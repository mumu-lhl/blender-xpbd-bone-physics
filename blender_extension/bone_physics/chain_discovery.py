"""Deterministic Blender-independent bone-chain discovery."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True, slots=True)
class BoneNode:
    """The hierarchy fields sampled from one Blender pose bone."""

    uuid: str
    name: str
    parent_uuid: str | None
    length: float


class BranchSelectionRequired(ValueError):
    """Raised when a head has more than one possible descendant leaf."""

    def __init__(self, head_uuid: str, leaves: tuple[str, ...]) -> None:
        self.head_uuid = head_uuid
        self.leaves = leaves
        super().__init__(
            f"branch selection required for head {head_uuid!r}: {', '.join(leaves)}"
        )


def chain_display_name(head_uuid: str | None, tail_uuid: str | None) -> str:
    """Return a stable, untranslated label for a configured chain path."""
    head = str(head_uuid or "Unnamed chain")
    tail = str(tail_uuid or "")
    return head if not tail or tail == head else f"{head} → {tail}"


def descendant_leaves(nodes: Iterable[BoneNode], head_uuid: str) -> tuple[str, ...]:
    """Return the stable-sorted leaves below ``head_uuid`` (including itself)."""
    by_id, children = _validated_topology(nodes)
    if head_uuid not in by_id:
        raise ValueError(f"head {head_uuid!r} not found")

    pending = [head_uuid]
    leaves: list[str] = []
    while pending:
        current = pending.pop()
        current_children = children[current]
        if not current_children:
            leaves.append(current)
            continue
        pending.extend(reversed(current_children))
    return tuple(sorted(leaves, key=lambda uuid: (by_id[uuid].name, uuid)))


def resolve_chain(
    nodes: Iterable[BoneNode], head_uuid: str, tail_uuid: str | None = None
) -> tuple[str, ...]:
    """Resolve one unique parent path, asking callers to choose at branches."""
    materialized = tuple(nodes)
    by_id, _children = _validated_topology(materialized)
    if head_uuid not in by_id:
        raise ValueError(f"head {head_uuid!r} not found")
    if tail_uuid is not None and tail_uuid not in by_id:
        raise ValueError(f"tail {tail_uuid!r} not found")

    leaves = descendant_leaves(materialized, head_uuid)
    if tail_uuid is None:
        if len(leaves) != 1:
            raise BranchSelectionRequired(head_uuid, leaves)
        tail_uuid = leaves[0]
    elif tail_uuid not in leaves:
        raise ValueError("tail must be a descendant leaf")

    return _parent_path(by_id, head_uuid, tail_uuid)


def validate_new_paths(
    existing_paths: Iterable[Iterable[str]], new_paths: Iterable[Iterable[str]]
) -> None:
    """Reject duplicate selections and any re-use of a physical bone."""
    existing = tuple(tuple(path) for path in existing_paths)
    new = tuple(tuple(path) for path in new_paths)
    paths = existing + new
    if any(not path for path in paths):
        raise ValueError("path must not be empty")

    duplicate_paths = sorted({path for path in new if new.count(path) > 1})
    if duplicate_paths:
        raise ValueError(f"duplicate new path: {duplicate_paths[0]!r}")

    owners: dict[str, int] = {}
    for path in paths:
        for uuid in path:
            owners[uuid] = owners.get(uuid, 0) + 1
    overlap = sorted(uuid for uuid, count in owners.items() if count > 1)
    if overlap:
        raise ValueError(f"physical bone overlap: {overlap[0]}")


def prune_chain_ids(
    chain_ids: Iterable[str], removed_ids: Iterable[str]
) -> tuple[str, ...]:
    """Remove selected chain IDs while preserving the authored order."""
    removed = set(removed_ids)
    return tuple(chain_id for chain_id in chain_ids if chain_id not in removed)


def _validated_topology(
    nodes: Iterable[BoneNode],
) -> tuple[dict[str, BoneNode], dict[str, tuple[str, ...]]]:
    materialized = tuple(nodes)
    by_id: dict[str, BoneNode] = {}
    duplicates: set[str] = set()
    for node in materialized:
        if node.uuid in by_id:
            duplicates.add(node.uuid)
        else:
            by_id[node.uuid] = node
    if duplicates:
        raise ValueError(f"duplicate bone UUID: {min(duplicates)}")

    missing = sorted(
        (node.uuid, node.parent_uuid)
        for node in by_id.values()
        if node.parent_uuid is not None and node.parent_uuid not in by_id
    )
    if missing:
        uuid, parent_uuid = missing[0]
        raise ValueError(f"missing parent {parent_uuid!r} for bone {uuid!r}")

    _raise_on_parent_cycle(by_id)
    children: dict[str, list[str]] = {uuid: [] for uuid in by_id}
    for node in by_id.values():
        if node.parent_uuid is not None:
            children[node.parent_uuid].append(node.uuid)
    return by_id, {
        uuid: tuple(sorted(child_ids, key=lambda child: (by_id[child].name, child)))
        for uuid, child_ids in children.items()
    }


def _raise_on_parent_cycle(by_id: dict[str, BoneNode]) -> None:
    completed: set[str] = set()
    for start in sorted(by_id):
        path: list[str] = []
        positions: dict[str, int] = {}
        current: str | None = start
        while current is not None and current not in completed:
            previous_position = positions.get(current)
            if previous_position is not None:
                cycle = path[previous_position:]
                canonical = min(
                    tuple(cycle[index:] + cycle[:index]) for index in range(len(cycle))
                )
                rendered = canonical + (canonical[0],)
                raise ValueError(f"parent cycle: {' -> '.join(rendered)}")
            positions[current] = len(path)
            path.append(current)
            current = by_id[current].parent_uuid
        completed.update(path)


def _parent_path(
    by_id: dict[str, BoneNode], head_uuid: str, tail_uuid: str
) -> tuple[str, ...]:
    path = [tail_uuid]
    current = tail_uuid
    while current != head_uuid:
        parent_uuid = by_id[current].parent_uuid
        if parent_uuid is None:
            raise ValueError("tail must be a descendant leaf")
        path.append(parent_uuid)
        current = parent_uuid
    path.reverse()
    return tuple(path)
