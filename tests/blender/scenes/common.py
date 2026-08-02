from __future__ import annotations

from dataclasses import dataclass

from blender_extension.bone_physics.model import (
    AnalyticColliderSpec,
    AnalyticShape,
    BoneSpec,
    WorldSpec,
)


@dataclass(frozen=True)
class ScenarioFixture:
    name: str
    body_count: int
    chain_count: int
    contact_required: bool = False
    expected_contacts: int = 0

    def build_batch(self) -> dict[str, object]:
        return {
            "scenario": self.name,
            "body_count": self.body_count,
            "chain_count": self.chain_count,
            "contact_required": self.contact_required,
        }

    def build_world_spec(self, seed: int) -> WorldSpec:
        """Build a deterministic native world for acceptance measurements."""
        if self.chain_count < 1 or self.body_count < self.chain_count:
            raise ValueError("scenario fixture must contain at least one body per chain")
        lengths = [self.body_count // self.chain_count] * self.chain_count
        for index in range(self.body_count % self.chain_count):
            lengths[index] += 1
        bones = []
        next_id = 1
        for chain_index, length in enumerate(lengths):
            x = chain_index * 0.60
            y = 0.0
            parent_id = None
            for _ in range(length):
                bone_id = next_id
                next_id += 1
                bones.append(
                    BoneSpec(
                        id=bone_id,
                        name=f"{self.name}_bone_{bone_id}",
                        local_head=(0.0, 0.0, 0.0),
                        local_tail=(0.0, 0.12, 0.0),
                        parent_id=parent_id,
                        joint_constraint_id=(2000 + bone_id) if parent_id is not None else None,
                        rest_translation=(x, y, 0.0),
                        initial_translation=(x, y, 0.0),
                        inverse_mass=0.0 if parent_id is None else 1.0,
                        radius=0.04,
                        margin=0.005,
                        restitution=0.0,
                    )
                )
                parent_id = bone_id
                y += 0.12
        colliders = ()
        if self.contact_required:
            colliders = (
                AnalyticColliderSpec(
                    id=1001,
                    shape=AnalyticShape.SPHERE,
                    # Tangent to the first capsule.  This produces a real
                    # contact without pinning a fixed root inside the proxy.
                    translation=(0.145, 0.12, 0.0),
                    radius=0.10,
                ),
            )
        return WorldSpec(
            bones=tuple(bones),
            analytic_colliders=colliders,
            gravity=(0.0, -9.81, 0.0),
            fixed_dt=1.0 / 120.0,
        )
