"""Deterministic collision-surface construction independent of Blender imports."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import struct
from typing import Iterable

from .model import CollisionSurfaceSpec

Vec3 = tuple[float, float, float]
IDENTITY_MATRIX = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)


@dataclass(frozen=True, slots=True)
class MeshSource:
    vertices: tuple[Vec3, ...]
    polygons: tuple[tuple[int, ...], ...]
    polygon_regions: tuple[int, ...] = ()
    polygon_materials: tuple[str | None, ...] = ()
    object_to_simulation: tuple[float, ...] = IDENTITY_MATRIX

    def __post_init__(self) -> None:
        vertices = tuple(_vec3("mesh vertex", vertex) for vertex in self.vertices)
        polygons = tuple(tuple(polygon) for polygon in self.polygons)
        for polygon in polygons:
            if len(polygon) < 3:
                continue
            for vertex in polygon:
                if (
                    isinstance(vertex, bool)
                    or not isinstance(vertex, int)
                    or not 0 <= vertex < len(vertices)
                ):
                    raise ValueError("mesh polygon vertex index is out of range")
        regions = tuple(self.polygon_regions)
        if regions and len(regions) != len(polygons):
            raise ValueError("polygon region count must match polygon count")
        for region in regions:
            _positive_id("polygon region", region)
        materials = tuple(self.polygon_materials)
        if materials and len(materials) != len(polygons):
            raise ValueError("polygon material count must match polygon count")
        if any(material == "" for material in materials):
            raise ValueError("polygon material names must not be empty")
        matrix = tuple(float(value) for value in self.object_to_simulation)
        if len(matrix) != 16 or not all(math.isfinite(value) for value in matrix):
            raise ValueError("object_to_simulation must be one finite 4x4 matrix")
        object.__setattr__(self, "vertices", vertices)
        object.__setattr__(self, "polygons", polygons)
        object.__setattr__(self, "polygon_regions", regions)
        object.__setattr__(self, "polygon_materials", materials)
        object.__setattr__(self, "object_to_simulation", matrix)


@dataclass(frozen=True, slots=True)
class CollisionMeshTopology:
    vertex_count: int
    triangles: tuple[tuple[int, int, int], ...]
    closed_hint: bool


@dataclass(frozen=True, slots=True)
class MeshBuildValidation:
    discarded_degenerate_faces: int = 0
    discarded_duplicate_faces: int = 0
    discarded_excluded_faces: int = 0
    source_vertex_count: int = 0
    output_vertex_count: int = 0


@dataclass(frozen=True, slots=True)
class CollisionMeshBatch:
    mesh_id: int
    topology: CollisionMeshTopology
    previous_vertices: tuple[Vec3, ...]
    current_vertices: tuple[Vec3, ...]
    regions: tuple[int, ...]
    thickness: float
    body_region_ignores: tuple[tuple[int, tuple[int, ...]], ...]
    topology_fingerprint: str
    deformation_fingerprint: str
    source_topology_fingerprint: str
    source_vertex_indices: tuple[int, ...]
    validation: MeshBuildValidation


def build_collision_surface(
    evaluated_object: MeshSource | object,
    policy: CollisionSurfaceSpec,
    *,
    previous: MeshSource | object | None = None,
    template: CollisionMeshBatch | None = None,
) -> CollisionMeshBatch:
    current_source = _mesh_source(evaluated_object, policy)
    previous_source = current_source if previous is None else _mesh_source(previous, policy)
    if len(previous_source.vertices) != len(current_source.vertices):
        raise ValueError("previous and current collision meshes must have equal vertex counts")
    if previous_source.polygons != current_source.polygons:
        raise ValueError("collision mesh topology changed between motion samples")
    source_topology_fingerprint = _fingerprint_source(current_source, policy)
    if template is not None:
        if template.mesh_id != policy.id:
            raise ValueError("collision mesh template belongs to a different mesh ID")
        if template.source_topology_fingerprint != source_topology_fingerprint:
            raise ValueError("collision mesh source topology changed")

    current_unmerged = tuple(
        _transform_point(current_source.object_to_simulation, vertex)
        for vertex in current_source.vertices
    )
    previous_unmerged = tuple(
        _transform_point(previous_source.object_to_simulation, vertex)
        for vertex in previous_source.vertices
    )
    if template is not None:
        current_vertices = tuple(
            current_unmerged[index] for index in template.source_vertex_indices
        )
        previous_vertices = tuple(
            previous_unmerged[index] for index in template.source_vertex_indices
        )
        return CollisionMeshBatch(
            mesh_id=policy.id,
            topology=template.topology,
            previous_vertices=previous_vertices,
            current_vertices=current_vertices,
            regions=template.regions,
            thickness=policy.thickness,
            body_region_ignores=policy.body_region_ignores,
            topology_fingerprint=template.topology_fingerprint,
            deformation_fingerprint=_fingerprint_vertices(
                previous_vertices,
                current_vertices,
            ),
            source_topology_fingerprint=source_topology_fingerprint,
            source_vertex_indices=template.source_vertex_indices,
            validation=template.validation,
        )
    current_vertices, source_to_output, representatives = _simplify_vertices(
        current_unmerged,
        policy.simplify_max_error,
    )
    previous_vertices = tuple(previous_unmerged[index] for index in representatives)

    triangles = []
    regions = []
    seen: set[tuple[int, int, int]] = set()
    discarded_degenerate = 0
    discarded_duplicate = 0
    discarded_excluded = 0
    excluded_materials = set(policy.excluded_materials)
    excluded_regions = set(policy.excluded_region_ids)
    for polygon_index, polygon in enumerate(current_source.polygons):
        material = (
            current_source.polygon_materials[polygon_index]
            if current_source.polygon_materials
            else None
        )
        region = (
            current_source.polygon_regions[polygon_index]
            if current_source.polygon_regions
            else policy.default_region_id
        )
        fan_count = max(1, len(polygon) - 2)
        if material in excluded_materials or region in excluded_regions:
            discarded_excluded += fan_count
            continue
        if len(polygon) < 3:
            discarded_degenerate += 1
            continue
        for offset in range(1, len(polygon) - 1):
            triangle = (
                source_to_output[polygon[0]],
                source_to_output[polygon[offset]],
                source_to_output[polygon[offset + 1]],
            )
            if _degenerate(triangle, current_vertices):
                discarded_degenerate += 1
                continue
            canonical = tuple(sorted(triangle))
            if canonical in seen:
                discarded_duplicate += 1
                continue
            seen.add(canonical)
            triangles.append(triangle)
            regions.append(region)
    if not triangles:
        raise ValueError("collision surface has no valid triangles")

    topology = CollisionMeshTopology(
        vertex_count=len(current_vertices),
        triangles=tuple(triangles),
        closed_hint=policy.closed_hint,
    )
    topology_fingerprint = _fingerprint_topology(
        topology,
        tuple(regions),
        policy,
    )
    deformation_fingerprint = _fingerprint_vertices(
        previous_vertices,
        current_vertices,
    )
    return CollisionMeshBatch(
        mesh_id=policy.id,
        topology=topology,
        previous_vertices=previous_vertices,
        current_vertices=current_vertices,
        regions=tuple(regions),
        thickness=policy.thickness,
        body_region_ignores=policy.body_region_ignores,
        topology_fingerprint=topology_fingerprint,
        deformation_fingerprint=deformation_fingerprint,
        source_topology_fingerprint=source_topology_fingerprint,
        source_vertex_indices=representatives,
        validation=MeshBuildValidation(
            discarded_degenerate_faces=discarded_degenerate,
            discarded_duplicate_faces=discarded_duplicate,
            discarded_excluded_faces=discarded_excluded,
            source_vertex_count=len(current_unmerged),
            output_vertex_count=len(current_vertices),
        ),
    )


def _mesh_source(value: MeshSource | object, policy: CollisionSurfaceSpec) -> MeshSource:
    if isinstance(value, MeshSource):
        return value
    return mesh_source_from_evaluated(value, policy)


def mesh_source_from_evaluated(
    evaluated_object: object,
    policy: CollisionSurfaceSpec,
    object_to_simulation: tuple[float, ...] | None = None,
) -> MeshSource:
    mesh = getattr(evaluated_object, "data", None)
    if mesh is None or not hasattr(mesh, "vertices") or not hasattr(mesh, "polygons"):
        raise ValueError("evaluated collision object must contain mesh data")
    vertices = tuple(tuple(vertex.co) for vertex in mesh.vertices)
    mesh.calc_loop_triangles()
    polygons = tuple(tuple(triangle.vertices) for triangle in mesh.loop_triangles)
    materials = tuple(
        _material_name(
            evaluated_object,
            mesh.polygons[triangle.polygon_index].material_index,
        )
        for triangle in mesh.loop_triangles
    )
    regions = tuple(
        _polygon_region(mesh, triangle.polygon_index, policy)
        for triangle in mesh.loop_triangles
    )
    matrix_world = getattr(evaluated_object, "matrix_world", None)
    object_matrix = (
        tuple(float(matrix_world[row][column]) for row in range(4) for column in range(4))
        if matrix_world is not None
        else IDENTITY_MATRIX
    )
    matrix = (
        object_matrix
        if object_to_simulation is None
        else _matrix_multiply(object_to_simulation, object_matrix)
    )
    return MeshSource(vertices, polygons, regions, materials, matrix)


def merge_upstream_weights(collision_object: object, policy: CollisionSurfaceSpec) -> None:
    groups = getattr(collision_object, "vertex_groups", None)
    mesh = getattr(collision_object, "data", None)
    if groups is None or mesh is None:
        raise ValueError("collision object must expose mesh vertices and vertex groups")
    for physical_name, upstream_name in policy.upstream_weight_merges:
        physical = groups.get(physical_name)
        if physical is None:
            continue
        upstream = groups.get(upstream_name) or groups.new(name=upstream_name)
        physical_index = physical.index
        for vertex in mesh.vertices:
            weight = next(
                (
                    assignment.weight
                    for assignment in vertex.groups
                    if assignment.group == physical_index
                ),
                0.0,
            )
            if weight > 0.0:
                upstream.add((vertex.index,), weight, "ADD")
                physical.remove((vertex.index,))


def prepare_collision_source(
    original_object: object,
    policy: CollisionSurfaceSpec,
    collection: object | None = None,
) -> object:
    if getattr(original_object, "type", None) != "MESH":
        raise ValueError("collision source object must be a mesh")
    collision_object = original_object.copy()
    collision_object.data = original_object.data.copy()
    collision_object.name = f"{original_object.name}__bone_physics_collision"
    collision_object.hide_render = True
    collision_object["bone_physics_collision_source"] = original_object.name
    if collection is None:
        collections = getattr(original_object, "users_collection", ())
        if not collections:
            raise ValueError("collision source object is not linked to a collection")
        collection = collections[0]
    collection.objects.link(collision_object)
    merge_upstream_weights(collision_object, policy)
    return collision_object


def _simplify_vertices(
    vertices: tuple[Vec3, ...],
    max_error: float,
) -> tuple[tuple[Vec3, ...], tuple[int, ...], tuple[int, ...]]:
    if max_error == 0.0:
        indices = tuple(range(len(vertices)))
        return vertices, indices, indices
    cell_size = max_error / math.sqrt(3.0)
    cells: dict[tuple[int, int, int], int] = {}
    output = []
    representatives = []
    source_to_output = []
    for source_index, vertex in enumerate(vertices):
        cell = tuple(math.floor(value / cell_size) for value in vertex)
        output_index = cells.get(cell)
        if output_index is None:
            output_index = len(output)
            cells[cell] = output_index
            output.append(vertex)
            representatives.append(source_index)
        source_to_output.append(output_index)
    return tuple(output), tuple(source_to_output), tuple(representatives)


def _transform_point(matrix: tuple[float, ...], point: Vec3) -> Vec3:
    x, y, z = point
    homogeneous = (
        matrix[12] * x + matrix[13] * y + matrix[14] * z + matrix[15]
    )
    if not math.isfinite(homogeneous) or abs(homogeneous) <= 1.0e-12:
        raise ValueError("collision transform produced a non-finite projective point")
    result = tuple(
        (
            matrix[row * 4] * x
            + matrix[row * 4 + 1] * y
            + matrix[row * 4 + 2] * z
            + matrix[row * 4 + 3]
        )
        / homogeneous
        for row in range(3)
    )
    return _vec3("transformed mesh vertex", result)


def _matrix_multiply(
    left: tuple[float, ...],
    right: tuple[float, ...],
) -> tuple[float, ...]:
    if len(left) != 16 or len(right) != 16:
        raise ValueError("matrix multiplication requires two 4x4 matrices")
    return tuple(
        sum(left[row * 4 + inner] * right[inner * 4 + column] for inner in range(4))
        for row in range(4)
        for column in range(4)
    )


def _degenerate(triangle: tuple[int, int, int], vertices: tuple[Vec3, ...]) -> bool:
    if len(set(triangle)) != 3:
        return True
    a, b, c = (vertices[index] for index in triangle)
    ab = tuple(right - left for left, right in zip(a, b))
    ac = tuple(right - left for left, right in zip(a, c))
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return sum(value * value for value in cross) <= 1.0e-16


def _fingerprint_topology(
    topology: CollisionMeshTopology,
    regions: tuple[int, ...],
    policy: CollisionSurfaceSpec,
) -> str:
    digest = hashlib.sha256()
    digest.update(struct.pack("<I?", topology.vertex_count, topology.closed_hint))
    digest.update(struct.pack("<d", policy.thickness))
    for triangle, region in zip(topology.triangles, regions):
        digest.update(struct.pack("<IIII", *triangle, region))
    for body_id, ignored_regions in policy.body_region_ignores:
        digest.update(struct.pack("<II", body_id, len(ignored_regions)))
        for region in ignored_regions:
            digest.update(struct.pack("<I", region))
    return digest.hexdigest()


def _fingerprint_vertices(
    previous: tuple[Vec3, ...],
    current: tuple[Vec3, ...],
) -> str:
    digest = hashlib.sha256()
    for vertex in previous + current:
        digest.update(struct.pack("<ddd", *vertex))
    return digest.hexdigest()


def _fingerprint_source(
    source: MeshSource,
    policy: CollisionSurfaceSpec,
) -> str:
    digest = hashlib.sha256()
    digest.update(struct.pack("<I", len(source.vertices)))
    digest.update(struct.pack("<d?", policy.simplify_max_error, policy.closed_hint))
    excluded_materials = set(policy.excluded_materials)
    excluded_regions = set(policy.excluded_region_ids)
    for index, polygon in enumerate(source.polygons):
        digest.update(struct.pack("<I", len(polygon)))
        for vertex in polygon:
            digest.update(struct.pack("<I", vertex))
        region = (
            source.polygon_regions[index]
            if source.polygon_regions
            else policy.default_region_id
        )
        material = (
            source.polygon_materials[index]
            if source.polygon_materials
            else None
        )
        digest.update(struct.pack("<I", region))
        digest.update(
            struct.pack(
                "<?",
                material in excluded_materials or region in excluded_regions,
            )
        )
    return digest.hexdigest()


def _polygon_region(mesh: object, polygon_index: int, policy: CollisionSurfaceSpec) -> int:
    attributes = getattr(mesh, "attributes", None)
    attribute = attributes.get(policy.region_attribute) if attributes is not None else None
    if attribute is None:
        return policy.default_region_id
    value = int(attribute.data[polygon_index].value)
    _positive_id("polygon region", value)
    return value


def _material_name(evaluated_object: object, material_index: int) -> str | None:
    slots = getattr(evaluated_object, "material_slots", ())
    if not 0 <= material_index < len(slots):
        return None
    material = getattr(slots[material_index], "material", None)
    return None if material is None else str(material.name)


def _vec3(name: str, values: Iterable[float]) -> Vec3:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} values must be three finite numbers")
    return result  # type: ignore[return-value]


def _positive_id(kind: str, value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value <= 0xFFFFFFFF:
        raise ValueError(f"{kind} ID must be an unsigned 32-bit integer greater than zero")
