import logging
from collections import defaultdict
from typing import TYPE_CHECKING, List, Literal, Optional

import cadquery as cq

from cq_cam.operations.pocket_cq import pocket_cq
from cq_cam.routers import route_polyface_outers
from cq_cam.utils.geometry_op import (
    OffsetInput,
    Path,
    PathFace,
    calculate_offset,
    difference_poly_tree,
    offset_path,
    offset_polyface,
    union_poly_tree,
)
from cq_cam.utils.tree import Tree
from cq_cam.utils.utils import flatten_list

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from cq_cam.fluent import Job


def pocket(
    job: "Job",
    op_areas: List[cq.Face],
    avoid_areas: Optional[List[cq.Face]] = None,
    outer_offset: Optional[OffsetInput] = None,
    inner_offset: Optional[OffsetInput] = None,
    avoid_outer_offset: Optional[OffsetInput] = None,
    avoid_inner_offset: Optional[OffsetInput] = None,
    stepover: Optional[OffsetInput] = None,
    stepdown: Optional[float] = None,
    engine: Literal["clipper", "cq"] = "clipper",
):
    if avoid_areas is None:
        avoid_areas = []

    if avoid_areas and outer_offset is None:
        outer_offset = 0

    # Determine absolute offsets
    outer_offset = calculate_offset(job.tool_radius, outer_offset, -1)
    inner_offset = calculate_offset(job.tool_radius, inner_offset, 1)
    avoid_outer_offset = calculate_offset(job.tool_radius, avoid_outer_offset, 1)
    avoid_inner_offset = calculate_offset(job.tool_radius, avoid_inner_offset, -1)
    stepover = calculate_offset(job.tool_radius, stepover, 0.5)

    # Transform to job plane
    op_areas = [face.transformShape(job.top.fG) for face in op_areas]
    avoid_areas = [face.transformShape(job.top.fG) for face in avoid_areas]

    # TODO stepdown
    if engine == "clipper":
        return pocket_clipper(
            job,
            [PathFace.from_cq_face(face) for face in op_areas],
            [PathFace.from_cq_face(face) for face in avoid_areas],
            outer_offset,
            inner_offset,
            avoid_outer_offset,
            avoid_inner_offset,
            stepover,
        )
    elif engine == "cq":
        return pocket_cq(
            job,
            op_areas,
            avoid_areas,
            outer_offset,
            inner_offset,
            avoid_outer_offset,
            avoid_inner_offset,
            stepover,
        )
    else:
        raise ValueError("Unknown engine")


def generate_depth_map(poly_faces: List[PathFace]):
    depth_map = defaultdict(list)

    for face in poly_faces:
        if face.depth > 0:
            raise ValueError("PolyFace depth is above job plane")

        depth_map[face.depth].append(face)

    depths = list(depth_map.keys())
    depths.sort(reverse=True)

    return depth_map, depths


def combine_poly_faces(poly_faces: List[PathFace], depth) -> List[PathFace]:
    outers = [poly_face.outer for poly_face in poly_faces]
    inners = flatten_list([poly_face.inners for poly_face in poly_faces])

    new_outers = [poly_face.outer for poly_face in union_poly_tree(outers, [], depth)]
    return difference_poly_tree(new_outers, inners, depth)


def combine_outers(poly_faces: List[PathFace], depth) -> List[Path]:
    outers = [poly_face.outer for poly_face in poly_faces]
    return [poly_face.outer for poly_face in union_poly_tree(outers, [], depth)]


def build_pocket_ops(
    job: "Job", op_areas: List[PathFace], avoid_areas: List[PathFace]
) -> List[PathFace]:
    # Determine depth of each face
    depth_map, depths = generate_depth_map(op_areas)
    avoid_depth_map, avoid_depths = generate_depth_map(avoid_areas)

    pocket_ops = []
    # Iterate though each depth and construct the depth geometry
    for i, depth in enumerate(depths):
        depth_faces = depth_map[depth]
        for sub_depth in depths[i + 1 :]:
            depth_faces += depth_map[sub_depth]

        depth_ops = combine_outers(depth_faces, depth)
        depth_inners = flatten_list(face.inners for face in depth_faces)

        if avoid_depths:
            active_avoid_depths = [
                avoid_depth for avoid_depth in avoid_depths if avoid_depth >= depth
            ]
            avoid_faces = []
            for avoid_depth in active_avoid_depths:
                avoid_faces += avoid_depth_map[avoid_depth]
            avoid_outers = [face.outer for face in avoid_faces]

            # A limitation here is that avoids only work with the outer polygon
            # Technically pyclipper does support multiple depths, so this could
            # be investigated further
            depth_ops_with_avoid = difference_poly_tree(
                depth_ops, depth_inners + avoid_outers, depth
            )

            pocket_ops += depth_ops_with_avoid

        else:
            pocket_ops += difference_poly_tree(depth_ops, depth_inners, depth)

    return pocket_ops


def fill_pocket_contour_shrink(pocket: PathFace, step: float) -> list[PathFace]:
    tree = Tree(PathFace(pocket.outer, [], depth=pocket.depth))
    i = 0

    # TODO sanify the variable names here
    try:
        while True:
            node = tree.next_unlocked
            next_outer_candidates = offset_path(node.obj.outer, -step)
            if next_outer_candidates and pocket.inners:
                next_outers = [
                    face.outer
                    for face in difference_poly_tree(
                        next_outer_candidates, pocket.inners, 0
                    )
                ]
            else:
                next_outers = next_outer_candidates

            if next_outers:
                node.branch(
                    [PathFace(outer, [], pocket.depth) for outer in next_outers]
                )
            else:
                node.lock()

            i += 1

    except StopIteration:
        pass

    return tree.sequences


def pocket_clipper(
    job,
    op_areas: list[PathFace],
    avoid_areas: list[PathFace],
    outer_offset: float,
    inner_offset: float,
    avoid_outer_offset: float,
    avoid_inner_offset: float,
    stepover: float,
):
    # Offset faces
    offset_op_areas: list[PathFace] = []
    offset_avoid_areas: list[PathFace] = []

    for face in op_areas:
        offset_op_areas += offset_polyface(face, outer_offset, inner_offset)

    for face in avoid_areas:
        offset_avoid_areas += offset_polyface(
            face, avoid_outer_offset, avoid_inner_offset
        )

    pocket_ops = build_pocket_ops(job, offset_op_areas, offset_avoid_areas)

    # Apply pocket fill
    sequences: List[List[PathFace]] = []
    for pocket_op in pocket_ops:
        sequences += fill_pocket_contour_shrink(pocket_op, stepover)

    # Route wires
    commands = []
    for sequence_polyfaces in sequences:
        commands += route_polyface_outers(job, sequence_polyfaces, stepover=stepover)

    return commands
