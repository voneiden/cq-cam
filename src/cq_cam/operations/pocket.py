import logging
from collections import defaultdict
from typing import TYPE_CHECKING, Literal

import cadquery as cq
import numpy as np

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
    op_areas: list[cq.Face],
    avoid_areas: list[cq.Face] | None = None,
    outer_offset: OffsetInput | None = None,
    inner_offset: OffsetInput | None = None,
    avoid_outer_offset: OffsetInput | None = None,
    avoid_inner_offset: OffsetInput | None = None,
    stepover: OffsetInput | None = None,
    stepdown: float | None = None,
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
            stepdown,
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


def generate_depth_map(poly_faces: list[PathFace]):
    depth_map = defaultdict(list)

    for face in poly_faces:
        if face.depth > 0:
            raise ValueError("PolyFace depth is above job plane")

        depth_map[face.depth].append(face)

    depths = list(depth_map.keys())
    depths.sort(reverse=True)

    return depth_map, depths


def combine_poly_faces(poly_faces: list[PathFace], depth) -> list[PathFace]:
    outers = [poly_face.outer for poly_face in poly_faces]
    inners = flatten_list([poly_face.inners for poly_face in poly_faces])

    new_outers = [poly_face.outer for poly_face in union_poly_tree(outers, [], depth)]
    return difference_poly_tree(new_outers, inners, depth)


def combine_outers(poly_faces: list[PathFace], depth) -> list[Path]:
    outers = [poly_face.outer for poly_face in poly_faces]
    return [poly_face.outer for poly_face in union_poly_tree(outers, [], depth)]


def determine_stepdown_start_depth(
    pocket_op: PathFace, shallower_pocket_ops: list[PathFace]
) -> float | None:
    stepdown_start_depth = [
        op.depth
        for op in shallower_pocket_ops
        if op.polygon.contains(pocket_op.polygon)
    ]
    if stepdown_start_depth:
        stepdown_start_depth.sort()
        return stepdown_start_depth[0]
    else:
        return None


def apply_stepdown(
    sequences: list[list[PathFace]], start_depth: float | None, stepdown: float
) -> list[list[PathFace]]:
    start_depth = -stepdown if start_depth is None else start_depth - stepdown

    end_depths = set([path.depth for sequence in sequences for path in sequence])
    if len(end_depths) != 1:
        raise RuntimeError("Multiple depths in sequences")

    step_depths = np.arange(start_depth, list(end_depths)[0], -stepdown)
    stepped_sequences = []
    for step_depth in step_depths:
        for sequence in sequences:
            step_sequence = []
            for path in sequence:
                step_sequence.append(path.clone_to_depth(step_depth))
            stepped_sequences.append(step_sequence)

    return stepped_sequences


def build_pocket_ops(
    op_areas: list[PathFace], avoid_areas: list[PathFace]
) -> list[PathFace]:
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


def fill_pocket_contour_shrink(pocket: PathFace, step: float) -> list[list[PathFace]]:
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
    job: "Job",
    op_areas: list[PathFace],
    avoid_areas: list[PathFace],
    outer_offset: float,
    inner_offset: float,
    avoid_outer_offset: float,
    avoid_inner_offset: float,
    stepover: float,
    stepdown: float,
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

    pocket_ops = build_pocket_ops(offset_op_areas, offset_avoid_areas)

    # Apply pocket fill
    sequences: list[list[PathFace]] = []
    shallower_pocket_ops = []
    for pocket_op in pocket_ops:
        fill_sequences = fill_pocket_contour_shrink(pocket_op, stepover)
        if stepdown:
            stepdown_start_depth = determine_stepdown_start_depth(
                pocket_op, shallower_pocket_ops
            )
            sequences += apply_stepdown(fill_sequences, stepdown_start_depth, stepdown)

        sequences += fill_sequences
        shallower_pocket_ops.append(pocket_op)

    # Route wires
    commands = []
    for sequence_polyfaces in sequences:
        commands += route_polyface_outers(job, sequence_polyfaces, stepover=stepover)

    return commands
