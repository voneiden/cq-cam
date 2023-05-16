import logging
from collections import defaultdict
from typing import TYPE_CHECKING

import cadquery as cq
from OCP.StdFail import StdFail_NotDone
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer

from cq_cam.command import CommandVector
from cq_cam.routers import route_wires
from cq_cam.utils.geometry_op import offset_face, offset_wire
from cq_cam.utils.tree import Tree
from cq_cam.utils.utils import break_compound_to_faces

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from cq_cam.fluent import Job


def generate_depth_map(faces: list[cq.Face]):
    depth_map = defaultdict(list)

    for face in faces:
        bbox = face.BoundingBox()
        if bbox.zmax > 0:
            raise ValueError("Face is above job plane")

        depth_map[bbox.zmax].append(face)

    depths = list(depth_map.keys())
    depths.sort(reverse=True)

    return depth_map, depths


def combine_faces(faces: list[cq.Face]) -> list[cq.Face]:
    """Given a list of faces, fuse them together to form
    bigger faces"""

    # This works also as a sanity check as it will
    # raise if the faces are not coplanar
    wp = cq.Workplane().add(faces).combine()
    new_faces = []
    explorer = TopExp_Explorer(wp.objects[0].wrapped, TopAbs_FACE)
    while explorer.More():
        face = explorer.Current()
        new_faces.append(cq.Face(face))
        explorer.Next()
    return new_faces


def build_pocket_ops(
    job: "Job", op_areas: list[cq.Face], avoid_areas: list[cq.Face]
) -> list[cq.Face]:
    # Determine depth of each face
    depth_map, depths = generate_depth_map(op_areas)
    avoid_depth_map, avoid_depths = generate_depth_map(avoid_areas)

    pocket_ops = []
    # Iterate though each depth and construct the depth geometry
    for i, depth in enumerate(depths):
        depth_faces = depth_map[depth]
        for sub_depth in depths[i + 1 :]:
            # Move faces UP
            depth_faces += [
                face.translate(job.top.zDir.multiply(depth - sub_depth))
                for face in depth_map[sub_depth]
            ]

        depth_ops = combine_faces(depth_faces)

        if avoid_depths:
            active_avoid_depths = [
                avoid_depth for avoid_depth in avoid_depths if avoid_depth >= depth
            ]
            avoid_faces = []
            for avoid_depth in active_avoid_depths:
                # Move faces DOWN
                avoid_faces += [
                    face.translate(job.top.zDir.multiply(depth - avoid_depth))
                    for face in avoid_depth_map[avoid_depth]
                ]
            depth_ops_with_avoid = []
            for i, depth_op in enumerate(depth_ops):
                depth_ops_with_avoid += break_compound_to_faces(
                    depth_op.cut(*avoid_faces)
                )

            pocket_ops += depth_ops_with_avoid

        else:
            pocket_ops += depth_ops

    return pocket_ops


def fill_pocket(pocket: cq.Face, offset: float) -> list[list[cq.Wire]]:
    return fill_pocket_contour_shrink(pocket, offset)


def fill_pocket_contour_shrink(pocket: cq.Face, step: float) -> list[list[cq.Wire]]:
    inner_faces = [cq.Face.makeFromWires(inner) for inner in pocket.innerWires()]
    tree = Tree(pocket.outerWire())
    i = 0

    # TODO sanify the variable names here
    try:
        while True:
            node = tree.next_unlocked
            next_outer_candidates = offset_wire(node.obj, -step, "arc")
            if inner_faces:
                next_outers = []
                for next_outer in next_outer_candidates:
                    try:
                        outer_face = cq.Face.makeFromWires(next_outer)
                    except StdFail_NotDone:
                        continue
                    outer_compound = outer_face.cut(*inner_faces)
                    compound_faces = break_compound_to_faces(outer_compound)
                    if compound_faces:
                        next_outers += [face.outerWire() for face in compound_faces]
            else:
                next_outers = next_outer_candidates

            if next_outers:
                node.branch(next_outers)
            else:
                node.lock()

            i += 1

    except StopIteration:
        pass

    return tree.sequences


def pocket_cq(
    job: "Job",
    op_areas: list[cq.Face],
    avoid_areas: list[cq.Face],
    outer_offset: float,
    inner_offset: float,
    avoid_outer_offset: float,
    avoid_inner_offset: float,
    stepover: float,
):
    # Offset faces
    offset_op_areas = []
    offset_avoid_areas = []

    for face in op_areas:
        offset_op_areas += offset_face(face, outer_offset, inner_offset)

    for face in avoid_areas:
        offset_avoid_areas += offset_face(face, avoid_outer_offset, avoid_inner_offset)

    # Build pocket boundary geometry (Face)
    pocket_ops = build_pocket_ops(job, offset_op_areas, offset_avoid_areas)

    # Apply pocket fill (convert to Wire)
    sequences: list[list[cq.Wire]] = []
    for pocket_op in pocket_ops:
        sequences += fill_pocket_contour_shrink(pocket_op, stepover)

    # Route wires
    for sequence_wires in sequences:
        commands += route_wires(job, sequence_wires, stepover=stepover)

    return commands
