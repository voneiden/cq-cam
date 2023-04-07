import logging
from collections import defaultdict
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

import cadquery as cq
import numpy as np
from OCP.BRepFeat import BRepFeat
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer

from cq_cam.command import Cut, Plunge, Rapid
from cq_cam.operations.base_operation import FaceBaseOperation, OperationError
from cq_cam.operations.mixin_operation import (
    ObjectsValidationMixin,
    PlaneValidationMixin,
)
from cq_cam.operations.strategy import Strategy, ZigZagStrategy
from cq_cam.utils.offset import OffsetInput, calculate_offset, offset_face, offset_wire
from cq_cam.utils.utils import WireClipper, flatten_list, flatten_wire_to_closed_2d

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from cq_cam.fluent import Job


def group_by_depth(wires: List[cq.Wire]):
    depth_map = defaultdict(list)
    for wire in wires:
        wire.po


def build_pocket_geometry_layers(
    top: cq.Plane,
    op_areas: List[cq.Wire],
    avoid_areas: List[cq.Wire],
):
    return []


def generate_depth_map(faces: List[cq.Face]):
    depth_map = defaultdict(list)

    for face in faces:
        bbox = face.BoundingBox()
        if bbox.zmax > 0:
            raise ValueError("Face is above job plane")

        depth_map[bbox.zmax].append(face)

    depths = list(depth_map.keys())
    depths.sort(reverse=True)

    return depth_map, depths


def combine_faces(faces: List[cq.Face]) -> List[cq.Face]:
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
    job: "Job", op_areas: List[cq.Face], avoid_areas: List[cq.Face]
) -> List[cq.Face]:
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
                for avoid_face in avoid_faces:
                    depth_op = depth_op.cut(avoid_face)
                depth_ops_with_avoid.append(depth_op)

            pocket_ops += depth_ops_with_avoid

        else:
            pocket_ops += depth_ops

    return pocket_ops


def fill_pocket(pocket: cq.Face, offset: float) -> List[List[cq.Wire]]:
    return fill_pocket_contour_shrink(pocket, offset)


def fill_pocket_contour_shrink(pocket: cq.Face, offset: float) -> List[List[cq.Wire]]:
    outer = pocket.outerWire()
    inners = pocket.innerWires()
    sequence = [outer]
    other_sequences = []
    i = 0

    while True:
        # TODO some kind of tree implementation would be nice

        if iteration_method(i) == "expand":
            offset_inners = flatten_list(
                [offset_wire(inner, offset) for inner in inners]
            )
            for outer in outers:
                new_outer_inners = cq.Face.makeFromWires(
                    outer, offset_inners
                ).innerWires()

        else:
            pass

    return sequence + other_sequences


def pocket2(
    job: "Job",
    op_areas: List[cq.Face],
    avoid_areas: List[cq.Face],
    outer_offset: Optional[OffsetInput] = None,
    inner_offset: Optional[OffsetInput] = None,
    avoid_outer_offset: Optional[OffsetInput] = None,
    avoid_inner_offset: Optional[OffsetInput] = None,
    show_object=None,
):
    if avoid_areas and outer_offset is None:
        outer_offset = 0

    # Determine absolute offsets
    outer_offset = calculate_offset(job.tool_radius, outer_offset, -1)
    inner_offset = calculate_offset(job.tool_radius, inner_offset, 1)
    avoid_outer_offset = calculate_offset(job.tool_radius, avoid_outer_offset, 1)
    avoid_inner_offset = calculate_offset(job.tool_radius, avoid_inner_offset, -1)

    # Transform to job plane
    op_areas = [face.transformShape(job.top.fG) for face in op_areas]
    avoid_areas = [face.transformShape(job.top.fG) for face in avoid_areas]

    # Offset faces
    offset_op_areas = []
    offset_avoid_areas = []

    for face in op_areas:
        offset_op_areas += offset_face(face, outer_offset, inner_offset)

    for face in avoid_areas:
        offset_avoid_areas += offset_face(face, avoid_outer_offset, avoid_inner_offset)

    # Build pocket boundary geometry
    pocket_ops = build_pocket_ops(job, offset_op_areas, offset_avoid_areas)

    # Apply pocket fill

    # Route wires

    # Route depths


@dataclass(kw_only=True)
class Pocket(PlaneValidationMixin, ObjectsValidationMixin, FaceBaseOperation):
    """2.5D Pocket operation

    All faces involved must be planes and parallel.
    """

    tool_diameter: float = 3.175
    strategy: Strategy = ZigZagStrategy
    """ Diameter of the tool that will be used to perform the operation.
    """

    # TODO rotation angle for zigzag

    @property
    def _tool_diameter(self) -> float:
        return self.tool_diameter

    def __post_init__(self):
        # Give each face an ID
        super().__post_init__()
        faces = [(i, face.copy()) for i, face in enumerate(self._faces)]
        features, coplanar_faces, depth_info = self._discover_pocket_features(faces)
        groups = self._group_faces_by_features(features, coplanar_faces)

        for group_faces in groups.values():
            boundaries = self._boundaries_by_group(
                group_faces, coplanar_faces, depth_info
            )
            for boundary, start_depth, end_depth in boundaries:
                self.process_boundary(boundary, start_depth, end_depth)

    @staticmethod
    def _group_faces_by_features(
        features: List[cq.Face], faces: List[Tuple[int, cq.Face]]
    ):
        feat = BRepFeat()
        remaining = faces[:]
        groups = defaultdict(lambda: [])
        for feature in features:
            for i, face in remaining[:]:
                # IsInside_s works regardless the faces are co-planar
                if feat.IsInside_s(face.wrapped, feature.wrapped):
                    groups[feature].append((i, face))
                    remaining.remove((i, face))

        assert not remaining
        return groups

    def _discover_pocket_features(self, faces: List[Tuple[int, cq.Face]]):
        """Given a list of faces, creates fused feature faces and returns depth information"""
        # Move everything on the same plane
        coplanar_faces = []
        job_zdir = self.job.top.zDir
        job_height = job_zdir.dot(self.job.top.origin)
        depth_info = {}
        for i, face in faces:
            face_height = job_zdir.dot(face.Center())
            # TODO rounding errors ([-0.10000000000000275, -0.1000000000000032])
            face_depth = face_height - job_height
            depth_info[i] = face_depth

            # Move face to same level with job plane
            translated_face = face.translate(job_zdir.multiply(-face_depth))
            coplanar_faces.append((i, translated_face))

        features = self._combine_coplanar_faces([f[1] for f in coplanar_faces])

        return features, coplanar_faces, depth_info

    @staticmethod
    def _combine_coplanar_faces(faces: List[cq.Face]) -> List[cq.Face]:
        """Given a list of (coplanar) faces, fuse them together to form
        bigger faces"""

        # This works also as a sanity check as it will
        # raise if the faces are not coplanar
        wp = cq.Workplane().add(faces).combine()
        features = []
        explorer = TopExp_Explorer(wp.objects[0].wrapped, TopAbs_FACE)
        while explorer.More():
            face = explorer.Current()
            features.append(cq.Face(face))
            explorer.Next()
        return features

    def _boundaries_by_group(
        self,
        group_faces: List[Tuple[int, cq.Face]],
        coplanar_faces: List[Tuple[int, cq.Face]],
        depth_info: Dict[int, float],
    ):
        face_depths = list(set(depth_info.values()))
        face_depths.sort()
        face_depths.reverse()
        current_depth = 0
        boundaries = []
        group_i = [i for i, _ in group_faces]
        for face_depth in face_depths:
            depth_i = [
                i
                for i, depth in depth_info.items()
                if depth <= face_depth and i in group_i
            ]
            feature_faces = [
                f for i, f in coplanar_faces if i in depth_i and i in group_i
            ]
            if not feature_faces:
                # Probably a bug
                logger.warning("Empty feature faces encountered")
                continue
            features = self._combine_coplanar_faces(feature_faces)
            assert len(features) == 1
            boundary = features[0]
            boundaries.append((boundary, current_depth, face_depth))
            current_depth = face_depth
        return boundaries

    def _generate_depths(self, start_depth: float, end_depth: float):
        if self.stepdown:
            depths = list(
                np.arange(start_depth - self.stepdown, end_depth, -self.stepdown)
            )
            if depths[-1] != end_depth:
                depths.append(end_depth)
            return depths
        else:
            return [end_depth]

    def _apply_avoid(
        self,
        outer_subject_wires,
        inner_subject_wires,
        avoid_objs,
        outer_offset,
        inner_offset,
    ):
        avoid_clip = WireClipper()
        for o in avoid_objs:
            if isinstance(o, cq.Face):
                # Use reverse offsets because avoid is like an anti-pocket
                outer_avoid_wires = o.outerWire().offset2D(inner_offset)
                inner_avoid_wires = flatten_list(
                    [wire.offset2D(outer_offset) for wire in o.innerWires()]
                )
            elif isinstance(o, cq.Wire):
                # TODO check this
                outer_avoid_wires = o.offset2D(inner_offset)
                inner_avoid_wires = []
            else:
                raise OperationError("Avoid can only be a wire or a face")

            for wire in outer_avoid_wires + inner_avoid_wires:
                avoid_clip.add_clip_wire(wire)

        for outer_subject in outer_subject_wires:
            avoid_clip.add_subject_wire(outer_subject)

        outer_boundaries = [
            list(boundary) for boundary in avoid_clip.execute_difference()
        ]
        avoid_clip.reset()

        for inner_subject in inner_subject_wires:
            avoid_clip.add_subject_wire(inner_subject)

        inner_boundaries = [
            list(boundary) for boundary in avoid_clip.execute_difference()
        ]

        return outer_boundaries, inner_boundaries

    def process_boundary(self, face: cq.Face, start_depth: float, end_depth: float):
        # TODO break this function down into easily testable sections
        # Perform validations
        self.validate_face_plane(face)
        face_workplane = cq.Workplane(obj=face)
        self.validate_plane(self.job, face_workplane)

        # Prepare profile paths
        tool_radius = self._tool_diameter / 2
        outer_wire_offset = (
            tool_radius * self.outer_boundary_offset[0] + self.outer_boundary_offset[1]
        )
        inner_wire_offset = (
            tool_radius * self.inner_boundary_offset[0] + self.inner_boundary_offset[1]
        )

        # These are the profile paths. They are done very last as a finishing pass
        outer_profiles = face.outerWire().offset2D(outer_wire_offset)
        inner_profiles = flatten_list(
            [wire.offset2D(inner_wire_offset) for wire in face.innerWires()]
        )

        # Prepare primary clearing regions
        if self.boundary_final_pass_stepover is None:
            self.boundary_final_pass_stepover = self.stepover
        final_pass_offset = tool_radius * self.boundary_final_pass_stepover

        # Generate the primary clearing regions with stepover from the above profiles
        # TODO these offsets can fail!
        outer_boundaries = flatten_list(
            [wire.offset2D(-final_pass_offset) for wire in outer_profiles]
        )
        inner_boundaries = []
        for inner_wire in inner_profiles:
            try:
                inner_boundaries.append(inner_wire.offset2D(final_pass_offset))
            except ValueError:
                # TODO failure mode
                logger.warning("Failed to offset wire")
                continue
        inner_boundaries = flatten_list(inner_boundaries)
        # TODO apply "avoid" here using wire clipper? or in the strategy?
        # Note: also apply avoid to the actual profiles
        if self.avoid:
            objs = self._o_objects(self.avoid)

            outer_profiles, inner_profiles = self._apply_avoid(
                outer_profiles,
                inner_profiles,
                objs,
                outer_wire_offset,
                inner_wire_offset,
            )

            outer_boundaries, inner_boundaries = self._apply_avoid(
                outer_boundaries,
                inner_boundaries,
                objs,
                outer_wire_offset - final_pass_offset,
                inner_wire_offset + final_pass_offset,
            )

        cut_sequences = self.strategy.process(self, outer_boundaries, inner_boundaries)
        if self.avoid:
            cut_sequences += outer_profiles
            cut_sequences += inner_profiles
        else:
            outer_polygons = tuple(
                flatten_wire_to_closed_2d(outer_profile)
                for outer_profile in outer_profiles
            )
            inner_polygons = tuple(
                flatten_wire_to_closed_2d(inner_profile)
                for inner_profile in inner_profiles
            )
            cut_sequences += outer_polygons
            cut_sequences += inner_polygons

        for i, depth in enumerate(self._generate_depths(start_depth, end_depth)):
            for cut_sequence in cut_sequences:
                cut_start = cut_sequence[0]
                self.commands.append(Rapid.abs(z=self.clearance_height))
                self.commands.append(Rapid.abs(x=cut_start[0], y=cut_start[1]))
                self.commands.append(
                    Rapid.abs(z=self.top_height)
                )  # TODO plunge or rapid?
                self.commands.append(Plunge.abs(z=depth))
                for cut in cut_sequence[1:]:
                    self.commands.append(Cut.abs(x=cut[0], y=cut[1]))


def pick_other_scanline_end(scanline, scanpoint):
    if scanline[0] == scanpoint:
        return scanline[1]
    return scanline[0]
