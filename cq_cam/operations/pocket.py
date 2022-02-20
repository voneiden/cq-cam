from collections import defaultdict
from dataclasses import dataclass
from typing import List, Tuple, Dict

import numpy as np
from OCP.BRepFeat import BRepFeat
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from cadquery import cq

from cq_cam.commands.base_command import Unit
from cq_cam.commands.command import Rapid, Cut, Plunge
from cq_cam.job import Job
from cq_cam.operations.base_operation import FaceBaseOperation
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils.utils import WireClipper, flatten_list
from cq_cam.visualize import visualize_task


@dataclass
class Pocket(PlaneValidationMixin, ObjectsValidationMixin, FaceBaseOperation):
    """ 2.5D Pocket operation

    All faces involved must be planes and parallel.
    """

    tool_diameter: float = 3.175
    """ Diameter of the tool that will be used to perform the operation.
    """

    # TODO rotation angle for zigzag

    @property
    def _tool_diameter(self) -> float:
        return self.tool_diameter

    def __post_init__(self):
        # Give each face an ID
        faces = [(i, face.copy()) for i, face in enumerate(self.faces)]
        features, coplanar_faces, depth_info = self._discover_pocket_features(faces)
        groups = self._group_faces_by_features(features, coplanar_faces)

        for group_faces in groups.values():
            boundaries = self._boundaries_by_group(group_faces, coplanar_faces, depth_info)
            for boundary, start_depth, end_depth in boundaries:
                self.process_boundary(boundary, start_depth, end_depth)

    @staticmethod
    def _group_faces_by_features(features: List[cq.Face], faces: List[Tuple[int, cq.Face]]):
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
        """ Given a list of faces, creates fused feature faces and returns depth information """
        # Move everything on the same plane
        coplanar_faces = []
        job_zdir = self.job.workplane.plane.zDir
        job_height = job_zdir.dot(self.job.workplane.plane.origin)
        depth_info = {}
        for i, face in faces:
            face_height = job_zdir.dot(face.Center())
            face_depth = face_height - job_height
            depth_info[i] = face_depth

            # Move face to same level with job plane
            translated_face = face.translate(job_zdir.multiply(-face_depth))
            coplanar_faces.append((i, translated_face))

        features = self._combine_coplanar_faces([f[1] for f in coplanar_faces])

        return features, coplanar_faces, depth_info

    @staticmethod
    def _combine_coplanar_faces(faces: List[cq.Face]) -> List[cq.Face]:
        """ Given a list of (coplanar) faces, fuse them together to form
         bigger faces """

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

    def _boundaries_by_group(self,
                             group_faces: List[Tuple[int, cq.Face]],
                             coplanar_faces: List[Tuple[int, cq.Face]],
                             depth_info: Dict[int, float]):
        face_depths = list(set(depth_info.values()))
        face_depths.sort()
        face_depths.reverse()
        current_depth = 0
        boundaries = []
        group_i = [i for i, _ in group_faces]
        for face_depth in face_depths:
            depth_i = [i for i, depth in depth_info.items() if depth <= face_depth and i in group_i]
            features = self._combine_coplanar_faces([f for i, f in coplanar_faces if i in depth_i and i in group_i])
            assert len(features) == 1
            boundary = features[0]
            boundaries.append((boundary, current_depth, face_depth))
            current_depth = face_depth
        return boundaries

    def _generate_depths(self, start_depth: float, end_depth: float):
        if self.stepdown:
            depths = list(np.arange(start_depth - self.stepdown, end_depth, -self.stepdown))
            if depths[-1] != end_depth:
                depths.append(end_depth)
            return depths
        else:
            return [end_depth]



    def process_boundary(self, face: cq.Face, start_depth: float, end_depth: float):
        # TODO break this function down into easily testable sections
        # Perform validations
        self.validate_face_plane(face)
        face_workplane = cq.Workplane(obj=face)
        self.validate_plane(self.job, face_workplane)

        # Prepare profile paths
        job_plane = self.job.workplane.plane
        tool_radius = self._tool_diameter / 2
        outer_wire_offset = tool_radius * self.outer_boundary_offset
        inner_wire_offset = tool_radius * self.inner_boundary_offset

        # These are the profile paths. They are done very last as a finishing pass
        outer_profiles = face.outerWire().offset2D(outer_wire_offset)
        inner_profiles = flatten_list([wire.offset2D(inner_wire_offset) for wire in face.innerWires()])

        # Prepare primary clearing regions
        if self.boundary_final_pass_stepover is None:
            self.boundary_final_pass_stepover = self.stepover
        final_pass_offset = tool_radius * self.boundary_final_pass_stepover

        # Generate the primary clearing regions with stepover from the above profiles
        outer_regions = flatten_list([wire.offset2D(-final_pass_offset) for wire in outer_profiles])
        inner_regions = flatten_list([wire.offset2D(final_pass_offset) for wire in inner_profiles])

        # TODO: Scanline orientation
        # Here we could rotate the regions so that we can keep the scanlines in standard XY plane

        # Vectorize regions and prepare scanline clipper
        clipper = WireClipper(job_plane)
        outer_polygons = []
        for outer_region in outer_regions:
            polygon = clipper.add_clip_wire(outer_region)
            outer_polygons.append(polygon)

        inner_polygons = []
        for inner_region in inner_regions:
            polygon = clipper.add_clip_wire(inner_region)
            inner_polygons.append(polygon)

        max_bounds = clipper.max_bounds()

        # Generate ZigZag scanlines
        y_scanpoints = list(np.arange(max_bounds['bottom'], max_bounds['top'], self._tool_diameter * self.stepover))
        scanline_templates = [((max_bounds['left'], y), (max_bounds['right'], y)) for y in y_scanpoints]

        for scanline_template in scanline_templates:
            clipper.add_subject_polygon(scanline_template)

        scanlines = clipper.execute()

        scanpoint_to_scanline, scanpoints = self._scanline_end_map(scanlines)

        linked_polygons, scanpoint_to_linked_polygon = self._link_scanpoints_to_boundaries(
            scanpoints, outer_polygons + inner_polygons)

        cut_sequences = self._route_zig_zag(linked_polygons,
                                            scanlines,
                                            scanpoint_to_linked_polygon,
                                            scanpoint_to_scanline)

        for i, depth in enumerate(self._generate_depths(start_depth, end_depth)):
            for cut_sequence in cut_sequences:
                cut_start = cut_sequence[0]
                self.commands.append(Rapid(None, None, self.clearance_height))
                self.commands.append(Rapid(*cut_start, None))
                self.commands.append(Rapid(None, None, self.top_height))  # TODO plunge or rapid?
                self.commands.append(Plunge(depth))
                for cut in cut_sequence[1:]:
                    self.commands.append(Cut(*cut, None))


def pick_other_scanline_end(scanline, scanpoint):
    if scanline[0] == scanpoint:
        return scanline[1]
    return scanline[0]


def demo():
    job_plane = cq.Workplane().box(15, 15, 10).faces('>Z').workplane()
    obj = (
        job_plane
            .rect(7.5, 7.5)
            .cutBlind(-4)
            .faces('>Z[1]')
            .rect(2, 2)
            .extrude(2)
            .faces('>Z').workplane()
            .moveTo(-5.75, 0)
            .rect(4, 2)
            .cutBlind(-6)
    )
    op_plane = obj.faces('>Z[1]')
    test = obj.faces('>Z[-3] or >Z[-2] or >Z[-4]')
    # obj = op_plane.workplane().rect(2, 2).extrude(4)

    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Pocket(job, 2, 0, test.objects, None, 1, 0.33, stepdown=1)

    toolpath = visualize_task(job, op)
    print(op.to_gcode())

    show_object(obj)
    # show_object(op_plane)
    show_object(toolpath, 'g')
    # for w in op._wires:
    #    show_object(w)

    show_object(test, 'test')


if 'show_object' in locals() or __name__ == '__main__':
    demo()
