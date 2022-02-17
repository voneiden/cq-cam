from collections import defaultdict
from dataclasses import dataclass
from typing import List, Union, Optional, Tuple, Dict

import numpy as np
from OCP.BRepFeat import BRepFeat
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from cadquery import cq
from cq_cam.commands.base_command import Unit
from cq_cam.commands.command import Rapid, Cut, Plunge
from cq_cam.job import Job
from cq_cam.operations.base_operation import Task
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils.linked_polygon import LinkedPolygon
from cq_cam.utils.utils import WireClipper, flatten_list, pairwise, \
    dist_to_segment_squared
from cq_cam.visualize import visualize_task


@dataclass
class Pocket(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """ 2.5D Pocket operation

    All faces involved must be planes and parallel.
    """

    faces: List[cq.Face]
    """ List of faces to operate on"""

    avoid: Optional[List[cq.Face]]
    """ [INOP] List of faces that the tool may not enter. This option
    can be relevant when using an `outer_boundary_offset` that
    would otherwise cause the tool to enter features you do
    not want to cut."""

    tool_diameter: float
    """ Diameter of the tool that will be used to perform the operation.
    """

    stepover: float = 0.8
    """ Stepover (cut width) as a fraction of tool diameter (0..1]. 
    For example a value of 0.5 means the operation tries to use 
    50% of the tool width."""

    outer_boundary_stepover: float = -1
    """ Typically -1 for closed pockets and 0 for open pockets.
    Setting `avoid` is generally necessary when doing open pockets.
    """

    inner_boundary_stepover: float = 1
    """ Typically 1 for any kind of pocket.  """

    boundary_final_pass_stepover: Union[float, None] = None
    """ Stepover for a final boundary (profile) pass.
    """

    stepdown: Union[float, None] = None
    """ Maximum distance to step down on each pass 
    """

    # todo angle

    def __post_init__(self):
        faces = [(i, face.copy()) for i, face in enumerate(self.faces)]
        features, coplanar_faces, depth_info = self._discover_pocket_features(faces)
        groups = self._group_faces_by_features(features, faces)

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
                if feat.IsInside_s(face.wrapped, feature.wrapped):
                    groups[feature].append((i, face))
                    remaining.remove((i, face))

        assert not remaining
        return groups

    def _discover_pocket_features(self, faces: List[Tuple[int, cq.Face]]):

        # Move everything on the same plane
        coplanar_faces = []
        job_zdir = self.job.workplane.plane.zDir
        job_height = job_zdir.dot(self.job.workplane.plane.origin)
        depth_info = {}
        for i, face in faces:
            face_depth = job_zdir.dot(face.Center())
            height_diff = job_height - face_depth
            depth_info[i] = height_diff
            translated_face = face.translate(job_zdir.multiply(height_diff))
            coplanar_faces.append((i, translated_face))

        features = self._combine_coplanar_faces([f[1] for f in coplanar_faces])

        return features, coplanar_faces, depth_info

    @staticmethod
    def _combine_coplanar_faces(faces: List[cq.Face]) -> List[cq.Face]:
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
        current_depth = 0
        boundaries = []
        group_i = [i for i, _ in group_faces]
        for face_depth in face_depths:
            depth_i = [i for i, depth in depth_info.items() if depth >= face_depth and i in group_i]
            features = self._combine_coplanar_faces([f for i, f in coplanar_faces if i in depth_i and i in group_i])
            assert len(features) == 1
            boundary = features[0]
            boundaries.append((boundary, current_depth, face_depth))
            current_depth = face_depth
        return boundaries

    def process_boundary(self, face: cq.Face, start_depth: float, end_depth: float):
        # TODO break this function down into easily testable sections
        # Perform validations
        self.validate_face_plane(face)
        face_workplane = cq.Workplane(obj=face)
        self.validate_plane(self.job, face_workplane)
        # bottom_height = plane_offset_distance(self.job.workplane.plane, face_workplane.workplane().plane)
        bottom_height = -end_depth

        if self.stepdown:
            depths = list(np.arange(-start_depth + self.stepdown, bottom_height, self.stepdown))
            if depths[-1] != bottom_height:
                depths.append(bottom_height)
        else:
            depths = [bottom_height]

        # Prepare profile paths
        job_plane = self.job.workplane.plane
        tool_radius = self.tool_diameter / 2
        outer_wire_offset = tool_radius * self.outer_boundary_stepover
        inner_wire_offset = tool_radius * self.inner_boundary_stepover

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
        y_scanpoints = list(np.arange(max_bounds['bottom'], max_bounds['top'], self.tool_diameter * self.stepover))
        scanline_templates = [((max_bounds['left'], y), (max_bounds['right'], y)) for y in y_scanpoints]

        for scanline_template in scanline_templates:
            clipper.add_subject_polygon(scanline_template)

        scanlines = clipper.execute()

        # Do a mapping of scanpoints to scanlines
        scanpoint_to_scanline = {}
        scanpoints = []
        for scanline in scanlines:
            sp1, sp2 = scanline
            scanpoint_to_scanline[sp1] = scanline
            scanpoint_to_scanline[sp2] = scanline
            scanpoints.append(sp1)
            scanpoints.append(sp2)

        # Link scanpoints to the boundary regions
        remaining_scanpoints = scanpoints[:]
        scanpoint_to_polynode = {}
        linked_polygons = []
        for polygon in outer_polygons + inner_polygons:
            linked_polygon = LinkedPolygon(polygon[:])
            linked_polygons.append(linked_polygon)
            for p1, p2 in pairwise(polygon):
                for scanpoint in remaining_scanpoints[:]:
                    d = dist_to_segment_squared(scanpoint, p1, p2)
                    # Todo pick a good number. Tests show values between 1.83e-19 and 1.38e-21
                    if d < 0.0000001:
                        remaining_scanpoints.remove(scanpoint)
                        linked_polygon.link_point(scanpoint, p1, p2)
                        scanpoint_to_polynode[scanpoint] = linked_polygon

        assert not remaining_scanpoints

        # Prepare to route the zigzag
        for linked_polygon in linked_polygons:
            linked_polygon.reset()

        scanlines = list(scanlines)

        # Pick a starting position. Clipper makes no guarantees about the orientation
        # of polylines it returns, so figure the top left scanpoint as the
        # starting position.
        starting_scanline = scanlines.pop(0)
        start_position, cut_position = starting_scanline
        if start_position[0] > cut_position[0]:
            start_position, cut_position = cut_position, start_position

        scanpoint_to_polynode[start_position].drop(start_position)
        cut_sequence = [start_position, cut_position]
        cut_sequences = []

        # Primary routing loop
        while scanlines:
            linked_polygon = scanpoint_to_polynode[cut_position]
            path = linked_polygon.nearest_linked(cut_position)
            if path is None:
                cut_sequences.append(cut_sequence)
                # TODO some optimization potential in picking the nearest scanpoint
                start_position, cut_position = scanlines.pop(0)
                # TODO some optimization potential in picking a direction
                cut_sequence = [start_position, cut_position]
                continue

            cut_sequence += path
            scanline = scanpoint_to_scanline[path[-1]]
            cut_sequence.append(scanline[1] if scanline[0] == path[-1] else scanline[0])
            cut_position = cut_sequence[-1]
            scanlines.remove(scanline)

        cut_sequences.append(cut_sequence)

        for i, depth in enumerate(depths):
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
    op = Pocket(job, 2, 0, test.objects, None, 1, 0.33, stepdown=-1)

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
