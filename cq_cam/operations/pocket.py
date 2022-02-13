from dataclasses import dataclass, field
from enum import Enum
from functools import cache
from typing import List, Union, Optional, Tuple

import numpy as np
import pyclipper
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from cadquery import cq

from cq_cam.commands.command import Rapid, Cut
from cq_cam.operations.base_operation import Job, Unit
from cq_cam.operations.base_operation import Task
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils.utils import vertex_to_vector, orient_vector, flatten_wire, WireClipper, flatten_list, pairwise, \
    dist_to_segment_squared, dist2, cached_dist2
from cq_cam.utils.linked_polygon import LinkedPolygon
from cq_cam.visualize import visualize_task


@dataclass
class Pocket(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """ 2.5D Pocket operation

    All faces involved must be planes and parallel.
    """

    faces: List[cq.Face]
    """ List of faces to operate on"""

    avoid: Optional[List[cq.Face]]
    """ List of faces that the tool may not enter. This option
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

        # Practice with single face

        f = self.faces[0]

        # All the faces should be flat!
        adaptor = BRepAdaptor_Surface(f.wrapped)
        if adaptor.GetType() != GeomAbs_SurfaceType.GeomAbs_Plane:
            raise RuntimeError("no plane no game")

        job_plane = self.job.workplane.plane

        tool_radius = self.tool_diameter / 2
        outer_wire_offset = tool_radius * self.outer_boundary_stepover
        inner_wire_offset = tool_radius * self.inner_boundary_stepover

        # These are the profile paths. They are done last as a finishing pass
        outer_profiles = f.outerWire().offset2D(outer_wire_offset)
        inner_profiles = flatten_list([wire.offset2D(inner_wire_offset) for wire in f.innerWires()])

        if self.boundary_final_pass_stepover is None:
            self.boundary_final_pass_stepover = self.stepover
        final_pass_offset = tool_radius * self.boundary_final_pass_stepover

        # Generate the primary clearing regions with stepover from the above profiles
        outer_regions = flatten_list([wire.offset2D(-final_pass_offset) for wire in outer_profiles])
        inner_regions = flatten_list([wire.offset2D(final_pass_offset) for wire in inner_profiles])

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

        # Ziggy zaggy
        # Generate scanlines
        y_scanpoints = list(np.arange(max_bounds['bottom'], max_bounds['top'], self.tool_diameter * self.stepover))
        scanline_templates = [((max_bounds['left'], y), (max_bounds['right'], y)) for y in y_scanpoints]

        # Feed the scanlines to clipper
        for scanline_template in scanline_templates:
            clipper.add_subject_polygon(scanline_template)

        scanlines = clipper.execute()

        # OK, the war plan for zigzag is
        # 1) Create a map scanpoints -> scanline
        # 2) Create a map of polygons -> scanpoints -> polygons
        # 3) Pick top left scanline
        # 4) Cut scanline
        # 5) Find nearest unused scanpoint neighbour from mapped polygon - if none, rapid to highest unused scanline and #4
        # 6) Repeat until all scanlines done
        # 7) Do finishing pass

        scanpoint_to_scanline = {}
        scanpoints = []
        # Generate a map of scanlines and boundary polygons
        print("Mapping scanpoints")
        for scanline in scanlines:
            sp1, sp2 = scanline
            scanpoint_to_scanline[sp1] = scanline
            scanpoint_to_scanline[sp2] = scanline
            scanpoints.append(sp1)
            scanpoints.append(sp2)

        print("Done")
        print("Mapping scanlines")
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

        # Reset
        for linked_polygon in linked_polygons:
            linked_polygon.reset()

        # Setup
        scanlines = list(scanlines)
        starting_scanline = scanlines.pop(0)
        start_position, cut_position = starting_scanline
        if start_position[0] > cut_position[0]:
            start_position, cut_position = cut_position, start_position

        scanpoint_to_polynode[start_position].drop(start_position)
        cut_sequence = [start_position, cut_position]
        cut_sequences = []
        while scanlines:
            linked_polygon = scanpoint_to_polynode[cut_position]
            path = linked_polygon.nearest_linked(cut_position)
            if path is None:
                break # TODO

            cut_sequence += path
            scanline = scanpoint_to_scanline[path[-1]]
            cut_sequence.append(scanline[1] if scanline[0] == path[-1] else scanline[0])
            cut_position = cut_sequence[-1]


        self.commands.append(Rapid(*cut_sequence[0], -1))
        for cut in cut_sequence[1:]:
            self.commands.append(Cut(*cut, None))

        #for (p1, p2) in scanlines:
        #    self.commands.append(Rapid(*p1, -1))
        #    self.commands.append(Rapid(None, None, -4))
        #    self.commands.append(Cut(*p2, -4))
        #    self.commands.append(Rapid(None, None, -1))

        # Assemble the scanlines into suitable work sequences with some nice algorithm

        # Determine face depths

        # Construct profile polygons

        # Generate operation layers
        self._wires = [*outer_profiles, *inner_profiles]

def pick_other_scanline_end(scanline, scanpoint):
    if scanline[0] == scanpoint:
        return scanline[1]
    return scanline[0]

def demo():
    job_plane = cq.Workplane().box(10, 10, 10).faces('>Z').workplane()
    obj = job_plane.rect(7.5, 7.5).cutBlind(-4).faces('>Z[1]').rect(2, 2).extrude(2)
    op_plane = obj.faces('>Z[1]')
    # obj = op_plane.workplane().rect(2, 2).extrude(4)

    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Pocket(job, 2, 0, op_plane.objects, None, 1, 0.33)

    toolpath = visualize_task(job, op)
    print(op.to_gcode())

    show_object(obj)
    # show_object(op_plane)
    show_object(toolpath, 'g')
    for w in op._wires:
        show_object(w)


if 'show_object' in locals() or __name__ == '__main__':
    demo()
