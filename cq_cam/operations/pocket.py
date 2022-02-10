from dataclasses import dataclass
from typing import List, Union

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
from cq_cam.utils import vertex_to_vector, orient_vector, flatten_wire, WireClipper
from cq_cam.visualize import visualize_task


@dataclass
class Pocket(PlaneValidationMixin, ObjectsValidationMixin, Task):
    faces: List[cq.Face]
    tool_diameter: float
    stepover: float
    stepdown: Union[float, None]

    # todo angle

    def __post_init__(self):
        # Practice with single face

        f = self.faces[0]

        # All the faces should be flat!
        adaptor = BRepAdaptor_Surface(f.wrapped)
        if adaptor.GetType() != GeomAbs_SurfaceType.GeomAbs_Plane:
            raise RuntimeError("no plane no game")

        # Do tool_diameter negative offset
        outer_profile = f.outerWire().offset2D(-self.tool_diameter/2)[0]  # TODO handle splitting
        inner_profiles = [wire.offset2D(self.tool_diameter/2)[0] for wire in f.innerWires()] # TODO same

        #outer_polygon = [orient_vector(v, self.job.workplane.plane) for v in flatten_wire(outer_profile)]
        #inner_polygons = []
        #for inner_profile in inner_profiles:
        #    inner_polygons.append(orient_vector(v, self.job.workplane.plane) for v in flatten_wire(inner_profile))


        clipper = WireClipper()
        clipper.add_wire(self.job.workplane.plane, outer_profile)

        for inner_profile in inner_profiles:
            clipper.add_wire(self.job.workplane.plane, inner_profile)
        #clipper = pyclipper.Pyclipper()
        #clipper.AddPath(pyclipper.scale_to_clipper(outer_polygon), pyclipper.PT_CLIP)
        #for inner_polygon in inner_polygons:
        #    clipper.AddPath(pyclipper.scale_to_clipper(inner_polygon), pyclipper.PT_CLIP)


        # Determine dimensions
        explorer = TopExp_Explorer(f.wrapped, TopAbs_VERTEX)
        vxs = []

        while explorer.More():
            vx = explorer.Current()
            vxs.append(vx)
            explorer.Next()

        # Wrangle the topo vertexes into vectors and align them on the job plane
        # Note this list contains duplicates, probably not worth it trying to get rid of them
        vxs = [orient_vector(vertex_to_vector(vx), self.job.workplane.plane) for vx in vxs]

        print("VXS", vxs)

        x = [vx.x for vx in vxs]
        y = [vx.y for vx in vxs]

        x_max = max(x)
        x_min = min(x)
        y_max = max(y)
        y_min = min(y)

        print("Top left", x_min, y_max)
        print("Bottom right", x_max, y_min)
        # Ziggy zaggy
        # Generate scanlines
        y_scanpoints = list(np.arange(y_min, y_max, self.tool_diameter * self.stepover))
        scanlines = [((x_min, y), (x_max, y)) for y in y_scanpoints]

        # Feed the scanlines to clipper
        for scanline in scanlines:
            clipper.add_polygon(scanline, subject=True)

        result = clipper.execute()



        for (p1, p2) in result:
            self.commands.append(Rapid(*p1, -1))
            self.commands.append(Rapid(None, None, -4))
            self.commands.append(Cut(*p2, -4))
            self.commands.append(Rapid(None, None, -1))

        # Use pyclipper to chop the scanlines

        # Assemble the scanlines into suitable work sequences with some nice algorithm

        pass

        # Determine face depths

        # Construct profile polygons

        # Generate operation layers
        self._wires = [outer_profile, *inner_profiles]


def demo():
    job_plane = cq.Workplane().box(10, 10, 10).faces('>Z').workplane()
    obj = job_plane.rect(7.5, 7.5).cutBlind(-4).faces('>Z[1]').rect(2, 2).extrude(2)
    op_plane = obj.faces('>Z[1]')
    #obj = op_plane.workplane().rect(2, 2).extrude(4)

    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Pocket(job, 2, 0, op_plane.objects, 1, 0.5, None)

    toolpath = visualize_task(job, op)
    show_object(obj)
    #show_object(op_plane)
    show_object(toolpath, 'g')
    for w in op._wires:
        show_object(w)



if 'show_object' in locals() or __name__ == '__main__':
    demo()
