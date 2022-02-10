from dataclasses import dataclass
from typing import List, Union

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from cadquery import cq

from cq_cam.operations.base_operation import Job, Unit
from cq_cam.operations.base_operation import Task
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils import vertex_to_vector, orient_vector


@dataclass
class Pocket(PlaneValidationMixin, ObjectsValidationMixin, Task):
    faces: List[cq.Face]
    tool_diameter: float
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

        # Generate scanlines

        # Use pyclipper to chop the scanlines

        # Assemble the scanlines into suitable work sequences with some nice algorithm

        pass

        # Determine face depths

        # Construct profile polygons

        # Generate operation layers


def demo():
    job_plane = cq.Workplane().box(10, 10, 10).faces('>Z').workplane()
    obj = job_plane.rect(5, 5).cutBlind(-4)
    op_plane = obj.faces('>Z[1]')

    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Pocket(job, 2, 0, op_plane.objects, 3.175, None)

    show_object(obj)
    show_object(op_plane)


if 'show_object' in locals() or __name__ == '__main__':
    demo()
