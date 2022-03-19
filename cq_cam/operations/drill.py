from dataclasses import dataclass, field
from typing import Optional, List, Union

from cq_cam.commands.command import Rapid, Plunge
from cq_cam.operations.base_operation import Operation, OperationError
import cadquery as cq

from cq_cam.operations.strategy import Strategy
from cq_cam.utils.utils import flatten_list

_op_o_shapes = Union[cq.Wire, cq.Face, cq.Vector]


@dataclass
class Drill(Operation):
    wp: cq.Workplane = None
    """ The cadquery Workplane containing faces and/or
    wires that the profile will operate on. 
    """
    o: Union[cq.Workplane, List[_op_o_shapes], _op_o_shapes] = None
    depth: float = None

    def __post_init__(self):
        # TODO max depth
        # TODO evacuate chips?
        transform_f = self.job.workplane.plane.toWorldCoords
        drill_vectors: List[cq.Vector] = []

        if self.o is None:
            raise OperationError("o must be defined")

        if self.depth is None:
            raise OperationError('depth must be defined')

        for obj in self._o_objects(self.o):
            if isinstance(obj, cq.Vector):
                drill_vectors.append(transform_f(obj))
            elif isinstance(obj, cq.Wire):
                drill_vectors.append(transform_f(cq.Face.makeFromWires(obj).Center()))
            elif isinstance(obj, cq.Face):
                if obj.innerWires():
                    for wire in obj.innerWires():
                        drill_vectors.append(transform_f(cq.Face.makeFromWires(wire).Center()))
                else:
                    drill_vectors.append(transform_f(cq.Face.makeFromWires(obj.outerWire()).Center()))
            else:
                raise OperationError(f'Object type "{type(obj)}" not supported by Profile operation')

        if not drill_vectors:
            raise OperationError("Given wp does not contain anything to do")


        drill_points = [(point.x, point.y) for point in drill_vectors]
        ordered_drill_points = []
        cut_sequences = []
        last = None
        while drill_points:
            drill_point = Strategy._pick_nearest(last, drill_points) if last else drill_points[0]
            ordered_drill_points.append(drill_point)
            drill_points.pop(drill_points.index(drill_point))
            last = drill_point

        depth = -abs(self.depth)
        for point in ordered_drill_points:
            cut_sequences.append([
                Rapid(x=None, y=None, z=self.clearance_height),
                Rapid(x=point[0], y=point[1], z=None),
                Rapid(x=None, y=None, z=self.top_height),
                Plunge(z=depth),  # TODO depth
                Rapid(x=None, y=None, z=self.clearance_height),
            ])
        cut_sequences = flatten_list(cut_sequences)
        self.commands = cut_sequences


def demo():
    from cq_cam.job import Job
    from cq_cam.commands.base_command import Unit
    from cq_cam.visualize import visualize_task

    result = cq.Workplane("front").box(20.0, 20.0, 2).faces('>Z').workplane().pushPoints([
        (3, 3), (-5, -8), (0, 0), (5, 2), (7, -3), (-8, 2)]).circle(1).cutThruAll()

    job_plane = result.faces('>Z').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Drill(job=job, clearance_height=5, top_height=0, o=result.faces('>Z').objects[0].innerWires())
    toolpath = visualize_task(job, op, as_edges=False)
    # result.objects += toolpath
    show_object(result)
    show_object(toolpath)
    show_object(result.faces('>Z'), 'avoid', {'color': 'red'})


if 'show_object' in locals() or __name__ == '__main__':
    demo()
