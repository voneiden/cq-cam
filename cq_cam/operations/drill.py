from dataclasses import dataclass, field
from typing import Optional, List, Union

from cq_cam.operations.base_operation import Task, OperationError
import cadquery as cq


@dataclass
class Drill(Task):
    wp: cq.Workplane = None
    """ The cadquery Workplane containing faces and/or
    wires that the profile will operate on. 
    """
    points: Optional[List[Union[cq.Location, cq.Vector]]] = None
    wires: Optional[List[cq.Wire]] = None
    faces: Optional[List[cq.Face]] = None
    depth: Optional[float] = None

    def __post__init__(self):
        # TODO max depth
        # TODO evacuate chips?
        transform_f = self.job.workplane.plane.toWorldCoords
        drill_points: List[Union[cq.Location, cq.Vector]] = []

        if self.wp is None:
            raise OperationError("wp must be defined")

        for obj in self.wp.objects:
            if isinstance(obj, cq.Location):
                drill_points.append(obj)
            elif isinstance(obj, cq.Vector):
                drill_points.append(transform_f(obj))
            elif isinstance(obj, cq.Wire):
                drill_points.append(transform_f(cq.Face.makeFromWires(obj).Center()))
            elif isinstance(obj, cq.Face):
                if obj.innerWires():
                    for wire in obj.innerWires():
                        drill_points.append(transform_f(cq.Face.makeFromWires(wire).Center()))
                else:
                    drill_points.append(transform_f(cq.Face.makeFromWires(obj.outerWire()).Center()))
            else:
                raise OperationError(f'Object type "{type(obj)}" not supported by Profile operation')

        if not drill_points:
            raise OperationError("Given wp does not contain anything to do")

        for point in self.points:
            drill_points.append(
                point if isinstance(point, cq.Location) else transform_f(point)
            )

        for wire in self.wires:
            drill_points.append(transform_f(cq.Face.makeFromWires(wire).Center()))

        for face in self.faces:
            for wire in face.innerWires():
                drill_points.append(transform_f(cq.Face.makeFromWires(wire).Center()))

        # TODO generate commands
        # TODO optimize order
if __name__ == '__main__':
    d = Drill(None, 0, [], )