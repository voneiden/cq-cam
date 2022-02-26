from dataclasses import dataclass, field
from typing import Optional, List, Union

from cq_cam.operations.base_operation import Task
import cadquery as cq


@dataclass
class Drill(Task):
    points: Optional[List[Union[cq.Location, cq.Vector]]] = None
    wires: Optional[List[cq.Wire]] = None
    faces: Optional[List[cq.Face]] = None
    depth: Optional[float] = None

    def __post__init__(self):
        # TODO max depth
        # TODO evacuate chips?
        transform_f = self.job.workplane.plane.toWorldCoords
        drill_points: List[Union[cq.Location, cq.Vector]] = []

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