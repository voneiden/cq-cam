import logging
from dataclasses import dataclass, field
from typing import Union, List, Optional

import numpy as np
from cadquery import cq


from cq_cam.commands.command import Rapid, Plunge
from cq_cam.commands.util_command import wire_to_command_sequence2

from cq_cam.operations.base_operation import Task, OperationError
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils.utils import (
    plane_offset_distance,
    cut_clockwise
)
from cq_cam.visualize import visualize_task

logger = logging.getLogger(__name__)


@dataclass
class Profile(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """
    Create a profiles based on selected wires and faces in a Workplane.
    """

    wp: cq.Workplane = None
    """ The cadquery Workplane containing faces and/or
    wires that the profile will operate on. 
    """

    stepdown: Union[float, None] = None
    """ Maximum Z stepdown. A value of None means straight to bottom. """

    tool_diameter: float = 3.175

    face_offset_outer: Optional[float] = 1
    """ Offset is in multiples of tool diameter
      * -1 for closed pockets and inside profiles
      * 0 for open pockets
      * 1 for outside profiles
      
    This offset is applied to any wires and the outer wires of any faces 
    given to the operation.
    """

    face_offset_inner: Optional[float] = None
    """ See `outer_offset`  """

    wire_offset: Optional[float] = 1

    def __post_init__(self):
        if not self.wp:
            raise OperationError("wp must be defined")

        faces: List[cq.Face] = []
        wires: List[cq.Wire] = []
        for obj in self.wp.objects:
            if isinstance(obj, cq.Face):
                faces.append(obj)
            elif isinstance(obj, cq.Wire):
                wires.append(obj)
            else:
                raise OperationError(f'Object type "{type(obj)}" not supported by Profile operation')

        if not faces and not wires:
            raise OperationError('wp selection must contain at least one face or wire')

        for face in faces:
            if self.face_offset_outer is None and self.face_offset_inner is None:
                raise OperationError("Define at least one of 'face_offset_outer' and 'face_offset_inner'")

            if self.face_offset_outer is not None:
                self.profile(face.outerWire(), self.face_offset_outer * self.tool_diameter)
            if self.face_offset_inner is not None:
                inner_wires = face.innerWires()
                if not inner_wires:
                    logger.warning("Face had no innerWires but inner_boundary_offset was set")
                for wire in inner_wires:
                    self.profile(wire, self.face_offset_inner * self.tool_diameter)

        for wire in wires:
            if self.face_offset_outer is None:
                raise OperationError("'wire_offset' must be defined when profiling wires")
            self.profile(wire, self.wire_offset * self.tool_diameter)

    def profile(self, wire: cq.Wire, offset: float):

        face = cq.Workplane(cq.Face.makeFromWires(wire))
        workplane, _ = self.validate_plane(self.job, face)

        # Originally I intended to use clipper to do the offset, however
        # I realized later that CadQuery/OpenCASCADE can do 2D offsets for
        # wires as well. This has the benefit that it conserves arcs
        # which can then be effortlessly be converted to G2/G3 commands.
        offset_wires = wire.offset2D(offset, 'arc')
        assert len(offset_wires) == 1

        # TODO, apply tabs here
        command_sequence = wire_to_command_sequence2(offset_wires[0])

        if command_sequence.is_clockwise() != cut_clockwise(True, True, True):
            command_sequence.reverse()
            pass



        start = command_sequence.start
        end = command_sequence.end

        # Rapid to clearance height
        self.commands.append(Rapid(x=start.x, y=start.y, z=self.clearance_height))
        bottom_height = plane_offset_distance(self.job.workplane.plane, workplane.plane)
        if self.stepdown:
            depths = list(np.arange(self.top_height + self.stepdown, bottom_height, self.stepdown))
            if depths[-1] != bottom_height:
                depths.append(bottom_height)
        else:
            depths = [bottom_height]

        for i, depth in enumerate(depths):
            # self.commands.append(profile[0])
            if getattr(command_sequence.commands[0], 'tab', False):
                self.commands.append(Plunge(-8)) # TODO
            else:
                self.commands.append(Plunge(depth))
            # TODO apply tabs
            self.commands += command_sequence.duplicate(depth, tab_z=-8).commands

        self.commands.append(Rapid(x=end.x, y=end.y, z=self.clearance_height))

        self.job.tasks.append(self)



def demo():
    from cq_cam.job import Job
    from cq_cam.commands.base_command import Unit
    wp = cq.Workplane().box(10, 20, 10)
    job = Job(wp.faces('>Z').workplane(), 300, 50, Unit.METRIC, 5)
    profile = Profile(job, wp=wp.wires('<Z'), stepdown=-1, clearance_height=2, top_height=0)
    print("OK")
    viz = visualize_task(job, profile)
    show_object(wp, 'wp')
    show_object(viz,'viz' )


if 'show_object' in locals() or __name__ == '__main__':
    demo()
