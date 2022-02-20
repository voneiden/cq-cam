import logging
from dataclasses import dataclass, field
from typing import Union, List, Optional

import numpy as np
from cadquery import cq

from cq_cam.commands.command import Rapid, Plunge
from cq_cam.commands.util_command import wire_to_command_sequence
from cq_cam.operations.base_operation import Task, OperationError
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils.utils import (
    plane_offset_distance,
    cut_clockwise
)

logger = logging.getLogger(__name__)


@dataclass
class Profile(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """
    Create a profiles based on wires and faces

    """
    wires: List[cq.Wire] = field(default_factory=list)
    """ List of wires to profile
    Note: Will use outer_offset as the offset value
    """

    faces: List[cq.Face] = field(default_factory=list)
    """ List of faces to profile
    Note: Will use outer_offset for the outer wire and inner_offset for the inner wires
    """

    stepdown: Union[float, None] = None
    """ Maximum Z stepdown. A value of None means straight to bottom. """

    tool_diameter: float = 3.175

    outer_offset: Optional[float] = 1
    """ Offset is in multiples of tool diameter
      * -1 for closed pockets and inside profiles
      * 0 for open pockets
      * 1 for outside profiles
      
    This offset is applied to any wires and the outer wires of any faces 
    given to the operation.
    """

    inner_offset: Optional[float] = None
    """ See `outer_offset`  """

    def __post_init__(self):
        if not self.faces and not self.wires:
            raise OperationError("At least one face or wire must be defined")

        faces = [self.faces] if isinstance(self.faces, cq.Face) else self.faces
        wires = [self.wires] if isinstance(self.wires, cq.Wire) else self.wires

        for face in faces:
            if isinstance(face, cq.Face):
                if self.outer_offset is None and self.inner_offset is None:
                    raise OperationError("Define at least one of 'outer_boundary_offset' and 'inner_boundary_offset'")

                if self.outer_offset is not None:
                    self.profile(face.outerWire(), self.outer_offset * self.tool_diameter)
                if self.inner_offset is not None:
                    inner_wires = face.innerWires()
                    if not inner_wires:
                        logger.warning("Face had no innerWires but inner_boundary_offset was set")
                    for wire in inner_wires:
                        self.profile(wire, self.inner_offset * self.tool_diameter)
            else:
                raise OperationError("'faces' may only contain cq.Face objects")

        for wire in wires:
            if isinstance(wire, cq.Wire):
                if self.outer_offset is None:
                    raise OperationError("'outer_offset' must be defined when profiling wires")
                self.profile(wire, self.outer_offset * self.tool_diameter)
            else:
                raise OperationError("'wires' may only contain cq.Wire objects")

    def profile(self, wire: cq.Wire, offset: float):

        face = cq.Workplane(cq.Face.makeFromWires(wire))
        workplane, _ = self.validate_plane(self.job, face)

        # Originally I intended to use clipper to do the offset, however
        # I realized later that CadQuery/OpenCASCADE can do 2D offsets for
        # wires as well. This has the benefit that it conserves arcs
        # which can then be effortlessly be converted to G2/G3 commands.
        offset_wires = wire.offset2D(offset, 'arc')
        assert len(offset_wires) == 1

        command_sequence = wire_to_command_sequence(offset_wires[0], self.job.workplane.plane)

        if command_sequence.is_clockwise() != cut_clockwise(True, True, True):
            command_sequence.reverse()
            pass

        start = command_sequence.start
        end = command_sequence.end

        # Rapid to clearance height
        self.commands.append(Rapid(start.x, start.y, self.clearance_height))
        bottom_height = plane_offset_distance(self.job.workplane.plane, workplane.plane)
        if self.stepdown:
            depths = list(np.arange(self.top_height + self.stepdown, bottom_height, self.stepdown))
            if depths[-1] != bottom_height:
                depths.append(bottom_height)
        else:
            depths = [bottom_height]

        for i, depth in enumerate(depths):
            # self.commands.append(profile[0])
            self.commands.append(Plunge(depth))
            self.commands += command_sequence.duplicate(depth).commands

        self.commands.append(Rapid(end.x, end.y, self.clearance_height))

        self.job.tasks.append(self)
