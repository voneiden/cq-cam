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
    Create a profiles based on selected wires and faces in a Workplane.
    """

    wp: cq.Workplane = None
    """ The cadquery Workplane containing faces and/or
    wires that the profile will operate on. 
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
