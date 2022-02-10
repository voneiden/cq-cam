from dataclasses import dataclass
from typing import Union

import numpy as np
from cadquery import cq

from cq_cam.commands.command import Rapid, Plunge
from cq_cam.commands.util_command import wire_to_command_sequence
from cq_cam.operations.base_operation import Task
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.utils import (
    plane_offset_distance,
    cut_clockwise
)


@dataclass
class Profile(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """
    Create a profile around the outer wire of a given face

    TODO profile should always work on a wire
    """
    face: cq.Workplane
    offset: float
    stepdown: Union[float, None]

    def __post_init__(self):

        wires = self.face.wires()
        self.validate_wires(wires, 1)
        workplane, _ = self.validate_plane(self.job, self.face)

        # Originally I intended to use clipper to do the offset, however
        # I realized later that CadQuery/OpenCASCADE can do 2D offsets for
        # wires as well. This has the benefit that it conserves arcs
        # which can then be effortlessly be converted to G2/G3 commands.

        wire: cq.Wire = wires.objects[0]
        offset_wires = wire.offset2D(self.offset, 'arc')
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
