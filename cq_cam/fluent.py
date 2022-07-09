from __future__ import annotations

from copy import copy
from typing import List, Optional

from cadquery import cq

from cq_cam import Pocket, Drill, Surface3D
from cq_cam.command import Command
from cq_cam.common import Unit
from cq_cam.operations.profile import profile
from cq_cam.utils.utils import extract_wires
from cq_cam.visualize import visualize_job


# TODO: Render breaks on empty moves
# --> to_gcode and to_ais_shape should be able to handle this

class Operation:
    def __init__(self, job, name: str, commands: List[Command]):
        self.job = job
        self.name = name
        self.commands = commands

    def to_gcode(self):
        # Set starting position above rapid height so that
        # we guarantee getting the correct Z rapid in the beginning
        position = cq.Vector(0, 0, self.job.rapid_height + 1)
        gcodes = [f'({self.job.name} - {self.name})']
        previous_command = None
        for command in self.commands:
            gcode, position = command.to_gcode(previous_command, position)
            previous_command = command
            gcodes.append(gcode)
        return '\n'.join(gcodes)


class JobV2:
    def __init__(self,
                 top: cq.Plane,
                 feed: float,
                 tool_diameter: Optional[float] = None,
                 name='Job',
                 plunge_feed: float = None,
                 rapid_height: float = None,
                 op_safe_height: float = None,
                 gcode_precision: int = 3,
                 unit: Unit = Unit.METRIC):
        self.top = top
        self.top_plane_face = cq.Face.makePlane(None, None, top.origin, top.zDir)
        self.feed = feed
        self.tool_diameter = tool_diameter
        self.name = name
        self.plunge_feed = feed if plunge_feed is None else plunge_feed
        self.rapid_height = self._default_rapid_height(unit) if rapid_height is None else rapid_height
        self.op_safe_height = self._default_op_safe_height(unit) if op_safe_height is None else op_safe_height
        self.gcode_precision = gcode_precision
        self.unit = unit

        self.max_stepdown_count = 100

        self.operations: List[Operation] = []

    @staticmethod
    def _default_rapid_height(unit: Unit):
        if unit == Unit.METRIC:
            return 10
        return 0.4

    @staticmethod
    def _default_op_safe_height(unit: Unit):
        if unit == Unit.METRIC:
            return 1
        return 0.04

    def to_gcode(self):
        task_break = "\n\n\n"
        return f"({self.name} - Feedrate: {self.feed} - Unit: {self.unit})\n{self.unit.to_gcode()}\n{task_break.join(task.to_gcode() for task in self.operations)}"

    def show(self, show_object):
        for operation in self.operations:
            show_object(visualize_job(self.top, operation.commands[1:]), f'{self.name} - {operation.name}')

    def _add_operation(self, name: str, commands: List[Command]):
        job = copy(self)
        job.operations = [*self.operations, Operation(job, name, commands)]
        return job

    def profile(self, shape, outer_offset=1, inner_offset=None, stepdown=None):
        if self.tool_diameter is None:
            raise ValueError('Profile requires tool_diameter to be est')

        if inner_offset is None:
            inner_offset = -outer_offset
        outer_wires, inner_wires = extract_wires(shape)

        commands = profile(
            job=self,
            outer_wires=outer_wires,
            inner_wires=inner_wires,
            outer_offset=outer_offset,
            inner_offset=inner_offset,
            stepdown=stepdown
        )
        return self._add_operation('Profile', commands)

    def pocket(self, *args, **kwargs):
        if self.tool_diameter is None:
            raise ValueError('Profile requires tool_diameter to be est')

        pocket = Pocket(self, *args, **kwargs)
        return self._add_operation('Pocket', pocket.commands)

    def drill(self, *args, **kwargs):
        drill = Drill(self, *args, **kwargs)
        return self._add_operation('Drill', drill.commands)

    def surface3d(self, *args, **kwargs):
        surface3d = Surface3D(self, *args, **kwargs)
        return self._add_operation('Surface 3D', surface3d.commands)
