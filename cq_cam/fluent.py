from __future__ import annotations

import itertools
from copy import copy
from typing import List

from cadquery import cq

from cq_cam.command import Command
from cq_cam.common import Unit
from cq_cam.routers import route
from cq_cam.utils.utils import extract_wires, compound_to_edges, flatten_list
from cq_cam.visualize import visualize_job


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
                 tool_diameter: float,
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

    def profile(self, shape, offset=1, offset_inner=None, stepdown=None):
        if offset_inner is None:
            offset_inner = -offset
        outers, inners = extract_wires(shape)

        # Transform to relative coordinates
        outers = [outer.transformShape(self.top.fG) for outer in outers]
        inners = [inner.transformShape(self.top.fG) for inner in inners]

        self.debug = outers + inners

        # Generate base features
        base_features = []
        for outer in outers:
            base_features += outer.offset2D(offset * self.tool_diameter)

        for inner in inners:
            base_features += inner.offset2D(offset_inner * self.tool_diameter)

        if stepdown:
            toolpaths = []
            for base_feature in base_features:
                step = cq.Vector(0, 0, 1) * stepdown
                layers: List[cq.Wire] = [base_feature]
                self.debug.append(base_feature)
                for i in itertools.count():
                    if i > self.max_stepdown_count:
                        raise RuntimeError('Job.max_stepdown_count exceeded')

                    i_op: cq.Wire = base_feature.moved(cq.Location(step * (i + 1)))
                    if i_op.BoundingBox().zmin >= 0:
                        break

                    edges = compound_to_edges(i_op.cut(self.top_plane_face))
                    edges = [edge for edge in edges if edge.Center().z < 0]
                    wires = cq.Wire.combine(edges)
                    if not wires:
                        break
                    self.debug.append(i_op)
                    layers.append(i_op)

                layers.reverse()
                toolpaths += layers

        else:
            toolpaths = base_features

        commands = route(self, toolpaths)
        return self._add_operation('Profile', commands)


if __name__ == 'temp' or __name__ == '__main__':
    wp = cq.Workplane().box(15, 10, 5)
    top = wp.faces('>X').workplane()
    bottom = wp.faces('<X')
    cam = JobV2(top.plane, 100, 3.175).profile(bottom, stepdown=3)
    print(cam.to_gcode())
    show_object(wp)
    cam.show(show_object)
