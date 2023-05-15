from __future__ import annotations

import logging
from copy import copy
from typing import Union

from cadquery import cq

from cq_cam.command import Command, SafetyBlock, StartSequence, StopSequence, ToolChange
from cq_cam.groups import (
    ArcDistanceMode,
    CoolantState,
    DistanceMode,
    PlannerControlMode,
    Unit,
    WorkOffset,
    WorkPlane,
)
from cq_cam.operations.pocket import pocket
from cq_cam.operations.profile import profile
from cq_cam.operations.tabs import Tabs
from cq_cam.tool import Tool
from cq_cam.utils.geometry_op import OffsetInput
from cq_cam.utils.utils import extract_wires, flatten_list
from cq_cam.visualize import visualize_job, visualize_job_as_edges

logger = logging.getLogger(__name__)


class Operation:
    def __init__(self, job: Job, name: str, commands: list[Command]):
        self.job = job
        self.name = name
        self.commands = commands

    def to_gcode(self):
        # Set starting position above rapid height so that
        # we guarantee getting the correct Z rapid in the beginning
        gcodes = [f"({self.job.name} - {self.name})"]
        for command in self.commands:
            gcode = str(command)

            # Skip blank lines. These can happen for example if we try to issue
            # a move to the same position where we already are
            if not gcode:
                continue
            gcodes.append(gcode)

        return "\n".join(gcodes)


class Job:
    def __init__(
        self,
        top: cq.Plane,
        feed: float,
        speed: float | None = None,
        tool_diameter: float | None = None,
        tool_number: int | None = None,
        name="Job",
        plunge_feed: float = None,
        rapid_height: float = None,
        op_safe_height: float = None,
        precision: int = 3,
        unit: Unit = Unit.METRIC,
        plane: WorkPlane = WorkPlane.XY,
        coordinate: WorkOffset = WorkOffset.OFFSET_1,
        distance: DistanceMode = DistanceMode.ABSOLUTE,
        arc_distance: ArcDistanceMode = ArcDistanceMode.ABSOLUTE,
        controller_motion: PlannerControlMode = PlannerControlMode.CONTINUOUS,
        coolant: CoolantState | None = None,
    ):
        self.top = top
        self.top_plane_face = cq.Face.makePlane(None, None, top.origin, top.zDir)
        self.feed = feed
        self.speed = speed
        self.tool_diameter = tool_diameter
        self.tool_number = tool_number
        self.name = name
        self.plunge_feed = feed if plunge_feed is None else plunge_feed
        self.rapid_height = (
            self._default_rapid_height(unit) if rapid_height is None else rapid_height
        )
        self.op_safe_height = (
            self._default_op_safe_height(unit)
            if op_safe_height is None
            else op_safe_height
        )
        self.precision = precision
        self.unit = unit
        self.plane = plane
        self.coordinate = coordinate
        self.distance = distance
        self.arc_distance = arc_distance
        self.controller_motion = controller_motion
        self.coolant = coolant

        self.max_stepdown_count = 100

        self.operations: list[Operation] = []

    """
    There are two checks that must happen between operations:
    1. Check if the tool_number has changed
        - Only tool_number needs to be checked here and not tool_diameter as different tools with the same diameter can be used for different operations (e.g. flat, ball, bull nose, etc.)
    2. Check if the speed setting has changed
        - The speed setting can change between roughing and finishing operation

    If either of these happens a ToolChange or a StartSequence command needs to be issued first.

    Note that only one one of these can happen between operations

    No special handling is need for the feed setting. It only needs to be updated for any subsequent operations that use it
    """

    def update_tool(self, tool: Tool | None = None) -> Job:
        if tool is not None:
            # Initilize variables
            tool_diameter = (
                self.tool_diameter if tool.tool_diameter is None else tool.tool_diameter
            )
            tool_number = (
                self.tool_number if tool.tool_number is None else tool.tool_number
            )
            feed = self.feed if tool.feed is None else tool.feed
            speed = self.speed if tool.speed is None else tool.speed

            # Check if any of the setting is changed changed to add necessary command
            if tool_number is not None and tool_number != self.tool_number:
                commands = [ToolChange(tool_number, speed, self.coolant)]
                self = self._add_operation("Tool Change", commands)
            elif speed is not None and speed != self.speed:
                commands = [StartSequence(speed, self.coolant)]
                self = self._add_operation("Speed Change", commands)

            # Update Job attributes
            self.tool_number = tool_number
            self.tool_diameter = tool_diameter
            self.feed = feed
            self.speed = speed

        return self

    # ##################
    # Fluent operations
    # ##################
    def profile(
        self,
        shape: Union[cq.Workplane, cq.Shape, list[cq.Shape]],
        outer_offset=1,
        inner_offset=-1,
        stepdown=None,
        tabs: Tabs | None = None,
        tool: Tool | None = None,
    ) -> Job:
        if self.tool_diameter is None:
            raise ValueError("Profile requires tool_diameter to be set")

        if outer_offset is None and inner_offset is None:
            raise ValueError('Set at least one of "outer_offset" or "inner_offset"')
        outer_wires, inner_wires = extract_wires(shape)

        self = self.update_tool(tool)
        # Prefer inner wires first
        commands = []
        if inner_wires and inner_offset is not None:
            for inner_wire in inner_wires:
                commands += profile(
                    job=self,
                    wire=inner_wire,
                    offset=inner_offset,
                    stepdown=stepdown,
                    tabs=tabs,
                )

        if outer_wires and outer_offset is not None:
            for outer_wire in outer_wires:
                commands += profile(
                    job=self,
                    wire=outer_wire,
                    offset=outer_offset,
                    stepdown=stepdown,
                    tabs=tabs,
                )

        return self._add_operation("Profile", commands)

    def wire_profile(
        self,
        wires: cq.Wire | list[cq.Wire],
        offset=1,
        stepdown=None,
        tabs=None,
        tool: Tool | None = None,
    ):
        self = self.update_tool(tool)
        if isinstance(wires, cq.Wire):
            wires = [wires]

        commands = []
        for wire in wires:
            commands += profile(
                job=self,
                wire=wire,
                offset=offset,
                stepdown=stepdown,
                tabs=tabs,
            )
        return self._add_operation("Wire Profile", commands)

    def pocket(
        self,
        op_areas: Union[cq.Workplane, cq.Face, list[cq.Face]],
        avoid_areas: Union[cq.Workplane, cq.Face, list[cq.Face]] | None = None,
        outer_offset: OffsetInput | None = None,
        inner_offset: OffsetInput | None = None,
        avoid_outer_offset: OffsetInput | None = None,
        avoid_inner_offset: OffsetInput | None = None,
        stepover: OffsetInput | None = None,
        stepdown: float | None = None,
        tool: Tool | None = None,
    ) -> Job:
        self = self.update_tool(tool)
        if isinstance(op_areas, cq.Workplane):
            op_areas = op_areas.objects
        elif isinstance(op_areas, cq.Face):
            op_areas = [op_areas]

        if isinstance(avoid_areas, cq.Workplane):
            avoid_areas = avoid_areas.objects
        elif isinstance(avoid_areas, cq.Face):
            avoid_areas = [avoid_areas]

        commands = pocket(
            self,
            op_areas,
            avoid_areas=avoid_areas,
            outer_offset=outer_offset,
            inner_offset=inner_offset,
            avoid_outer_offset=avoid_outer_offset,
            avoid_inner_offset=avoid_inner_offset,
            stepover=stepover,
            stepdown=stepdown,
        )
        return self._add_operation("Pocket", commands)

    def drill(self, op_areas, tool: Tool | None = None, **kwargs) -> Job:
        from cq_cam.operations.drill import Drill

        self = self.update_tool(tool)
        drill = Drill(self, o=op_areas, **kwargs)
        return self._add_operation("Drill", drill.commands)

    def surface3d(self, *args, **kwargs) -> Job:
        from cq_cam.operations.op3d import Surface3D

        surface3d = Surface3D(self, *args, **kwargs)
        return self._add_operation("Surface 3D", surface3d.commands)

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
        return (
            f"({self.name} - Feedrate: {self.feed} - Unit: {repr(self.unit)})\n"
            f"{SafetyBlock()}\n"
            f"{StartSequence(speed=self.speed, coolant=self.coolant)}\n"
            f"{task_break.join(task.to_gcode() for task in self.operations)}\n"
            f"{SafetyBlock()}\n"
            f"{StopSequence(coolant=self.coolant)}"
        )

    def save_gcode(self, file_name):
        gcode = str(self)
        with open(file_name, "w") as f:
            f.write(gcode)

    def show(self, show_object=None):
        if show_object is None:
            import __main__

            show_object = __main__.__dict__.get("show_object")

        if show_object is None:
            raise ValueError(
                "Unable to visualize job, no show_object provided or found"
            )

        match source_module := show_object.__module__.split(".")[0]:
            case "cq_editor":
                visualize_f = visualize_job
            case "ocp_vscode":
                visualize_f = visualize_job_as_edges
            case _:
                logger.warning(
                    f"Unsupported show_object source module ({source_module}) - visualizing as edges"
                )
                visualize_f = visualize_job_as_edges
        for i, operation in enumerate(self.operations):
            show_object(
                visualize_f(self.top, operation.commands[1:]),
                f"{self.name} #{i} {operation.name}",
            )

    def to_shapes(self, as_edges=False):
        if as_edges:
            return flatten_list(
                [
                    visualize_job_as_edges(self.top, operation.commands[1:])
                    for operation in self.operations
                ]
            )
        return [
            visualize_job(self.top, operation.commands[1:])
            for operation in self.operations
        ]

    def _add_operation(self, name: str, commands: list[Command]):
        job = copy(self)
        job.operations = [*self.operations, Operation(job, name, commands)]
        return job

    @property
    def tool_radius(self):
        return self.tool_diameter / 2 if self.tool_diameter else None
