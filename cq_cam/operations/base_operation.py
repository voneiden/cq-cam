from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
from typing import List

from cadquery import cq

from cq_cam.commands.base_command import Command
from cq_cam.job.job import Job


class OperationError(Exception):
    pass


@dataclass
class Task(ABC):
    job: Job
    commands: List[Command] = field(init=False, default_factory=list)
    clearance_height: float
    top_height: float

    def __post_init__(self):
        self.job.tasks.append(self)

    def to_gcode(self):
        start = cq.Vector(10, 10, 10)

        def command_gcode_generator(start):
            previous_command = None
            for command in self.commands:
                gcode, start = command.to_gcode(previous_command, start, self.job)
                yield gcode
                previous_command = command

        return "\n".join(command_gcode_generator(start))


class Unit(Enum):
    METRIC = 20
    IMPERIAL = 21

    def to_gcode(self) -> str:
        if self == Unit.METRIC:
            return "G20"
        else:
            return "G21"
