from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
from typing import List

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
        def command_gcode_generator():
            previous = None
            for command in self.commands:
                yield command.to_gcode(previous, self.job)
                previous = command

        return "\n".join(command_gcode_generator())


class Unit(Enum):
    METRIC = 20
    IMPERIAL = 21

    def to_gcode(self) -> str:
        if self == Unit.METRIC:
            return "G20"
        else:
            return "G21"
