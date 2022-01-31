from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Union

from cadquery import cq


class PlaneNotAligned(Exception):
    pass

class OperationError(Exception):
    pass

class Command(ABC):
    @abstractmethod
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        pass


@dataclass
class XYZData:
    __slots__ = ['x', 'y', 'z']
    x: Union[float, None]
    y: Union[float, None]
    z: Union[float, None]

    def diff(self, other):
        d = []
        if self.x is not None:
            if not isinstance(other, XYZData) or self.x != other.x:
                d.append(f'X{self.x}')
        if self.y is not None:
            if not isinstance(other, XYZData) or self.y != other.y:
                d.append(f'Y{self.y}')
        if self.z is not None:
            if not isinstance(other, XYZData) or self.z != other.z:
                d.append(f'Z{self.z}')
        return "".join(d)


class Rapid(XYZData, Command):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        if isinstance(previous, Rapid):
            return self.diff(previous)
        else:
            return f"G0{self.diff(previous)}"


class Linear(Command):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        if isinstance(previous, Linear):
            return ""
        else:
            return "G1"


class Cut(XYZData, Linear):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        feed = "" if isinstance(previous, Cut) else f"F{job.feed}"
        return f"{feed}{super().to_gcode(previous, job)}{self.diff(previous)}"


@dataclass
class Plunge(Linear):
    __slots__ = ['z']
    z: Union[float, None]

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        plunge_feed = "" if isinstance(previous, Plunge) else f"F{job.plunge_feed}"
        return f"{plunge_feed}{super().to_gcode(previous, job)}Z{self.z}"


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


@dataclass
class Job:
    workplane: cq.Workplane
    tasks: List[Task] = field(init=False, default_factory=list)
    feed: float
    plunge_feed: float
    unit: Unit
    rapid_height: float


    def to_gcode(self):
        task_break = "\n\n\n"
        return f"{self.unit.to_gcode()}\n{task_break.join(task.to_gcode() for task in self.tasks)}"
