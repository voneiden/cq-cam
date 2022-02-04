from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from cq_cam.job.job import Job


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


class Linear(Command):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        if isinstance(previous, Linear):
            return ""
        else:
            return "G1"


@dataclass
class CircularData:
    # TODO think about this a bit, need to support helix too
    __slots__ = ['sx', 'sy', 'ex', 'ey', 'r']
    sx: float
    sy: float
    ex: float
    ey: float
    c: Tuple[float, float]
    r: float


class CircularBase(CircularData, Command):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        return f'X{self.ex}Y{self.ey}R{self.r}'
