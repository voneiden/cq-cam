from __future__ import annotations

from dataclasses import dataclass
from typing import Union, TYPE_CHECKING

from cq_cam.commands.base_command import XYZData, Command, Linear, CircularBase

if TYPE_CHECKING:
    from cq_cam.job.job import Job


class Rapid(XYZData, Command):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        if isinstance(previous, Rapid):
            return self.diff(previous)
        else:
            return f"G0{self.diff(previous)}"


class Cut(XYZData, Linear):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        feed = "" if isinstance(previous, Cut) else f"F{job.feed}"
        return f"{feed}{super().to_gcode(previous, job)}{self.diff(previous)}"


class CircularCW(CircularBase):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        cmd = '' if isinstance(previous, CircularCW) else 'G2'
        return f'{cmd}{super().to_gcode(previous, job)}'


class CircularCCW(CircularBase):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        cmd = '' if isinstance(previous, CircularCCW) else 'G3'
        return f'{cmd}{super().to_gcode(previous, job)}'


@dataclass
class Plunge(Linear):
    __slots__ = ['z']
    z: Union[float, None]

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        plunge_feed = "" if isinstance(previous, Plunge) else f"F{job.plunge_feed}"
        return f"{plunge_feed}{super().to_gcode(previous, job)}Z{self.z}"
