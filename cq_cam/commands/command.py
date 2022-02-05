from __future__ import annotations

from dataclasses import dataclass
from typing import Union, TYPE_CHECKING

from cadquery import cq

from cq_cam.commands.base_command import EndData, Command, Linear, Circular

if TYPE_CHECKING:
    from cq_cam.job.job import Job


class Rapid(EndData, Command):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        if isinstance(previous, Rapid):
            return self.diff(previous)
        else:
            return f"G0{self.diff(previous)}"

    def duplicate(self, z: float):
        end = cq.Vector(self.end)
        end.z = z
        return Rapid(end)


class Cut(EndData, Linear):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        feed = "" if isinstance(previous, Cut) else f"F{job.feed}"
        return f"{feed}{super().to_gcode(previous, job)}{self.diff(previous)}"

    def duplicate(self, z: float):
        end = cq.Vector(self.end)
        end.z = z
        return Cut(end)


class CircularCW(Circular):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        cmd = '' if isinstance(previous, CircularCW) else 'G2'
        return f'{cmd}{super().to_gcode(previous, job)}'

    def duplicate(self, z: float):
        end = cq.Vector(self.end)
        end.z = z
        return CircularCW(end, self.radius)


class CircularCCW(Circular):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        cmd = '' if isinstance(previous, CircularCCW) else 'G3'
        return f'{cmd}{super().to_gcode(previous, job)}'

    def duplicate(self, z: float):
        end = cq.Vector(self.end)
        end.z = z
        return CircularCCW(end, self.radius)


@dataclass
class Plunge(EndData, Linear):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        plunge_feed = "" if isinstance(previous, Plunge) else f"F{job.plunge_feed}"
        return f"{plunge_feed}{super().to_gcode(previous, job)}Z{self.end.z}"

    def duplicate(self, z: float):
        end = cq.Vector(self.end)
        end.z = z
        return Plunge(end)
