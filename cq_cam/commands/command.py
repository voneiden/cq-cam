from __future__ import annotations

from dataclasses import dataclass
from typing import Union, TYPE_CHECKING, Tuple

from cadquery import cq

from cq_cam.commands.base_command import EndData, Command, Linear, Circular, MotionCommand

if TYPE_CHECKING:
    from cq_cam.job import Job


class Rapid(EndData, MotionCommand):
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        if isinstance(previous_command, Rapid) or isinstance(previous_command, Retract):
            return self.diff(start, job)
        else:
            diff, end = self.diff(start, job)
            return f"G0{diff}", end

    def duplicate(self, z: float):
        return Rapid(self.x, self.y, z)


class Cut(EndData, Linear):

    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        feed = "" if isinstance(previous_command, Cut) else f"F{job.feed}"
        diff, end = self.diff(start, job)
        return f"{feed}{super().to_gcode(previous_command, start, job)}{diff}", end

    def duplicate(self, z: float):
        return Cut(self.x, self.y, z)


class CircularCW(Circular):
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        cmd = '' if isinstance(previous_command, CircularCW) else 'G2'
        diff, end = super().to_gcode(previous_command, start, job)
        return f'{cmd}{diff}', end

    def duplicate(self, z: float):
        return CircularCW(self.x, self.y, z, self.radius, self.ijk, (self.mid[0], self.mid[1], z))


class CircularCCW(Circular):
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        cmd = '' if isinstance(previous_command, CircularCCW) else 'G3'
        diff, end = super().to_gcode(previous_command, start, job)
        return f'{cmd}{diff}', end

    def duplicate(self, z: float):
        return CircularCCW(self.x, self.y, z, self.radius, self.ijk, (self.mid[0], self.mid[1], z))


@dataclass
class Plunge(Linear):
    __slots__ = ['z']
    z: float

    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        plunge_feed = "" if isinstance(previous_command, Plunge) else f"F{job.plunge_feed}"
        diff, end = self.diff(start, job)
        return f"{plunge_feed}{super().to_gcode(previous_command, start, job)}{diff}", end

    def duplicate(self, z: float):
        return Plunge(self.z)

    def end(self, start: cq.Vector) -> cq.Vector:
        return cq.Vector(
            start.x,
            start.y,
            self.z if self.z else start.z
        )


@dataclass
class Retract(Plunge):
    """ Rapid retract """
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        if isinstance(previous_command, Rapid) or isinstance(previous_command, Retract):
            return self.diff(start, job)
        else:
            diff, end = self.diff(start, job)
            return f"G0{diff}", end
