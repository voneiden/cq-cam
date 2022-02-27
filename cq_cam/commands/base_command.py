from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Union, TYPE_CHECKING, List, Optional, Tuple

from cadquery import cq


from cq_cam.commands.util_command import same_to_none, vector_same_to_none, equal_within_tolerance, \
    normalize

if TYPE_CHECKING:
    from cq_cam.job import Job


@dataclass
class CommandSequence:
    start: cq.Vector
    commands: List[Command]
    end: cq.Vector

    def reverse(self):
        # Flip the start and end properties
        start = self.start
        end = self.end
        next_end = start

        # Flip each command direction
        for i, command in enumerate(self.commands):
            if isinstance(command, MotionCommand):
                command, next_end = command.flip(next_end)
                self.commands[i] = command

        # Flip the list so that we can iterate naturally
        self.commands.reverse()
        self.start = next_end
        self.end = start

    def is_clockwise(self):
        """ Only works on sequences that form a simple polygon! """
        # TODO implement logic for circles or some other weird stuff

        if len(self.commands) == 1:
            from cq_cam.commands.command import CircularCW, CircularCCW
            cmd = self.commands[0]

            if isinstance(cmd, CircularCW):
                return True
            elif isinstance(cmd, CircularCCW):
                return False
            else:
                raise NotImplementedError('Unable to determine if path is clockwise')

        if len(self.commands) < 3:
            raise NotImplementedError('Unable to determine if path is clockwise')

        # Find the smallest y, biggest x
        # https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order/1180256#1180256
        motion_commands = [command for command in self.commands if isinstance(command, MotionCommand)]
        ends = []
        previous_end = self.start
        for command in motion_commands:
            end = command.end(previous_end)
            ends.append(end)
            previous_end = end

        # TODO filter also commands that don't move on XY plane?
        b = sorted(ends, key=lambda e: (e.y, -e.x))[0]
        b_i = ends.index(b)

        a = ends[b_i - 1]
        c = ends[(b_i + 1) % len(ends)]

        det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)

        return det < 0

    def duplicate(self, z: float):
        # TODO tab support here?
        start = cq.Vector(self.start)
        end = cq.Vector(self.end)
        start.z = z
        end.z = z
        commands = []
        for command in self.commands:
            commands.append(command.duplicate(z))
        return CommandSequence(start, commands, end)


class Command(ABC):
    @abstractmethod
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        """ Output all the necessary G-Code required to perform the command """
        pass

    @abstractmethod
    def duplicate(self, z: float):
        pass


class MotionCommand(Command, ABC):
    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        from cq_cam.commands.command import Cut, Plunge, Retract

        # This is a bit hard to conceptualize
        # "new_end" is the old start (so it's the previous)
        # "start" below is actually the old end
        new_start = self.end(new_end)

        x_eq, y_eq, z_eq = new_start.x == new_end.x, new_start.y == new_end.y, new_start.z == new_end.z
        if x_eq and y_eq:
            # Going up, use Cut
            if new_end.z > new_start.z:
                return Retract(same_to_none(new_end.z, new_start.z)), new_start
                # Going down, use Plunge
            else:
                return Plunge(same_to_none(new_end.z, new_start.z)), new_start

        if not z_eq:
            # TODO Ramp!
            pass
        return Cut(*vector_same_to_none(new_end, new_start)), new_start

    @abstractmethod
    def end(self, start: cq.Vector) -> cq.Vector:
        pass

    def diff(self, start: Optional[cq.Vector], job: Job) -> Tuple[str, cq.Vector]:
        """ Output X Y and Z coordinates as necessary """
        end = self.end(start)
        coordinates = []

        if start is None or not equal_within_tolerance(end.x, start.x, job.gcode_precision):
            coordinates.append(f'X{normalize(round(end.x, job.gcode_precision))}')
        if start is None or not equal_within_tolerance(end.y, start.y, job.gcode_precision):
            coordinates.append(f'Y{normalize(round(end.y, job.gcode_precision))}')
        if start is None or not equal_within_tolerance(end.z, start.z, job.gcode_precision):
            coordinates.append(f'Z{normalize(round(end.z, job.gcode_precision))}')

        return "".join(coordinates), end


@dataclass
class EndData(Command, ABC):
    __slots__ = ['x', 'y', 'z']
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]

    def end(self, previous_end: cq.Vector) -> cq.Vector:
        return cq.Vector(
            previous_end.x if self.x is None else self.x,
            previous_end.y if self.y is None else self.y,
            previous_end.z if self.z is None else self.z
        )


@dataclass
class InitialReference(MotionCommand):
    __slots__ = ['reference']
    reference: cq.Vector

    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        raise NotImplemented('InitialReference can not output gcode')

    def duplicate(self, z: float):
        raise NotImplemented('InitialReference can not be duplicated')

    def end(self, start: cq.Vector) -> cq.Vector:
        return self.reference


class Linear(MotionCommand, ABC):
    """ Linear interpolation (G01) """

    def to_gcode(self, previous_command: MotionCommand, start: cq.Vector, job: Job) -> str:
        if isinstance(start, Linear):
            return ""
        else:
            return "G1"


@dataclass
class CircularData(EndData, ABC):
    __slots__ = ['radius', 'ijk', 'mid']
    # TODO in py3.10 dataclass supports __slots__ better with default values (?)
    radius: Optional[float]
    ijk: Optional[Tuple[float, float, float]]
    mid: Optional[Tuple[float, float, float]]

    def __post_init__(self):
        if self.radius is None and self.ijk is None:
            raise RuntimeError('Either radius or ijk must be given to a circular command')


class Circular(CircularData, MotionCommand, ABC):
    def to_gcode(self, previous_command: MotionCommand, start: cq.Vector, job: Job) -> Tuple[str, cq.Vector]:
        diff, end = self.diff(start, job)

        if self.ijk is not None:
            return f'{diff}{self.diff_ijk(job)}', end

        return f'{diff}R{self.radius}', end

    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        from cq_cam.commands.command import CircularCW, CircularCCW

        new_start = self.end(new_end)

        if isinstance(self, CircularCW):
            cls = CircularCCW
        else:
            cls = CircularCW

        # TODO ijk needs to be flipped?
        # TODO save center instead of ijk?
        ijk = cq.Vector(self.ijk)
        ijk = new_end.add(ijk).sub(new_start)
        x, y, z = vector_same_to_none(new_end, new_start)
        return cls(x, y, z, self.radius, (ijk.x, ijk.y, ijk.z), self.mid), new_start

    def diff_ijk(self, job: Job):
        ijk = []
        i = normalize(round(self.ijk[0], job.gcode_precision))
        j = normalize(round(self.ijk[1], job.gcode_precision))
        k = normalize(round(self.ijk[2], job.gcode_precision))
        if i:
            ijk.append(f'I{i}')
        if j:
            ijk.append(f'J{j}')
        if k:
            ijk.append(f'K{k}')
        return ''.join(ijk)


class Unit(Enum):
    METRIC = 20
    IMPERIAL = 21

    def to_gcode(self) -> str:
        if self == Unit.METRIC:
            return "G20"
        else:
            return "G21"
