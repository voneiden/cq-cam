from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union, TYPE_CHECKING, List, Optional

from cadquery import cq

if TYPE_CHECKING:
    from cq_cam.job.job import Job


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
            command, next_end = command.flip(next_end)
            self.commands[i] = command

        # Flip the list so that we can iterate naturally
        self.commands.reverse()
        self.start = next_end
        self.end = start

    def is_clockwise(self):
        """ Only works on sequences that form a simple polygon! """
        # TODO implement logic for circles or some other weird stuff
        if len(self.commands) < 3:
            raise NotImplemented

        # Find the smallest y, biggest x
        # https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order/1180256#1180256
        commands = [command for command in self.commands if isinstance(command, EndData)]
        # TODO filter also commands that don't move on XY plane?
        b_cmd = sorted(commands, key=lambda cmd: (cmd.end.y, -cmd.end.x))[0]
        b_i = commands.index(b_cmd)
        b = b_cmd.end
        a = commands[b_i - 1].end
        c = commands[(b_i + 1) % len(commands)].end

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
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        pass

    @abstractmethod
    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        pass

    @abstractmethod
    def duplicate(self, z: float):
        pass


@dataclass
class EndData(Command, ABC):
    __slots__ = ['end']
    end: cq.Vector

    def x_equal(self, other: cq.Vector):
        return self.end.x == other.x

    def y_equal(self, other: cq.Vector):
        return self.end.y == other.y

    def z_equal(self, other: cq.Vector):
        return self.end.z == other.z

    def diff(self, other: Optional[EndData]):
        d = []
        if other is None or not self.x_equal(other.end):
            d.append(f'X{self.end.x}')
        if other is None or not self.y_equal(other.end):
            d.append(f'Y{self.end.y}')
        if other is None or not self.z_equal(other.end):
            d.append(f'Z{self.end.z}')
        return "".join(d)

    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        from cq_cam.commands.command import Cut, Plunge

        x_eq, y_eq, z_eq = self.x_equal(new_end), self.y_equal(new_end), self.z_equal(new_end)
        if x_eq and y_eq:
            # Going up, use Cut
            if new_end.z > self.end.z:
                return Cut(new_end), self.end
            # Going down, use Plunge
            else:
                return Plunge(new_end), self.end

        if not z_eq:
            # TODO Ramp!
            pass
        return Cut(new_end), self.end


class Linear(Command, ABC):

    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        if isinstance(previous, Linear):
            return ""
        else:
            return "G1"


@dataclass
class CircularData(EndData, ABC):
    __slots__ = ['radius']
    radius: float

    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        from cq_cam.commands.command import CircularCW, CircularCCW
        if isinstance(self, CircularCW):
            cls = CircularCCW
        else:
            cls = CircularCW

        return cls(new_end, self.radius), self.end


class Circular(CircularData, Command, ABC):
    def to_gcode(self, previous: Union[Command, None], job: Job) -> str:
        return f'X{self.end.x}Y{self.end.y}R{self.radius}'
