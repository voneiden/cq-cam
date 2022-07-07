from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Union, Optional, Tuple

import cadquery as cq
from OCP.AIS import AIS_Shape

from cq_cam.commands.util_command import normalize
from cq_cam.visualize import to_occ_color

'''

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
        """
Only
works
on
sequences
that
form
a
simple
polygon! """
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

    def duplicate(self, z: float, tab_z=None):
        from cq_cam.commands.command import Retract, Plunge

        start = cq.Vector(self.start)
        end = cq.Vector(self.end)
        start.z = z
        end.z = z
        commands = []
        tabbing = False
        for command in self.commands:
            if tab_z is not None and getattr(command, 'tab', False) and z < tab_z:
                if not tabbing:
                    commands.append(Retract(tab_z))
                    tabbing = True
                commands.append(command.duplicate(tab_z))
            else:
                if tabbing:
                    commands.append(Plunge(z))
                    tabbing = False
                commands.append(command.duplicate(z))

        return CommandSequence(start, commands, end)
'''


class CommandVector(ABC):
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    @abstractmethod
    def to_vector(self, origin: cq.Vector, relative=False):
        pass


class RelativeCV(CommandVector):
    def to_vector(self, origin: cq.Vector, relative=False):
        if relative:
            x = 0 if self.x is None else self.x
            y = 0 if self.y is None else self.y
            z = 0 if self.z is None else self.z
        else:
            x = origin.x + self.x if self.x else origin.x
            y = origin.y + self.y if self.y else origin.y
            z = origin.z + self.z if self.z else origin.z
        return cq.Vector(x, y, z)


class AbsoluteCV(CommandVector):
    @classmethod
    def from_vector(cls, v: cq.Vector):
        return cls(v.x, v.y, v.z)

    def to_vector(self, origin: cq.Vector, relative=False):
        if relative:
            x = 0 if self.x is None else self.x - origin.x
            y = 0 if self.y is None else self.y - origin.y
            z = 0 if self.z is None else self.z - origin.z
        else:
            x = origin.x if self.x is None else self.x
            y = origin.y if self.y is None else self.y
            z = origin.z if self.z is None else self.z
        return cq.Vector(x, y, z)


class Command(ABC):
    modal = None
    max_depth: Optional[float]
    end: CommandVector
    tab: bool  # TODO not the right place to carry tab information imo?

    def __init__(self, end: CommandVector, tab=False):
        self.end = end
        self.tab = tab
        self.max_depth = None  # TODO whats this?

    @classmethod
    def abs(cls, x=None, y=None, z=None, tab=False):
        return cls(end=AbsoluteCV(x=x, y=y, z=z), tab=tab)

    @classmethod
    def rel(cls, x=None, y=None, z=None, tab=False):
        return cls(end=RelativeCV(x=x, y=y, z=z), tab=tab)

    def print_modal(self, previous: Optional[Command]):
        if self.modal and (previous is None or previous.modal != self.modal):
            return self.modal
        return ''

    def xyz_gcode(self, start: cq.Vector) -> (str, cq.Vector):
        coords = []
        end = self.end.to_vector(start)

        if start.x != end.x:
            coords.append(f'X{end.x}')

        if start.y != end.y:
            coords.append(f'Y{end.y}')

        if start.z != end.z:
            coords.append(f'Z{end.z}')

        return ''.join(coords), end

    @abstractmethod
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job) -> (str, cq.Vector):
        """ Output all the necessary G-Code required to perform the command """
        pass

    @abstractmethod
    def to_ais_shape(self, start: cq.Vector, transform: cq.Matrix) -> (AIS_Shape, cq.Vector):
        pass

    # @abstractmethod
    # def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
    #    pass

    # def diff(self, start: Optional[cq.Vector], job: Job) -> Tuple[str, cq.Vector]:
    #    """ Output X Y and Z coordinates as necessary """
    #    end = self.end(start)
    #    coordinates = []
    #
    #    if start is None or not equal_within_tolerance(end.x, start.x, job.gcode_precision):
    #        coordinates.append(f'X{normalize(round(end.x, job.gcode_precision))}')
    #    if start is None or not equal_within_tolerance(end.y, start.y, job.gcode_precision):
    #        coordinates.append(f'Y{normalize(round(end.y, job.gcode_precision))}')
    #    if start is None or not equal_within_tolerance(end.z, start.z, job.gcode_precision):
    #        coordinates.append(f'Z{normalize(round(end.z, job.gcode_precision))}')
    #
    #    return "".join(coordinates), end


class ReferencePosition(Command):
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector, job) -> (str, cq.Vector):
        raise RuntimeError('Reference position may not generate gcode')

    def to_ais_shape(self, start: cq.Vector, transform: cq.Matrix) -> (AIS_Shape, cq.Vector):
        raise RuntimeError('Reference position may not generate shape')


class Linear(Command, ABC):
    def to_gcode(self, previous: Optional[Command], start: cq.Vector, job) -> Tuple[str, cq.Vector]:
        xyz, end = self.xyz_gcode(start)
        return f'{self.print_modal(previous)}{xyz}', end

    def to_ais_shape(self, start, transform):
        end = self.end.to_vector(start)
        edge = cq.Edge.makeLine(start, end)
        shape = AIS_Shape(edge.wrapped)
        shape.SetColor(to_occ_color(getattr(self, 'ais_color', 'red')))
        return shape, end

    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        start = new_end - self.relative_end
        return self.__class__(-self.relative_end), start


class Rapid(Linear):
    modal = 'G0'
    ais_color = 'green'


class Cut(Linear):
    modal = 'G1'

    # @staticmethod
    # def from_edge(edge: cq.Edge, transitions):
    #    orientation = edge.wrapped.Orientation()
    #    reversed = orientation == TopAbs_REVERSED
    #
    #    commands = []
    #    for start, end in pairwise_open(transitions):
    #        _, transition = start
    #        end_d, _ = end
    #        position = edge.positionAt((1 - end_d) if reversed else end_d)
    #        if transition == Transition.TAB:
    #            commands.append(Cut(x=position.x, y=position.y, z=position.z, tab=True))
    #        else:
    #            commands.append(Cut(x=position.x, y=position.y, z=position.z))
    #    return commands


class Plunge(Cut):
    ais_color = 'yellow'

    # TODO apply plunge feed rate

    def __init__(self, end, tab=False):
        if end.x is not None or end.y is not None:
            raise RuntimeError('Plunge can only operate on z axis')
        super().__init__(end, tab)

    @classmethod
    def abs(cls, z=None, tab=False, **kwargs):
        return cls(end=AbsoluteCV(z=z), tab=tab)

    @classmethod
    def rel(cls, z=None, tab=False, **kwargs):
        return cls(end=RelativeCV(z=z), tab=tab)



class Retract(Rapid):
    """ Rapid retract """
    modal = 'G0'
    ais_color = 'blue'

    def __init__(self, z, tab=False):
        super().__init__(AbsoluteCV(z=z), tab)


# CIRCULAR MOTION

class Circular(Command, ABC):
    end: CommandVector
    center: CommandVector
    mid: CommandVector

    def __init__(self, end: CommandVector, center: CommandVector, mid: CommandVector, tab=False):
        super().__init__(end, tab)
        self.center = center
        self.mid = mid

    def ijk_gcode(self, start: cq.Vector, precision=3):
        center = self.center.to_vector(start)
        i = normalize(round(center.x, precision))
        j = normalize(round(center.y, precision))
        k = normalize(round(center.z, precision))

        ijk = []
        if self.center.x is not None:
            ijk.append(f'I{i}')

        if self.center.y is not None:
            ijk.append(f'J{j}')

        if self.center.z is not None:
            ijk.append(f'K{k}')

        return ''.join(ijk)

    def to_gcode(self, previous: Command, start: cq.Vector, job) -> Tuple[str, cq.Vector]:
        xyz, end = self.xyz_gcode(start)
        ijk = self.ijk_gcode(start)
        return f'{self.print_modal(previous)}{xyz}{ijk}', end

    def to_ais_shape(self, start, transform):
        end = self.end.to_vector(start)
        mid = self.mid.to_vector(start)
        edge = cq.Edge.makeThreePointArc(start, mid, end)
        shape = AIS_Shape(edge.wrapped)
        shape.SetColor(to_occ_color('red'))
        return shape, end

    """
    @classmethod
    def from_edge(cls, edge: cq.Edge, transitions):
        orientation = edge.wrapped.Orientation()
        reversed = orientation == TopAbs_REVERSED
        commands = []
        for start, end in pairwise_open(transitions):
            start_d, transition = start
            end_d, _ = end
            mid_d = (end_d + start_d) / 2

            start = edge.positionAt((1 - start_d) if reversed else start_d)
            mid = edge.positionAt((1 - mid_d) if reversed else mid_d)
            end = edge.positionAt((1 - end_d) if reversed else end_d)
            center = edge.Center()

            commands.append(cls._from_vectors(start, mid, end, center, tab=transition == Transition.TAB))
        return commands
    """
    """
    @staticmethod
    def _from_vectors(start, mid, end, center, tab):
        from cq_cam.commands.command import CircularCW, CircularCCW
        mid_relative = mid.sub(start)
        ijk = center.sub(start)

        if start.x == end.x and start.y == end.y:
            raise NotImplemented('Full circles are not implemented')

        if is_arc_clockwise(start, mid, end):
            return CircularCW(x=end.x, y=end.y, ijk=vector_to_tuple(ijk),
                              mid=vector_to_tuple(mid_relative), tab=tab)
        else:
            return CircularCCW(x=end.x, y=end.y, ijk=vector_to_tuple(ijk),
                               mid=vector_to_tuple(mid_relative), tab=tab)
    """


class CircularCW(Circular):
    modal = 'G2'

    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        # TODO TODO TODO
        start = new_end - self.relative_end
        return CircularCCW(-self.relative_end), start


class CircularCCW(Circular):
    modal = 'G3'
