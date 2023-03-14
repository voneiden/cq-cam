from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Union, Optional, Tuple

import cadquery as cq
from OCP.AIS import AIS_Shape, AIS_Line
from OCP.Geom import Geom_CartesianPoint

from cq_cam.utils.utils import optimize_float
from src.cq_cam.visualize import to_occ_color


class CommandVector(ABC):
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    @abstractmethod
    def to_vector(self, origin: cq.Vector, relative=False) -> cq.Vector:
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
    ais_color = to_occ_color('red')
    end: CommandVector
    tab: bool  # TODO not the right place to carry tab information imo?

    def __init__(self, end: CommandVector, arrow=False, tab=False):
        self.end = end
        self.arrow = arrow
        self.tab = tab
        self.max_depth = None  # TODO whats this?

    @classmethod
    def abs(cls, x=None, y=None, z=None, **kwargs):
        return cls(end=AbsoluteCV(x=x, y=y, z=z), **kwargs)

    @classmethod
    def rel(cls, x=None, y=None, z=None, **kwargs):
        return cls(end=RelativeCV(x=x, y=y, z=z), **kwargs)

    def print_modal(self, previous: Optional[Command]):
        if self.modal and (previous is None or previous.modal != self.modal):
            return self.modal
        return ''

    def xyz_gcode(self, start: cq.Vector, precision=3) -> (str, cq.Vector):
        coords = []
        end = self.end.to_vector(start)
        # TODO precision

        # TODO use isclose

        if start.x != end.x:
            coords.append(f'X{optimize_float(round(end.x, precision))}')

        if start.y != end.y:
            coords.append(f'Y{optimize_float(round(end.y, precision))}')

        if start.z != end.z:
            coords.append(f'Z{optimize_float(round(end.z, precision))}')

        return ''.join(coords), end

    @abstractmethod
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector) -> (str, cq.Vector):
        """ Output all the necessary G-Code required to perform the command """
        pass

    @abstractmethod
    def to_ais_shape(self, start: cq.Vector, as_edges=False) -> (AIS_Shape, cq.Vector):
        pass


class ReferencePosition(Command):
    def to_gcode(self, previous_command: Union[Command, None], start: cq.Vector) -> (str, cq.Vector):
        raise RuntimeError('Reference position may not generate gcode')

    def to_ais_shape(self, start: cq.Vector, as_edges=False) -> (AIS_Shape, cq.Vector):
        raise RuntimeError('Reference position may not generate shape')


class Linear(Command, ABC):
    def to_gcode(self, previous_command: Optional[Command], start: cq.Vector) -> Tuple[str, cq.Vector]:
        xyz, end = self.xyz_gcode(start)
        return f'{self.print_modal(previous_command)}{xyz}', end

    def to_ais_shape(self, start, as_edges=False):
        end = self.end.to_vector(start)
        if start == end:
            return None, end

        if as_edges:
            return cq.Edge.makeLine(start, end), end

        shape = AIS_Line(
            Geom_CartesianPoint(start.toPnt()),
            Geom_CartesianPoint(end.toPnt())
        )
        if self.arrow:
            shape.Attributes().SetLineArrowDraw(True)
        shape.SetColor(self.ais_color)
        return shape, end

    def flip(self, new_end: cq.Vector) -> (Command, cq.Vector):
        start = new_end - self.relative_end
        return self.__class__(-self.relative_end), start


class Rapid(Linear):
    modal = 'G0'
    ais_color = to_occ_color('green')


class Cut(Linear):
    modal = 'G1'


class Plunge(Cut):
    ais_color = to_occ_color('yellow')

    # TODO apply plunge feed rate

    def __init__(self, end, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError('Plunge can only operate on z axis')
        super().__init__(end, **kwargs)

    @classmethod
    def abs(cls, z=None, **kwargs):
        return cls(end=AbsoluteCV(z=z), **kwargs)

    @classmethod
    def rel(cls, z=None, **kwargs):
        return cls(end=RelativeCV(z=z), **kwargs)


class Retract(Rapid):
    """ Rapid retract """
    ais_color = to_occ_color('blue')

    def __init__(self, end, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError('Retract can only operate on z axis')
        super().__init__(end, **kwargs)

    @classmethod
    def abs(cls, z=None, **kwargs):
        return cls(end=AbsoluteCV(z=z), **kwargs)

    @classmethod
    def rel(cls, z=None, **kwargs):
        return cls(end=RelativeCV(z=z), **kwargs)


# CIRCULAR MOTION

class Circular(Command, ABC):
    end: CommandVector
    center: CommandVector
    mid: CommandVector

    def __init__(self, end: CommandVector, center: CommandVector, mid: CommandVector, **kwargs):
        super().__init__(end, **kwargs)
        self.center = center
        self.mid = mid

    def ijk_gcode(self, start: cq.Vector, precision=3):
        center = self.center.to_vector(start, relative=True)
        i = optimize_float(round(center.x, precision))
        j = optimize_float(round(center.y, precision))
        k = optimize_float(round(center.z, precision))

        ijk = []
        if self.center.x is not None:
            ijk.append(f'I{i}')

        if self.center.y is not None:
            ijk.append(f'J{j}')

        if self.center.z is not None:
            ijk.append(f'K{k}')

        return ''.join(ijk)

    def to_gcode(self, previous_command: Optional[Command], start: cq.Vector) -> Tuple[str, cq.Vector]:
        xyz, end = self.xyz_gcode(start)
        ijk = self.ijk_gcode(start)
        return f'{self.print_modal(previous_command)}{xyz}{ijk}', end

    def to_ais_shape(self, start, as_edges=False):
        end = self.end.to_vector(start)
        mid = self.mid.to_vector(start)
        if start == end:
            center = self.center.to_vector(start).toPnt()
            radius = (start - center).Length
            # uh oh

            edge = cq.Edge.makeCircle(radius, center, zDir)
        else:
            edge = cq.Edge.makeThreePointArc(start, mid, end)
        if as_edges:
            return edge, end
        shape = AIS_Shape(edge.wrapped)
        shape.SetColor(self.ais_color)
        return shape, end


class CircularCW(Circular):
    modal = 'G2'


class CircularCCW(Circular):
    modal = 'G3'
