"""
command.py builts upon the gcode letter addresses introduced in common.py to form full gcode command abstractions.
It is organised in a similiar fashion, into motion and non motion commands.

The code is strctured around the abstract classes CommandVector and Command:
- CommandVector
    - AbsouluteCV
    - RelativeCV
- Command
    - MotionCommand (abstract)
        - Linear (abstract)
            - Rapid
            - Cut
            - Plunge
            - Retract
        - Circular (abstract)
            - CircularCW
            - CircularCCW
    - ConfigCommand (abstract)
        - StartSequence
        - StopSequence
        - SafetyBlock
        - ToolChange

MotionCommands keep track of the previous command and position to optimise the generated gcode into a smaller size.

In the following example the G0 command in the second line along with X1 and Z1 are unnecessarily issued:
```
G0 X1 Y1 Z1
G0 X1 Y2 Z1
```

A more optimised version would look like this:
```
G0 X1 Y1 Z1
Y2
```

MotionCommands are consumed by routers.py
"""
from __future__ import annotations

import warnings
from abc import ABC, abstractmethod
from typing import Optional, Union

import cadquery as cq
from OCP.AIS import AIS_Line, AIS_Shape
from OCP.Geom import Geom_CartesianPoint

from cq_cam.common import (
    ArcDistanceMode,
    AutomaticChangerMode,
    CannedCycle,
    CoolantState,
    CutterState,
    DistanceMode,
    FeedRateControlMode,
    HomePosition,
    LengthCompensation,
    Path,
    PlannerControlMode,
    ProgramControlMode,
    RadiusCompensation,
    SpindleControlMode,
    Unit,
    WorkOffset,
    WorkPlane,
)
from cq_cam.utils.utils import optimize_float
from cq_cam.visualize import cached_occ_color


class CommandVector(ABC):
    __slots__ = ("x", "y", "z")

    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    @abstractmethod
    def to_vector(self, origin: cq.Vector, relative=False) -> cq.Vector:
        pass


class RelativeCV(CommandVector):
    def to_vector(self, origin: cq.Vector, relative=False):
        warnings.warn("Relative CV is deprecated", DeprecationWarning)
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


class MotionCommand(Command, ABC):
    max_depth: Optional[float]
    ais_color = "red"
    ais_alt_color = "darkred"
    previous_command: Union[MotionCommand, None]
    start: cq.Vector
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
        warnings.warn("Relative CV is deprecated", DeprecationWarning)
        return cls(end=RelativeCV(x=x, y=y, z=z), **kwargs)

    def print_modal(self, previous: Optional[MotionCommand]):
        if self.modal and (previous is None or previous.modal != self.modal):
            return self.modal
        return ""

    def xyz_gcode(self, start: cq.Vector, precision=3) -> tuple[str, cq.Vector]:
        coords = []
        end = self.end.to_vector(start)
        # TODO precision

        # TODO use isclose

        if start.x != end.x:
            coords.append(f"X{optimize_float(round(end.x, precision))}")

        if start.y != end.y:
            coords.append(f"Y{optimize_float(round(end.y, precision))}")

        if start.z != end.z:
            coords.append(f"Z{optimize_float(round(end.z, precision))}")

        return "".join(coords), end

    @abstractmethod
    def to_gcode(self) -> tuple[str, cq.Vector]:
        """Output all the necessary G-Code required to perform the command"""
        pass

    @abstractmethod
    def to_ais_shape(
        self, as_edges=False, alt_color=False
    ) -> tuple[AIS_Shape, cq.Vector]:
        pass


class ConfigCommand(Command, ABC):
    @abstractmethod
    def to_gcode(self) -> str:
        pass


class Linear(MotionCommand, ABC):
    def to_gcode(self) -> tuple[str, cq.Vector]:
        xyz, end = self.xyz_gcode(self.start)
        return f"{self.print_modal(self.previous_command)}{xyz}", end

    def to_ais_shape(self, as_edges=False, alt_color=False):
        end = self.end.to_vector(self.start)
        if self.start == end:
            return None, end

        if as_edges:
            return cq.Edge.makeLine(self.start, end), end

        shape = AIS_Line(
            Geom_CartesianPoint(self.start.toPnt()), Geom_CartesianPoint(end.toPnt())
        )
        if self.arrow:
            shape.Attributes().SetLineArrowDraw(True)

        shape.SetColor(
            cached_occ_color(self.ais_alt_color if alt_color else self.ais_color)
        )

        return shape, end

    def flip(self, new_end: cq.Vector) -> tuple[MotionCommand, cq.Vector]:
        start = new_end - self.relative_end
        return self.__class__(-self.relative_end), start


class Rapid(Linear):
    modal = Path.RAPID.to_gcode()
    ais_color = "green"


class Cut(Linear):
    modal = Path.LINEAR.to_gcode()


class Plunge(Cut):
    ais_color = "yellow"

    # TODO apply plunge feed rate

    def __init__(self, end, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError("Plunge can only operate on z axis")
        super().__init__(end, **kwargs)

    @classmethod
    def abs(cls, z=None, **kwargs):
        return cls(end=AbsoluteCV(z=z), **kwargs)

    @classmethod
    def rel(cls, z=None, **kwargs):
        warnings.warn("Relative CV is deprecated", DeprecationWarning)
        return cls(end=RelativeCV(z=z), **kwargs)


class Retract(Rapid):
    """Rapid retract"""

    ais_color = "blue"

    def __init__(self, end, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError("Retract can only operate on z axis")
        super().__init__(end, **kwargs)

    @classmethod
    def abs(cls, z=None, **kwargs):
        return cls(end=AbsoluteCV(z=z), **kwargs)

    @classmethod
    def rel(cls, z=None, **kwargs):
        warnings.warn("Relative CV is deprecated", DeprecationWarning)
        return cls(end=RelativeCV(z=z), **kwargs)


# CIRCULAR MOTION


class Circular(MotionCommand, ABC):
    end: CommandVector
    center: CommandVector
    mid: CommandVector

    def __init__(
        self, end: CommandVector, center: CommandVector, mid: CommandVector, **kwargs
    ):
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
            ijk.append(f"I{i}")

        if self.center.y is not None:
            ijk.append(f"J{j}")

        if self.center.z is not None:
            ijk.append(f"K{k}")

        return "".join(ijk)

    def to_gcode(self) -> tuple[str, cq.Vector]:
        xyz, end = self.xyz_gcode(self.start)
        ijk = self.ijk_gcode(self.start)
        return f"{self.print_modal(self.previous_command)}{xyz}{ijk}", end

    def to_ais_shape(self, as_edges=False, alt_color=False):
        end = self.end.to_vector(self.start)
        mid = self.mid.to_vector(self.start)

        # Note: precision of __eq__ on vectors can cause false positive circles with very small arcs
        # TODO: Neutralise small arcs, these can cause similar problem with grbl as far as I remember
        # if start == end:
        #    center = self.center.to_vector(start)
        #    radius = center.Length
        #    # uh oh
        #
        #    edge = cq.Edge.makeCircle(radius, center, cq.Vector(0,0,1))
        # else:
        try:
            edge = cq.Edge.makeThreePointArc(self.start, mid, end)
        except:
            try:
                edge = cq.Edge.makeLine(self.start, end)
            except:
                # Too small to render ?
                return None, end
        if as_edges:
            return edge, end
        shape = AIS_Shape(edge.wrapped)
        shape.SetColor(
            cached_occ_color(self.ais_alt_color if alt_color else self.ais_color)
        )
        return shape, end


class CircularCW(Circular):
    modal = Path.ARC_CW.to_gcode()


class CircularCCW(Circular):
    modal = Path.ARC_CCW.to_gcode()


class StartSequence(ConfigCommand):
    spindle: Optional[int] = None
    coolant: Optional[CoolantState] = None

    def __init__(
        self, spindle: Optional[int] = None, coolant: Optional[CoolantState] = None
    ) -> None:
        self.spindle = spindle
        self.coolant = coolant
        super().__init__()

    def to_gcode(self) -> str:
        gcode_str = CutterState.ON_CW.to_gcode()

        if self.spindle is not None:
            gcode_str += f" S{self.spindle}"

        if self.coolant is not None:
            gcode_str += f" {self.coolant.to_gcode()}"

        return gcode_str


class StopSequence(ConfigCommand):
    coolant: Optional[CoolantState] = None

    def __init__(self, coolant: Optional[CoolantState] = None):
        self.coolant = coolant
        super().__init__()

    def to_gcode(self) -> str:
        gcode_str = CutterState.OFF.to_gcode()
        if self.coolant is not None:
            gcode_str += f" {CoolantState.OFF.to_gcode()}"

        return gcode_str


class SafetyBlock(ConfigCommand):
    def to_gcode(self) -> str:
        return "\n".join(
            (
                " ".join(
                    (
                        DistanceMode.ABSOLUTE.to_gcode(),
                        WorkOffset.OFFSET_1.to_gcode(),
                        PlannerControlMode.BLEND.to_gcode(),
                        SpindleControlMode.MAX_SPINDLE_SPEED.to_gcode(),
                        WorkPlane.XY.to_gcode(),
                        FeedRateControlMode.UNITS_PER_MINUTE.to_gcode(),
                    )
                ),
                " ".join(
                    (
                        LengthCompensation.OFF.to_gcode(),
                        RadiusCompensation.OFF.to_gcode(),
                        CannedCycle.CANCEL.to_gcode(),
                    )
                ),
                Unit.METRIC.to_gcode(),
                HomePosition.HOME_2.to_gcode(),
            )
        )


class ToolChange(ConfigCommand):
    tool_number: Optional[int] = None
    spindle: Optional[int] = None
    coolant: Optional[CoolantState] = None

    def __init__(
        self,
        tool_number: int,
        spindle: Optional[int] = None,
        coolant: Optional[CoolantState] = None,
    ):
        self.tool_number = tool_number
        self.spindle = spindle
        self.coolant = coolant

        super().__init__()

    def to_gcode(self) -> str:
        return "\n".join(
            (
                StopSequence(self.coolant).to_gcode(),
                HomePosition.HOME_2.to_gcode(),
                ProgramControlMode.PAUSE_OPTIONAL.to_gcode(),
                " ".join(
                    (
                        f"T{self.tool_number}",
                        LengthCompensation.ON.to_gcode(),
                        f"H{self.tool_number}",
                        AutomaticChangerMode.TOOL_CHANGE.to_gcode(),
                    )
                ),
                StartSequence(self.spindle, self.coolant).to_gcode(),
            )
        )
