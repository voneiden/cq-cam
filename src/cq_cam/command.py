"""
The RS274/NGC language is based on lines of code. Each line (command block) includes commands that change the internal state of the machine and/or move the machining center. Lines of code may be collected in a file to make a program.
G-code programs are read and executed by the gcode interpreter. The interpreter reads a line (command block) of gcode, builds an internal representation and changes the internal state of the machine and/or calls one or more canonical machining function.
G-code command can be organised into two levels of abstraction; the lower level, canonical machining functions and the higher level, RS274/NGC language.
The interpreter is responsible for lowering the RS274//NGC language into its equivalent canonical machining function representation (e.g. canned cycles are converted into G0 and G1 commands).
The interpreter should reject input commands or canonical functions that address non-existent equipment.

The command syntax for various commands is shown below:
Rapid Linear Motioun: G0 X… Y… Z…
Linear Motion at Feed Rate: G1 X… Y… Z… F…
Arc at Feed Rate (Center Format Arc): G2 X… Y… Z… I… J… K… F…
Simple Drill: <G98> G81 X… Y… Z… R… L…
Drill with Dwell: G82 X… Y… Z… R… L… P…
Peck Drill: G83 X… Y… Z… R… L… Q…
Right-hand Tap: G84 X… Y… Z… R… L…
Reaming cycle: G85 X… Y… Z… R… L…
Boring with spindle stop cycle: G86 X… Y… Z… R… L… P…
Back Boring cycle: G87 X… Y… Z… R… L… I… J… K…
Boring with manual retraction: G88 X… Y… Z… R… L… P…
Boring cycle: G89 X… Y… Z… R… L… P…
Dwell: G4 P…
Return to Home: G28 X… Y… Z…
Move in absolute coordinates: G1 G53 X… Y… Z…

When there are multiple words on the same line those words are executed in the following order:
1. Comment
2. Feed Rate Mode (G93, G94)
3. Set Feed Rate (F)
4. Set Spindle Speed (S)
5. Select Tool (T)
6. Change Tool (M6)
7. Spindle Control (M3, M4, M5)
8. Coolant Control (M7, M8, M9)
9. Enable or disable limit switches (M48, M49)
10. Dwell (G4)
11. Set Active Plane (G17, G18, G19)
12. Set Length Units (G20, G21)
13. Cutter Radius Compensation (G40, G41, G42)
14. Cutter Length Compensation (G43, G49)
15. Coordinate System Selection (G54-G59.3)
16. Set Path Control Mode (G61, G61.1, G64)
17. Set Distance Mode (G90, G91)
18. Set Retract Mode (G98, G99)
19. Home (G28, G30)
20. Motion Control (G0-G3 and G80-G89)
21. Stop (M0, M1, M2, M30, M60)

These are some of the rules that we must be aware off when constructing G-code programs:
- Demarcating a file with percents (%) is optional if the file has an M2 or M30 in it, but is is required if not
- Spaces and tabs are allowed anywhere on a line of code and do not change the meaning of the line
- Input is case insensitive
- Initial and trailing zeros are allowed but not required
- A line may have any number of G words, but two G words from the same modal group may not appear on the same line
- It is an error to a put a G-code from group 1 and group 0 (G10, G28, G30, G92) on the same line if both of them use axis words.
- A line may have zero to four M words, but two G words from the same modal group may not appear on the same line
- For all other legal letter, a line may have only one words beginning with that letter
- Axis words (ABC, XYZ) specify a destination point.
- Axis numbers are in the currently active coordinate system, unless explicitly described as being in the aboslute coordinate system (G53)
- Any optional ommited axes will have their current value
- It is an error if a required axis is ommited
- It is an error if all axis words are ommited
- It is common practice to put the T word for the next tool after the previous tool change to maximize the time available for the carousel to move
- It is an error if inverse time feed rate mode is active and line with G1, G2, or G3 motion does not have an F word

`command.py` builts upon the gcode modal groups and letter addresses introduced in `groups.py` and `address.py` respectively, to form full gcode command abstractions.
It is organised in a similiar fashion, into motion and non motion commands. MotionCommands are consumed by routers.py
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
"""

from __future__ import annotations

import warnings
from abc import ABC, abstractmethod

import cadquery as cq
from OCP.AIS import AIS_Line, AIS_Shape
from OCP.Geom import Geom_CartesianPoint

from cq_cam.address import (
    ArcXAxis,
    ArcYAxis,
    ArcZAxis,
    Feed,
    Speed,
    ToolLengthOffset,
    ToolNumber,
    XAxis,
    YAxis,
    ZAxis,
)
from cq_cam.groups import (
    ArcDistanceMode,
    AutomaticChangerMode,
    CannedCycle,
    CoolantState,
    CutterState,
    DistanceMode,
    FeedRateControlMode,
    LengthCompensation,
    Path,
    PlannerControlMode,
    Position,
    ProgramControlMode,
    RadiusCompensation,
    SpindleControlMode,
    Unit,
    WorkOffset,
    WorkPlane,
)
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

    def __str__(self) -> str:
        return f"{self.x} {self.y} {self.z}"


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

    @abstractmethod
    def to_gcode(self) -> tuple[str, cq.Vector | None]:
        """Output all the necessary G-Code required to perform the command"""
        pass


class MotionCommand(Command, ABC):
    max_depth: float | None
    ais_color = "red"
    ais_alt_color = "darkred"
    previous_command: MotionCommand | None
    feed: float | None
    start: cq.Vector
    end: CommandVector
    tab: bool  # TODO not the right place to carry tab information imo?

    def __init__(
        self, end: CommandVector, arrow=False, tab=False, feed: float | None = None
    ):
        self.end = end
        self.arrow = arrow
        self.tab = tab
        self.feed = feed
        self.max_depth = None  # TODO whats this?

    def __str__(self) -> str:
        return f"{self.modal} {self.end}"

    @classmethod
    def abs(cls, x=None, y=None, z=None, **kwargs):
        return cls(end=AbsoluteCV(x=x, y=y, z=z), **kwargs)

    @classmethod
    def rel(cls, x=None, y=None, z=None, **kwargs):
        warnings.warn("Relative CV is deprecated", DeprecationWarning)
        return cls(end=RelativeCV(x=x, y=y, z=z), **kwargs)

    def print_feed(self):
        if self.feed and self.modal != str(Path.RAPID):
            return f"{Feed(self.feed)}"
        return ""

    def xyz_gcode(self, precision=3) -> tuple[str, cq.Vector]:
        coords = []
        end = self.end
        # TODO precision

        # TODO use isclose

        if end.x is not None:
            coords.append(f"{XAxis(end.x, precision)}")

        if end.y is not None:
            coords.append(f"{YAxis(end.y, precision)}")

        if end.z is not None:
            coords.append(f"{ZAxis(end.z, precision)}")

        return " ".join(coords), end

    @abstractmethod
    def to_ais_shape(
        self, as_edges=False, alt_color=False
    ) -> tuple[AIS_Shape, cq.Vector]:
        pass


class ConfigCommand(Command, ABC):
    pass


class Linear(MotionCommand, ABC):
    def to_gcode(self) -> tuple[str, cq.Vector]:
        modal = self.modal
        xyz, end = self.xyz_gcode()
        feed = self.print_feed()
        words = [modal, xyz]
        if feed != "":
            words.append(feed)

        return (
            " ".join(words),
            end,
        )

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
    modal = str(Path.RAPID)
    ais_color = "green"


class Cut(Linear):
    modal = str(Path.LINEAR)


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
        ijk = []
        if self.center.x is not None:
            ijk.append(f"{ArcXAxis(center.x)}")

        if self.center.y is not None:
            ijk.append(f"{ArcYAxis(center.y)}")

        if self.center.z is not None:
            ijk.append(f"{ArcZAxis(center.z)}")

        return " ".join(ijk)

    def to_gcode(self) -> tuple[str, cq.Vector]:
        modal = self.modal
        xyz, end = self.xyz_gcode()
        ijk = self.ijk_gcode(self.start)
        feed = self.print_feed()
        words = [modal, xyz, ijk]
        if feed != "":
            words.append(feed)
        return " ".join(words), end

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
    modal = str(Path.ARC_CW)


class CircularCCW(Circular):
    modal = str(Path.ARC_CCW)


class StartSequence(ConfigCommand):
    speed: int | None
    coolant: CoolantState | None

    def __init__(
        self, speed: int | None = None, coolant: CoolantState | None = None
    ) -> None:
        self.speed = speed
        self.coolant = coolant
        super().__init__()

    def __str__(self) -> str:
        words = [f"{CutterState.ON_CW}"]

        if self.speed is not None:
            words.append(f"{Speed(self.speed)}")

        if self.coolant is not None:
            words.append(f"{self.coolant}")

        gcode_str = " ".join(words)

        return gcode_str

    def to_gcode(self) -> tuple[str, cq.Vector]:
        words = [f"{CutterState.ON_CW}"]

        if self.speed is not None:
            words.append(f"{Speed(self.speed)}")

        if self.coolant is not None:
            words.append(f"{self.coolant}")

        gcode_str = " ".join(words)
        return (gcode_str, None)


class StopSequence(ConfigCommand):
    coolant: CoolantState | None

    def __init__(self, coolant: CoolantState | None = None):
        self.coolant = coolant
        super().__init__()

    def __str__(self) -> str:
        words = [f"{CutterState.OFF}"]
        if self.coolant is not None:
            words.append(f"{CoolantState.OFF}")

        gcode_str = " ".join(words)
        return gcode_str

    def to_gcode(self) -> tuple[str, cq.Vector]:
        words = [f"{CutterState.OFF}"]
        if self.coolant is not None:
            words.append(f"{CoolantState.OFF}")

        gcode_str = " ".join(words)
        return (gcode_str, None)


class SafetyBlock(ConfigCommand):
    def __str__(self) -> str:
        return "\n".join(
            (
                " ".join(
                    (
                        str(DistanceMode.ABSOLUTE),
                        str(WorkOffset.OFFSET_1),
                        str(PlannerControlMode.CONTINUOUS),
                        str(SpindleControlMode.MAX_SPINDLE_SPEED),
                        str(WorkPlane.XY),
                        str(FeedRateControlMode.UNITS_PER_MINUTE),
                    )
                ),
                " ".join(
                    (
                        str(LengthCompensation.OFF),
                        str(RadiusCompensation.OFF),
                        str(CannedCycle.CANCEL),
                    )
                ),
                str(Unit.METRIC),
                str(Position.SECONDARY_HOME),
            )
        )

    def to_gcode(self) -> tuple[str, cq.Vector]:
        return (
            "\n".join(
                (
                    " ".join(
                        (
                            str(DistanceMode.ABSOLUTE),
                            str(WorkOffset.OFFSET_1),
                            str(PlannerControlMode.CONTINUOUS),
                            str(SpindleControlMode.MAX_SPINDLE_SPEED),
                            str(WorkPlane.XY),
                            str(FeedRateControlMode.UNITS_PER_MINUTE),
                        )
                    ),
                    " ".join(
                        (
                            str(LengthCompensation.OFF),
                            str(RadiusCompensation.OFF),
                            str(CannedCycle.CANCEL),
                        )
                    ),
                    str(Unit.METRIC),
                    str(Position.SECONDARY_HOME),
                )
            ),
            None,
        )


class ToolChange(ConfigCommand):
    tool_number: int | None
    speed: int | None
    coolant: CoolantState | None

    def __init__(
        self,
        tool_number: int,
        speed: int | None = None,
        coolant: CoolantState | None = None,
    ):
        self.tool_number = tool_number
        self.speed = speed
        self.coolant = coolant

        super().__init__()

    def __str__(self) -> str:
        return "\n".join(
            (
                str(StopSequence(self.coolant)),
                str(Position.SECONDARY_HOME),
                str(ProgramControlMode.PAUSE_OPTIONAL),
                " ".join(
                    (
                        f"{ToolNumber(self.tool_number)}",
                        str(LengthCompensation.ON),
                        f"{ToolLengthOffset(self.tool_number)}",
                        str(AutomaticChangerMode.TOOL_CHANGE),
                    )
                ),
                str(StartSequence(self.speed, self.coolant)),
            )
        )

    def to_gcode(self) -> tuple[str, cq.Vector]:
        return (
            "\n".join(
                (
                    str(StopSequence(self.coolant)),
                    str(Position.SECONDARY_HOME),
                    str(ProgramControlMode.PAUSE_OPTIONAL),
                    " ".join(
                        (
                            f"{ToolNumber(self.tool_number)}",
                            str(LengthCompensation.ON),
                            f"{ToolLengthOffset(self.tool_number)}",
                            str(AutomaticChangerMode.TOOL_CHANGE),
                        )
                    ),
                    str(StartSequence(self.speed, self.coolant)),
                )
            ),
            None,
        )
