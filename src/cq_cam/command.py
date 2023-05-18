"""
The RS274/NGC language is based on lines of code. Each line (command block) includes commands that change the internal state of the machine and/or move the machining center. Lines of code may be collected in a file to make a program.
G-code programs are read and executed by the gcode interpreter. The interpreter reads a line (command block) of gcode, builds an internal representation and changes the internal state of the machine and/or calls one or more canonical machining function.
G-code command can be organised into two levels of abstraction; the lower level, canonical machining functions and the higher level, RS274/NGC language.
The interpreter is responsible for lowering the RS274//NGC language into its equivalent canonical machining function representation (e.g. canned cycles are converted into G0 and G1 commands).
The interpreter should reject input commands or canonical functions that address non-existent equipment.

The command syntax for various commands is shown below:
Rapid Linear Motion: G0 X… Y… Z…
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
- A line may have zero to four M words, but two M words from the same modal group may not appear on the same line
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
    IJK,
    XYZ,
    AddressVector,
    Feed,
    Speed,
    ToolLengthOffset,
    ToolNumber,
)
from cq_cam.groups import (
    ArcDistanceMode,
    AutomaticChangerMode,
    CannedCycle,
    CoolantState,
    CutterState,
    DistanceMode,
    FeedRateControlMode,
    GCodeGroup,
    LengthCompensation,
    MotionControl,
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


class Command(ABC):
    pass


class MotionCommand(Command, ABC):
    modal: MotionControl
    start: AddressVector
    end: AddressVector
    ais_color = "red"
    ais_alt_color = "darkred"

    def __init__(
        self,
        end: AddressVector,
        start: AddressVector | None,
        arrow=False,
        **kwargs,
    ):
        if start is not None:
            if end.x is None:
                end.x = start.x
            if end.y is None:
                end.y = start.y
            if end.z is None:
                end.z = start.z
        else:
            start = AddressVector()
        self.start = start
        self.end = end
        self.arrow = arrow

        super().__init__(**kwargs)

    @classmethod
    def abs(cls, x=None, y=None, z=None, start: AddressVector | None = None, **kwargs):
        return cls(end=AddressVector(x=x, y=y, z=z), start=start, **kwargs)

    @abstractmethod
    def to_ais_shape(self, as_edges=False, alt_color=False) -> AIS_Shape:
        pass


class RapidCommand(MotionCommand, ABC):
    modal = Path.RAPID

    def to_ais_shape(self, as_edges=False, alt_color=False) -> AIS_Shape:
        start = cq.Vector(self.start.x, self.start.y, self.start.z)
        end = self.end.to_vector(start)
        if start == end:
            return None

        if as_edges:
            return cq.Edge.makeLine(start, end)

        shape = AIS_Line(
            Geom_CartesianPoint(start.toPnt()), Geom_CartesianPoint(end.toPnt())
        )
        if self.arrow:
            shape.Attributes().SetLineArrowDraw(True)

        shape.SetColor(
            cached_occ_color(self.ais_alt_color if alt_color else self.ais_color)
        )

        return shape

    def __str__(self) -> str:
        modal = str(self.modal)
        xyz = str(XYZ(self.end))
        words = [modal, xyz]

        return " ".join(words)


class Rapid(RapidCommand):
    ais_color = "green"


class PlungeRapid(Rapid):
    ais_color = "yellow"

    def __init__(self, end, start, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError("Plunge can only operate on z axis")
        super().__init__(end, start, **kwargs)

    @classmethod
    def abs(cls, z=None, start: AddressVector | None = None, **kwargs):
        return cls(end=AddressVector(z=z), start=start, **kwargs)


class Retract(Rapid):
    """Rapid retract"""

    ais_color = "blue"

    def __init__(self, end, start, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError("Retract can only operate on z axis")
        super().__init__(end, start, **kwargs)

    @classmethod
    def abs(cls, z=None, start: AddressVector | None = None, **kwargs):
        return cls(end=AddressVector(z=z), start=start, **kwargs)


class FeedRateCommand(MotionCommand, ABC):
    feed: float | None

    def __init__(
        self,
        end: AddressVector,
        start: AddressVector,
        arrow=False,
        feed: float | None = None,
        **kwargs,
    ):
        self.feed = feed
        super().__init__(end, start, arrow, **kwargs)


class Cut(FeedRateCommand):
    modal = Path.LINEAR

    def __str__(self) -> str:
        modal = str(self.modal)
        xyz = str(XYZ(self.end))
        feed = str(Feed(self.feed))
        words = [modal, xyz]
        if feed != "":
            words.append(feed)

        return " ".join(words)

    def to_ais_shape(self, as_edges=False, alt_color=False) -> AIS_Shape:
        start = cq.Vector(self.start.x, self.start.y, self.start.z)
        end = self.end.to_vector(start)
        if start == end:
            return None

        if as_edges:
            return cq.Edge.makeLine(start, end)

        shape = AIS_Line(
            Geom_CartesianPoint(start.toPnt()), Geom_CartesianPoint(end.toPnt())
        )
        if self.arrow:
            shape.Attributes().SetLineArrowDraw(True)

        shape.SetColor(
            cached_occ_color(self.ais_alt_color if alt_color else self.ais_color)
        )

        return shape


class PlungeCut(Cut):
    ais_color = "yellow"

    def __init__(self, end, start, **kwargs):
        if end.x is not None or end.y is not None:
            raise RuntimeError("Plunge can only operate on z axis")
        super().__init__(end, start, **kwargs)

    @classmethod
    def abs(cls, z=None, start: AddressVector | None = None, **kwargs):
        return cls(end=AddressVector(z=z), start=start, **kwargs)


# CIRCULAR MOTION
class Circular(FeedRateCommand, ABC):
    center: AddressVector
    mid: AddressVector

    def __init__(
        self,
        center: AddressVector,
        mid: AddressVector,
        **kwargs,
    ):
        self.center = center
        self.mid = mid
        super().__init__(**kwargs)

    def __str__(self) -> str:
        modal = str(self.modal)
        xyz = str(XYZ(self.end))
        center = self.center.to_vector(self.start, relative=True)
        ijk = str(IJK(center))
        feed = str(Feed(self.feed))
        words = [modal, xyz, ijk]
        if feed != "":
            words.append(feed)
        return " ".join(words)

    def to_ais_shape(self, as_edges=False, alt_color=False) -> AIS_Shape:
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
                return None
        if as_edges:
            return edge
        shape = AIS_Shape(edge.wrapped)
        shape.SetColor(
            cached_occ_color(self.ais_alt_color if alt_color else self.ais_color)
        )
        return shape


class CircularCW(Circular):
    modal = Path.ARC_CW


class CircularCCW(Circular):
    modal = Path.ARC_CCW


class ConfigCommand(Command, ABC):
    pass


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

        return " ".join(words)


class StopSequence(ConfigCommand):
    coolant: CoolantState | None

    def __init__(self, coolant: CoolantState | None = None):
        self.coolant = coolant
        super().__init__()

    def __str__(self) -> str:
        words = [f"{CutterState.OFF}"]
        if self.coolant is not None:
            words.append(f"{CoolantState.OFF}")

        return " ".join(words)


class SafetyBlock(ConfigCommand):
    def __str__(self) -> str:
        return "\n".join(
            (
                " ".join(
                    (
                        str(DistanceMode.ABSOLUTE),
                        # str(ArcDistanceMode.INCREMENTAL),
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
