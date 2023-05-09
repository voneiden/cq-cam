"""
G-code (also RS-274) is the most widely-used computer numerical control (CNC) programming language. 
It is used mainly in computer-aided manufacturing to control automated machine tools, as well as from a 3D-printing slicer app.
Here we concentrate on a subset of G-code relevant for 3-axis CNC machining.

The G-code syntax includes single letter addresses to configure the machine state and control the motors. 
The two primary letter addresses are M-codes and G-codes. M-codes are known as machine codes (or more accurately miscalleneous codes) and G-codes are called preparatory codes.
M-codes allow for state changes of the machine components and the running program. G-codes control the motion of the motors and the internal configuration of the machine. G-code commands can be catergorised as modal or non-modal.
Modal commands remain in effect until they are replaced or cancelled by another command. Non-modal commands execute only once.
M-code and G-code are further organised into modal groups.

G-code Modal Groups
- Group 0 - Non-modal codes: G4, G10 G28, G30, G52, G53, G92, G92.1, G92.2, G92.3 
- Group 1 - Motion: G0, G1, G2, G3, G33, G38.n, G73, G76, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89
- Group 2 - Plane: G17, G18, G19, G17.1, G18.1, G19.1
- Group 3 - Distance Mode: G90, G91
- Group 4 - Arc Distance Mode: G90.1, G91.1
- Group 5 - Feed Rate Mode: G93, G94, G95
- Group 6 - Units: G20, G21
- Group 7 - Cutter Diameter Compensation: G40, G41, G42, G41.1, G42.1
- Group 8 - Tool Length Offset: G43, G43.1, G49
- Group 10 - Canned Cycle Return Mode: G98, G99
- Group 12 - Coordinate System: G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3
- Group 13 - Planner Control Mode: G61, G61.1, G64
- Group 14 - Spindle Speed Mode: G96, G97

M-code Modal Groups:
- Group 4 - Stopping: M0, M1, M2, M30, M60
- Group 7 - Spindle: M3, M4, M5
- Group 8 - Coolant: M7, M8, M9

In addition to the G and M address there are other letter addresses that are used in conjuction with them:
- F - Feed Rate: G1, G2, G3
- X - X Axis: G1, G2, G3
- Y - Y Axis: G1, G2, G3
- Z - Y Axis: G1, G2, G3
- I - Arc Center in X Axis: G2, G3
- J - Arc Center in Y Axis: G2, G3
- K - Arc Center in Z Axis: G2, G3
- R - Canned Cycle Arc Radius: G80, G81, G82, G83, G84, G85, G86, G88, G89
- P - Canned Cycle Dwell Time: G80, G81, G82, G83, G84, G85, G86, G88, G89
- L - Canned Cycle Loop Count: G80, G81, G82, G83, G84, G85, G86, G88, G89
- S - Spindle: M3, M4
- T - Tool Selection: M6
- H - Tool Length Offset: G43
- D - Radius Offset: G41, G42
"""

from enum import Enum


class GCodeAddress(Enum):
    def __repr__(self):
        return f"{self.__class__.__name__}.{self._name_}"

    def __str__(self):
        return self._value_


"""
The G-codes have been organised differently from the proposed modal groups.

## Motion Control
- Paths: G00 (Rapid), G01 (Linear), G02 (Arc CW), G03 (Arc CCW), G04 (Pause/Dwell), G05 (Cubic Spline)
- Home Position: G28 (Home 1), G30 (Home 2)
- CannedCycle: G80 (Cancel), G81 (Simple), G82 (Simple Dwell), G83 (Peck), G84 (Tap), G98 (Retract to initial Z), G99 (Retruct to last Z)
- CannedCycleReturnMode: G98, G99

## Machine Configuration
- Work Plane: G17 (XY), G18 (XZ), G19 (YZ)
- Units: G20 (inch/min), G21 (mm/min)
- Length Compensation: G43 (ON), G49 (OFF)
- Radius Compensation: G40 (OFF), G41 (Left), G42 (Right)
- WorkOffset: G53, G54, G55, G56, G57, G58, G59
- MotionControlMode: G61 (Exact Stop Check), G64 (Blend)
- DistanceMode: G90 (Absolute), G91 (Incremental)
- ArcDistanceMode: G90.1 (Absolute), G91.1 (Incremental)
- FeedRateControlMode: G93 (Inverse), G94 (Time), G95 (Revolution)
- SpindleControlMode: G96 (Surface Speed), G97 (RPM)

## Tool Configuration
- LengthCompensation: G40 (OFF), G41 (Left), G42 (Right)
- RadiusCompensation: G43 (ON), G49 (OFF)
"""


class Path(GCodeAddress):
    RAPID = "G0"
    LINEAR = "G1"
    ARC_CW = "G2"
    ARC_CCW = "G3"
    PAUSE = "G4"
    CUBIC = "G5"
    QUADRATIC_SPLINE = "G5.1"
    NURBS = "G5.2"


class WorkPlane(GCodeAddress):
    XY = "G17"
    XZ = "G18"
    YZ = "G19"


class Unit(GCodeAddress):
    IMPERIAL = "G20"
    METRIC = "G21"


class HomePosition(GCodeAddress):
    HOME_1 = "G28"
    HOME_2 = "G30"


class ProbeMode(GCodeAddress):
    ON_CONTACT_ERROR = "38.2"
    ON_CONTACT = "G38.3"
    LOSE_CONTACT_ERROR = "G38.4"
    LOSE_CONTACT = "G38.5"


class RadiusCompensation(GCodeAddress):
    OFF = "G40"
    LEFT = "G41"
    RIGHT = "G42"


class LengthCompensation(GCodeAddress):
    ON = "G43"
    OFF = "G49"


class WorkOffset(GCodeAddress):
    ABSOLUTE = "G53"
    OFFSET_1 = "G54"
    OFFSET_2 = "G55"
    OFFSET_3 = "G56"
    OFFSET_4 = "G57"
    OFFSET_6 = "G59"
    OFFSET_5 = "G58"


class PlannerControlMode(GCodeAddress):
    EXACT = "G61"
    BLEND = "G64"


class CannedCycle(GCodeAddress):
    CANCEL = "G80"
    DRILL_SIMPLE = "G81"
    DRILL_DWELL = "G82"
    DRILL_PECK = "G83"
    DRILL_TAP = "G84"
    BORE = "G85"
    BORE_DWELL_STOP = "G86"
    BORE_DWELL_STOP_MANUAL = "G88"
    BORE_DWELL = "G89"


class DistanceMode(GCodeAddress):
    ABSOLUTE = "G90"
    INCREMENTAL = "G91"


class ArcDistanceMode(GCodeAddress):
    ABSOLUTE = "G90.1"
    INCREMENTAL = "G91.1"


class FeedRateControlMode(GCodeAddress):
    INVERSE_TIME = "G93"
    UNITS_PER_MINUTE = "G94"
    UNITS_PER_REVOLUTION = "G95"


class SpindleControlMode(GCodeAddress):
    MAX_SPINDLE_SPEED = "G50"
    SURFACE_SPEED = "G96"
    RPM = "G97"


class CannedCycleReturnMode(GCodeAddress):
    INITIAL = "G98"
    LAST = "G99"


"""
Common M-Codes
- Program End: M02 (END), M30 (END RESET)
- Cutter State: M03 (ON CCW), M04 (ON CCW), M05 (OFF)
- Tool Change: M06
- Coolant State: M07 (Mist), M08 (Flood), M09 (OFF)
- Vacuum State: M10 (ON), M11 (OFF)
"""


class ProgramControlMode(GCodeAddress):
    PAUSE = "M0"
    PAUSE_OPTIONAL = "M1"
    END = "M2"
    END_RESET = "M30"


class CutterState(GCodeAddress):
    ON_CW = "M3"
    ON_CCW = "M4"
    OFF = "M5"


class AutomaticChangerMode(GCodeAddress):
    TOOL_CHANGE = "M6"
    PALLET_CHANGE = "M60"


class CoolantState(GCodeAddress):
    MIST = "M7"
    FLOOD = "M8"
    OFF = "M9"


class VacuumState(GCodeAddress):
    ON = "M10"
    OFF = "M11"
