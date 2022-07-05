from enum import Enum


class Unit(Enum):
    METRIC = 20
    IMPERIAL = 21

    def to_gcode(self) -> str:
        if self == Unit.METRIC:
            return "G20"
        else:
            return "G21"
