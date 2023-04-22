from enum import Enum


class Unit(Enum):
    IMPERIAL = 20
    METRIC = 21

    def to_gcode(self) -> str:
        if self == Unit.METRIC:
            return "G21"
        else:
            return "G20"
