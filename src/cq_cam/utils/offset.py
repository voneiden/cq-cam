from typing import Optional, Tuple, TypeAlias, Union

OffsetToolRadiusMultiplier: TypeAlias = float
OffsetDistance: TypeAlias = float
OffsetInput: TypeAlias = Union[
    OffsetToolRadiusMultiplier, Tuple[OffsetToolRadiusMultiplier, OffsetDistance]
]


def calculate_offset(tool_radius: float, offset: Optional[OffsetInput], default=None):
    if offset is None:
        return default or 0

    try:
        multiplier, distance = offset
        return tool_radius * multiplier + distance

    except TypeError:
        return tool_radius * offset
