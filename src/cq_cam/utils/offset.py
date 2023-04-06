from typing import List, Optional, Tuple, TypeAlias, Union

import cadquery as cq

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


def offset_face(
    face: cq.Face, outer_offset: float, inner_offset: float
) -> List[cq.Face]:
    offset_faces = []
    op_outers = face.outerWire().offset2D(outer_offset)
    op_inners = []
    for inner in face.innerWires():
        op_inners += inner.offset2D(inner_offset)
    for op_outer in op_outers:
        offset_faces.append(cq.Face.makeFromWires(op_outer, op_inners))
    return offset_faces
