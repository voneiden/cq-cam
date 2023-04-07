from typing import List, Literal, Optional, Tuple, TypeAlias, Union

import cadquery as cq

from cq_cam.utils.circle_bug_workaround import circle_bug_workaround

OffsetToolRadiusMultiplier: TypeAlias = float
OffsetDistance: TypeAlias = float
OffsetInput: TypeAlias = Union[
    OffsetToolRadiusMultiplier, Tuple[OffsetToolRadiusMultiplier, OffsetDistance]
]


def calculate_offset(tool_radius: float, offset: Optional[OffsetInput], default=None):
    if offset is None:
        offset = default or 0

    try:
        multiplier, distance = offset
        return tool_radius * multiplier + distance

    except TypeError:
        return tool_radius * offset


def offset_wire(
    wire: cq.Wire, offset, kind: Literal["arc", "intersection", "tangent"] = "arc"
) -> List[cq.Wire]:
    offset_wires = wire.offset2D(offset, kind)
    return circle_bug_workaround(wire, offset_wires)


def offset_face(
    face: cq.Face, outer_offset: float, inner_offset: float
) -> List[cq.Face]:
    offset_faces = []
    op_outers = offset_wire(face.outerWire(), outer_offset)
    op_inners = []
    for inner in face.innerWires():
        op_inners += offset_wire(inner, inner_offset)
    for op_outer in op_outers:
        offset_faces.append(cq.Face.makeFromWires(op_outer, op_inners))
    return offset_faces
