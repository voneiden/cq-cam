from typing import List, Literal, Optional, Tuple, TypeAlias, Union

import cadquery as cq
import pyclipper as pc

from cq_cam.utils.circle_bug_workaround import circle_bug_workaround
from cq_cam.utils.interpolation import vectors_to_2d_tuples, wire_to_vectors
from cq_cam.utils.utils import pairwise

OffsetToolRadiusMultiplier: TypeAlias = float
OffsetDistance: TypeAlias = float
OffsetInput: TypeAlias = Union[
    OffsetToolRadiusMultiplier, Tuple[OffsetToolRadiusMultiplier, OffsetDistance]
]

Polygon: TypeAlias = List[Tuple[float, float]]


class PolyFace:
    def __init__(self, outer: Polygon, inners: List[Polygon]):
        self.outer: Polygon = outer
        self.inners: List[Polygon] = inners

    @classmethod
    def from_cq_face(cls, cq_face: cq.Face):
        return cls(
            wire_to_polygon(cq_face.outerWire()),
            [wire_to_polygon(wire) for wire in cq_face.innerWires()],
        )


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
    try:
        offset_wires = wire.offset2D(offset, kind)
    except ValueError:
        # If stars align, OCCT can hit a bug where it fails to offset with
        # the exact value, but will succeed with a slightly bigger or
        # smaller value
        try:
            offset_wires = wire.offset2D(offset + 0.000001, kind)
        except ValueError:
            return []
    circle_bug_workaround(wire, offset_wires)
    return offset_wires


def offset_polygon(polygon: Polygon, offset: float) -> List[Polygon]:
    # noinspection PyArgumentList
    scaled_polygon = pc.scale_to_clipper(polygon)

    # noinspection PyArgumentList
    scaled_offset = pc.scale_to_clipper(offset)
    # TODO precision?

    # noinspection PyArgumentList
    precision = pc.scale_to_clipper(0.01)

    # noinspection PyArgumentList
    pco = pc.PyclipperOffset(pc.scale_to_clipper(2.0), precision)
    pco.AddPath(scaled_polygon, pc.JT_ROUND, pc.ET_CLOSEDPOLYGON)
    offset_polygons = [
        pc.scale_from_clipper(offset_polygon)
        for offset_polygon in pc.CleanPolygons(
            pco.Execute(scaled_offset), precision / 100
        )
    ]
    return offset_polygons


def wire_to_polygon(wire: cq.Wire) -> Polygon:
    return vectors_to_2d_tuples(wire_to_vectors(wire))


def polygon_to_wire(polygon: Polygon, reference: Union[cq.Wire, float]) -> cq.Wire:
    if isinstance(reference, cq.Wire):
        z = reference.startPoint().z
    else:
        z = reference

    edges = [
        cq.Edge.makeLine((a[0], a[1], z), (b[0], b[1], z)) for a, b in pairwise(polygon)
    ]

    # This can be fairly slow
    wire = cq.Wire.assembleEdges(edges)
    return wire


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


def polygon_boolean_op(subjects: List[Polygon], clips: List[Polygon], clip_type: int):
    clipper = pc.Pyclipper()
    # noinspection PyArgumentList
    scaled_subjects = [pc.scale_to_clipper(subject) for subject in subjects]
    # noinspection PyArgumentList
    scaled_clips = [pc.scale_to_clipper(clip) for clip in clips]

    clipper.AddPaths(scaled_subjects, pc.PT_SUBJECT)
    clipper.AddPaths(scaled_clips, pc.PT_CLIP)
    results = [pc.scale_from_clipper(result) for result in clipper.Execute(clip_type)]
    return results
