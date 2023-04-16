import logging
from itertools import pairwise
from math import sqrt
from typing import List, Literal, Optional, Tuple, TypeAlias, Union

import cadquery as cq
import pyclipper as pc
from OCP.StdFail import StdFail_NotDone

from cq_cam.utils.circle_bug_workaround import circle_bug_workaround
from cq_cam.utils.interpolation import vectors_to_2d_tuples, wire_to_vectors
from cq_cam.utils.utils import dist_to_segment_squared, flatten_list

logger = logging.getLogger(__name__)
OffsetToolRadiusMultiplier: TypeAlias = float
OffsetDistance: TypeAlias = float
OffsetInput: TypeAlias = Union[
    OffsetToolRadiusMultiplier, Tuple[OffsetToolRadiusMultiplier, OffsetDistance]
]

Point: TypeAlias = Tuple[float, float]
Polygon: TypeAlias = List[Point]
PolyPosition: TypeAlias = Tuple[int, float]


class PolyFace:
    __slots__ = ("outer", "inners", "depth")

    def __init__(self, outer: Polygon, inners: List[Polygon], depth: float):
        self.outer: Polygon = outer
        self.inners: List[Polygon] = inners
        self.depth = depth

    @classmethod
    def from_cq_face(cls, cq_face: cq.Face):
        bbox = cq_face.BoundingBox()
        if bbox.zmin != bbox.zmax:
            raise ValueError("PolyFace supports only flat faces")

        return cls(
            wire_to_polygon(cq_face.outerWire()),
            [wire_to_polygon(wire) for wire in cq_face.innerWires()],
            bbox.zmax,
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

    validated_wires = []
    for wire in offset_wires:
        try:
            face = cq.Face.makeFromWires(wire)
            # Todo arbitrary..
            # How should one detect whether offset is possible??
            if face.Area() > 0.1:
                validated_wires.append(wire)
        except StdFail_NotDone:
            logger.warning("Failed to offset wire")
        except:
            logger.error("NOT PLANAR!")

    circle_bug_workaround(wire, validated_wires)
    return validated_wires


def offset_polygon(polygon: Polygon, offset: float) -> list[Polygon]:
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

    # TODO determine good value for the CleanPolygons
    offset_polygons = [
        close_polygon(pc.scale_from_clipper(offset_polygon))
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


def tuplify_polygon(polygon: Polygon):
    return [tuple(point) for point in polygon]


def close_polygon(polygon: Polygon):
    if polygon[0] != polygon[-1]:
        polygon.append(polygon[0])
    return polygon


def poly_tree_to_poly_faces(poly_tree: pc.PyPolyNode, depth: float) -> List[PolyFace]:
    polyfaces: List[PolyFace] = []
    for face in poly_tree.Childs:
        if face.depth > 1:
            logger.warning("Deep face encountered in make_polyface")
        outer = tuplify_polygon(close_polygon(pc.scale_from_clipper(face.Contour)))
        inners = [
            tuplify_polygon(close_polygon(pc.scale_from_clipper(child.Contour)))
            for child in face.Childs
        ]
        polyfaces.append(PolyFace(outer, inners, depth))
    return polyfaces


def make_polyfaces(
    outers: List[Polygon], inners: List[Polygon], depth: float
) -> List[PolyFace]:
    return difference_poly_tree(outers, inners, depth)


def offset_polyface(
    polyface: PolyFace, outer_offset: float, inner_offset: float
) -> List[PolyFace]:
    outers = offset_polygon(polyface.outer, outer_offset)
    if polyface.inners:
        inners = flatten_list(
            [offset_polygon(inner, inner_offset) for inner in polyface.inners]
        )
        return make_polyfaces(outers, inners, polyface.depth)
    else:
        return [PolyFace(outer, inners=[], depth=polyface.depth) for outer in outers]


def prepare_polygon_boolean_op(
    subjects: List[Polygon], clips: List[Polygon]
) -> pc.Pyclipper:
    clipper = pc.Pyclipper()
    # noinspection PyArgumentList
    scaled_subjects = [pc.scale_to_clipper(subject) for subject in subjects]
    # noinspection PyArgumentList
    scaled_clips = [pc.scale_to_clipper(clip) for clip in clips]

    clipper.AddPaths(scaled_subjects, pc.PT_SUBJECT)

    if scaled_clips:
        clipper.AddPaths(scaled_clips, pc.PT_CLIP)

    return clipper


def boolean_op_polygon_list(
    subjects: List[Polygon], clips: List[Polygon], clip_type: int
) -> List[Polygon]:
    clipper = prepare_polygon_boolean_op(subjects, clips)
    results = [pc.scale_from_clipper(result) for result in clipper.Execute(clip_type)]
    return results


def boolean_op_poly_tree(
    subjects: List[Polygon], clips: List[Polygon], clip_type: int
) -> pc.PyPolyNode:
    clipper = prepare_polygon_boolean_op(subjects, clips)
    results = clipper.Execute2(clip_type)
    return results


def union_poly_tree(
    subjects: List[Polygon], clips: List[Polygon], depth: float
) -> List[PolyFace]:
    return poly_tree_to_poly_faces(
        boolean_op_poly_tree(subjects, clips, pc.CT_UNION), depth
    )


def difference_poly_tree(
    subjects: List[Polygon], clips: List[Polygon], depth: float
) -> List[PolyFace]:
    return poly_tree_to_poly_faces(
        boolean_op_poly_tree(subjects, clips, pc.CT_DIFFERENCE), depth
    )


def segment_length_squared(s1: Point, s2: Point):
    return (s2[0] - s1[0]) ** 2 + (s2[1] - s1[1]) ** 2


def distance_to_polygon(
    point: Point, polygon: Polygon
) -> Tuple[float, Point, PolyPosition]:
    distance, closest_point, poly_position = None, None, None
    for i, (segment_start, segment_end) in enumerate(pairwise(polygon)):
        s_distance, s_closest_point = dist_to_segment_squared(
            point, segment_start, segment_end
        )
        if distance is None or s_distance < distance:
            distance = s_distance
            closest_point = s_closest_point
            segment_length = segment_length_squared(segment_start, segment_end)
            point_length = segment_length_squared(segment_start, closest_point)

            # the lengths are squared, so to get the correct ratio
            # sqrt needs to be applied
            poly_param = sqrt(point_length / segment_length)
            poly_position = (i, poly_param)

    return distance, closest_point, poly_position
