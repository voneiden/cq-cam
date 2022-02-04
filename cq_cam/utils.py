import abc
import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Tuple

import numpy as np
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.TopAbs import TopAbs_REVERSED
from cadquery import cq, Edge


def end_point(edge: cq.Edge):
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.startPoint()
    return edge.endPoint()


def start_point(edge: cq.Edge):
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.endPoint()
    return edge.startPoint()


def edge_start_end(edge: cq.Edge):
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.endPoint(), edge.startPoint()
    return edge.startPoint(), edge.endPoint()


def vectors_to_xy(plane: cq.Plane, *vectors: cq.Vector):
    return tuple((plane.xDir.dot(vector),
                  plane.yDir.dot(vector))
                 for vector in vectors)


def position_space(edge: cq.Edge, tolerance=0.1):
    # TODO orientation check
    return np.linspace(0, 1, math.ceil(edge.Length() / tolerance))


def flatten_edges(edges: List[cq.Edge]):
    vxs = []
    for i, edge in enumerate(edges):
        # LINE ARC CIRCLE SPLINE
        # if i == 0:
        #    vxs.append(edge.startPoint())

        if edge.geomType() == "LINE":
            vxs.append(end_point(edge))
        elif edge.geomType() in ["ARC", "CIRCLE"]:
            positions = edge.positions(position_space(edge))
            vxs += positions
    return vxs


@dataclass
class TypeVector(abc.ABC):
    __slots__ = ['start', 'end']
    start: Tuple[float, float]
    end: Tuple[float, float]


@dataclass
class LineTypeVector(TypeVector):
    pass


@dataclass
class ArcTypeVector(TypeVector, abc.ABC):
    __slots__ = ['start', 'end', 'mid', 'radius']
    start: Tuple[float, float]
    end: Tuple[float, float]
    mid: Tuple[float, float]  # Obsolete?
    center: Tuple[float, float]
    radius: float


class CWArcTypeVector(ArcTypeVector):
    pass


class CCWArcTypeVector(ArcTypeVector):
    pass


def reverse_type_vectors(tvs: List[TypeVector]):
    for tv in tvs:
        start = tv.start
        tv.start = tv.end
        tv.end = start
    tvs.reverse()
    return tvs


def wire_to_type_vectors(plane: cq.Plane, wire: cq.Wire) -> List[TypeVector]:
    """
    Convert a wire into ordered sequence of type vectors. Type vectors
    are also transformed from job plane to XY plane.

    :param plane: job plane
    :param wire: wire to convert
    :return:
    """
    ordered_edges = wire_to_ordered_edges(wire)
    tvs = []
    for edge in ordered_edges:
        if edge.geomType() == "LINE":
            tvs.append(
                LineTypeVector(
                    *(vectors_to_xy(plane, *edge_start_end(edge)))
                )
            )
        elif edge.geomType() in ["CIRCLE"]:
            start, end, center = vectors_to_xy(plane, *edge_start_end(edge), edge.arcCenter())
            radius = magnitude_2d(subtract_2d(start, center))

            if start[0] == end[0] and start[1] == end[1]:
                raise NotImplemented('Full circles are not implemented')

            mid, = vectors_to_xy(plane, edge.positionAt(0.5))

            if is_arc_clockwise(start, mid, end):
                tvs.append(CWArcTypeVector(start, end, mid, center, radius))
            else:
                tvs.append(CCWArcTypeVector(start, end, mid, center, radius))

        elif edge.geomType() == 'ARC':
            raise NotImplemented('ARC geom type is not implemented')

        elif edge.geomType() == 'SPLINE':
            raise NotImplemented('SPLINE geom type is not implemented')

        else:
            raise NotImplemented(f'Unknown geom type "{edge.geomType()}"')

    return tvs


def is_tvs_clockwise(type_vectors: List[TypeVector]):
    """
    Provide type_vector list that has been normalized to XY plane

    :param type_vectors:
    :return:
    """
    # TODO implement logic for circles or some other weird stuff
    if len(type_vectors) < 3:
        raise NotImplemented

    # Find the smallest y, biggest x
    # https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order/1180256#1180256

    b_tv = sorted(type_vectors, key=lambda tv: (tv.start[1], -tv.start[0]))[0]
    b_i = type_vectors.index(b_tv)
    b = b_tv.start
    a = type_vectors[b_i - 1].start
    c = type_vectors[(b_i + 1) % len(type_vectors)].start

    det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])

    return det < 0


def is_arc_clockwise(start: Tuple[float, float], mid: Tuple[float, float], end: Tuple[float, float]):
    # https://stackoverflow.com/questions/33960924/is-arc-clockwise-or-counter-clockwise

    # start -> end
    se = (end[0] - start[0], end[1] - start[1])

    # start -> mid
    sm = (mid[0] - start[0], mid[1] - mid[0])

    # cross product
    cp = se[0] * sm[1] - se[1] * sm[0]

    # Arc is clockwise if cross product is positive
    return cp > 0


def is_parallel_plane(plane1: cq.Plane, plane2: cq.Plane):
    # Based on Plane._eq_iter
    def _eq_iter():
        cls = type(plane1)
        yield isinstance(plane1, cq.Plane)
        yield isinstance(plane2, cq.Plane)
        # origins are the same
        # yield abs(plane1.origin - plane2.origin) < cls._eq_tolerance_origin
        # z-axis vectors are parallel (assumption: both are unit vectors, ignore direction)
        yield abs(plane1.zDir.dot(plane2.zDir)) < cls._eq_tolerance_dot + 1
        # x-axis vectors are parallel (assumption: both are unit vectors, ignore direction)
        yield abs(plane1.xDir.dot(plane2.xDir)) < cls._eq_tolerance_dot + 1

    return all(_eq_iter())


def dot(v1: cq.Vector, v2: cq.Vector):
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z


def plane_offset_distance(plane1: cq.Plane, plane2: cq.Plane):
    if not is_parallel_plane(plane1, plane2):
        return None

    height1 = dot(plane1.origin, plane1.zDir)
    height2 = dot(plane2.origin, plane1.zDir)
    return height2 - height1


def wire_to_ordered_edges(wire: cq.Wire) -> List[cq.Edge]:
    """
    It's a trap.

    OpenCASCADE topology doesn't mind wire edges not being in order.

    https://dev.opencascade.org/content/connectivity-edges-sequence
    :param wire: wire to explore edges from
    :return: list of ordered Edges
    """

    explorer = BRepTools_WireExplorer(wire.wrapped)
    ordered_edges = []
    while not explorer.Current().IsNull():
        ordered_edges.append(Edge(explorer.Current()))
        explorer.Next()

    return ordered_edges


def cut_clockwise(positive_offset: bool, spindle_clockwise: bool, climb: bool):
    """
    If all 3 are true, then cut must be done clockwise.
    Changing one to false, the cut must be done counter-clockwise.
    Changing two to false, the cut must be done clockwise.
    Changing all three to false, the cut must be done counter-clockwise.

    You get the idea..

    :param positive_offset: Positive offset = outside cut, negative offset = inside cut
    :param spindle_clockwise: Spindle spinning clockwise (top->down)
    :param climb: climb milling (vs conventional milling)
    :return: cut clockwise (or counter-clockwise)
    """
    return bool((positive_offset + spindle_clockwise + climb) % 2)


def subtract_2d(a: Tuple[float, float], b: Tuple[float, float]):
    return a[0] - b[0], a[1] - b[1]


def magnitude_2d(a: Tuple[float, float]):
    return math.sqrt(a[0] ** 2 + a[1] ** 2)
