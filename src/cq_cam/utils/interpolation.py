import cadquery as cq
import numpy as np
from OCP import Geom
from OCP.TopAbs import TopAbs_REVERSED

from cq_cam.utils.utils import edge_start_end, wire_to_ordered_edges


def get_edge_basis_curve(edge: cq.Edge):
    # AdaptorCurve -> GeomCurve
    curve = edge._geomAdaptor().Curve().Curve()
    while True:
        try:
            curve = curve.BasisCurve()
        except AttributeError:
            break
    return curve


geom_LUT_CURVE = {
    Geom.Geom_Line: "LINE",
    Geom.Geom_Circle: "CIRCLE",
    Geom.Geom_Ellipse: "ELLIPSE",
    Geom.Geom_Hyperbola: "HYPERBOLA",
    Geom.Geom_Parabola: "PARABOLA",
    Geom.Geom_BezierCurve: "BEZIER",
    Geom.Geom_BSplineCurve: "BSPLINE",
}


def get_underlying_geom_type(edge: cq.Edge):
    curve = get_edge_basis_curve(edge)
    return geom_LUT_CURVE[curve.__class__]


def interpolate_edge_to_vectors(edge: cq.Edge, precision: int) -> list[cq.Vector]:
    # Interpolation must have at least two edges
    n = edge_interpolation_count(edge, precision)

    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        i, j = 1, 0
    else:
        i, j = 0, 1

    interpolations = []
    for length in np.linspace(i, j, n):
        interpolations.append(edge.positionAt(length))

    return interpolations


def vectors_to_2d_tuples(vectors: list[cq.Vector]) -> list[tuple[float, float]]:
    return [(vector.x, vector.y) for vector in vectors]


def edge_to_vectors(edge: cq.Edge, precision: int) -> list[cq.Vector]:
    geom_type = edge.geomType()
    if geom_type == "OFFSET":
        geom_type = get_underlying_geom_type(edge)

    if geom_type == "LINE":
        return list(edge_start_end(edge))
    else:
        return interpolate_edge_to_vectors(edge, precision)


def wire_to_vectors(wire: cq.Wire, precision: int, close=True) -> list[cq.Vector]:
    edges = wire_to_ordered_edges(wire)

    if not edges:
        return []

    vectors = edge_to_vectors(edges[0], precision)

    for edge in edges[1:]:
        vectors += edge_to_vectors(edge, precision)[1:]

    if len(vectors) == 1:
        raise ValueError("Wire resulted only in one vector")

    if close and vectors[0] != vectors[-1]:
        vectors.append(vectors[0])

    elif not close and vectors[0] == vectors[-1]:
        vectors = vectors[:-1]

    return vectors


def edge_interpolation_count(edge: cq.Edge, precision: int):
    return max(int(edge.Length() * 10**precision), 2)
