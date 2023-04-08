from typing import List

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


def interpolate_edge_to_vectors(
    edge: cq.Edge, precision: float = 0.1
) -> List[cq.Vector]:
    # Interpolation must have at least two edges
    n = max(int(edge.Length() / precision), 2)

    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        i, j = 1, 0
    else:
        i, j = 0, 1

    interpolations = []
    for length in np.linspace(i, j, n):
        interpolations.append(edge.positionAt(length))

    return interpolations


def interpolate_edge(edge: cq.Edge, precision: float = 0.1) -> List[cq.Edge]:
    interpolations = interpolate_edge_to_vectors(edge, precision)

    result = []
    for a, b in zip(interpolations, interpolations[1:]):
        result.append(cq.Edge.makeLine(a, b))

    return result


def interpolate_edges_with_unstable_curves(
    edges: List[cq.Edge], precision: float = 0.1
):
    """
    It appears some curves do not offset nicely with OCCT. BSPLINE is an example.
    These curves unfortunately need to be interpolated to ensure stable offset performance.
    :param edges:
    :param precision:
    :return:
    """
    result = []
    interpolated = False
    for edge in edges:
        geom_type = edge.geomType()
        if geom_type == "OFFSET":
            geom_type = get_underlying_geom_type(edge)

        if geom_type == "BSPLINE":
            edges = interpolate_edge(edge, precision)
            result += edges
            interpolated = True
        else:
            result.append(edge)
    return result, interpolated


def edge_to_vectors(edge: cq.Edge, precision: float = 0.1) -> List[cq.Vector]:
    geom_type = edge.geomType()
    if geom_type == "OFFSET":
        geom_type = get_underlying_geom_type(edge)

    if geom_type == "LINE":
        return list(edge_start_end(edge))
    else:
        return interpolate_edge_to_vectors(edge, precision)


def wire_to_vectors(
    wire: cq.Wire, precision: float = 0.1, close=True
) -> List[cq.Vector]:
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


def interpolate_wire_with_unstable_edges(
    wire: cq.Wire, precision: float = 0.1
) -> cq.Wire:
    edges, interpolated = interpolate_edges_with_unstable_curves(
        wire.Edges(), precision
    )
    if not interpolated:
        return wire
    return cq.Wire.assembleEdges(edges)
