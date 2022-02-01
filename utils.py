import math
from typing import List

import numpy as np
from OCP.TopAbs import TopAbs_REVERSED
from cadquery import cq


def end_point(edge: cq.Edge):
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.startPoint()
    return edge.endPoint()


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


def is_parallel_plane(plane1: cq.Plane, plane2: cq.Plane):
    # Based on Plane._eq_iter
    def _eq_iter():
        cls = type(plane1)
        yield isinstance(plane1, cq.Plane)
        yield isinstance(plane2, cq.Plane)
        # origins are the same
        # yield abs(plane1.origin - plane2.origin) < cls._eq_tolerance_origin
        # z-axis vectors are parallel (assumption: both are unit vectors)
        yield abs(plane1.zDir.dot(plane2.zDir) - 1) < cls._eq_tolerance_dot
        # x-axis vectors are parallel (assumption: both are unit vectors)
        yield abs(plane1.xDir.dot(plane2.xDir) - 1) < cls._eq_tolerance_dot

    return all(_eq_iter())


def dot(v1: cq.Vector, v2: cq.Vector):
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z


def plane_offset_distance(plane1: cq.Plane, plane2: cq.Plane):
    if not is_parallel_plane(plane1, plane2):
        return None

    height1 = dot(plane1.origin, plane1.zDir)
    height2 = dot(plane2.origin, plane2.zDir)
    return height2 - height1
