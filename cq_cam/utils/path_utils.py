from copy import copy
from typing import List, Tuple

import cadquery as cq
import numpy as np
import pyclipper
from OCP.TopAbs import TopAbs_REVERSED

from cq_cam.utils.utils import (
    wire_to_ordered_edges, edge_start_point, get_underlying_geom_type, edge_end_point,
    closest_point_and_distance_squared_to_segment
)

"""
path is a list of cq.Vectors
"""


def wire_to_path(wire: cq.Wire):
    edges = wire_to_ordered_edges(wire)
    sp = edge_start_point(edges[0])
    path = [[sp.x, sp.y]]
    for edge in edges:
        geom_type = edge.geomType()
        if geom_type == 'OFFSET':
            geom_type = get_underlying_geom_type(edge)
        ep = edge_end_point(edge)
        if geom_type == 'LINE':
            path.append([ep.x, ep.y])

        elif geom_type in ['ARC', 'CIRCLE', 'BSPLINE', 'BEZIER']:
            path += interpolate_edge_as_2d_path(edge)[1:]

        else:
            raise RuntimeError(f'Unsupported geom type: {geom_type}')
    return path


def interpolate_edge_as_2d_path(edge: cq.Edge, precision: float = 0.1) -> List[Tuple[float, float]]:
    # Interpolation must have at least two edges
    n = max(int(edge.Length() / precision), 2)

    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        i, j = 1, 0
    else:
        i, j = 0, 1

    interpolations = []
    for length in np.linspace(i, j, n):
        position = edge.positionAt(length)
        interpolations.append([position.x, position.y])

    return interpolations


def path_i_and_point_at_d(path: List[cq.Vector], d: float):
    assert 0 <= d < 1
    segment_lengths = [(s2 - s1).Length for s1, s2 in zip(path, path[1:])]
    total_length = sum(segment_lengths)

    current_d = 0
    for i, segment_length in enumerate(segment_lengths):
        segment_d = segment_length / total_length
        segment_end_d = segment_d + current_d
        if current_d <= d < segment_end_d:
            s1 = path[i]
            s2 = path[i + 1]
            uv = (s2 - s1).normalized()
            point = s1 + uv * ((d - current_d) / segment_d * segment_length)
            return i, point
        current_d += segment_d
    raise ValueError('Failed to determine point ')


def find_closest_in_path(point: cq.Vector, path: List[cq.Vector]) -> Tuple[cq.Vector, float, float, int]:
    """
    Returns the closest point in a path and it's parameter d [0..1]
    :param point: find closest to this point
    :param path: list of vectors to search from
    :return:
    """
    closest_point = None
    closest_distance = None
    closest_segment_i = None
    path_length = 0
    segment_lengths = []
    for segment_i, (s1, s2) in enumerate(zip(path, path[1:])):
        cp, d2 = closest_point_and_distance_squared_to_segment(point, s1, s2)
        segment_length = (s2 - s1).Length
        path_length += segment_length
        segment_lengths.append(segment_length)

        if closest_distance is None or d2 <= closest_distance:
            closest_distance = d2
            closest_point = cp
            closest_segment_i = segment_i

    if closest_point:
        earlier_segment_ds = sum(
            [segment_length / path_length for segment_length in segment_lengths[:closest_segment_i]])
        d = earlier_segment_ds + (closest_point - path[closest_segment_i]).Length / path_length
    else:
        d = None

    return closest_point, closest_distance, d, closest_segment_i


class ClipperPathSimulator:
    # TODO: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Orientation.htm

    def __init__(self, tool_radius: float, precision=0.1):
        self.tool_radius = pyclipper.scale_to_clipper(tool_radius)
        self.cleared_area = []
        self.arc_precision = pyclipper.scale_to_clipper(precision)

    def _empty_copy(self):
        cps = copy(self)
        cps.cleared_area = []
        return cps

    def add_clipper_path(self, clipper_path: List[List[int]]):
        rv = self._empty_copy()
        offset = pyclipper.PyclipperOffset(arc_tolerance=self.arc_precision)
        offset.AddPath(path=clipper_path, join_type=pyclipper.JT_ROUND, end_type=pyclipper.ET_CLOSEDLINE)
        cleared_path = offset.Execute(self.tool_radius)
        if self.cleared_area:
            clipper = pyclipper.Pyclipper()
            for cleared_area_path in self.cleared_area:
                clipper.AddPath(path=cleared_area_path, poly_type=pyclipper.PT_CLIP, closed=True)
            clipper.AddPath(path=cleared_path, poly_type=pyclipper.PT_SUBJECT, closed=True)
            rv.cleared_area = clipper.Execute(pyclipper.CT_UNION)

        else:
            rv.cleared_area = [cleared_path]

        return rv

    def safe_area(self):
        offset = pyclipper.PyclipperOffset(arc_tolerance=self.arc_precision)
        for cleared_area_path in self.cleared_area:
            offset.AddPath(path=cleared_area_path, join_type=pyclipper.JT_ROUND, end_type=pyclipper.ET_CLOSEDPOLYGON)

        safe_area = offset.Execute(-self.tool_radius)
        return safe_area

    def check_safe_move(self, v1: cq.Vector, v2: cq.Vector, clipper_path_to_be_cleared: List[List[int]]):
        clear_result = self.add_clipper_path(clipper_path_to_be_cleared)
        safe_area = clear_result.safe_area()

        clipper = pyclipper.Pyclipper()
        for path in safe_area:
            clipper.AddPath(path=path, poly_type=pyclipper.PT_CLIP, closed=True)

        clipper.AddPath(path=pyclipper.scale_to_clipper([[v1.x, v1.y], [v2.x, v2.y]]),
                        poly_type=pyclipper.PT_SUBJECT,
                        closed=False)

        outside_segments = clipper.Execute(pyclipper.CT_DIFFERENCE)

        return not outside_segments

