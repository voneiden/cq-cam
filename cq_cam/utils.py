import math
from typing import List, Tuple, Iterable

import numpy as np
import pyclipper
from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopoDS import TopoDS_Shape, TopoDS_Vertex, TopoDS
from cadquery import cq, Edge


def end_point(edge: cq.Edge) -> cq.Vector:
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.startPoint()
    return edge.endPoint()


def start_point(edge: cq.Edge) -> cq.Vector:
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.endPoint()
    return edge.startPoint()


def edge_start_end(edge: cq.Edge) -> Tuple[cq.Vector, cq.Vector]:
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.endPoint(), edge.startPoint()
    return edge.startPoint(), edge.endPoint()


def vertex_to_vector(vertex: TopoDS_Shape) -> cq.Vector:
    geom_point = BRep_Tool.Pnt_s(TopoDS.Vertex_s(vertex))
    return cq.Vector(geom_point.X(), geom_point.Y(), geom_point.Z())


def orient_vector(vector: cq.Vector, plane: cq.Plane):
    return cq.Vector(plane.xDir.dot(vector), plane.yDir.dot(vector), plane.zDir.dot(vector))


def drop_z(vector: cq.Vector) -> Tuple[float, float]:
    return vector.x, vector.y


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
            # TODO handle full circles
            positions = edge.positions(position_space(edge))
            vxs += positions
    return vxs


def flatten_wire(wire: cq.Wire) -> List[cq.Vector]:
    return flatten_edges(wire_to_ordered_edges(wire))


def is_arc_clockwise(start: cq.Vector, mid: cq.Vector, end: cq.Vector):
    # https://stackoverflow.com/questions/33960924/is-arc-clockwise-or-counter-clockwise
    if start.z != mid.z:
        raise NotImplemented('Helical arcs not supported yet')

    # start -> end
    se = (end.x - start.x, end.y - start.y)

    # start -> mid
    sm = (mid.x - start.x, mid.y - start.y)

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


def plane_offset_distance(plane1: cq.Plane, plane2: cq.Plane):
    if not is_parallel_plane(plane1, plane2):
        return None

    height1 = plane1.origin.dot(plane1.zDir)
    height2 = plane2.origin.dot(plane1.zDir)
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


class WireClipper:
    def __init__(self):
        self._clipper = pyclipper.Pyclipper()
        self._pt_clip_cache = []

    def add_clip_wire(self, plane: cq.Plane, wire: cq.Wire):
        return self._add_wire(plane, wire, pyclipper.PT_CLIP)

    def add_subject_wire(self, plane: cq.Plane, wire: cq.Wire):
        return self._add_wire(plane, wire, pyclipper.PT_SUBJECT)

    def _add_wire(self, plane: cq.Plane, wire: cq.Wire, pt):
        polygon = [drop_z(orient_vector(v, plane)) for v in flatten_wire(wire)]
        self._add_path(pyclipper.scale_to_clipper(polygon), pt, wire.IsClosed())

    def add_clip_polygon(self, polygon: Iterable[Tuple[float, float]], is_closed=False):
        return self._add_polygon(polygon, pyclipper.PT_CLIP, is_closed)

    def add_subject_polygon(self, polygon: Iterable[Tuple[float, float]], is_closed=False):
        return self._add_polygon(polygon, pyclipper.PT_SUBJECT, is_closed)

    def _add_polygon(self, polygon: Iterable[Tuple[float, float]], pt, is_closed=False):
        self._add_path(pyclipper.scale_to_clipper(polygon), pt, is_closed)

    def _add_path(self, path, pt, closed):
        if pt == pyclipper.PT_CLIP:
            self._pt_clip_cache.append((path, pt, closed))
        self._clipper.AddPath(path, pt, closed)

    def reset(self):
        self._clipper.Clear()
        for cached in self._pt_clip_cache:
            self._clipper.AddPath(*cached)

    def bounds(self):
        bounds: pyclipper.PyIntRect = self._clipper.GetBounds()
        return {
            'left': pyclipper.scale_from_clipper(bounds.left),
            'top': pyclipper.scale_from_clipper(bounds.top),
            'right': pyclipper.scale_from_clipper(bounds.right),
            'bottom': pyclipper.scale_from_clipper(bounds.bottom),
        }

    def max_bounds(self):
        bounds = self.bounds()
        diagonal_length = self._bounds_diagonal_length(bounds)
        half_diagonal = diagonal_length / 2
        x_center = (bounds['left'] + bounds['right']) / 2
        y_center = (bounds['top'] + bounds['bottom']) / 2

        return {
            'left': math.floor(x_center - half_diagonal),
            'top': math.ceil(y_center + half_diagonal),
            'right': math.ceil(x_center + half_diagonal),
            'bottom': math.floor(y_center - half_diagonal),
        }

    def _bounds_diagonal_length(self, bounds=None):
        bounds = bounds if bounds else self.bounds()
        top_left = cq.Vector(bounds['left'], bounds['top'])
        bottom_right = cq.Vector(bounds['right'], bounds['bottom'])
        diagonal = top_left.sub(bottom_right)
        return diagonal.Length

    def execute(self):
        polytree = self._clipper.Execute2(pyclipper.CT_INTERSECTION)
        paths = pyclipper.scale_from_clipper(pyclipper.PolyTreeToPaths(polytree))
        return paths
