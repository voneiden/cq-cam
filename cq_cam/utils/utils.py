import itertools
import math
from functools import cache
from typing import List, Tuple, Iterable

import numpy as np
import pyclipper
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from cadquery import cq, Edge
from cadquery.occ_impl.shapes import TOLERANCE


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
            # TODO make sure position_space ends up returning something (really tiny arcs?)
            positions = edge.positions(position_space(edge)[1:])
            vxs += positions
    return vxs


def flatten_wire(wire: cq.Wire) -> List[cq.Vector]:
    return flatten_edges(wire_to_ordered_edges(wire))


def is_arc_clockwise(start: cq.Vector, mid: cq.Vector, end: cq.Vector):
    # https://stackoverflow.com/questions/33960924/is-arc-clockwise-or-counter-clockwise
    if start.z != mid.z:
        raise NotImplementedError('Helical arcs not supported yet', start.z, mid.z, end.z)

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


def flatten_list(lst: List[Iterable]) -> List:
    return [element for nested_lst in lst for element in nested_lst]


class WireClipper:
    def __init__(self):
        self._clipper = pyclipper.Pyclipper()
        self._pt_clip_cache = []

    def add_clip_wire(self, wire: cq.Wire, cache=True):
        return self._add_wire(wire, pyclipper.PT_CLIP, cache=cache, is_closed=wire.IsClosed())

    def add_subject_wire(self, wire: cq.Wire, is_closed=None):
        is_closed = is_closed if is_closed is not None else wire.IsClosed()
        return self._add_wire(wire, pyclipper.PT_SUBJECT, False, is_closed)

    def _add_wire(self, wire: cq.Wire, pt, cache, is_closed):
        polygon = [drop_z(v) for v in flatten_wire(wire)]
        # Not sure if i'll shoot myself in the leg with this
        if not is_closed:
            polygon.append(polygon[0])
        self._add_path(pyclipper.scale_to_clipper(polygon), pt, is_closed, cache)
        return polygon

    def add_clip_polygon(self, polygon: Iterable[Tuple[float, float]], is_closed=False):
        return self._add_polygon(polygon, pyclipper.PT_CLIP, is_closed)

    def add_subject_polygon(self, polygon: Iterable[Tuple[float, float]], is_closed=False):
        return self._add_polygon(polygon, pyclipper.PT_SUBJECT, is_closed)

    def _add_polygon(self, polygon: Iterable[Tuple[float, float]], pt, is_closed=False):
        self._add_path(pyclipper.scale_to_clipper(polygon), pt, is_closed, False)

    def _add_path(self, path, pt, closed, cache):
        if pt == pyclipper.PT_CLIP and cache:
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

    def execute(self, clip_type=pyclipper.CT_INTERSECTION) -> Tuple[Tuple[Tuple[float, float], Tuple[float, float]]]:
        # TODO detect if there's nothing to do?
        polytree = self._clipper.Execute2(clip_type)

        open_paths = pyclipper.scale_from_clipper(pyclipper.OpenPathsFromPolyTree(polytree))
        closed_paths = pyclipper.scale_from_clipper(pyclipper.ClosedPathsFromPolyTree(polytree))
        # noinspection PyTypeChecker
        for closed_path in closed_paths:
            closed_path.append(closed_path[0])

        return tuple(tuple(tuple(point) for point in path) for path in (closed_paths + open_paths))

    def execute_difference(self):
        return self.execute(pyclipper.CT_DIFFERENCE)


def dist2(v, w):
    return (v[0] - w[0]) ** 2 + (v[1] - w[1]) ** 2


@cache
def cached_dist2(p1: Tuple[float, float], p2: Tuple[float, float]):
    return dist2(p1, p2)


def dist_to_segment_squared(p, v, w):
    """https://stackoverflow.com/a/1501725"""
    l2 = dist2(v, w)
    t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2
    t = max(0, min(1, t))
    closest_point = (v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1]))
    return dist2(p, closest_point)


def pairwise(iterable):
    """s -> (s0,s1), (s1,s2), (s2, s3), ...
    builtin in py3.10"""
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, itertools.chain(b, [iterable[0]]))

def pairwise_open(iterable):
    """s -> (s0,s1), (s1,s2), (s2, s3), ...
    builtin in py3.10"""
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def project_face(face: cq.Face, projection_dir=(0, 0, 1)) -> cq.Face:
    """
    Based on CQ SVG export function, thanks to adam-urbanczyk.
    """

    hlr = HLRBRep_Algo()
    hlr.Add(face.wrapped)

    projector = HLRAlgo_Projector(gp_Ax2(gp_Pnt(), gp_Dir(*projection_dir)))

    hlr.Projector(projector)
    hlr.ShowAll()
    hlr.Update()
    hlr.Hide()

    hlr_shapes = HLRBRep_HLRToShape(hlr)

    visible_shapes = []

    # VCompound contains the hard edges
    # Rg1LineVCompound contains soft edges
    # * Not interested
    # OutLineVCompound contains outline edges
    # * These are not edges on the original shape but edges created by the projection

    visible_edges = hlr_shapes.VCompound()
    if not visible_edges.IsNull():
        visible_shapes.append(visible_edges)

    visible_outlines = hlr_shapes.OutLineVCompound()
    if not visible_outlines.IsNull():
        visible_shapes.append(visible_outlines)

    # Fix the underlying geometry - otherwise we will get segfaults
    for el in visible_shapes:
        BRepLib.BuildCurves3d_s(el, TOLERANCE)

    # convert to native CQ objects
    visible_shapes = list(map(cq.Shape, visible_shapes))
    visible_wires = cq.Wire.combine(visible_shapes)

    # Calculate Area for each wire and use the biggest as the outer wire of the final result
    wires_with_area = [(cq.Face.makeFromWires(wire).Area(), wire) for wire in visible_wires]
    wires_with_area.sort(key=lambda v: v[0], reverse=True)

    wires = [wire for (_, wire) in wires_with_area]

    return cq.Face.makeFromWires(wires[0], wires[1:])
