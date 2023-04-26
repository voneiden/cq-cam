import itertools
import math
from functools import cache
from typing import Iterable, List, T, Tuple, Union

import numpy as np
import pyclipper
from cadquery import Edge, cq
from cadquery.occ_impl.shapes import TOLERANCE
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_ShapeEnum
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape


def edge_end_point(edge: cq.Edge, precision=3) -> cq.Vector:
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.startPoint()
    return edge.endPoint()


def edge_start_point(edge: cq.Edge, precision=3) -> cq.Vector:
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.endPoint()
    return edge.startPoint()


def edge_start_param(edge: cq.Edge) -> float:
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.paramAt(1)
    return edge.paramAt(0)


def edge_end_param(edge: cq.Edge) -> float:
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.paramAt(0)
    return edge.paramAt(1)


def edge_oriented_param(edge: cq.Edge, p1, p2):
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return (1 - p2), (1 - p1), True
    return p1, p2, False


def edge_start_end(edge: cq.Edge) -> Tuple[cq.Vector, cq.Vector]:
    # https://github.com/CadQuery/cadquery/issues/831
    orientation = edge.wrapped.Orientation()
    if orientation == TopAbs_REVERSED:
        return edge.endPoint(), edge.startPoint()
    return edge.startPoint(), edge.endPoint()


def drop_z(vector: cq.Vector) -> Tuple[float, float]:
    return vector.x, vector.y


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
            vxs.append(edge_end_point(edge))
            # TODO tmp
        elif edge.geomType() in ["ARC", "CIRCLE", "OFFSET"]:
            # TODO handle full circles
            # TODO make sure position_space ends up returning something (really tiny arcs?)
            positions = edge.positions(position_space(edge)[1:])
            vxs += positions
        else:
            print("UNKNOWN TYPE", edge.geomType())
            raise ValueError(f"UNKNOWN TYPE {edge.geomType()}")
    return vxs


def flatten_wire(wire: cq.Wire) -> List[cq.Vector]:
    return flatten_edges(wire_to_ordered_edges(wire))


def is_arc_clockwise(start: cq.Vector, mid: cq.Vector, end: cq.Vector):
    # https://stackoverflow.com/questions/33960924/is-arc-clockwise-or-counter-clockwise
    if start.z != mid.z:
        raise NotImplementedError(
            "Helical arcs not supported yet", start.z, mid.z, end.z
        )

    # start -> end
    se = (end.x - start.x, end.y - start.y)

    # start -> mid
    sm = (mid.x - start.x, mid.y - start.y)

    # cross product
    cp = se[0] * sm[1] - se[1] * sm[0]

    # Arc is clockwise if cross product is positive
    return cp > 0


def is_arc_clockwise2(arc: cq.Edge):
    normal = arc.normal()
    # TODO support other axes besides Z?
    if normal.z == 0:
        raise RuntimeError("Only Z axis arcs are supported")
    if arc.wrapped.Orientation() == TopAbs_REVERSED:
        return normal.z > 0
    return normal.z < 0


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
    TODO This code is not used currently, but is left here
    TODO as a reference.
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


def flatten_list(lst: Iterable[Iterable[T]]) -> List[T]:
    return [element for nested_lst in lst for element in nested_lst]


class WireClipper:
    def __init__(self):
        self._clipper = pyclipper.Pyclipper()
        self._pt_clip_cache = []

    def add_clip_wire(self, wire: cq.Wire, cache=True):
        return self._add_wire(
            wire, pyclipper.PT_CLIP, cache=cache, is_closed=wire.IsClosed()
        )

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

    def add_clip_polygon(
        self, polygon: Iterable[Tuple[float, float]], is_closed=False, cache=False
    ):
        return self._add_polygon(polygon, pyclipper.PT_CLIP, is_closed, cache)

    def add_subject_polygon(
        self, polygon: Iterable[Tuple[float, float]], is_closed=False
    ):
        return self._add_polygon(polygon, pyclipper.PT_SUBJECT, is_closed)

    def _add_polygon(
        self, polygon: Iterable[Tuple[float, float]], pt, is_closed=False, cache=False
    ):
        self._add_path(pyclipper.scale_to_clipper(polygon), pt, is_closed, cache)

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
            "left": pyclipper.scale_from_clipper(bounds.left),
            "top": pyclipper.scale_from_clipper(bounds.top),
            "right": pyclipper.scale_from_clipper(bounds.right),
            "bottom": pyclipper.scale_from_clipper(bounds.bottom),
        }

    def max_bounds(self):
        bounds = self.bounds()
        diagonal_length = self._bounds_diagonal_length(bounds)
        half_diagonal = diagonal_length / 2
        x_center = (bounds["left"] + bounds["right"]) / 2
        y_center = (bounds["top"] + bounds["bottom"]) / 2

        return {
            "left": math.floor(x_center - half_diagonal),
            "top": math.ceil(y_center + half_diagonal),
            "right": math.ceil(x_center + half_diagonal),
            "bottom": math.floor(y_center - half_diagonal),
        }

    def _bounds_diagonal_length(self, bounds=None):
        bounds = bounds if bounds else self.bounds()
        top_left = cq.Vector(bounds["left"], bounds["top"])
        bottom_right = cq.Vector(bounds["right"], bounds["bottom"])
        diagonal = top_left.sub(bottom_right)
        return diagonal.Length

    def execute(
        self, clip_type=pyclipper.CT_INTERSECTION
    ) -> Tuple[Tuple[Tuple[float, float], Tuple[float, float]]]:
        # TODO detect if there's nothing to do?
        polytree = self._clipper.Execute2(clip_type)
        # TODO option to return nested structure?
        open_paths = pyclipper.scale_from_clipper(
            pyclipper.OpenPathsFromPolyTree(polytree)
        )
        closed_paths = pyclipper.scale_from_clipper(
            pyclipper.ClosedPathsFromPolyTree(polytree)
        )
        # noinspection PyTypeChecker
        for closed_path in closed_paths:
            closed_path.append(closed_path[0])

        return tuple(
            tuple(tuple(point) for point in path)
            for path in (closed_paths + open_paths)
        )

    def execute_difference(self):
        return self.execute(pyclipper.CT_DIFFERENCE)


def dist2(v: Tuple[float, float], w: Tuple[float, float]) -> float:
    return (v[0] - w[0]) ** 2 + (v[1] - w[1]) ** 2


@cache
def cached_dist2(p1: Tuple[float, float], p2: Tuple[float, float]):
    return dist2(p1, p2)


def dist_to_segment_squared(
    point: Tuple[float, float],
    segment_start: Tuple[float, float],
    segment_end: Tuple[float, float],
) -> Tuple[float, Tuple[float, float]]:
    p, v, w = point, segment_start, segment_end
    """https://stackoverflow.com/a/1501725"""
    l2 = dist2(v, w)
    t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2
    t = max(0.0, min(1.0, t))
    closest_point = (v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1]))
    return dist2(p, closest_point), closest_point


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
    wires_with_area = [
        (cq.Face.makeFromWires(wire).Area(), wire) for wire in visible_wires
    ]
    wires_with_area.sort(key=lambda v: v[0], reverse=True)

    wires = [wire for (_, wire) in wires_with_area]

    return cq.Face.makeFromWires(wires[0], wires[1:])


def extract_wires(
    shape: Union[cq.Workplane, cq.Shape, List[cq.Shape]]
) -> Tuple[List[cq.Wire], List[cq.Wire]]:
    if isinstance(shape, cq.Workplane):
        return extract_wires(shape.objects)

    if isinstance(shape, cq.Shape):
        shape_objs = [shape]
    else:
        shape_objs = [shape_obj for shape_obj in shape]

    outers = []
    inners = []

    for shape_obj in shape_objs:
        if isinstance(shape_obj, cq.Wire):
            outers.append(shape_obj)
        elif isinstance(shape_obj, cq.Face):
            outers.append(shape_obj.outerWire())
            inners += shape_obj.innerWires()
        else:
            raise ValueError(f"Unsupported shape {type(shape_obj)}")

    return outers, inners


def compound_to_edges(compound: cq.Compound):
    """
    Break a compound into a list of edges.

    :param compound:
    :return:
    """
    edges = []
    explorer = TopExp_Explorer(compound.wrapped, TopAbs_EDGE)
    while explorer.More():
        edge = explorer.Current()
        edges.append(cq.Edge(edge))
        explorer.Next()
    return edges


def optimize_float(v: float):
    """Drop trailing zeroes from a float that is exactly an int to save some bytes in the gcode"""
    iv = int(v)
    return iv if v == iv else v


def break_compound_to(
    compound: cq.Compound, shape_type: TopAbs_ShapeEnum
) -> List[TopoDS_Shape]:
    shapes = []
    explorer = TopExp_Explorer(compound.wrapped, shape_type)
    while explorer.More():
        shape = explorer.Current()
        shapes.append(shape)
        explorer.Next()
    return shapes


def break_compound_to_faces(compound: cq.Compound):
    return [cq.Face(shape) for shape in break_compound_to(compound, TopAbs_FACE)]
