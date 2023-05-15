from math import isclose
from typing import TYPE_CHECKING, Union

import cadquery as cq
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape, BRepExtrema_SupportType
from OCP.TopAbs import TopAbs_REVERSED

from cq_cam.command import (
    CircularCCW,
    CircularCW,
    CommandVector,
    Cut,
    MotionCommand,
    PlungeCut,
    PlungeRapid,
    Rapid,
    Retract,
)
from cq_cam.utils.geometry_op import Path, PathFace, distance_to_path
from cq_cam.utils.interpolation import edge_interpolation_count
from cq_cam.utils.utils import (
    edge_end_param,
    edge_end_point,
    edge_start_param,
    edge_start_point,
    is_arc_clockwise2,
    wire_to_ordered_edges,
)

if TYPE_CHECKING:
    from cq_cam.fluent import Job

"""
def vertical_plunge(job: "Job", layer1: cq.Wire, layer2: cq.Wire, p=0.0):
    p1 = layer1.positionAt(p, "parameter")
    p2 = layer2.positionAt(p, "parameter")

    assert p1.z > p2.z

    if p1.x == p2.x and p1.y == p2.y:
        return [Plunge(p1.z - p2.z)]

    p3 = layer2.positionAt(0)
    return [Rapid(z=job.op_safe_height), Rapid(x=p3.x, y=p3.y), Plunge(p3.z)]


def vertical_ramp(
    job: "Job", layer1: cq.Wire, layer2: cq.Wire, p=0.0, ramp_angle=10, reverse=False
):
    p1 = layer1.positionAt(p, "parameter")
    p2 = layer2.positionAt(p, "parameter")

    assert p1.z > p2.z
    if p1.x != p2.x or p1.y != p2.y:
        return vertical_plunge(job, layer1, layer2, p)

    depth = p1.z - p2.z
    distance = depth / np.tan(np.deg2rad(ramp_angle))
    p2 = distance / layer1.Length()
"""


def rapid_to(
    start: CommandVector,
    end: cq.Vector,
    rapid_height: float,
    safe_plunge_height=None,
    plunge_feed: float | None = None,
):
    commands = [Retract.abs(z=rapid_height, start=start)]
    start.z = rapid_height
    commands.append(Rapid.abs(x=end.x, y=end.y, start=start, arrow=True))
    start.x = end.x
    start.y = end.y
    if safe_plunge_height is None:
        commands.append(
            PlungeCut.abs(z=end.z, start=start, arrow=True, feed=plunge_feed)
        )
    else:
        commands.append(PlungeRapid.abs(z=safe_plunge_height, start=start, arrow=True))
        start.z = safe_plunge_height
        if safe_plunge_height > end.z:
            commands.append(
                PlungeCut.abs(z=end.z, start=start, arrow=True, feed=plunge_feed)
            )
    return commands


def vertical_edge(edge: cq.Edge):
    p1 = edge.startPoint()
    p2 = edge.endPoint()
    return p1.x == p2.x and p1.y == p2.y


def distance_to_wire(v: cq.Vector, wire2: cq.Wire):
    vx = cq.Vertex.makeVertex(*v.toTuple())
    extrema = BRepExtrema_DistShapeShape(vx.wrapped, wire2.wrapped)
    extrema.Perform()
    if extrema.IsDone():
        distance = extrema.Value()
        if extrema.SupportTypeShape2(1) == BRepExtrema_SupportType.BRepExtrema_IsVertex:
            vertex = cq.Vertex(extrema.SupportOnShape2(1))
            return distance, vertex, None
        edge = cq.Edge(extrema.SupportOnShape2(1))
        param = extrema.ParOnEdgeS2(1)[0]
        return distance, edge, param

    return None, None, None


def shift_edges(
    edges: list[cq.Edge], target: Union[cq.Edge, cq.Vertex]
) -> list[cq.Edge]:
    if isinstance(target, cq.Edge):
        i = edges.index(target)
        return edges[i:] + edges[:i]
    else:
        target = cq.Vector(target.toTuple())
        for i, edge in enumerate(edges):
            if edge_start_point(edge) == target:
                return edges[i:] + edges[:i]

    raise ValueError("Failed to shift")


def route_edge(
    edge: cq.Edge,
    precision: int,
    start_p=None,
    end_p=None,
    arrow=False,
    feed: float | None = None,
) -> tuple[list[MotionCommand], cq.Vector]:
    commands = []
    geom_type = edge.geomType()
    sp = (
        edge_start_point(edge)
        if start_p is None
        else edge.positionAt(start_p, "parameter")
    )
    ep = edge_end_point(edge) if end_p is None else edge.positionAt(end_p, "parameter")

    start_cv = CommandVector.from_vector(sp)
    end_cv = CommandVector.from_vector(ep)
    if geom_type == "LINE":
        # commands.append(Cut(end_cv, arrow=edge_i % 5 == 0))
        commands.append(Cut(end_cv, start_cv, arrow=arrow, feed=feed))

    elif geom_type == "ARC" or geom_type == "CIRCLE":
        if start_p is None:
            start_p = edge_start_param(edge)
        if end_p is None:
            end_p = edge_end_param(edge)

        center = CommandVector.from_vector(edge.arcCenter())
        cmd = CircularCW if is_arc_clockwise2(edge) else CircularCCW

        # Actual circles are closed
        # CIRCLE geom type does not guarantee that the edge is a full circle!
        # TODO ARC's are not necessarily circular, so the gcode representation can be wrong!
        if edge.Closed():
            mid1_p, end1_p, mid2_p = np.linspace(start_p, end_p, 5)[1:-1]
            mid1 = CommandVector.from_vector(edge.positionAt(mid1_p, "parameter"))
            end1 = CommandVector.from_vector(edge.positionAt(end1_p, "parameter"))
            start_cv = CommandVector.from_vector(edge.positionAt(start_p, "parameter"))
            commands.append(
                cmd(
                    end=end1,
                    center=center,
                    mid=mid1,
                    start=start_cv,
                    arrow=arrow,
                    feed=feed,
                )
            )
            start_cv = end1
            mid2 = CommandVector.from_vector(edge.positionAt(mid2_p, "parameter"))
            commands.append(
                cmd(
                    end=end_cv,
                    center=center,
                    mid=mid2,
                    start=start_cv,
                    arrow=arrow,
                    feed=feed,
                )
            )
        elif sp == ep:
            # Really tiny arc, might as well just cut it straight
            # TODO verify the sanity
            commands.append(Cut(end=end_cv, start=start_cv, arrow=arrow, feed=feed))
        else:
            mid_p = np.linspace(start_p, end_p, 3)[1]
            mid = CommandVector.from_vector(edge.positionAt(mid_p, "parameter"))
            commands.append(
                cmd(
                    end=end_cv,
                    center=center,
                    mid=mid,
                    start=start_cv,
                    arrow=arrow,
                    feed=feed,
                )
            )

    elif geom_type == "BSPLINE" or geom_type == "SPLINE" or geom_type == "OFFSET":
        n = edge_interpolation_count(edge, precision)

        orientation = edge.wrapped.Orientation()
        if orientation == TopAbs_REVERSED:
            i, j = 1, 0
        else:
            i, j = 0, 1

        for length in np.linspace(i, j, n):
            # [e._geomAdaptor().Curve().Curve().BasisCurve().BasisCurve() for e in pocket.DEBUG[0].Edges()]
            end_cv_int = CommandVector.from_vector(edge.positionAt(length))
            commands.append(
                Cut(
                    end_cv_int,
                    start_cv,
                    arrow=arrow,
                    feed=feed,
                )
            )
            start_cv = end_cv_int

    else:
        raise RuntimeError(f"Unsupported geom type: {geom_type}")

    return commands, ep


def route_wires(job: "Job", wires: list[Union[cq.Wire, cq.Edge]], stepover=None):
    commands = []
    previous_wire_end = None
    start_cv = CommandVector()

    for wire in wires:
        # Convert wires to edges
        if isinstance(wire, cq.Edge):
            edges = [wire]
        else:
            edges = wire_to_ordered_edges(wire)
        if not edges:
            return []
        # Set start and end point of wire
        start = edge_start_point(edges[0])
        end = edge_end_point(edges[-1])

        # Determine how to access the wire
        # Direct plunge option
        if previous_wire_end:
            distance, target, param = distance_to_wire(previous_wire_end, wire)
        else:
            distance, target, param = None, None, None

        if stepover and distance and distance <= stepover:
            # Determine the index of the edge, shift edges and use param?
            edges = shift_edges(edges, target)
            start = edge_start_point(edges[0])
            if param:
                start = edges[0].positionAt(param, "parameter")
            commands.append(
                Cut(CommandVector.from_vector(start), start_cv, feed=job.feed)
            )
        else:
            # Create simple transition between toolpaths
            # TODO Implement Ramping (Horizontal/Vertical)
            # TODO Implement orientation (Climb/Conventional)
            commands += rapid_to(
                start_cv,
                start,
                job.rapid_height,
                job.op_safe_height,
                plunge_feed=job.plunge_feed,
            )

        for edge_i, edge in enumerate(edges):
            if edge_i == 0:
                if param is not None:
                    new_commands, end = route_edge(
                        edge, job.precision, start_p=param, arrow=True, feed=job.feed
                    )
                    commands += new_commands
                else:
                    new_commands, end = route_edge(
                        edge, job.precision, arrow=True, feed=job.feed
                    )
                    commands += new_commands
            else:
                new_commands, end = route_edge(edge, job.precision, feed=job.feed)
                commands += new_commands
        if param:
            new_commands, end = route_edge(
                edges[0], job.precision, end_p=param, feed=job.feed
            )
            commands += new_commands

        previous_wire_end = end
        start_cv.x = end.x
        start_cv.y = end.y
        start_cv.z = end.z
    return commands


def shift_polygon(polygon: Path, i: int):
    polygon = polygon[:-1]
    polygon = polygon[i:] + polygon[: i + 1]
    return polygon


def route_polyface_outers(job: "Job", polyfaces: list[PathFace], stepover=None):
    commands = []
    previous_wire_end = None
    start_cv = CommandVector()

    for polyface in polyfaces:
        poly = polyface.outer
        start = cq.Vector(*poly[0], polyface.depth)

        # Determine how to access the wire
        # Direct plunge option
        if previous_wire_end:
            distance, closest_point, poly_position = distance_to_path(
                previous_wire_end, poly
            )
        else:
            distance, closest_point, poly_position = None, None, None

        # Same comment applies here as in the same section in route_wires
        if stepover and distance and distance <= stepover:
            # Determine the index of the edge, shift edges and use param?
            # edges = shift_edges(edges, target)
            index = poly_position[0]
            poly = shift_polygon(poly, index)
            start = closest_point
            commands.append(Cut.abs(*start, polyface.depth, start_cv, feed=job.feed))
            pass
        else:
            # Create simple transition between toolpaths
            commands += rapid_to(
                start_cv, start, job.rapid_height, job.op_safe_height, job.plunge_feed
            )

        for x, y in poly[1:]:
            commands.append(
                Cut.abs(x, y, polyface.depth, start=start_cv, feed=job.feed)
            )
            start_cv.x = x
            start_cv.y = y

        if closest_point:
            commands.append(
                Cut.abs(*closest_point, polyface.depth, start_cv, feed=job.feed)
            )
            previous_wire_end = closest_point
        else:
            previous_wire_end = poly[-1]

        start_cv.x = previous_wire_end[0]
        start_cv.y = previous_wire_end[1]
        start_cv.z = polyface.depth

    return commands
