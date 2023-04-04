from math import isclose
from typing import TYPE_CHECKING, List

import cadquery as cq
import numpy as np

from cq_cam.command import AbsoluteCV, CircularCCW, CircularCW, Cut, Plunge, Rapid
from cq_cam.utils.utils import (
    edge_end_point,
    edge_start_point,
    is_arc_clockwise2,
    wire_to_ordered_edges,
)

if TYPE_CHECKING:
    from cq_cam.fluent import Job


def vertical_plunge(job: "Job", layer1: cq.Wire, layer2: cq.Wire, p=0.0):
    p1 = layer1.positionAt(p, "parameter")
    p2 = layer2.positionAt(p, "parameter")

    assert p1.z > p2.z

    if p1.x == p2.x and p1.y == p1.y:
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


def rapid_to(v: cq.Vector, rapid_height: float, safe_plunge_height=None):
    commands = [Rapid.abs(z=rapid_height), Rapid.abs(x=v.x, y=v.y, arrow=True)]

    if safe_plunge_height is None:
        commands.append(Plunge.abs(z=v.z, arrow=True))
    else:
        commands.append(Rapid.abs(z=safe_plunge_height, arrow=True))
        if safe_plunge_height > v.z:
            commands.append(Plunge.abs(z=v.z, arrow=True))
    return commands


def vertical_edge(edge: cq.Edge):
    p1 = edge.startPoint()
    p2 = edge.endPoint()
    return p1.x == p2.x and p1.y == p2.y


def route(job: "Job", wires: List[cq.Wire]):
    commands = []
    previous_wire = None
    previous_wire_start = None
    ep = None
    for wire in wires:
        edges = wire_to_ordered_edges(wire)
        if not edges:
            return []
        start = edge_start_point(edges[0])

        # Determine how to access the wire
        # Direct plunge option
        if (
            ep
            and isclose(ep.x, start.x, abs_tol=0.001)
            and isclose(ep.y, start.y, abs_tol=0.001)
        ):
            commands.append(Plunge.abs(start.z, arrow=True))
        else:
            commands += rapid_to(start, job.rapid_height, job.op_safe_height)
        for edge_i, edge in enumerate(edges):
            geom_type = edge.geomType()
            ep = edge_end_point(edge)
            end_cv = AbsoluteCV.from_vector(ep)
            if geom_type == "LINE":
                commands.append(Cut(end_cv, arrow=edge_i % 5 == 0))

            elif geom_type == "ARC" or geom_type == "CIRCLE":
                sp = edge_start_point(edge)
                center = AbsoluteCV.from_vector(edge.arcCenter())
                cmd = CircularCW if is_arc_clockwise2(edge) else CircularCCW
                # Let's break circles into half
                if sp == ep:
                    mid1 = AbsoluteCV.from_vector(edge.positionAt(0.25))
                    end1 = AbsoluteCV.from_vector(edge.positionAt(0.5))
                    commands.append(cmd(end=end1, center=center, mid=mid1))
                    mid2 = AbsoluteCV.from_vector(edge.positionAt(0.75))
                    commands.append(cmd(end=end_cv, center=center, mid=mid2))
                else:
                    mid = AbsoluteCV.from_vector(edge.positionAt(0.5))
                    commands.append(cmd(end=end_cv, center=center, mid=mid))

            elif geom_type == "SPLINE":
                # TODO?
                raise RuntimeError("Unsupported geom type: SPLINE")

            else:
                raise RuntimeError(f"Unsupported geom type: {geom_type}")

        previous_wire = wire
        previous_wire_start = start
    return commands
