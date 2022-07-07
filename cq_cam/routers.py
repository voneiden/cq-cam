from typing import TYPE_CHECKING, List

import cadquery as cq
import numpy as np

from cq_cam.command import Plunge, Rapid, ReferencePosition, AbsoluteCV, Cut, CircularCW, CircularCCW, Retract
from cq_cam.utils.utils import wire_to_ordered_edges, end_point, start_point, is_arc_clockwise2

if TYPE_CHECKING:
    from cq_cam.fluent import JobV2


def vertical_plunge(job: 'JobV2', layer1: cq.Wire, layer2: cq.Wire, p=0.0):
    p1 = layer1.positionAt(p, 'parameter')
    p2 = layer2.positionAt(p, 'parameter')

    assert p1.z > p2.z

    if p1.x == p2.x and p1.y == p1.y:
        return [Plunge(p1.z - p2.z)]

    p3 = layer2.positionAt(0)
    return [Rapid(z=job.op_safe_height), Rapid(x=p3.x, y=p3.y), Plunge(p3.z)]


def vertical_ramp(job: 'JobV2', layer1: cq.Wire, layer2: cq.Wire, p=0.0, ramp_angle=10, reverse=False):
    p1 = layer1.positionAt(p, 'parameter')
    p2 = layer2.positionAt(p, 'parameter')

    assert p1.z > p2.z
    if p1.x != p2.x or p1.y != p2.y:
        return vertical_plunge(job, layer1, layer2, p)

    depth = p1.z - p2.z
    distance = depth / np.tan(np.deg2rad(ramp_angle))
    p2 = distance / layer1.Length()


def rapid_to(v: cq.Vector, rapid_height: float, safe_plunge_height=None):
    commands = [
        Rapid.abs(z=rapid_height),
        Rapid.abs(x=v.x, y=v.y)
    ]


    if safe_plunge_height is None:
        commands.append(Plunge.abs(z=v.z))
    else:
        commands.append(Rapid.abs(z=safe_plunge_height))
        if safe_plunge_height > v.z:
            commands.append(Plunge.abs(z=v.z))
    return commands


def route(job, wires: List[cq.Wire]):
    commands = []
    for wire in wires:
        edges = wire_to_ordered_edges(wire)
        if not edges:
            return []
        start = start_point(edges[0])
        # TODO ways to move to next wire
        # 1) Direct plunge or ramp (next wire is straight below)
        # 2) Rapid retract
        commands += rapid_to(start, job.rapid_height, job.op_safe_height)
        for edge in edges:
            geom_type = edge.geomType()
            end = AbsoluteCV.from_vector(end_point(edge))
            if geom_type == 'LINE':
                commands.append(Cut(end))

            elif geom_type == 'ARC' or geom_type == 'CIRCLE':
                center = AbsoluteCV.from_vector(edge.arcCenter())
                mid = AbsoluteCV.from_vector(edge.positionAt(0.5))
                if is_arc_clockwise2(edge):
                    commands.append(CircularCW(end=end, center=center, mid=mid))
                else:
                    commands.append(CircularCCW(end=end, center=center, mid=mid))

            elif geom_type == 'SPLINE':
                # TODO?
                raise RuntimeError('Unsupported geom type: SPLINE')

            else:
                raise RuntimeError(f'Unsupported geom type: {geom_type}')

    return commands