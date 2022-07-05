import cadquery as cq
import numpy as np

from cq_cam.commands.command import Plunge, Rapid
from cq_cam.fluent import JobV2


def vertical_plunge(job: JobV2, layer1: cq.Wire, layer2: cq.Wire, p=0.0):
    p1 = layer1.positionAt(p, 'parameter')
    p2 = layer2.positionAt(p, 'parameter')

    assert p1.z > p2.z

    if p1.x == p2.x and p1.y == p1.y:
        return [Plunge(p1.z - p2.z)]

    p3 = layer2.positionAt(0)
    return [Rapid(z=job.op_safe_height), Rapid(x=p3.x, y=p3.y), Plunge(p3.z)]


def vertical_ramp(job: JobV2, layer1: cq.Wire, layer2: cq.Wire, p=0.0, ramp_angle=10, reverse=False):
    p1 = layer1.positionAt(p, 'parameter')
    p2 = layer2.positionAt(p, 'parameter')

    assert p1.z > p2.z
    if p1.x != p2.x or p1.y != p2.y:
        return vertical_plunge(job, layer1, layer2, p)

    depth = p1.z - p2.z
    distance = depth / np.tan(np.deg2rad(ramp_angle))
    p2 = distance / layer1.Length()
