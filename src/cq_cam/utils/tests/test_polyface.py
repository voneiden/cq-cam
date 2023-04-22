from typing import List

import cadquery as cq

from cq_cam.utils.geometry_op import make_polyfaces, wire_to_polygon
from cq_cam.utils.tests.conftest import shift_polygon


def test_make_polyface():
    wp = (
        cq.Workplane()
        .rect(10, 10)
        .extrude(2)
        .faces(">Z")
        .workplane()
        .pushPoints([(-2, 0), (2, 0)])
        .rect(1, 1)
        .cutBlind(-1)
        .pushPoints([(0, 0)])
        .rect(1, 10)
        .cutBlind(-1)
        .faces(">Z")
    )

    outers = []
    inners = []
    faces: List[cq.Face] = wp.objects
    for face in faces:
        outers.append(wire_to_polygon(face.outerWire()))
        inners += [wire_to_polygon(inner) for inner in face.innerWires()]

    polyfaces = make_polyfaces(outers, inners, 0)
    assert len(polyfaces) == 2

    # The order changes a bit during processing with clipper
    assert polyfaces[0].outer == shift_polygon(outers[0], 3)
    assert polyfaces[0].inners[0] == shift_polygon(inners[0], 1)
    assert polyfaces[1].outer == shift_polygon(outers[1], 3)
    assert polyfaces[1].inners[0] == shift_polygon(inners[1], 1)
