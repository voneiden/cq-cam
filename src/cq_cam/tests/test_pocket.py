import cadquery as cq

from cq_cam.operations.pocket import offset_face
from cq_cam.utils.tests.conftest import round_array


def test_offset_face():
    wp = (
        cq.Workplane()
        .box(10, 10, 10)
        .faces(">Z")
        .workplane()
        .rect(2, 2)
        .cutThruAll()
        .faces(">Z")
    )
    # noinspection PyTypeChecker
    faces = offset_face(wp.objects[0], -1, 2)
    assert len(faces) == 1
    face = faces[0]
    assert len(face.innerWires()) == 1

    expected_outer_vx = {
        (-4.0, -4.0, 5.0),
        (4.0, -4.0, 5.0),
        (-4.0, 4.0, 5.0),
        (4.0, 4.0, 5.0),
    }
    outer_vx = set([vx.toTuple() for vx in face.outerWire().Vertices()])
    assert outer_vx == expected_outer_vx

    expected_inner_vx = {
        (-3.0, 1.0, 5.0),
        (-3.0, -1.0, 5.0),
        (1.0, 3.0, 5.0),
        (1.0, -3.0, 5.0),
        (3.0, -1.0, 5.0),
        (-1.0, -3.0, 5.0),
        (3.0, 1.0, 5.0),
        (-1.0, 3.0, 5.0),
    }
    inner_vx = round_array([vx.toTuple() for vx in face.innerWires()[0].Vertices()])
    assert inner_vx == expected_inner_vx
