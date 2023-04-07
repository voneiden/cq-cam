import cadquery as cq

from cq_cam.utils.offset import calculate_offset, offset_face


def test_calculate_offset():
    assert calculate_offset(1, 1) == 1
    assert calculate_offset(1, (1, 1)) == 2
    assert calculate_offset(1, None, 3) == 3
    assert calculate_offset(1, -1) == -1
    assert calculate_offset(1, (-1, 0.5)) == -0.5
    assert calculate_offset(1, (0, 1.05)) == 1.05
    assert calculate_offset(0.5, None, 3) == 1.5


def test_offset_face():
    rect1 = cq.Workplane().rect(10, 10).objects[0]
    rect2 = (
        cq.Workplane()
        .rect(
            5,
            5,
        )
        .objects[0]
    )
    # noinspection PyTypeChecker
    face = cq.Face.makeFromWires(rect1, [rect2])
    new_faces = offset_face(face, outer_offset=-1, inner_offset=1)
    assert len(new_faces) == 1
    new_face = new_faces[0]

    outer_points = set([vx.toTuple() for vx in new_face.outerWire().Vertices()])

    assert len(new_face.innerWires()) == 1
    inner_points = set([vx.toTuple() for vx in new_face.innerWires()[0].Vertices()])

    assert {
        (4.0, 4.0, 0.0),
        (-4.0, -4.0, 0.0),
        (4.0, -4.0, 0.0),
        (-4.0, 4.0, 0.0),
    } == outer_points
    assert {
        (-3.5, -2.5, 0.0),
        (-3.5, 2.5, 0.0),
        (-2.5, -3.5, 0.0),
        (-2.5, 3.5, 0.0),
        (2.5, -3.5, 0.0),
        (2.5, 3.5, 0.0),
        (3.5, -2.5, 0.0),
        (3.5, 2.5, 0.0),
    } == inner_points
