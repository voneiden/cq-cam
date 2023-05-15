import cadquery as cq
import pytest

from cq_cam import Job
from cq_cam.operations.pocket import apply_stepdown, determine_stepdown_start_depth
from cq_cam.utils.geometry_op import PathFace, offset_face
from cq_cam.utils.tests.conftest import round_array
from cq_cam.utils.utils import break_compound_to_faces

tool_change_label = "(Job - Tool Change)\n"
speed_change_label = "(Job - Speed Change)\n"


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


def test_make_from_wire_with_bigger_inner_than_outer():
    outer = cq.Workplane().rect(5, 5).objects[0]
    inner = cq.Workplane().rect(10, 10).objects[0]
    outer_face = cq.Face.makeFromWires(outer)
    inner_face = cq.Face.makeFromWires(inner)

    # with pytest.raises(ValueError):
    outer_compound = outer_face.cut(inner_face)
    assert outer_compound.Area() == 0
    compound_faces = break_compound_to_faces(outer_compound)
    assert len(compound_faces) == 0


def test_stepdown(job: Job, box):
    wp = box.faces(">Z").workplane().rect(2, 2).cutBlind(-1)
    face = wp.faces(">Z[1]")
    job = job.pocket(face, stepdown=0.5)
    assert job.operations[0].to_gcode() == (
        "(Job - Pocket)\n"
        "G0 Z10\n"
        "G0 X0.25 Y0.25 Z10\n"
        "G0 X0.25 Y0.25 Z1\n"
        "G1 X0.25 Y0.25 Z-0.5 F200\n"
        "G1 X-0.25 Y0.25 Z-0.5 F200\n"
        "G1 X-0.25 Y-0.25 Z-0.5 F200\n"
        "G1 X0.25 Y-0.25 Z-0.5 F200\n"
        "G1 X0.25 Y0.25 Z-0.5 F200\n"
        "G0 Z10\n"
        "G0 X0.25 Y0.25 Z10\n"
        "G0 X0.25 Y0.25 Z1\n"
        "G1 X0.25 Y0.25 Z-1 F200\n"
        "G1 X-0.25 Y0.25 Z-1 F200\n"
        "G1 X-0.25 Y-0.25 Z-1 F200\n"
        "G1 X0.25 Y-0.25 Z-1 F200\n"
        "G1 X0.25 Y0.25 Z-1 F200"
    )


def test_apply_stepdown_invalid_depths():
    with pytest.raises(RuntimeError):
        apply_stepdown([[PathFace([], [], -5), PathFace([], [], -6)]], None, 1)


def test_determine_stepdown_start_depth():
    upper_container = PathFace([(0, 0), (1, 0), (1, 1), (0, 1)], [], -3)
    upper_non_container = PathFace([(0, 0), (-1, 0), (-1, -1), (0, -1)], [], -3)
    bottom = PathFace([(0, 0), (1, 0), (1, 1), (0, 1)], [], -5)
    assert determine_stepdown_start_depth(bottom, []) is None
    assert determine_stepdown_start_depth(bottom, [upper_container]) == -3
    assert determine_stepdown_start_depth(bottom, [upper_non_container]) is None
