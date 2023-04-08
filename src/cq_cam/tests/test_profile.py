import cadquery as cq
import pytest

from cq_cam.fluent import Job
from cq_cam.utils.circle_bug_workaround import circle_bug_workaround


@pytest.fixture
def box():
    return cq.Workplane().rect(5, 5).extrude(1)


@pytest.fixture
def top_and_face(box):
    top_face = box.faces(">Z")
    top_plane = top_face.workplane().plane
    return top_plane, top_face.objects[0]


@pytest.fixture
def job(top_and_face):
    top, _ = top_and_face
    return Job(top, 200, 1.5)


def test_circle_bug_workaround():
    wp = cq.Workplane("XY")
    wp = wp.box(1, 1, 1)
    wp = wp.faces(">Z")
    c1_wp = wp.circle(0.75)
    c2_wp = wp.offset2D(0.1)

    # noinspection PyTypeChecker
    circle_bug_workaround(c1_wp.objects[0], c2_wp.objects)
    assert (
        c1_wp.objects[0].location().toTuple() != c2_wp.objects[0].location().toTuple()
    )


def test_profile_square_outside(job: Job, top_and_face):
    _, face = top_and_face
    code = job.profile(face).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90\n"
        "G21\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X-2.5Y-3.25\n"
        "Z1\n"
        "G1Z0\n"
        "X2.5\n"
        "G3X3.25Y-2.5I0J0.75K0\n"
        "G1Y2.5\n"
        "G3X2.5Y3.25I-0.75J0K0\n"
        "G1X-2.5\n"
        "G3X-3.25Y2.5I0J-0.75K0\n"
        "G1Y-2.5\n"
        "G3X-2.5Y-3.25I0.75J0K0G1Z0\n"
        "G0Z10\n"
        "X0Y0"
    )


def test_profile_square_midline(job: Job, top_and_face):
    _, face = top_and_face
    code = job.profile(face, outer_offset=0).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90\n"
        "G21\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X-2.5Y-2.5\n"
        "Z1\n"
        "G1Z0\n"
        "X2.5\n"
        "Y2.5\n"
        "X-2.5\n"
        "Y-2.5G1Z0\n"
        "G0Z10\n"
        "X0Y0"
    )


def test_profile_square_inside(job: Job, top_and_face):
    _, face = top_and_face
    code = job.profile(face, outer_offset=-1).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90\n"
        "G21\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X-1.75Y-1.75\n"
        "Z1\n"
        "G1Z0\n"
        "X1.75\n"
        "Y1.75\n"
        "X-1.75\n"
        "Y-1.75G1Z0\n"
        "G0Z10\n"
        "X0Y0"
    )
