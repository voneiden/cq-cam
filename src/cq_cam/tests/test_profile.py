import cadquery as cq

from cq_cam.fluent import Job
from cq_cam.operations.tabs import EdgeTabs, WireTabs
from cq_cam.utils.circle_bug_workaround import circle_bug_workaround


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


def test_profile_square_outside(job: Job, top_face):
    code = job.profile(top_face).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X-2.5Y-3.25\n"
        "Z1\n"
        "G1Z0F200\n"
        "X2.5\n"
        "G3X3.25Y-2.5I0J0.75K0\n"
        "G1Y2.5\n"
        "G3X2.5Y3.25I-0.75J0K0\n"
        "G1X-2.5\n"
        "G3X-3.25Y2.5I0J-0.75K0\n"
        "G1Y-2.5\n"
        "G3X-2.5Y-3.25I0.75J0K0\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_square_midline(job: Job, top_face):
    code = job.profile(top_face, outer_offset=0).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X-2.5Y-2.5\n"
        "Z1\n"
        "G1Z0F200\n"
        "X2.5\n"
        "Y2.5\n"
        "X-2.5\n"
        "Y-2.5\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_square_inside(job: Job, top_face):
    code = job.profile(top_face, outer_offset=-1).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X-1.75Y-1.75\n"
        "Z1\n"
        "G1Z0F200\n"
        "X1.75\n"
        "Y1.75\n"
        "X-1.75\n"
        "Y-1.75\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_stepdown(job: Job, bottom_face):
    code = job.profile(bottom_face, outer_offset=0, stepdown=1).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X2.5Y-2.5\n"
        "Z1\n"
        "G1Z-1F200\n"
        "X-2.5\n"
        "Y2.5\n"
        "X2.5\n"
        "Y-2.5\n"
        "G0Z10\n"
        "Z1\n"
        "G1Z-2\n"
        "X-2.5\n"
        "Y2.5\n"
        "X2.5\n"
        "Y-2.5\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_tabs(job, bottom_face):
    code = job.profile(
        bottom_face, outer_offset=0, tabs=EdgeTabs(spacing=2, width=1, height=1)
    ).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X2.5Y-2.5\n"
        "Z1\n"
        "G1Z-2F200\n"
        "X1.333\n"
        "Z-1\n"
        "X0.333\n"
        "Z-2\n"
        "X-0.333\n"
        "Z-1\n"
        "X-1.333\n"
        "Z-2\n"
        "X-2.5\n"
        "Y-1.333\n"
        "Z-1\n"
        "Y-0.333\n"
        "Z-2\n"
        "Y0.333\n"
        "Z-1\n"
        "Y1.333\n"
        "Z-2\n"
        "Y2.5\n"
        "X-1.333\n"
        "Z-1\n"
        "X-0.333\n"
        "Z-2\n"
        "X0.333\n"
        "Z-1\n"
        "X1.333\n"
        "Z-2\n"
        "X2.5\n"
        "Y1.333\n"
        "Z-1\n"
        "Y0.333\n"
        "Z-2\n"
        "Y-0.333\n"
        "Z-1\n"
        "Y-1.333\n"
        "Z-2\n"
        "Y-2.5\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_wire_tabs(job, bottom_face):
    code = job.profile(
        bottom_face, outer_offset=0, tabs=WireTabs(count=2, width=1, height=1)
    ).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0Z10\n"
        "X2.5Y-2.5\n"
        "Z1\n"
        "G1Z-1F200\n"
        "X2\n"
        "Z-2\n"
        "X-2.5\n"
        "Y2\n"
        "Z-1\n"
        "Y2.5\n"
        "X-2\n"
        "Z-2\n"
        "X2.5\n"
        "Y-2\n"
        "Z-1\n"
        "Y-2.5\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )
