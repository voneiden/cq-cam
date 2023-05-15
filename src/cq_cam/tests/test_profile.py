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
        "G0 Z10\n"
        "G0 X-2.5 Y-3.25\n"
        "G0 Z1\n"
        "G1 Z0 F200\n"
        "G1 X2.5 Y-3.25 Z0 F200\n"
        "G3 X3.25 Y-2.5 Z0 I0 J0.75 K0 F200\n"
        "G1 X3.25 Y2.5 Z0 F200\n"
        "G3 X2.5 Y3.25 Z0 I-0.75 J0 K0 F200\n"
        "G1 X-2.5 Y3.25 Z0 F200\n"
        "G3 X-3.25 Y2.5 Z0 I0 J-0.75 K0 F200\n"
        "G1 X-3.25 Y-2.5 Z0 F200\n"
        "G3 X-2.5 Y-3.25 Z0 I0.75 J0 K0 F200\n"
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
        "G0 Z10\n"
        "G0 X-2.5 Y-2.5\n"
        "G0 Z1\n"
        "G1 Z0 F200\n"
        "G1 X2.5 Y-2.5 Z0 F200\n"
        "G1 X2.5 Y2.5 Z0 F200\n"
        "G1 X-2.5 Y2.5 Z0 F200\n"
        "G1 X-2.5 Y-2.5 Z0 F200\n"
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
        "G0 Z10\n"
        "G0 X-1.75 Y-1.75\n"
        "G0 Z1\n"
        "G1 Z0 F200\n"
        "G1 X1.75 Y-1.75 Z0 F200\n"
        "G1 X1.75 Y1.75 Z0 F200\n"
        "G1 X-1.75 Y1.75 Z0 F200\n"
        "G1 X-1.75 Y-1.75 Z0 F200\n"
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
        "G0 Z10\n"
        "G0 X2.5 Y-2.5\n"
        "G0 Z1\n"
        "G1 Z-1 F200\n"
        "G1 X-2.5 Y-2.5 Z-1 F200\n"
        "G1 X-2.5 Y2.5 Z-1 F200\n"
        "G1 X2.5 Y2.5 Z-1 F200\n"
        "G1 X2.5 Y-2.5 Z-1 F200\n"
        "G0 Z10\n"
        "G0 X2.5 Y-2.5\n"
        "G0 Z1\n"
        "G1 Z-2 F200\n"
        "G1 X-2.5 Y-2.5 Z-2 F200\n"
        "G1 X-2.5 Y2.5 Z-2 F200\n"
        "G1 X2.5 Y2.5 Z-2 F200\n"
        "G1 X2.5 Y-2.5 Z-2 F200\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_tabs(job: Job, bottom_face):
    code = job.profile(
        bottom_face, outer_offset=0, tabs=EdgeTabs(spacing=2, width=1, height=1)
    ).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0 Z10\n"
        "G0 X2.5 Y-2.5\n"
        "G0 Z1\n"
        "G1 Z-2 F200\n"
        "G1 X1.333 Y-2.5 Z-2 F200\n"
        "G1 X1.333 Y-2.5 Z-1 F200\n"
        "G1 X0.333 Y-2.5 Z-1 F200\n"
        "G1 X0.333 Y-2.5 Z-2 F200\n"
        "G1 X-0.333 Y-2.5 Z-2 F200\n"
        "G1 X-0.333 Y-2.5 Z-1 F200\n"
        "G1 X-1.333 Y-2.5 Z-1 F200\n"
        "G1 X-1.333 Y-2.5 Z-2 F200\n"
        "G1 X-2.5 Y-2.5 Z-2 F200\n"
        "G1 X-2.5 Y-1.333 Z-2 F200\n"
        "G1 X-2.5 Y-1.333 Z-1 F200\n"
        "G1 X-2.5 Y-0.333 Z-1 F200\n"
        "G1 X-2.5 Y-0.333 Z-2 F200\n"
        "G1 X-2.5 Y0.333 Z-2 F200\n"
        "G1 X-2.5 Y0.333 Z-1 F200\n"
        "G1 X-2.5 Y1.333 Z-1 F200\n"
        "G1 X-2.5 Y1.333 Z-2 F200\n"
        "G1 X-2.5 Y2.5 Z-2 F200\n"
        "G1 X-1.333 Y2.5 Z-2 F200\n"
        "G1 X-1.333 Y2.5 Z-1 F200\n"
        "G1 X-0.333 Y2.5 Z-1 F200\n"
        "G1 X-0.333 Y2.5 Z-2 F200\n"
        "G1 X0.333 Y2.5 Z-2 F200\n"
        "G1 X0.333 Y2.5 Z-1 F200\n"
        "G1 X1.333 Y2.5 Z-1 F200\n"
        "G1 X1.333 Y2.5 Z-2 F200\n"
        "G1 X2.5 Y2.5 Z-2 F200\n"
        "G1 X2.5 Y1.333 Z-2 F200\n"
        "G1 X2.5 Y1.333 Z-1 F200\n"
        "G1 X2.5 Y0.333 Z-1 F200\n"
        "G1 X2.5 Y0.333 Z-2 F200\n"
        "G1 X2.5 Y-0.333 Z-2 F200\n"
        "G1 X2.5 Y-0.333 Z-1 F200\n"
        "G1 X2.5 Y-1.333 Z-1 F200\n"
        "G1 X2.5 Y-1.333 Z-2 F200\n"
        "G1 X2.5 Y-2.5 Z-2 F200\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )


def test_profile_wire_tabs(job: Job, bottom_face):
    code = job.profile(
        bottom_face, outer_offset=0, tabs=WireTabs(count=2, width=1, height=1)
    ).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Profile)\n"
        "G0 Z10\n"
        "G0 X2.5 Y-2.5\n"
        "G0 Z1\n"
        "G1 Z-1 F200\n"
        "G1 X2 Y-2.5 Z-1 F200\n"
        "G1 X2 Y-2.5 Z-2 F200\n"
        "G1 X-2.5 Y-2.5 Z-2 F200\n"
        "G1 X-2.5 Y2 Z-2 F200\n"
        "G1 X-2.5 Y2 Z-1 F200\n"
        "G1 X-2.5 Y2.5 Z-1 F200\n"
        "G1 X-2 Y2.5 Z-1 F200\n"
        "G1 X-2 Y2.5 Z-2 F200\n"
        "G1 X2.5 Y2.5 Z-2 F200\n"
        "G1 X2.5 Y-2 Z-2 F200\n"
        "G1 X2.5 Y-2 Z-1 F200\n"
        "G1 X2.5 Y-2.5 Z-1 F200\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )
