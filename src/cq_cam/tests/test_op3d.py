from opencamlib import ocl

from cq_cam import Job


def test_profile_square_outside(job: Job, box):
    box = box.faces(">Z").workplane().rect(2, 2).cutBlind(1)

    code = job.surface3d(o=box.faces(), tool=ocl.CylCutter(1, 15)).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Surface 3D)\n"
        "G0 Z10\n"
        "G0 X-1.5 Y1 Z10\n"
        "G0 X-1.5 Y1 Z1\n"
        "G1 X-1.5 Y1 Z0\n"
        "G1 X-1 Y1 Z0\n"
        "G1 X-0.5 Y1 Z0\n"
        "G1 X0 Y1 Z0\n"
        "G1 X0.5 Y1 Z0\n"
        "G1 X1 Y1 Z0\n"
        "G1 X1.5 Y1 Z0\n"
        "G1 X1.5 Y1 Z0\n"
        "G1 X1.5 Y0.5 Z0\n"
        "G1 X1.5 Y0.2 Z0\n"
        "G1 X1.5 Y0.2 Z0\n"
        "G1 X1 Y0.2 Z0\n"
        "G1 X0.5 Y0.2 Z0\n"
        "G1 X0 Y0.2 Z0\n"
        "G1 X-0.5 Y0.2 Z0\n"
        "G1 X-1 Y0.2 Z0\n"
        "G1 X-1.5 Y0.2 Z0\n"
        "G1 X-1.5 Y0.2 Z0\n"
        "G1 X-1.5 Y-0.3 Z0\n"
        "G1 X-1.5 Y-0.6 Z0\n"
        "G1 X-1.5 Y-0.6 Z0\n"
        "G1 X-1 Y-0.6 Z0\n"
        "G1 X-0.5 Y-0.6 Z0\n"
        "G1 X0 Y-0.6 Z0\n"
        "G1 X0.5 Y-0.6 Z0\n"
        "G1 X1 Y-0.6 Z0\n"
        "G1 X1.5 Y-0.6 Z0\n"
        "G1 X1.5 Y-0.6 Z0\n"
        "G1 X1.5 Y-1.1 Z0\n"
        "G1 X1.5 Y-1.4 Z0\n"
        "G1 X1.5 Y-1.4 Z0\n"
        "G1 X1 Y-1.4 Z0\n"
        "G1 X0.5 Y-1.4 Z0\n"
        "G1 X0 Y-1.4 Z0\n"
        "G1 X-0.5 Y-1.4 Z0\n"
        "G1 X-1 Y-1.4 Z0\n"
        "G1 X-1.5 Y-1.4 Z0\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5\n"
        "M30"
    )
