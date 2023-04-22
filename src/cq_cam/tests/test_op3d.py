from opencamlib import ocl

from cq_cam import Job


def test_profile_square_outside(job: Job, box):
    box = box.faces(">Z").workplane().rect(2, 2).cutBlind(1)

    code = job.surface3d(o=box.faces(), tool=ocl.CylCutter(1, 15)).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90\n"
        "G21\n"
        "(Job - Surface 3D)\n"
        "G0Z10\n"
        "X-1.5Y1\n"
        "Z1\n"
        "G1Z0\n"
        "X-1\n"
        "X-0.5\n"
        "X0\n"
        "X0.5\n"
        "X1\n"
        "X1.5\n"
        "Y0.5\n"
        "Y0.2\n"
        "X1\n"
        "X0.5\n"
        "X0\n"
        "X-0.5\n"
        "X-1\n"
        "X-1.5\n"
        "Y-0.3\n"
        "Y-0.6\n"
        "X-1\n"
        "X-0.5\n"
        "X0\n"
        "X0.5\n"
        "X1\n"
        "X1.5\n"
        "Y-1.1\n"
        "Y-1.4\n"
        "X1\n"
        "X0.5\n"
        "X0\n"
        "X-0.5\n"
        "X-1\n"
        "X-1.5G1Z0\n"
        "G0Z10\n"
        "X0Y0"
    )
