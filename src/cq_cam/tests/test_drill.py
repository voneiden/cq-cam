import cadquery as cq

from cq_cam.fluent import Job
from cq_cam.operations.tabs import EdgeTabs
from cq_cam.utils.circle_bug_workaround import circle_bug_workaround


def test_drill_superficially(box, job):
    box = box.faces(">Z").workplane().circle(1).cutThruAll()
    hole = [w for w in box.wires(">Z").objects if len(w.Edges()) == 1][0]
    code = job.drill(hole, depth=1).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90\n"
        "G21\n"
        "(Job - Drill)\n"
        "G0Z1\n"
        "X0Y0\n"
        "Z0\n"
        "G1Z-1G1Z0\n"
        "G0Z10\n"
        "X0Y0"
    )
