from cq_cam import Job


def test_drill_superficially(box, job: Job):
    box = box.faces(">Z").workplane().circle(1).cutThruAll()
    hole = [w for w in box.wires(">Z").objects if len(w.Edges()) == 1][0]
    code = job.drill(hole, depth=1).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Drill)\n"
        "G0 Z1\n"
        "G0 X0 Y0 Z1\n"
        "G0 X0 Y0 Z0\n"
        "G1 X0 Y0 Z-1 F200\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5\n"
        "M30"
    )
