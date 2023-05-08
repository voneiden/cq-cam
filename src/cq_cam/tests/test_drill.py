def test_drill_superficially(box, job):
    box = box.faces(">Z").workplane().circle(1).cutThruAll()
    hole = [w for w in box.wires(">Z").objects if len(w.Edges()) == 1][0]
    code = job.drill(hole, depth=1).to_gcode()
    assert code == (
        "(Job - Feedrate: 200 - Unit: Unit.METRIC)\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M3\n"
        "(Job - Drill)\n"
        "G0Z1\n"
        "X0Y0\n"
        "Z0\n"
        "G1Z-1\n"
        "G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30\n"
        "M5"
    )
