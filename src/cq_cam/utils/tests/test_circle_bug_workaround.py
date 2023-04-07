import cadquery as cq


def test_circle_bug():
    wp = cq.Workplane("XY")
    wp = wp.box(1, 1, 1)
    wp = wp.faces(">Z")
    c1_wp = wp.circle(0.75)
    c2_wp = wp.offset2D(0.1)

    assert (
        c1_wp.objects[0].location().toTuple() != c2_wp.objects[0].location().toTuple()
    )
