import cadquery as cq
from cq_cam.fluent import Job


def demo():
    job_plane = cq.Workplane().box(15, 15, 10).faces('>Z').workplane()
    obj = (
        job_plane
        .rect(7.5, 7.5)
        .cutBlind(-4)
        .faces('>Z[1]')
        .rect(2, 2)
        .extrude(2)
        .faces('>Z').workplane()
        .moveTo(-5.75, 0)
        .rect(4, 2)
        .cutBlind(-6)
    )
    op_plane = obj.faces('>Z[1] or >Z[2]')
    # test = obj.faces('>Z[-3] or >Z[-2] or >Z[-4]')
    # obj = op_plane.workplane().rect(2, 2).extrude(4)

    cam = (
        Job(job_plane.plane, feed=300, tool_diameter=3.175)
        .pocket(op_plane)
    )




def demo2():
    result = cq.Workplane("front").box(20.0, 20.0, 2).faces('>Z').workplane().rect(15, 15).cutBlind(-1)
    result = result.moveTo(0, -10).rect(5, 5).cutBlind(-1)
    # show_object(result.faces('<Z[1]'))
    job_wp = result.faces('>Z').workplane()
    cam = (
        Job(job_wp.plane, feed=300, tool_diameter=1)
        .pocket(clearance_height=5, top_height=0, o=result.faces('<Z[1]'),
                outer_boundary_offset=1, avoid=result.faces('>Z'))
    )

    show_object(result, 'obj')
    show_object(result.faces('>Z'), 'avoid', {'color': 'red'})
    cam.show(show_object)


if 'show_object' in locals() or __name__ == '__main__':
    demo2()
