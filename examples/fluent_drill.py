import cadquery as cq

from cq_cam.fluent import JobV2


def demo():

    result = cq.Workplane("front").box(20.0, 20.0, 2).faces('>Z').workplane().pushPoints([
        (3, 3), (-5, -8), (0, 0), (5, 2), (7, -3), (-8, 2)]).circle(1).cutThruAll()

    job_wp = result.faces('>Z').workplane()
    cam = (
        JobV2(job_wp.plane, feed=300, tool_diameter=3)
        .drill(depth=2, clearance_height=5, top_height=0, o=result.faces('>Z').objects[0].innerWires())
    )
    show_object(result, 'obj')
    show_object(result.faces('>Z'), 'avoid', {'color': 'red'})
    cam.show(show_object)

if 'show_object' in locals() or __name__ == '__main__':
    demo()
