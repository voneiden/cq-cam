import cadquery as cq
import ocl

from cq_cam.fluent import Job


def demo():
    wp = cq.Workplane('XZ').lineTo(100, 0).lineTo(100, 120).lineTo(80, 120).lineTo(0, 0).close().extrude(50)
    job_wp = wp.faces('>Z').workplane()
    faces = wp.faces('(not +X) and (not -X) and (not -Y) and (not +Y) and (not -Z)')
    cam = (Job(job_wp.plane,
               feed=300,
               plunge_feed=100,
               rapid_height=10)
           .surface3d(clearance_height=2, top_height=0, o=faces, tool=ocl.CylCutter(3.175, 10),
                      avoid=None, stepdown=-5)
           )

    show_object(wp, 'part')


def demo2():
    result = (
        cq.Workplane('XY').rect(30, 30).extrude(20)
        .faces('>Z').workplane().rect(20, 20).cutBlind(-5)
        .faces('>Z[1]').workplane().rect(10, 10).extrude(3)
        .faces('>Z[1]').fillet(1)
        .faces('>Z[2]').fillet(1)
        .faces('>Z')
    )
    result.objects = result.objects[0].innerWires()
    result = result.fillet(1)
    job_wp = result.faces('>Z').workplane()
    cam = (Job(job_wp.plane,
               feed=300,
               plunge_feed=100,
               rapid_height=10)
           .surface3d(clearance_height=2, top_height=0, o=result.faces(), tool=ocl.CylCutter(3.175, 10),
                      interpolation_step=0.1, outer_boundary_offset=0)
           )

    show_object(result)
    cam.show(show_object)


if 'show_object' in locals() or __name__ == '__main__':
    demo2()
