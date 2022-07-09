import cadquery as cq

from cq_cam.fluent import JobV2

if __name__ == 'temp' or __name__ == '__main__':
    wp = cq.Workplane().box(15, 10, 5)
    top = wp.faces('>X').workplane()
    bottom = wp.faces('<X')
    cam = JobV2(top.plane, 100, 3.175).profile(bottom, stepdown=3)
    show_object(wp)
    cam.show(show_object)
