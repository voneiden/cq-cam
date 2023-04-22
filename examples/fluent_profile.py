import cadquery as cq

from cq_cam.fluent import Job
from cq_cam.operations import profile
from cq_cam.operations.tabs import EdgeTabs

if __name__ == "temp" or __name__ == "__main__":
    wp = cq.Workplane().box(15, 10, 5)
    top = wp.faces(">X").workplane()
    bottom = wp.faces("<X")
    cam = Job(top.plane, 100, 3.175).profile(
        bottom, stepdown=3, tabs=EdgeTabs(spacing=3, width=2, height=6)
    )
    show_object(wp)
    cam.show(show_object)
    for i, o in enumerate(profile.DEBUG):
        show_object(o, f"edge-{i}")
