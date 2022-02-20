from cadquery import cq

from cq_cam.commands.base_command import Unit
from cq_cam.operations.base_operation import Job
from cq_cam.operations.profile import Profile
from cq_cam.visualize import visualize_task

L = cq.Workplane('YZ').lineTo(20, -5).lineTo(20, 10).lineTo(15, 10).lineTo(15, 5).lineTo(0, 5).close().extrude(10)
L_top = L.faces('<X').workplane()

box = cq.Workplane().box(10, 10, 10)
box_top = box.faces('<X').workplane()

job = Job(workplane=box_top, feed=300, plunge_feed=50, unit=Unit.METRIC, rapid_height=10)
profile = Profile(job=job, clearance_height=5, top_height=0, faces=box.faces('>X').objects, outer_offset=4, stepdown=-2.77)

print(job.to_gcode())

toolpath = visualize_task(job, profile)
# show_object(box, 'box')
show_object(toolpath, 'toolpath', {'color': 'red'})
# show_object(visual_profile_cuts, 'visual_profile_cuts', {'color': 'red'})
# show_object(visual_profile_plunges, 'visual_profile_plunges', {'color': 'red'})
# show_object(L, 'L')
# show_object(L.faces('>X'), 'ltop')
# show_object(L.faces('<X'), 'bottom')
show_object(box, 'bottombox')
