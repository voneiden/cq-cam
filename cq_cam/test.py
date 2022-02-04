import cadquery as cq
from cadquery import Face

from lib import cqlib

# Setup
from cq_cam.utils import wire_to_type_vectors, is_tvs_clockwise

cqlib.setup(locals())
cq.Workplane = cqlib.CustomWorkplane
cq.Assembly = cqlib.Assembly
show_object = cqlib.CustomFunctions.show_object
debug = cqlib.CustomFunctions.debug

cw = cq.Workplane().lineTo(0, 20).lineTo(10, 15).lineTo(10, 0).close().wire()
ccw = cq.Workplane().lineTo(-20, 0).lineTo(-15, -10).lineTo(0, -10).close().wire()
box = cq.Workplane().box(10, 10, 10, True)
box_face = box.faces('<Z')

cw_face = Face.makeFromWires(cw.objects[0])
ccw_face = Face.makeFromWires(ccw.objects[0])
show_object(cw_face, 'cw_face')
show_object(ccw_face, 'ccw_face')
show_object(box_face, 'box_face')
print("OK")

icw = is_tvs_clockwise(wire_to_type_vectors(box.plane, cw.objects[0]))
iccw = is_tvs_clockwise(wire_to_type_vectors(box.plane, ccw.objects[0]))
print("ok")