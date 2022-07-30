import unittest
import cadquery as cq

from cq_cam.utils.utils import is_arc_clockwise2, wire_to_offset_safe_wire, wire_to_ordered_edges


class TestUtils(unittest.TestCase):
    def test_arc_clockwise(self):
        ccw_arc = cq.Edge.makeThreePointArc(
            cq.Vector(1, 0, 0),
            cq.Vector(0.707, 0.707, 0),
            cq.Vector(0, 1, 0),
        )
        self.assertFalse(is_arc_clockwise2(ccw_arc))

        cw_arc = cq.Edge.makeThreePointArc(
            cq.Vector(1, 0, 0),
            cq.Vector(0.707, -0.707, 0),
            cq.Vector(0, -1, 0),
        )
        self.assertTrue(is_arc_clockwise2(cw_arc))

    def test_non_z_arcs(self):
        x_axis_arc = cq.Edge.makeThreePointArc(
            cq.Vector(0, 0, 1),
            cq.Vector(0, 0.707, 0.707),
            cq.Vector(0, 1, 0),
        )
        self.assertRaises(RuntimeError, lambda: is_arc_clockwise2(x_axis_arc))

    def test_helical_arcs(self):
        helical_ccw_arc = cq.Edge.makeThreePointArc(
            cq.Vector(1, 0, 0),
            cq.Vector(0.707, 0.707, -0.5),
            cq.Vector(0, 1, -1),
        )
        self.assertFalse(is_arc_clockwise2(helical_ccw_arc))

        helical_cw_arc = cq.Edge.makeThreePointArc(
            cq.Vector(1, 0, 0),
            cq.Vector(0.707, -0.707, -0.5),
            cq.Vector(0, -1, -1),
        )
        self.assertTrue(is_arc_clockwise2(helical_cw_arc))

    def test_wire_to_offset_safe_wire(self):
        circle = cq.Workplane().circle(5).objects[0]
        wire = wire_to_offset_safe_wire(circle)

        original_edges = wire_to_ordered_edges(circle)
        new_edges = wire_to_ordered_edges(wire)
        self.assertEqual(len(original_edges), 1)
        self.assertEqual(len(new_edges), 2)

        bspline = cq.Workplane().spline([[0, 0], [1, 1], [5, 0], [2, 3]]).close().objects[0]
        wire = wire_to_offset_safe_wire(bspline)
        original_edges = wire_to_ordered_edges(bspline)
        new_edges = wire_to_ordered_edges(wire)
        self.assertEqual(len(original_edges), 2)
        self.assertEqual(len(new_edges), 107)
