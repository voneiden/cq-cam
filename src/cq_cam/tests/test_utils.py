import unittest
import cadquery as cq

from cq_cam.utils.utils import is_arc_clockwise2


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
