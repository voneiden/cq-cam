import unittest

import cadquery as cq

from cq_cam.command import AbsoluteCV, CircularCCW, CircularCW, Cut


class TestUtils(unittest.TestCase):
    def test_linear(self):
        cmd = Cut.abs(10, 5, 1)
        cmd.previous_command = None
        cmd.start = cq.Vector(0, 0, 0)
        gcode, position = cmd.to_gcode()
        self.assertEqual("G1X10Y5Z1", gcode)
        self.assertEqual(cq.Vector(10.0, 5.0, 1.0), position)

    def test_cw_arc(self):
        start = cq.Vector(-1, 0, 0)
        mid = AbsoluteCV(x=0, y=1, z=None)
        end = AbsoluteCV(x=1, y=0, z=None)
        center = AbsoluteCV(x=0, y=0, z=None)
        cmd = CircularCW(end=end, center=center, mid=mid)
        cmd.previous_command = None
        cmd.start = start
        gcode, position = cmd.to_gcode()
        self.assertEqual("G2X1I1J0", gcode)
        self.assertEqual(cq.Vector(1, 0, 0), position)

    def test_ccw_arc(self):
        start = cq.Vector(-1, 0, 0)
        mid = AbsoluteCV(x=0, y=-1, z=None)
        end = AbsoluteCV(x=1, y=0, z=None)
        center = AbsoluteCV(x=0, y=0, z=None)
        cmd = CircularCCW(end=end, center=center, mid=mid)
        cmd.previous_command = None
        cmd.start = start
        gcode, position = cmd.to_gcode()
        self.assertEqual("G3X1I1J0", gcode)
        self.assertEqual(cq.Vector(1, 0, 0), position)
