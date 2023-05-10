import unittest

import cadquery as cq

from cq_cam.address import (
    IJK,
    XYZ,
    ArcXAxis,
    ArcYAxis,
    ArcZAxis,
    DwellTime,
    Feed,
    Speed,
    ToolLengthOffset,
    ToolNumber,
    ToolRadiusOffset,
    XAxis,
    YAxis,
    ZAxis,
)


class TestUtils(unittest.TestCase):
    def test_x_axis(self):
        gcode = f"{XAxis(10.0)}"
        self.assertEqual("X10", gcode)

    def test_y_axis(self):
        gcode = f"{YAxis(10.0)}"
        self.assertEqual("Y10", gcode)

    def test_z_axis(self):
        gcode = f"{ZAxis(10.0)}"
        self.assertEqual("Z10", gcode)

    def test_i_axis(self):
        gcode = f"{ArcXAxis(10.0)}"
        self.assertEqual("I10", gcode)

    def test_j_axis(self):
        gcode = f"{ArcYAxis(10.0)}"
        self.assertEqual("J10", gcode)

    def test_k_axis(self):
        gcode = f"{ArcZAxis(10.0)}"
        self.assertEqual("K10", gcode)

    def test_feed(self):
        gcode = f"{Feed(10.0)}"
        self.assertEqual("F10.0", gcode)

    def test_speed(self):
        gcode = f"{Speed(10)}"
        self.assertEqual("S10", gcode)

    def test_dwell(self):
        gcode = f"{DwellTime(10.0)}"
        self.assertEqual("P10.0", gcode)

    def test_tool_number(self):
        gcode = f"{ToolNumber(10)}"
        self.assertEqual("T10", gcode)

    def test_tool_length(self):
        gcode = f"{ToolLengthOffset(10)}"
        self.assertEqual("H10", gcode)

    def test_tool_radius(self):
        gcode = f"{ToolRadiusOffset(10)}"
        self.assertEqual("D10", gcode)

    def test_xyz(self):
        end = cq.Vector(10.0, 20.0, 30.0)
        gcode = f"{XYZ(end)}"
        self.assertEqual("X10 Y20 Z30", gcode)

    def test_ijk(self):
        start = cq.Vector(10.0, 20.0, 30.0)
        center = cq.Vector(5.0, 5.0, 5.0)
        gcode = f"{IJK(start, center)}"
        self.assertEqual("I-5 J-15 K-25", gcode)
