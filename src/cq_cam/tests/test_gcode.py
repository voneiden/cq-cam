import unittest

import cadquery as cq

from cq_cam.command import AbsoluteCV, CircularCCW, CircularCW, Cut, StopSequence, StartSequence, SafetyBlock, ToolChange, CoolantState


class TestUtils(unittest.TestCase):
    def test_linear(self):
        cmd = Cut.abs(10, 5, 1)
        gcode, position = cmd.to_gcode(None, cq.Vector(0, 0, 0))
        self.assertEqual("G1X10Y5Z1", gcode)
        self.assertEqual(cq.Vector(10.0, 5.0, 1.0), position)

    def test_cw_arc(self):
        start = cq.Vector(-1, 0, 0)
        mid = AbsoluteCV(x=0, y=1, z=None)
        end = AbsoluteCV(x=1, y=0, z=None)
        center = AbsoluteCV(x=0, y=0, z=None)
        cmd = CircularCW(end=end, center=center, mid=mid)
        gcode, position = cmd.to_gcode(None, start)
        self.assertEqual("G2X1I1J0", gcode)
        self.assertEqual(cq.Vector(1, 0, 0), position)

    def test_ccw_arc(self):
        start = cq.Vector(-1, 0, 0)
        mid = AbsoluteCV(x=0, y=-1, z=None)
        end = AbsoluteCV(x=1, y=0, z=None)
        center = AbsoluteCV(x=0, y=0, z=None)
        cmd = CircularCCW(end=end, center=center, mid=mid)
        gcode, position = cmd.to_gcode(None, start)
        self.assertEqual("G3X1I1J0", gcode)
        self.assertEqual(cq.Vector(1, 0, 0), position)

    def test_stop_sequence_default(self):
        cmd = StopSequence()
        gcode = cmd.to_gcode()
        self.assertEqual("M5", gcode)

    def test_stop_sequence_coolant(self):
        cmd = StopSequence(CoolantState.FLOOD)
        gcode = cmd.to_gcode()
        self.assertEqual("M5 M9", gcode)

    def test_start_sequence_default(self):
        cmd = StartSequence()
        gcode = cmd.to_gcode()
        self.assertEqual("M3", gcode)

    def test_start_sequence_spindle(self):
        cmd = StartSequence(spindle=1000)
        gcode = cmd.to_gcode()
        self.assertEqual("M3 S1000", gcode)

    def test_start_sequence_coolant_flood(self):
        cmd = StartSequence(coolant=CoolantState.FLOOD)
        gcode = cmd.to_gcode()
        self.assertEqual("M3 M8", gcode)
    
    def test_start_sequence_coolant_mist(self):
        cmd = StartSequence(coolant=CoolantState.MIST)
        gcode = cmd.to_gcode()
        self.assertEqual("M3 M7", gcode)

    def test_start_sequence_spindle_coolant(self):
        cmd = StartSequence(spindle=1000, coolant=CoolantState.FLOOD)
        gcode = cmd.to_gcode()
        self.assertEqual("M3 S1000 M8", gcode)

    def test_safety_block(self):
        cmd  = SafetyBlock()
        gcode = cmd.to_gcode()
        self.assertEqual("G90 G54 G64 G50 G17 G94\nG49 G40 G80\nG21\nG30", gcode)

    def test_tool_change_simple(self):
        cmd = ToolChange(2)
        gcode = cmd.to_gcode()
        self.assertEqual("M5\nG30\nM1\nT2 G43 H2 M6\nM3", gcode)

    def test_tool_change_spindle(self):
        cmd = ToolChange(2, spindle = 1000)
        gcode = cmd.to_gcode()
        self.assertEqual("M5\nG30\nM1\nT2 G43 H2 M6\nM3 S1000", gcode)

    def test_tool_change_coolant_flood(self):
        cmd = ToolChange(2, coolant=CoolantState.FLOOD)
        gcode = cmd.to_gcode()
        self.assertEqual("M5 M9\nG30\nM1\nT2 G43 H2 M6\nM3 M8", gcode)

    def test_tool_change_coolant_mist(self):
        cmd = ToolChange(2, coolant=CoolantState.MIST)
        gcode = cmd.to_gcode()
        self.assertEqual("M5 M9\nG30\nM1\nT2 G43 H2 M6\nM3 M7", gcode)

    def test_tool_change_spindle_coolant(self):
        cmd = ToolChange(2, spindle=1000, coolant=CoolantState.FLOOD)
        gcode = cmd.to_gcode()
        self.assertEqual("M5 M9\nG30\nM1\nT2 G43 H2 M6\nM3 S1000 M8", gcode)