import cadquery as cq
import pytest

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
from cq_cam.command import CommandVector


def test_x_axis():
    gcode = f"{XAxis(10.0)}"
    assert gcode == "X10"


def test_y_axis():
    gcode = f"{YAxis(10.0)}"
    assert gcode == "Y10"


def test_z_axis():
    gcode = f"{ZAxis(10.0)}"
    assert gcode == "Z10"


def test_i_axis():
    gcode = f"{ArcXAxis(10.0)}"
    assert gcode == "I10"


def test_j_axis():
    gcode = f"{ArcYAxis(10.0)}"
    assert gcode == "J10"


def test_k_axis():
    gcode = f"{ArcZAxis(10.0)}"
    assert gcode == "K10"


def test_feed():
    gcode = f"{Feed(10.0)}"
    assert gcode == "F10.0"


def test_speed():
    gcode = f"{Speed(10)}"
    assert gcode == "S10"


def test_dwell():
    gcode = f"{DwellTime(10.0)}"
    assert gcode == "P10.0"


def test_tool_number():
    gcode = f"{ToolNumber(10)}"
    assert gcode == "T10"


def test_tool_length():
    gcode = f"{ToolLengthOffset(10)}"
    assert gcode == "H10"


def test_tool_radius():
    gcode = f"{ToolRadiusOffset(10)}"
    assert gcode == "D10"


def test_xyz():
    end = cq.Vector(10.0, 20.0, 30.0)
    gcode = f"{XYZ(end)}"
    assert gcode == "X10 Y20 Z30"


def test_ijk():
    start = cq.Vector(10.0, 20.0, 30.0)
    center_cv = CommandVector(5.0, 5.0, 5.0)
    center = center_cv.to_vector(start, relative=True)
    gcode = f"{IJK(center)}"
    assert gcode == "I-5 J-15 K-25"
