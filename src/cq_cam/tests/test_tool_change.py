import pytest

from cq_cam.command import StartSequence, ToolChange
from cq_cam.fluent import Job
from cq_cam.tool import Tool

tool_change_label = "(Job - Tool Change)\n"
speed_change_label = "(Job - Speed Change)\n"


@pytest.mark.parametrize("operation", [Job.pocket, Job.profile])
def test_same_tool_diameter(job: Job, big_box, operation):
    wp = big_box.faces(">Z").workplane().rect(8, 8).cutBlind(-1)

    tool_1 = Tool(tool_diameter=5, tool_number=1)
    tool_2 = Tool(tool_diameter=5, tool_number=2)

    face = wp.faces(">Z[1]").val()
    print(operation.__name__)

    job = operation(job, face, tool=tool_1)
    job = operation(job, face, tool=tool_2)

    assert job.operations[1].to_gcode() == job.operations[3].to_gcode()


@pytest.mark.parametrize("operation", [Job.pocket, Job.profile])
def test_different_tool_diameter(job: Job, big_box, operation):
    wp = big_box.faces(">Z").workplane().rect(8, 8).cutBlind(-1)

    tool_1 = Tool(tool_diameter=5, tool_number=1)
    tool_2 = Tool(tool_diameter=1, tool_number=2)

    face = wp.faces(">Z[1]").val()

    job = operation(job, face, tool=tool_1)
    job = operation(job, face, tool=tool_2)

    assert job.operations[1].to_gcode() != job.operations[3].to_gcode()


@pytest.mark.parametrize("operation", [Job.pocket, Job.profile, Job.drill])
def test_same_tool_number(job: Job, big_box, operation):
    wp = (
        big_box.faces(">Z")
        .workplane()
        .rect(8, 8)
        .cutBlind(-1)
        .tag("operation-1")
        .faces(">Z[1]")
        .workplane()
        .rect(6, 6)
        .cutBlind(-1)
    )

    output_1 = ToolChange(tool_number=1)
    tool_1 = Tool(tool_diameter=5, tool_number=1)

    face_1 = wp.faces(">Z[1]", tag="operation-1").val()
    face_2 = wp.faces(">Z[1]").val()

    if operation.__name__ == "drill":
        job = operation(job, face_1, tool=tool_1, depth=1)
        job = operation(job, face_2, tool=tool_1, depth=1)
    else:
        job = operation(job, face_1, tool=tool_1)
        job = operation(job, face_2, tool=tool_1)

    gcode_str, _ = output_1.to_gcode()
    assert job.operations[0].to_gcode() == f"{tool_change_label}{gcode_str}"
    assert job.operations[2].to_gcode() != f"{tool_change_label}{gcode_str}"


@pytest.mark.parametrize("operation", [Job.pocket, Job.profile, Job.drill])
@pytest.mark.parametrize(
    "output_1, output_2, tool_1, tool_2, label",
    [
        (
            ToolChange(tool_number=1),
            ToolChange(tool_number=2),
            Tool(tool_diameter=3, tool_number=1),
            Tool(tool_diameter=3, tool_number=2),
            tool_change_label,
        ),
        (
            StartSequence(spindle=1000),
            StartSequence(spindle=2000),
            Tool(speed=1000),
            Tool(speed=2000),
            speed_change_label,
        ),
        (
            ToolChange(tool_number=1, spindle=1000),
            ToolChange(tool_number=2, spindle=2000),
            Tool(tool_diameter=3, tool_number=1, speed=1000),
            Tool(tool_diameter=3, tool_number=2, speed=2000),
            tool_change_label,
        ),
    ],
)
def test_different_tool_number_(
    job: Job, big_box, operation, output_1, output_2, tool_1, tool_2, label
):
    wp = (
        big_box.faces(">Z")
        .workplane()
        .rect(8, 8)
        .cutBlind(-1)
        .tag("operation-1")
        .faces(">Z[1]")
        .workplane()
        .rect(6, 6)
        .cutBlind(-1)
    )

    face_1 = wp.faces(">Z[1]", tag="operation-1").val()
    face_2 = wp.faces(">Z[1]").val()

    if operation.__name__ == "drill":
        job = operation(job, face_1, tool=tool_1, depth=1)
        job = operation(job, face_2, tool=tool_2, depth=1)
    else:
        job = operation(job, face_1, tool=tool_1)
        job = operation(job, face_2, tool=tool_2)

    gcode_str_1, _ = output_1.to_gcode()
    gcode_str_2, _ = output_2.to_gcode()

    assert job.operations[0].to_gcode() == f"{label}{gcode_str_1}"
    assert job.operations[2].to_gcode() == f"{label}{gcode_str_2}"
