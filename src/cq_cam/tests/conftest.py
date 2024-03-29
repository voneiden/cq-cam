import cadquery as cq
import pytest

from cq_cam.fluent import Job


@pytest.fixture
def box():
    return cq.Workplane("XY").rect(5, 5).extrude(2)


@pytest.fixture
def big_box():
    return cq.Workplane("XY").box(10, 10, 5)


@pytest.fixture
def top_face_workplane(box):
    return box.faces(">Z")


@pytest.fixture
def top_plane(top_face_workplane):
    return top_face_workplane.workplane().plane


@pytest.fixture
def top_face(top_face_workplane):
    return top_face_workplane.objects[0]


@pytest.fixture
def bottom_face(box):
    return box.faces("<Z").objects[0]


@pytest.fixture
def job(top_plane):
    return Job(top_plane, 200, tool_diameter=1.5)
