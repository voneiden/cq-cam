import sys
from unittest.mock import Mock

import pytest
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB

from cq_cam import Job


@pytest.fixture(autouse=True)
def to_occ_color():
    _cq_utils = Mock()
    sys.modules["cq_editor.cq_utils"] = _cq_utils
    _cq_utils.to_occ_color = lambda x: Quantity_Color(0, 0, 0, Quantity_TOC_RGB)


@pytest.fixture
def show_object():
    def show_object(*args, **kwargs):
        return None

    return show_object


def test_fluent_show__no_show_object(job: Job, top_face):
    job = job.profile(top_face)
    with pytest.raises(ValueError):
        job.show()


def test_fluent_show__cq_editor(job: Job, top_face, caplog, show_object):
    show_object.__module__ = "cq_editor.test"

    job = job.profile(top_face)
    job.show(show_object)
    assert "Unsupported show_object source module" not in caplog.text


def test_fluent_show__ocp_vscode(job: Job, top_face, caplog, show_object):
    show_object.__module__ = "ocp_vscode.test"

    job = job.profile(top_face)
    job.show(show_object)
    assert "Unsupported show_object source module" not in caplog.text


def test_fluent_show__unknown(job: Job, top_face, caplog, show_object):
    show_object.__module__ = "unknown.test"

    job = job.profile(top_face)
    job.show(show_object)

    assert "Unsupported show_object source module (unknown)" in caplog.text


def test_fluent_show__autodiscovery(job: Job, top_face, caplog, show_object):
    show_object.__module__ = "cq_editor.test"
    import __main__

    __main__.__dict__["show_object"] = show_object

    job = job.profile(top_face)
    job.show()

    assert "Unsupported show_object source module (unknown)" not in caplog.text
