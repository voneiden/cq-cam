from cq_cam.common import Unit
from cq_cam.fluent import Job
from cq_cam.operations.strategy import ContourStrategy, ZigZagStrategy

METRIC = Unit.METRIC
IMPERIAL = Unit.IMPERIAL

__all__ = [
    "Job",
    "METRIC",
    "IMPERIAL",
]
