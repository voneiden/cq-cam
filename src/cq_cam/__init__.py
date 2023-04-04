from cq_cam.common import Unit
from cq_cam.operations.strategy import ContourStrategy, ZigZagStrategy

_extra = []
try:
    from cq_cam.operations.op3d import Surface3D

    _extra.append("Surface3D")
except ModuleNotFoundError:
    pass

METRIC = Unit.METRIC
IMPERIAL = Unit.IMPERIAL

__all__ = [
    "JobV2",
    "Pocket",
    "Drill",
    "Unit",
    "ZigZagStrategy",
    "ContourStrategy",
    "EdgeTabs",
    "WireTabs",
    "METRIC",
    "IMPERIAL",
] + _extra
