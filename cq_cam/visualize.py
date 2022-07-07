import logging

from OCP.AIS import AIS_MultipleConnectedInteractive
import cadquery as cq

logger = logging.getLogger(__name__)


def to_occ_color(*args):
    from cq_editor.cq_utils import to_occ_color as _to_occ_color
    return _to_occ_color(*args)


def visualize_job(job_plane: cq.Plane, commands, start_height=10):
    inverse_transform = job_plane.rG
    position = cq.Vector(0, 0, start_height)
    group = AIS_MultipleConnectedInteractive()
    for command in commands:
        shape, position = command.to_ais_shape(position, inverse_transform)
        group.Connect(shape)

    group.SetLocalTransformation(inverse_transform.wrapped.Trsf())
    return group
