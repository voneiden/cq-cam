import logging

from OCP.AIS import AIS_MultipleConnectedInteractive, AIS_Shape, AIS_TextLabel
import cadquery as cq
from OCP.TCollection import TCollection_ExtendedString

logger = logging.getLogger(__name__)


def to_occ_color(*args):
    from cq_editor.cq_utils import to_occ_color as _to_occ_color
    return _to_occ_color(*args)


def visualize_job_plane(job_plane: cq.Plane, length=1):
    x_edge = cq.Edge.makeLine(job_plane.origin, job_plane.origin + job_plane.xDir * length)
    x_shape = AIS_Shape(x_edge.wrapped)
    x_shape.SetColor(to_occ_color('red'))

    y_edge = cq.Edge.makeLine(job_plane.origin, job_plane.origin + job_plane.yDir * length)
    y_shape = AIS_Shape(y_edge.wrapped)
    y_shape.SetColor(to_occ_color('green'))

    z_edge = cq.Edge.makeLine(job_plane.origin, job_plane.origin + job_plane.zDir * length)
    z_shape = AIS_Shape(z_edge.wrapped)
    z_shape.SetColor(to_occ_color('blue'))

    group = AIS_MultipleConnectedInteractive()
    group.Connect(x_shape)
    group.Connect(y_shape)
    group.Connect(z_shape)
    return group


def visualize_job(job_plane: cq.Plane, commands, start_height=10):
    inverse_transform = job_plane.rG
    position = cq.Vector(0, 0, start_height)
    command_group = AIS_MultipleConnectedInteractive()

    for command in commands:
        shape, position = command.to_ais_shape(position)
        if shape:
            command_group.Connect(shape)

    command_group.SetLocalTransformation(inverse_transform.wrapped.Trsf())

    group = AIS_MultipleConnectedInteractive()
    group.Connect(command_group)
    group.Connect(visualize_job_plane(job_plane))
    return group


def visualize_job_as_edges(job_plane: cq.Plane, commands, start_height=10):
    """ To be used mainly for documentation purposes

    :param job_plane:
    :param commands:
    :param start_height:
    :return:
    """
    inverse_transform = job_plane.rG
    position = cq.Vector(0, 0, start_height)
    edges = []

    for command in commands:
        shape, position = command.to_ais_shape(position, as_edges=True)
        if shape:
            edge = shape.transformShape(inverse_transform)
            edges.append(edge)

    return edges
