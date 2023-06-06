import logging

import cadquery as cq
from OCP.AIS import AIS_MultipleConnectedInteractive, AIS_Shape

logger = logging.getLogger(__name__)


def to_occ_color(*args):
    from cq_editor.cq_utils import to_occ_color as _to_occ_color

    return _to_occ_color(*args)


_occ_color_cache = {}


def cached_occ_color(color: str):
    return _occ_color_cache.setdefault(color, to_occ_color(color))


def visualize_job_plane(job_plane: cq.Plane, length=1):
    x_edge = cq.Edge.makeLine(
        job_plane.origin, job_plane.origin + job_plane.xDir * length
    )
    x_shape = AIS_Shape(x_edge.wrapped)
    x_shape.SetColor(cached_occ_color("red"))

    y_edge = cq.Edge.makeLine(
        job_plane.origin, job_plane.origin + job_plane.yDir * length
    )
    y_shape = AIS_Shape(y_edge.wrapped)
    y_shape.SetColor(cached_occ_color("green"))

    z_edge = cq.Edge.makeLine(
        job_plane.origin, job_plane.origin + job_plane.zDir * length
    )
    z_shape = AIS_Shape(z_edge.wrapped)
    z_shape.SetColor(cached_occ_color("blue"))

    group = AIS_MultipleConnectedInteractive()
    group.Connect(x_shape)
    group.Connect(y_shape)
    group.Connect(z_shape)
    return group


def visualize_job(
    job_plane: cq.Plane, commands, start_height=10
) -> AIS_MultipleConnectedInteractive:
    """
    Visualize commands as AIS_Line objects grouped inside AIS_MultipleConnectedInteractive
    This method is suitable for cq-editor as it enables the use of colours.
    """
    inverse_transform = job_plane.rG
    command_group = AIS_MultipleConnectedInteractive()

    for i, command in enumerate(commands):
        shape = command.to_ais_shape(alt_color=i % 2)
        if shape:
            command_group.Connect(shape)

    command_group.SetLocalTransformation(inverse_transform.wrapped.Trsf())

    group = AIS_MultipleConnectedInteractive()
    group.Connect(command_group)
    group.Connect(visualize_job_plane(job_plane))
    return group


def visualize_job_as_edges(
    job_plane: cq.Plane, commands, start_height=10
) -> list[cq.Edge]:
    """
    Slower generation but slightly more performant rendering compared to AIS_Line

    Used for documentation and ocp_vscode.
    """
    inverse_transform = job_plane.rG
    edges = []

    for command in commands:
        shape = command.to_ais_shape(as_edges=True)
        if shape:
            edge = shape.transformShape(inverse_transform)
            edges.append(edge)

    return edges
