import logging

from OCP.AIS import AIS_MultipleConnectedInteractive, AIS_Line, AIS_Shape
from OCP.Geom import Geom_CartesianPoint
from cadquery import cq, Edge

from cq_cam.commands.base_command import MotionCommand
from cq_cam.commands.command import Plunge, Cut, Rapid, Circular
from cq_cam.commands.util_command import arc_center_midpoint
from cq_cam.job import Job
from cq_cam.operations.base_operation import Operation

logger = logging.getLogger(__name__)


class VisualizeError(Exception):
    pass


def to_occ_color(*args):
    from cq_editor.cq_utils import to_occ_color as _to_occ_color
    return _to_occ_color(*args)


def visualize_task(job: Job, task: Operation, as_edges=False):
    root_workplane = job.workplane
    root_plane = root_workplane.plane

    if as_edges:
        group = []
    else:
        group = AIS_MultipleConnectedInteractive()

    # x = 0
    # y = 0
    # z = job.rapid_height
    start = cq.Vector(0, 0, job.rapid_height)

    last_plunge = True
    for command in task.commands:
        if isinstance(command, MotionCommand):
            end = command.end(start)
            # cx = command.end.x
            # cy = command.end.y
            # cz = command.end.z

            # New viz method coords are in world
            world_start = root_plane.toWorldCoords((start.x, start.y, start.z))
            world_end = root_plane.toWorldCoords((end.x, end.y, end.z))
            #world_start = start
            #world_end = end
            if isinstance(command, Circular):

                # W
                try:
                    # TODO one way of doing things, please
                    if command.mid:
                        relative_midpoint = cq.Vector(command.mid)
                    else:
                        raise RuntimeError('who is using this?')
                        _, midpoint = arc_center_midpoint(command, start, end)

                    # radius = command.radius if isinstance(command, CircularCW) else -command.radius
                    # wp = (
                    #    root_workplane.workplane(offset=start.z)
                    #        .moveTo(start.x, start.y)
                    #        .radiusArc((end.x, end.y), radius)
                    # )
                    if world_start == world_end:
                        # TODO this will blow up circles that are not flat in world coodinates
                        ijk = cq.Vector(command.ijk)
                        assert ijk.Length > 0
                        center = start + ijk
                        world_center = root_plane.toWorldCoords((center.x, center.y, center.z))
                        circle = Edge.makeCircle(ijk.Length, world_center)
                        if as_edges:
                            line = circle
                        else:
                            line = AIS_Shape(circle.wrapped)

                    else:
                        midpoint = relative_midpoint.add(start)
                        arc = Edge.makeThreePointArc(world_start,
                                                     root_plane.toWorldCoords((midpoint.x, midpoint.y, midpoint.z)),
                                                     world_end)
                        # line = AIS_Shape(wp.objects[0].wrapped)
                        if as_edges:
                            line = arc
                        else:
                            line = AIS_Shape(arc.wrapped)
                except Exception as ex:
                    logger.error("Failed circular render", ex)
                    if world_start == world_end:
                        logger.warning("encountered zero length")
                        continue

                    if as_edges:
                        line = Edge.makeLine(world_start, world_end)
                    else:
                        line = AIS_Line(
                            Geom_CartesianPoint(world_start.x, world_start.y, world_start.z),
                            Geom_CartesianPoint(world_end.x, world_end.y, world_end.z)
                        )
                        line.SetColor(to_occ_color('yellow'))
            else:
                if world_start == world_end:
                    logger.warning("encountered zero length")
                    continue
                if as_edges:
                    line = Edge.makeLine(world_start, world_end)
                else:
                    line = AIS_Line(
                        Geom_CartesianPoint(world_start.x, world_start.y, world_start.z),
                        Geom_CartesianPoint(world_end.x, world_end.y, world_end.z)
                    )
            if as_edges:
                group.append(line)
            else:
                if isinstance(command, Rapid):
                    line.SetColor(to_occ_color('green'))
                elif isinstance(command, Cut):
                    line.SetColor(to_occ_color('red'))
                elif isinstance(command, Plunge):
                    line.SetColor(to_occ_color('blue'))

                if isinstance(command, Plunge):
                    line.Attributes().SetLineArrowDraw(True)
                    last_plunge = True
                elif last_plunge and isinstance(line, AIS_Line):
                    line.Attributes().SetLineArrowDraw(True)
                    last_plunge = False

                group.Connect(line)

            start = end

    return group
