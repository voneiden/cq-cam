import math

from OCP.AIS import AIS_MultipleConnectedInteractive, AIS_Line, AIS_Shape
from OCP.Geom import Geom_CartesianPoint
from cadquery import cq, Edge
from cq_editor.cq_utils import to_occ_color

from cq_cam.commands.base_command import MotionCommand
from cq_cam.commands.command import Plunge, Cut, Rapid, Circular, CircularCW
from cq_cam.job.job import Job
from cq_cam.operations.base_operation import Task


class VisualizeError(Exception):
    pass


def visualize_task(job: Job, task: Task):
    root_workplane = job.workplane
    root_plane = root_workplane.plane

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

            world_start = root_plane.toWorldCoords((start.x, start.y, start.z))
            world_end = root_plane.toWorldCoords((end.x, end.y, end.z))
            if isinstance(command, Circular):

                # W
                try:
                    # Vector start->end
                    se = end.sub(start)

                    # Normal (TODO, this should optional param with arc commands? helix support)
                    if isinstance(command, CircularCW):
                        normal = cq.Vector(0, 0, -1)
                    else:
                        normal = cq.Vector(0, 0, 1)

                    # Midpoint unit vector
                    # TODO CW/CCW
                    midpoint_unit = se.cross(normal).normalized()
                    center_to_midpoint = midpoint_unit * command.radius

                    # Determine center

                    # Calculate bisector point
                    bisector = se / 2

                    # Bisector point center distance
                    bisect_center_distance = math.sqrt(command.radius ** 2 - abs(bisector) ** 2)

                    center = start + bisector + (-midpoint_unit * bisect_center_distance)

                    midpoint = center + center_to_midpoint

                    radius = command.radius if isinstance(command, CircularCW) else -command.radius
                    wp = (
                        root_workplane.workplane(offset=start.z)
                            .moveTo(start.x, start.y)
                            .radiusArc((end.x, end.y), radius)
                    )
                    arc = Edge.makeThreePointArc(world_start, root_plane.toWorldCoords((midpoint.x, midpoint.y, midpoint.z)), world_end)
                    #line = AIS_Shape(wp.objects[0].wrapped)
                    line = AIS_Shape(arc.wrapped)
                except:
                    line = AIS_Line(
                        Geom_CartesianPoint(world_start.x, world_start.y, world_start.z),
                        Geom_CartesianPoint(world_end.x, world_end.y, world_end.z)
                    )
                    line.SetColor(to_occ_color('yellow'))
            else:
                line = AIS_Line(
                    Geom_CartesianPoint(world_start.x, world_start.y, world_start.z),
                    Geom_CartesianPoint(world_end.x, world_end.y, world_end.z)
                )
                print(line)
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
