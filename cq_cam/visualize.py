from OCP.AIS import AIS_MultipleConnectedInteractive, AIS_Line, AIS_Shape
from OCP.Geom import Geom_CartesianPoint
from cq_editor.cq_utils import to_occ_color

from cq_cam.commands.base_command import EndData
from cq_cam.commands.command import Plunge, Cut, Rapid, Circular, CircularCW
from cq_cam.job.job import Job
from cq_cam.operations.base_operation import Task


class VisualizeError(Exception):
    pass


def visualize_task(job: Job, task: Task):
    root_workplane = job.workplane
    root_plane = root_workplane.plane

    group = AIS_MultipleConnectedInteractive()

    x = 0
    y = 0
    z = job.rapid_height
    last_plunge = True
    for command in task.commands:
        if isinstance(command, EndData):
            cx = command.end.x
            cy = command.end.y
            cz = command.end.z

            start = root_plane.toWorldCoords((x, y, z))
            end = root_plane.toWorldCoords((cx, cy, cz))
            if isinstance(command, Circular):
                # TODO this approach does not support helical arcs
                # W
                try:
                    radius = command.radius if isinstance(command, CircularCW) else -command.radius
                    wp = (
                        root_workplane.workplane(offset=z)
                        .moveTo(x, y)
                        .radiusArc((cx, cy), radius)
                    )
                    line = AIS_Shape(wp.objects[0].wrapped)
                except:
                    line = AIS_Line(
                        Geom_CartesianPoint(start.x, start.y, start.z),
                        Geom_CartesianPoint(end.x, end.y, end.z)
                    )
                    line.SetColor(to_occ_color('yellow'))
            else:
                line = AIS_Line(
                    Geom_CartesianPoint(start.x, start.y, start.z),
                    Geom_CartesianPoint(end.x, end.y, end.z)
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

            x = cx
            y = cy
            z = cz

    return group
