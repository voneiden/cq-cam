from OCP.AIS import AIS_MultipleConnectedInteractive, AIS_Line
from OCP.Geom import Geom_CartesianPoint
from cq_editor.cq_utils import to_occ_color

from base import Task, Job, Plunge, Cut, Rapid


class VisualizeError(Exception):
    pass


def visualize_task(job: Job, task: Task):
    root_workplane = job.workplane
    root_plane = root_workplane.plane

    rapids = AIS_MultipleConnectedInteractive()
    cuts = AIS_MultipleConnectedInteractive()
    plunges = AIS_MultipleConnectedInteractive()

    x = 0
    y = 0
    z = job.rapid_height
    last_plunge = True
    for command in task.commands:
        if isinstance(command, Rapid):
            group = rapids
        elif isinstance(command, Cut):
            group = cuts
        elif isinstance(command, Plunge):
            group = plunges
        else:
            raise VisualizeError("Unknown command instance")

        cx = x if (cx := getattr(command, 'x', None)) is None else cx
        cy = y if (cy := getattr(command, 'y', None)) is None else cy
        cz = z if (cz := getattr(command, 'z', None)) is None else cz

        start = root_plane.toWorldCoords((x, y, z))
        end = root_plane.toWorldCoords((cx, cy, cz))

        line = AIS_Line(
            Geom_CartesianPoint(start.x, start.y, start.z),
            Geom_CartesianPoint(end.x, end.y, end.z)
        )
        if isinstance(command, Rapid):
            line.SetColor(to_occ_color('green'))
        elif isinstance(command, Cut):
            line.SetColor(to_occ_color('red'))
        elif isinstance(command, Plunge):
            line.SetColor(to_occ_color('blue'))

        if isinstance(command, Plunge):
            line.Attributes().SetLineArrowDraw(True)
            last_plunge = True
        elif last_plunge:
            line.Attributes().SetLineArrowDraw(True)
            last_plunge = False

        group.Connect(line)

        x = cx
        y = cy
        z = cz

    return rapids, cuts, plunges
