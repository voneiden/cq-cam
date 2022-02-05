from cadquery import cq

from cq_cam.commands.base_command import CommandSequence
from cq_cam.commands.command import Cut, CircularCW, CircularCCW
from cq_cam.utils import (
    wire_to_ordered_edges,
    start_point, orient_vector,
    end_point,
    is_arc_clockwise,
)


def wire_to_command_sequence(wire: cq.Wire, plane: cq.Plane) -> CommandSequence:
    """
    Convert a wire into ordered sequence of commands. Type vectors
    are also transformed from job plane to XY plane.

    :param plane: job plane
    :param wire: wire to convert
    :return:
    """
    ordered_edges = wire_to_ordered_edges(wire)
    commands = []

    sequence_start = orient_vector(start_point(ordered_edges[0]), plane)
    sequence_end = orient_vector(start_point(ordered_edges[-1]), plane)

    for edge in ordered_edges:
        command_end = orient_vector(end_point(edge), plane)
        if edge.geomType() == "LINE":
            commands.append(Cut(command_end))

        elif edge.geomType() in ["CIRCLE"]:
            command_start = orient_vector(start_point(edge), plane)
            command_mid = orient_vector(edge.positionAt(0.5), plane)
            command_center = orient_vector(edge.arcCenter(), plane)
            radius = abs(command_start.sub(command_center))

            if command_start.x == command_end.x and command_start.y == command_end.y:
                raise NotImplemented('Full circles are not implemented')

            if is_arc_clockwise(command_start, command_mid, command_end):
                commands.append(CircularCW(command_end, radius))
            else:
                commands.append(CircularCCW(command_end, radius))

        elif edge.geomType() == 'ARC':
            raise NotImplemented('ARC geom type is not implemented')

        elif edge.geomType() == 'SPLINE':
            raise NotImplemented('SPLINE geom type is not implemented')

        else:
            raise NotImplemented(f'Unknown geom type "{edge.geomType()}"')

    return CommandSequence(sequence_start, commands, sequence_end)
