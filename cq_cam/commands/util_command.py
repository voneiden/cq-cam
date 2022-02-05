from typing import Optional, Tuple, TYPE_CHECKING

from cadquery import cq

from cq_cam.utils import (
    wire_to_ordered_edges,
    start_point, orient_vector,
    end_point,
    is_arc_clockwise,
)

if TYPE_CHECKING:
    from cq_cam.commands.base_command import CommandSequence


def wire_to_command_sequence(wire: cq.Wire, plane: cq.Plane) -> 'CommandSequence':
    """
    Convert a wire into ordered sequence of commands. Type vectors
    are also transformed from job plane to XY plane.

    :param plane: job plane
    :param wire: wire to convert
    :return:
    """
    from cq_cam.commands.base_command import CommandSequence
    from cq_cam.commands.command import Cut, CircularCW, CircularCCW

    ordered_edges = wire_to_ordered_edges(wire)
    commands = []

    sequence_start = orient_vector(start_point(ordered_edges[0]), plane)
    sequence_end = orient_vector(start_point(ordered_edges[-1]), plane)

    for edge in ordered_edges:
        command_end = orient_vector(end_point(edge), plane)
        if edge.geomType() == "LINE":
            commands.append(Cut(command_end.x, command_end.y, None))

        elif edge.geomType() in ["CIRCLE"]:
            command_start = orient_vector(start_point(edge), plane)
            command_mid = orient_vector(edge.positionAt(0.5), plane)
            command_center = orient_vector(edge.arcCenter(), plane)
            radius = abs(command_start.sub(command_center))

            if command_start.x == command_end.x and command_start.y == command_end.y:
                raise NotImplemented('Full circles are not implemented')

            if is_arc_clockwise(command_start, command_mid, command_end):
                commands.append(CircularCW(command_end.x, command_end.y, None, radius))
            else:
                commands.append(CircularCCW(command_end.x, command_end.y, None, radius))

        elif edge.geomType() == 'ARC':
            raise NotImplemented('ARC geom type is not implemented')

        elif edge.geomType() == 'SPLINE':
            raise NotImplemented('SPLINE geom type is not implemented')

        else:
            raise NotImplemented(f'Unknown geom type "{edge.geomType()}"')

    return CommandSequence(sequence_start, commands, sequence_end)


def same_to_none(result: float, compare: float) -> Optional[float]:
    if result == compare:
        return None
    return result


def vector_same_to_none(end: cq.Vector, start: cq.Vector) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    return same_to_none(end.x, start.y), same_to_none(end.y, start.y), same_to_none(end.z, start.z)
