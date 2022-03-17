import math
from typing import Optional, Tuple, TYPE_CHECKING

from cadquery import cq

from cq_cam.operations.tabs import WireTabs, EdgeTabs, NoTabs, Tabs
from cq_cam.utils.utils import (
    wire_to_ordered_edges,
    start_point, orient_vector,
    end_point,
    is_arc_clockwise,
)

if TYPE_CHECKING:
    from cq_cam.commands.base_command import CommandSequence, Circular


def wire_to_command_sequence(wire: cq.Wire, plane: cq.Plane) -> 'CommandSequence':
    """
    Convert a wire into ordered sequence of commands. Type vectors
    are also transformed from job plane to XY plane.

    :param plane: job plane
    :param wire: wire to convert
    :return:
    """
    print("wire_to_command_sequence is deprecated")
    from cq_cam.commands.base_command import CommandSequence
    from cq_cam.commands.command import Cut, CircularCW, CircularCCW

    ordered_edges = wire_to_ordered_edges(wire)
    commands = []

    sequence_start = orient_vector(start_point(ordered_edges[0]), plane)
    sequence_end = orient_vector(end_point(ordered_edges[-1]), plane)

    for edge in ordered_edges:
        # TODO refactor the use of orient_vector to shapeTransform
        command_end = orient_vector(end_point(edge), plane)
        if edge.geomType() == "LINE":
            commands.append(Cut(command_end.x, command_end.y, None))

        elif edge.geomType() in ["CIRCLE"]:
            # TODO put some safe lower limit for arc length
            # grbl for example can do completely unexpected things
            # with tiny arcs
            command_start = orient_vector(start_point(edge), plane)
            command_mid = orient_vector(edge.positionAt(0.5), plane)
            command_center = orient_vector(edge.arcCenter(), plane)
            # radius = abs(command_start.sub(command_center))
            ijk = command_center.sub(command_start)

            if command_start.x == command_end.x and command_start.y == command_end.y:
                raise NotImplemented('Full circles are not implemented')

            if is_arc_clockwise(command_start, command_mid, command_end):
                commands.append(CircularCW(command_end.x, command_end.y, None, None, vector_to_tuple(ijk),
                                           vector_to_tuple(command_mid)))
            else:
                commands.append(CircularCCW(command_end.x, command_end.y, None, None, vector_to_tuple(ijk),
                                            vector_to_tuple(command_mid)))

        elif edge.geomType() == 'ARC':
            raise NotImplemented('ARC geom type is not implemented')

        elif edge.geomType() == 'SPLINE':
            raise NotImplemented('SPLINE geom type is not implemented')

        else:
            raise NotImplemented(f'Unknown geom type "{edge.geomType()}"')

    return CommandSequence(sequence_start, commands, sequence_end)


def wire_to_command_sequence2(wire: cq.Wire, tabs: Tabs = NoTabs) -> 'CommandSequence':
    """
    Convert a wire into ordered sequence of commands.
    """
    from cq_cam.commands.base_command import CommandSequence, Circular
    from cq_cam.commands.command import Cut

    ordered_edges = wire_to_ordered_edges(wire)
    commands = []

    sequence_start = start_point(ordered_edges[0])
    sequence_end = end_point(ordered_edges[-1])

    if isinstance(tabs, WireTabs):
        tabs.load_wire(wire)

    tabs.load_ordered_edges(ordered_edges)

    for edge_i, edge in enumerate(ordered_edges):
        edge_transitions = tabs.edge_tab_transitions(edge_i)

        if edge.geomType() == "LINE":
            commands += Cut.from_edge(edge, edge_transitions)

        elif edge.geomType() in ["CIRCLE"]:
            # TODO put some safe lower limit for arc length
            # TODO support 3d arcs
            # grbl for example can do completely unexpected things
            # with tiny arcs
            commands += Circular.from_edge(edge, edge_transitions)

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
    return same_to_none(end.x, start.x), same_to_none(end.y, start.y), same_to_none(end.z, start.z)


def arc_center_midpoint(command: 'Circular', start: cq.Vector, end: Optional[cq.Vector] = None):
    from cq_cam.commands.command import CircularCW

    if end is None:
        end = command.end(start)

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

    return center, midpoint


def vector_to_tuple(vector: cq.Vector) -> Tuple[float, float, float]:
    return (vector.x, vector.y, vector.z)


def equal_within_tolerance(a: float, b: float, tolerance: int):
    return abs(a - b) < (1 / (10 * tolerance))


def normalize(v: float):
    iv = int(v)
    return iv if v == iv else v
