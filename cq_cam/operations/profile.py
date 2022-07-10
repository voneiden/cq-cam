import itertools
from typing import List, Optional, TYPE_CHECKING

import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.Geom import Geom_Plane
from OCP.GeomProjLib import GeomProjLib

from cq_cam.operations.tabs import Tabs, Transition
from cq_cam.routers import route
from cq_cam.utils.utils import compound_to_edges, wire_to_ordered_edges

if TYPE_CHECKING:
    from cq_cam.fluent import JobV2

DEBUG = []


def profile(job: 'JobV2',
            outer_wires: List[cq.Wire] = None,
            inner_wires: List[cq.Wire] = None,
            outer_offset: float = 1,
            inner_offset: float = -1,
            stepdown: Optional[int] = None,
            tabs: Optional[Tabs] = None,
            ):
    # Transform to relative coordinates
    outers = [outer.transformShape(job.top.fG) for outer in outer_wires]
    inners = [inner.transformShape(job.top.fG) for inner in inner_wires]

    # Generate base features
    base_features = []
    for outer in outers:
        base_features += outer.offset2D(outer_offset * job.tool_diameter)

    for inner in inners:
        base_features += inner.offset2D(inner_offset * job.tool_diameter)

    toolpaths = []
    for base_feature in base_features:
        base_zmin = base_feature.BoundingBox().zmin
        tab_z = base_zmin + tabs.height if tabs else None
        layers: List[cq.Wire] = [base_feature]
        if stepdown:
            step = cq.Vector(0, 0, 1) * stepdown

            for i in itertools.count():
                if i > job.max_stepdown_count:
                    raise RuntimeError('Profile max_stepdown_count exceeded')

                i_op: cq.Wire = base_feature.moved(cq.Location(step * (i + 1)))
                if i_op.BoundingBox().zmin >= 0:
                    break

                edges = compound_to_edges(i_op.cut(job.top_plane_face))
                edges = [edge for edge in edges if edge.Center().z < 0]
                wires = cq.Wire.combine(edges)

                if not wires:
                    break

                # TODO use the wires above..
                layers.append(i_op)

            layers.reverse()
        if tabs:
            layers = [apply_tabs(wire, tab_z, tabs) for wire in layers]

        toolpaths += layers

    commands = route(job, toolpaths)
    return commands


def apply_tabs(wire: cq.Wire, tab_z: float, tabs: Tabs):
    if tabs is None:
        return wire

    if wire.BoundingBox().zmin >= tab_z:
        return wire

    tab_plane = cq.Plane((0, 0, tab_z))

    edges = wire_to_ordered_edges(wire)
    tabs.load_ordered_edges(edges)

    new_edges = []
    for i, edge in enumerate(edges):
        transitions = tabs.edge_tab_transitions(i)
        if all(t[1] == Transition.NORMAL for t in transitions):
            new_edges.append(edge)
            continue

        previous_edge = None
        for t_start, t_end in zip(transitions, transitions[1:]):
            p1 = edge.paramAt(t_start[0])
            p2 = edge.paramAt(t_end[0])
            curve = edge._geomAdaptor().Curve().Curve()

            if t_start[1] == Transition.TAB:
                projected_curve = GeomProjLib.ProjectOnPlane_s(curve, Geom_Plane(tab_plane.toPln()),
                                                               cq.Vector(0, 0, -1).toDir(), True)
                projected_edge = cq.Edge(BRepBuilderAPI_MakeEdge(projected_curve, p1, p2).Edge())
                if previous_edge:
                    # Retract
                    retract = cq.Edge.makeLine(previous_edge.endPoint(), projected_edge.startPoint())
                    new_edges.append(retract)

                previous_edge = projected_edge
                new_edges.append(projected_edge)
            else:
                sliced_edge = cq.Edge(BRepBuilderAPI_MakeEdge(curve, p1, p2).Edge())
                # The location of the edge needs to be copied because it doesn't
                # affect the underlying curve..
                sliced_edge.move(edge.location())
                # Plunge
                if previous_edge:
                    plunge = cq.Edge.makeLine(previous_edge.endPoint(), sliced_edge.startPoint())
                    new_edges.append(plunge)

                previous_edge = sliced_edge
                new_edges.append(sliced_edge)

    new_wire = cq.Wire.assembleEdges(new_edges)
    return new_wire
