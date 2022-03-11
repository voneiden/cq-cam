from typing import Optional, List

import cadquery as cq
from enum import Enum

import numpy as np
from cq_cam.utils.utils import WireClipper, wire_to_ordered_edges


class Tabs:
    height: float

    def __init__(self,
                 height: float,
                 width: float,
                 count: Optional[int] = None,
                 distance: Optional[float] = None,
                 positions: Optional[List[float]] = None):
        from cq_cam.operations.base_operation import OperationError # TODO
        self.height = height
        self.width = width
        self.count = count
        self.distance = distance
        self.positions = positions or []

        if count is None and distance is None and not positions:
            raise OperationError('At least one of "count", "distance" or "positions" must be defined')

        if count is not None and distance is not None:
            raise OperationError('Only one of "count" and "distance" may be defined')

        if distance is not None and distance <= 0:
            raise OperationError('Distance must be greater than zero')

        if count is not None and count < 1:
            raise OperationError('Count must be 1 or more')

    def process(self, wire: cq.Wire):
        center = wire.Center()
        length = wire.Length()
        half = self.width / 2
        half_d = 1 / length * half
        step_d = 1 / length * 0.1
        outer = wire.offset2D(0.5, 'intersection')[0]
        inner = wire.offset2D(-0.5)[0]
        # Use Wire.locationAt ?
        # Determine top center somehow
        # Each location will produce a segment of desired width
        # Convert into a rectangle
        # Use clipper to break the wire

        # TODO convert self.distance to count
        positions = []
        clip_polygons = []
        if self.count:
            for d in np.linspace(0, 1, self.count, endpoint=False):
                ds = np.arange(d - half_d, d + half_d + step_d, step_d)
                ds = [d % 1 for d in ds]
                ds_inner = [(-d + 0.25) % 1 for d in ds]
                vecs = wire.positions(ds)
                tangents = [wire.tangentAt(d) for d in ds]
                for tangent in tangents:
                    tangent.x, tangent.y = tangent.y, -tangent.x
                outer_vecs = [v.add(t.multiply(0.1)) for v, t in zip(vecs, tangents)]
                inner_vecs = [v.sub(t.multiply(0.1)) for v, t in zip(vecs, tangents)]
                outer_vecs = outer.positions(ds)
                inner_vecs = inner.positions(ds_inner)
                outer_t = [(v.x, v.y) for v in outer_vecs]
                inner_t = [(v.x, v.y) for v in inner_vecs]
                inner_t.reverse()
                polygon = outer_t + inner_t + [outer_t[0]]
                clip_polygons.append(polygon)

        clipper = WireClipper()
        for clip_polygon in clip_polygons:
            clipper.add_clip_polygon(clip_polygon, True)

        clipper.add_subject_wire(wire, is_closed=False)
        paths = clipper.execute()
        limit_paths = clipper.execute_difference()
        show_object(outer, 'outer')
        show_object(inner, 'inner')
        for i, path in enumerate(clip_polygons):
            wp = cq.Workplane().moveTo(*path[0])
            for p in path[1:]:
                wp = wp.lineTo(*p)
            show_object(wp.close(), f'clip-{i}')

        for i, path in enumerate(paths):
            wp = cq.Workplane().moveTo(*path[0])
            for p in path[1:]:
                wp = wp.lineTo(*p)
            show_object(wp.close(), f'path-{i}')

        for i, path in enumerate(limit_paths):
            wp = cq.Workplane().moveTo(*path[0])
            for p in path[1:]:
                wp = wp.lineTo(*p)
            show_object(wp.close(), f'limit-path-{i}')
        print(paths)



class EdgeTabs:
    def __init__(self, spacing: float, width: float = 2, only=None):
        self.edges_ds = None
        self.spacing = spacing
        self.only = only
        self.width = width

    def load_ordered_edges(self, ordered_edges):
        self.edges_ds = []
        half = self.width / 2
        for edge in ordered_edges:
            length = edge.Length()
            edge_ds = []
            self.edges_ds.append(edge_ds)
            geom_type = edge.geomType()
            if self.only:
                if geom_type != self.only:
                    continue
            count = int(length // self.spacing)
            if count <= 0:
                continue

            half_d = 1 / length * half
            for d in np.linspace(0, 1, count + 2, endpoint=True)[1:-1]:
                edge_ds.append((d - half_d, d + half_d))

    def edge_tab_transitions(self, edge_index):
        edge_ds = self.edges_ds[edge_index]
        transitions = [(0, Transition.NORMAL)]
        for tab_start_d, tab_end_d in edge_ds:
            transitions.append((tab_start_d, Transition.TAB))
            transitions.append((tab_end_d, Transition.NORMAL))
        transitions.append((1, Transition.NORMAL))
        return transitions


class Transition(Enum):
    NORMAL = 0
    TAB = 1
    IGNORE = 2


class WireTabs:
    def __init__(self):
        self.edge_lengths = None
        self.edge_ranges = None
        self.wire_tab_ds = None

    def wire_tab_count(self, wire, count, width=2):
        length = wire.Length()
        half = width / 2
        half_d = 1 / length * half

        wire_tab_ds = []

        for d in np.linspace(0, 1, count, endpoint=False):
            range = (d - half_d, d + half_d)
            wire_tab_ds.append(range)

            # Handle edge cases of ranges crossing 0 or 1 by adding extra range(s)
            if range[0] < 0:
                wire_tab_ds.append((range[0] + 1, range[1] + 1))
            if range[1] > 1:
                wire_tab_ds.append((range[0] - 1, range[1] - 1))

        self.wire_tab_ds = wire_tab_ds

    def load_ordered_edges(self, ordered_edges):
        edge_lengths = [edge.Length() for edge in ordered_edges]
        wire_length = sum(edge_lengths)
        edge_ranges = []
        previous_range_end = 0
        for edge_length in edge_lengths:
            edge_fraction = edge_length / wire_length
            edge_range_end = previous_range_end + edge_fraction
            edge_ranges.append((previous_range_end, min(edge_range_end, 1)))
            previous_range_end = edge_range_end

        self.edge_lengths = edge_lengths
        self.edge_ranges = edge_ranges

    def edge_tab_transitions(self, edge_index):
        edge_range = self.edge_ranges[edge_index]
        edge_start_d, edge_end_d = edge_range
        transitions = []
        for tab_start_d, tab_end_d in self.wire_tab_ds:
            # Tab crosses start
            if tab_start_d <= edge_start_d < tab_end_d:
                transitions.append((0, Transition.TAB))

            # Tab starts inside edge
            if edge_start_d < tab_start_d <= edge_end_d:
                edge_d = min(max(self.wire_d_to_edge_d(tab_start_d, edge_range), 0), 1)
                transitions.append((edge_d, Transition.TAB))

            # Tab ends inside edge
            if edge_start_d < tab_end_d <= edge_end_d:
                edge_d = min(max(self.wire_d_to_edge_d(tab_end_d, edge_range), 0), 1)
                transitions.append((edge_d, Transition.NORMAL))

        # Add helper transitions for later processing
        if not transitions or transitions[0][0] != 0:
            transitions.insert(0, (0, Transition.NORMAL))
        if transitions[-1][0] != 1:
            transitions.append((1, Transition.IGNORE))
        return transitions

    @staticmethod
    def wire_edge_d_ranges(wire: cq.Wire):
        wire_length = wire.Length()
        edges = wire_to_ordered_edges(wire)
        edge_lengths = [edge.Length() for edge in edges]
        edge_ranges = []
        previous_range_end = 0
        for edge_length in edge_lengths:
            edge_fraction = edge_length / wire_length
            edge_range_end = previous_range_end + edge_fraction
            edge_ranges.append((previous_range_end, min(edge_range_end, 1)))
            previous_range_end = edge_range_end

        return edges, edge_ranges

    @staticmethod
    def wire_d_to_edge_d(wire_d, edge_range):
        return (wire_d - edge_range[0]) / (edge_range[1] - edge_range[0])


# TODO WireTabs EdgeTabs+ .,n
if 'show_object' in locals() or __name__ == '__main__':
    box = cq.Workplane().box(5, 5, 5)
    bottom = box.wires('<Z')
    tabs = Tabs(1, 1, 4)
    tabs.process(bottom.objects[0])
    show_object(box, 'box')
