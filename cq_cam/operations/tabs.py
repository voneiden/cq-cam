from enum import Enum

import cadquery as cq
import numpy as np

from cq_cam.utils.utils import wire_to_ordered_edges


class Transition(Enum):
    NORMAL = 0
    TAB = 1
    IGNORE = 2


class Tabs:
    def __init__(self, *, width: float, height: float):
        self.width = width
        self.height = height

    def load_ordered_edges(self, ordered_edges):
        raise NotImplementedError()

    def edge_tab_transitions(self, edge_index):
        raise NotImplementedError()


class NoTabs(Tabs):
    def __init__(self):
        super().__init__(width=0, height=0)

    def load_ordered_edges(self, ordered_edges):
        pass

    def edge_tab_transitions(self, edge_index):
        return [
            (0, Transition.NORMAL),
            (1, Transition.NORMAL)
        ]


class EdgeTabs(Tabs):
    def __init__(self, *, spacing: float, width: float, height: float, only=None):
        super().__init__(width=width, height=height)
        self.edges_ds = None
        self.spacing = spacing
        self.only = only

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


class WireTabs(Tabs):
    def __init__(self, *, count: int, width: float, height: float):
        super().__init__(width=width, height=height)
        self.edge_lengths = None
        self.edge_ranges = None
        self.wire_tab_ds = None
        self.count = count
        self.width = width

    def load_wire(self, wire):
        length = wire.Length()
        half = self.width / 2
        half_d = 1 / length * half

        wire_tab_ds = []

        for d in np.linspace(0, 1, self.count, endpoint=False):
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
