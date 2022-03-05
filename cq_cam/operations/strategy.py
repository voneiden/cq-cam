from random import random
from typing import List, Tuple, Dict, Iterable

import cadquery as cq
import numpy as np

from cq_cam.commands.base_command import CommandSequence
from cq_cam.commands.util_command import wire_to_command_sequence, wire_to_command_sequence2
from cq_cam.job import Job
from cq_cam.operations.base_operation import OperationError
from cq_cam.utils.linked_polygon import LinkedPolygon
from cq_cam.utils.utils import WireClipper, pairwise, dist_to_segment_squared, flatten_list

Scanpoint = Tuple[float, float]
Scanline = List[Scanpoint]


class ZigZagStrategy:

    @classmethod
    def process(cls, task, outer_boundaries: List, inner_boundaries: List):
        # TODO: Scanline orientation
        # Here we could rotate the regions so that we can keep the scanlines in standard XY plane
        clipper = WireClipper(cq.Workplane().plane)

        outer_polygons = []
        for outer_boundary in outer_boundaries:
            polygon = clipper.add_clip_wire(outer_boundary)
            outer_polygons.append(polygon)

        inner_polygons = []
        for inner_boundary in inner_boundaries:
            polygon = clipper.add_clip_wire(inner_boundary)
            inner_polygons.append(polygon)

        max_bounds = clipper.max_bounds()

        # Generate ZigZag scanlines
        y_scanpoints = list(np.arange(max_bounds['bottom'], max_bounds['top'], task._tool_diameter * task.stepover))
        scanline_templates = [((max_bounds['left'], y), (max_bounds['right'], y)) for y in y_scanpoints]

        for scanline_template in scanline_templates:
            clipper.add_subject_polygon(scanline_template)


        scanlines = clipper.execute()

        scanpoint_to_scanline, scanpoints = cls._scanline_end_map(scanlines)

        linked_polygons, scanpoint_to_linked_polygon = cls._link_scanpoints_to_boundaries(
            scanpoints, outer_polygons + inner_polygons)

        cut_sequences = cls._route_zig_zag(linked_polygons,
                                            scanlines,
                                            scanpoint_to_linked_polygon,
                                            scanpoint_to_scanline)
        return cut_sequences

    @staticmethod
    def _scanline_end_map(scanlines: List[Scanline]):
        scanline_end_map = {}
        scanpoints = []
        for scanline in scanlines:
            scanline_start, scanline_end = scanline[0], scanline[-1]
            scanline_end_map[scanline_start] = scanline
            scanline_end_map[scanline_end] = scanline
            scanpoints.append(scanline_start)
            scanpoints.append(scanline_end)
        return scanline_end_map, scanpoints


    @staticmethod
    def _link_scanpoints_to_boundaries(scanpoints: List[Scanpoint],
                                       boundaries: List[List[Tuple[float, float]]]):
        remaining_scanpoints = scanpoints[:]
        scanpoint_to_linked_polygon = {}
        linked_polygons = []
        for polygon in boundaries:
            linked_polygon = LinkedPolygon(polygon[:])
            linked_polygons.append(linked_polygon)
            for p1, p2 in pairwise(polygon):
                for scanpoint in remaining_scanpoints[:]:
                    d = dist_to_segment_squared(scanpoint, p1, p2)
                    # Todo pick a good number. Tests show values between 1.83e-19 and 1.38e-21
                    if d < 0.0000001:
                        remaining_scanpoints.remove(scanpoint)
                        linked_polygon.link_point(scanpoint, p1, p2)
                        scanpoint_to_linked_polygon[scanpoint] = linked_polygon

        assert not remaining_scanpoints
        return linked_polygons, scanpoint_to_linked_polygon

    @staticmethod
    def _route_zig_zag(linked_polygons: List[LinkedPolygon],
                       scanlines: Tuple[Tuple[Tuple[float, float], Tuple[float, float]]],
                       scanpoint_to_linked_polygon: Dict[Tuple[float, float], LinkedPolygon],
                       scanpoint_to_scanline: Dict[Tuple[float, float], List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:

        # Prepare to route the zigzag
        for linked_polygon in linked_polygons:
            linked_polygon.reset()

        scanlines = list(scanlines)

        # Pick a starting position. Clipper makes no guarantees about the orientation
        # of polylines it returns, so figure the top left scanpoint as the
        # starting position.
        starting_scanline = scanlines.pop(0)
        start_position, cut_position = starting_scanline
        if start_position[0] > cut_position[0]:
            start_position, cut_position = cut_position, start_position

        scanpoint_to_linked_polygon[start_position].drop(start_position)
        cut_sequence = [start_position, cut_position]
        cut_sequences = []

        # Primary routing loop
        while scanlines:
            linked_polygon = scanpoint_to_linked_polygon[cut_position]
            path = linked_polygon.nearest_linked(cut_position)
            if path is None:
                cut_sequences.append(cut_sequence)
                # TODO some optimization potential in picking the nearest scanpoint
                start_position, cut_position = scanlines.pop(0)
                # TODO some optimization potential in picking a direction
                cut_sequence = [start_position, cut_position]
                continue

            cut_sequence += path
            scanline = scanpoint_to_scanline[path[-1]]
            cut_sequence.append(scanline[1] if scanline[0] == path[-1] else scanline[0])
            cut_position = cut_sequence[-1]
            scanlines.remove(scanline)

        cut_sequences.append(cut_sequence)
        return cut_sequences

ContourDefinition = Tuple[cq.Wire, List['ContourDefinition']]

class ContourStrategy:
    """
    Contour strategy uses the outer boundary to generate incrementally shrinking contours.
    """
    #class Contour:

    @classmethod
    def process(cls, task, outer_boundaries: List[cq.Wire], inner_boundaries: List[cq.Wire], show_object) -> List[ContourDefinition]:
        # We generate shrinking contours from all the outer boundaries
        offset_step = -abs(task._tool_diameter * task.stepover)
        # TODO we gotta clip them!

        cut_groups: List[ContourDefinition] = []
        for obi, outer_boundary in enumerate(outer_boundaries):
            show_object(outer_boundary, f"outerboundary-{obi}-{random()}")
            root = (outer_boundary, [])
            cut_groups.append(root)

            queue = [root]
            while queue:
                contour, sub_contours = queue.pop(0)
                new_sub_contours = contour.offset2D(offset_step)
                for new_sub_contour in new_sub_contours:
                    new_sub_contour_definition = (new_sub_contour, [])
                    sub_contours.append(new_sub_contour_definition)
                    queue.append(new_sub_contour_definition)

        return cut_groups

    @staticmethod
    def flatten(cut_groups: List[ContourDefinition], job: Job, show_object) -> List[CommandSequence]:
        # wire_to_command_sequence

        def _wires(cd: ContourDefinition):
            c, scs = cd
            return [c, *flatten_list([_wires(sc) for sc in scs])]
        wires = flatten_list([_wires(cut_group) for cut_group in cut_groups])
        for i, wire in enumerate(wires):
            show_object(wire, f"wire-{i}-{random()}")
        return [wire_to_command_sequence2(wire) for wire in wires]
