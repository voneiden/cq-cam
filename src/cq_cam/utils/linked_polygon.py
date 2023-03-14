from dataclasses import dataclass, field
from typing import List, Tuple, Optional

from cq_cam.utils.utils import cached_dist2


@dataclass
class LinkedPolygon:
    polygon: List[Tuple[float, float]]
    linked_points: List[Tuple[float, float]] = field(default_factory=list)
    _linked_points: List[Tuple[float, float]] = field(default_factory=list)

    def link_point(self, point: Tuple[float, float], segment_start: Tuple[float, float],
                   segment_end: Tuple[float, float]):

        assert point not in self.linked_points

        i = self.polygon.index(segment_start)
        i_1 = i + 1
        j = self.polygon.index(segment_end)

        if i_1 == j:
            self.polygon.insert(i_1, point)
        else:
            point_range = self.polygon[i_1:j]
            point_range.append(point)
            dist2_range = [(cached_dist2(p, segment_start), p) for p in point_range]
            dist2_range.sort(key=lambda k: k[0])
            point_range = [p for _, p in dist2_range]
            self.polygon.insert(i_1 + point_range.index(point), point)

        self.linked_points.append(point)

    def reset(self, start_point: Optional[Tuple[float, float]] = None):
        """
        Reset the internal state for `nearest_linked`

        :param start_point: optional starting point that will be immediately dropped
                            from candidates
        :return:
        """
        self._linked_points = self.linked_points[:]
        if start_point:
            self._linked_points.remove(start_point)

    def drop(self, point: Optional[Tuple[float, float]]):
        self._linked_points.remove(point)

    def nearest_linked(self, point: Tuple[float, float]):
        """
        Given a (unused) linked point, find the nearest other (unused) linked point
        and return the path sequence to it.

        Use `reset` to reset the unused linked points.

        :param point: entry point (must be linked!)
        :return: exit point or None if no more points remain
        """
        self._linked_points.remove(point)
        if not self._linked_points:
            return None

        i = self.polygon.index(point)
        search_polygon = self.polygon[i + 1:] + self.polygon[:i]
        # Search forward
        forward_distance = 0
        forward_sequence = []
        last_point = point
        for next_point in search_polygon:
            forward_distance += cached_dist2(last_point, next_point)
            forward_sequence.append(next_point)
            if next_point in self._linked_points:
                break

        reverse_distance = 0
        reverse_sequence = []
        last_point = point
        search_polygon.reverse()
        for next_point in search_polygon:
            # Reverse order of arguments to avoid missing cache
            reverse_distance += cached_dist2(next_point, last_point)
            reverse_sequence.append(next_point)
            if next_point in self._linked_points:
                break

        if forward_distance < reverse_distance:
            self._linked_points.remove(forward_sequence[-1])
            return forward_sequence

        self._linked_points.remove(reverse_sequence[-1])
        return reverse_sequence
