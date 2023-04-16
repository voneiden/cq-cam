from cq_cam.utils.geometry_op import distance_to_polygon
from cq_cam.utils.utils import dist_to_segment_squared


def test_dist_to_segment_squared():
    point = (0, 1)
    segment_start = (10, 0)
    segment_end = (-10, 0)
    assert dist_to_segment_squared(point, segment_start, segment_end) == (1, (0, 0))

    segment_start = (0, 0)
    assert dist_to_segment_squared(point, segment_start, segment_end) == (1, (0, 0))

    segment_start = (-1, 0)
    assert dist_to_segment_squared(point, segment_start, segment_end) == (2, (-1, 0))

    segment_end = (0.5, 1.5)
    assert dist_to_segment_squared(point, segment_start, segment_end) == (0, (0, 1))

    segment_end = (1, 1.5)
    a, b = dist_to_segment_squared(point, segment_start, segment_end)
    assert a, (round(b[0], 2), round(b[1], 2)) == (0.04, (0.12, 0.84))


def test_distance_to_polygon():
    point = (0, 1)
    polygon = [(-1, 0), (0, 0), (1, 1), (1, 2)]
    distance, closest_point, poly_position = distance_to_polygon(point, polygon)

    assert distance == 0.5
    assert closest_point == (0.5, 0.5)
    assert poly_position == (1, 0.5)
