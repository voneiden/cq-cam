import cadquery as cq
import pyclipper as pc
import pytest

from cq_cam.utils.geometry_op import offset_path, wire_to_path


def round_path(path: list[list[float]], precision: int) -> list[list[float]]:
    return [[round(x, precision), round(y, precision)] for x, y in path]


@pytest.mark.parametrize(
    ["path", "expected_path", "precision"],
    [
        [[[0, 0], [1, 0.01], [2, 0], [0, 1]], [[0, 0], [2, 0], [0, 1]], 1],
        [
            [[0, 0], [1, 0.1], [2, 0], [0, 1]],
            [[0, 0], [1, 0.1], [2, 0], [0, 1]],
            1,
        ],
        [
            [[0, 0], [1, 0.09], [1.1, -0.09], [2, 0], [0, 1]],
            [[0, 0], [1, 0.1], [1.1, -0.1], [2, 0], [0, 1]],
            1,
        ],
        [
            [[0, 0], [1, 0.05], [1.01, 0.1], [2, 0], [0, 1]],
            [[0, 0], [1, 0.1], [2, 0], [0, 1]],
            1,
        ],
    ],
)
def test_clean_precision(path, expected_path, precision):
    """
    This is a sanity test to determine how CleanPolygon actually works
    """
    scaled_path = pc.scale_to_clipper(path)
    scaled_precision = pc.scale_to_clipper(10 ** (-precision))
    cleaned_path = pc.CleanPolygon(scaled_path, scaled_precision)
    final_path = pc.scale_from_clipper(cleaned_path)
    rounded_path = round_path(final_path, precision)
    assert rounded_path == expected_path


def test_offset_path_precision():
    path = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
    precision_1 = round_path(offset_path(path, 0.5, 1)[0], 1)

    """
    # For debug
    from matplotlib import pyplot as plt
    plt.plot(*zip(*path))
    plt.plot(*zip(*precision_1))
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    plt.show()
    """

    assert [
        [1.4, -0.3],
        [1.5, 0.0],
        [1.5, 1.0],
        [1.3, 1.4],
        [1.0, 1.5],
        [0.0, 1.5],
        [-0.4, 1.3],
        [-0.5, 1.0],
        [-0.5, 0.0],
        [-0.3, -0.4],
        [0.0, -0.5],
        [1.0, -0.5],
        [1.4, -0.3],
    ] == precision_1

    precision_2 = round_path(offset_path(path, 0.5, 2)[0], 2)
    assert [
        [1.14, -0.48],
        [1.27, -0.42],
        [1.38, -0.33],
        [1.45, -0.21],
        [1.49, -0.08],
        [1.5, 0.0],
        [1.5, 1.0],
        [1.48, 1.14],
        [1.42, 1.27],
        [1.33, 1.38],
        [1.21, 1.45],
        [1.08, 1.49],
        [1.0, 1.5],
        [0.0, 1.5],
        [-0.14, 1.48],
        [-0.27, 1.42],
        [-0.38, 1.33],
        [-0.45, 1.21],
        [-0.49, 1.08],
        [-0.5, 1.0],
        [-0.5, 0.0],
        [-0.48, -0.14],
        [-0.42, -0.27],
        [-0.33, -0.38],
        [-0.21, -0.45],
        [-0.08, -0.49],
        [0.0, -0.5],
        [1.0, -0.5],
        [1.14, -0.48],
    ] == precision_2


def test_interpolation_precision():
    wire = cq.Workplane().circle(0.5).objects[0]
    precision_1 = wire_to_path(wire, 1)
    assert [
        [0.5, 0.0],
        [0.5, 0.1],
        [0.5, 0.2],
        [0.4, 0.3],
        [0.3, 0.4],
        [0.3, 0.4],
        [0.2, 0.5],
        [0.1, 0.5],
        [-0.1, 0.5],
        [-0.2, 0.5],
        [-0.2, 0.4],
        [-0.3, 0.4],
        [-0.4, 0.3],
        [-0.5, 0.2],
        [-0.5, 0.1],
        [-0.5, 0.0],
        [-0.5, -0.1],
        [-0.5, -0.2],
        [-0.4, -0.3],
        [-0.3, -0.4],
        [-0.3, -0.4],
        [-0.2, -0.5],
        [-0.1, -0.5],
        [0.1, -0.5],
        [0.2, -0.5],
        [0.3, -0.4],
        [0.3, -0.4],
        [0.4, -0.3],
        [0.5, -0.2],
        [0.5, -0.1],
        [0.5, -0.0],
    ] == round_path(precision_1, 1)

    assert 31 == len(precision_1)

    precision_2 = wire_to_path(wire, 2)
    assert 314 == len(precision_2)

    precision_3 = wire_to_path(wire, 3)
    assert 3141 == len(precision_3)

    precision_4 = wire_to_path(wire, 4)
    assert 31415 == len(precision_4)

    # A wild PI appears!
