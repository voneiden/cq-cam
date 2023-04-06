from cq_cam.utils.offset import calculate_offset


def test_calculate_offset():
    assert calculate_offset(1, 1) == 1
    assert calculate_offset(1, (1, 1)) == 2
    assert calculate_offset(1, None, 3) == 3
    assert calculate_offset(1, -1) == -1
    assert calculate_offset(1, (-1, 0.5)) == -0.5
    assert calculate_offset(1, (0, 1.05)) == 1.05
