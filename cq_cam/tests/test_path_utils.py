import unittest
import cadquery as cq

from cq_cam.utils.path_utils import path_i_and_point_at_d, find_closest_in_path, ClipperPathSimulator


class TestPathUtils(unittest.TestCase):
    def test_path_i_and_point_at_d(self):
        path = [
            cq.Vector(0, 0, 0),
            cq.Vector(10, 0, 0),
            cq.Vector(10, 10, 0),
            cq.Vector(0, 10, 0),
            cq.Vector(0, 0, 0)
        ]
        i, p = path_i_and_point_at_d(path, 0.25)
        self.assertEqual(1, i)
        self.assertEqual(cq.Vector(10, 0, 0), p)

        i, p = path_i_and_point_at_d(path, 0.125)
        self.assertEqual(0, i)
        self.assertEqual(cq.Vector(5, 0, 0), p)

        i, p = path_i_and_point_at_d(path, 0.625)
        self.assertEqual(2, i)
        self.assertEqual(cq.Vector(5, 10, 0), p)

    def test_find_closest_in_path(self):
        closest = find_closest_in_path(cq.Vector(0, 0, 0), [
            cq.Vector(4, 2, 0),
            cq.Vector(2, 2, 0),
            cq.Vector(2, 4, 0),
            cq.Vector(4, 4, 0),
            cq.Vector(4, 2, 0)
        ])
        self.assertEqual((cq.Vector(2, 2, 0), 8.0, 0.25, 1), closest)


    def test_clipper_path_simulator(self):
        cps = ClipperPathSimulator(tool_radius=0.5)
