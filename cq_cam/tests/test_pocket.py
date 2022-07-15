import unittest
import cadquery as cq

from cq_cam import JobV2
from cq_cam.operations.pocket import pocket


class TestPocket(unittest.TestCase):
    def setUp(self):
        self.job_wp = cq.Workplane()
        self.job = JobV2(self.job_wp.plane, 300, tool_diameter=1)

    def test_simple_pocket(self):
        face = cq.Face.makePlane(5, 5, (0, 0, -1))
        result = pocket(self.job, [face])

        self.assertEqual(face.Wires()[0].Length(), 20)
        self.assertEqual(len(result), 3)
        self.assertEqual(16, result[0].Length())
        self.assertEqual(10, result[1].Length())
        self.assertEqual(4, result[2].Length())

