import unittest

from cq_cam.utils.linked_polygon import LinkedPolygon


class LinkedPolygonTest(unittest.TestCase):
    def test_linking_order(self):
        # Not actually a polygon but doesn't matter for the test case
        seg_start = (0, 0)
        seg_end = (10, 0)
        test_polygon = [seg_start, seg_end]

        p1 = (1, 0)
        p2 = (2, 0)
        p3 = (3, 0)
        p4 = (4, 0)
        p5 = (5, 0)

        lp = LinkedPolygon(test_polygon)
        self.assertSequenceEqual(lp.polygon, [seg_start, seg_end])

        lp.link_point(p3, seg_start, seg_end)
        self.assertSequenceEqual(lp.polygon, [seg_start, p3, seg_end])

        lp.link_point(p5, seg_start, seg_end)
        self.assertSequenceEqual(lp.polygon, [seg_start, p3, p5, seg_end])

        lp.link_point(p4, seg_start, seg_end)
        self.assertSequenceEqual(lp.polygon, [seg_start, p3, p4, p5, seg_end])

        lp.link_point(p1, seg_start, seg_end)
        self.assertSequenceEqual(lp.polygon, [seg_start, p1, p3, p4, p5, seg_end])

        lp.link_point(p2, seg_start, seg_end)
        self.assertSequenceEqual(lp.polygon, [seg_start, p1, p2, p3, p4, p5, seg_end])