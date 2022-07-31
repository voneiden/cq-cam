import unittest
import cadquery as cq

from cq_cam.routers import ContourChain
from cq_cam.utils.path_utils import path_i_and_point_at_d, find_closest_in_path


class TestRouters(unittest.TestCase):
    def test_route_contour_chain(self):
        offset = 5
        #ContourChain()
