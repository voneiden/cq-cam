import unittest
from unittest.mock import patch

from cadquery import cq, Edge

from cq_cam.job import Job
from cq_cam.commands.base_command import Unit
from cq_cam.operations.profile import Profile
from cq_cam.visualize import visualize_task, Geom_CartesianPoint


def infinite_generator(f):
    while True:
        yield f()


class ProfileTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_box(self):
        """
        Big Integration Test
        """
        box = cq.Workplane().box(10, 10, 10)
        box_top = box.faces('<Z').workplane()
        self.maxDiff = None
        job = Job(workplane=box_top, feed=300, plunge_feed=50, unit=Unit.METRIC, rapid_height=10)
        profile = Profile(job, o=box.faces('>Z'), clearance_height=5, top_height=0, tool_diameter=4, stepdown=-7.5)
        make_three_point_arc = Edge.makeThreePointArc
        geom_cartesian_point = Geom_CartesianPoint
        with patch('cq_cam.visualize.Edge.makeThreePointArc',
                   side_effect=lambda *args: make_three_point_arc(*args)) as arc, \
                patch('cq_cam.visualize.Geom_CartesianPoint',
                      side_effect=lambda *args: geom_cartesian_point(*args)) as cartesian_point:
            visualize_task(job, profile)

        expected_arcs = (
            ((-9.0, -5.0, 2.5), (-7.83, -7.83, 2.5), (-5.0, -9.0, 2.5)),
            ((5.0, -9.0, 2.5), (7.83, -7.83, 2.5), (9.0, -5.0, 2.5)),
            ((9.0, 5.0, 2.5), (7.83, 7.83, 2.5), (5.0, 9.0, 2.5)),
            ((-5.0, 9.0, 2.5), (-7.83, 7.83, 2.5), (-9.0, 5.0, 2.5)),
            ((-9.0, -5.0, 5.0), (-7.83, -7.83, 5.0), (-5.0, -9.0, 5.0)),
            ((5.0, -9.0, 5.0), (7.83, -7.83, 5.0), (9.0, -5.0, 5.0)),
            ((9.0, 5.0, 5.0), (7.83, 7.83, 5.0), (5.0, 9.0, 5.0)),
            ((-5.0, 9.0, 5.0), (-7.83, 7.83, 5.0), (-9.0, 5.0, 5.0))
        )
        called_arcs = []
        for call in arc.call_args_list:
            a, b, c = call.args
            called_arcs.append((
                (round(a.x, 2), round(a.y, 2), round(a.z, 2)),
                (round(b.x, 2), round(b.y, 2), round(b.z, 2)),
                (round(c.x, 2), round(c.y, 2), round(c.z, 2)),
            ))

        self.assertSequenceEqual(expected_arcs, called_arcs)

        expected_points = (
            (0.0, 0.0, -15.0), (-9.0, 5.0, -10.0), (-9.0, 5.0, -10.0), (-9.0, 5.0, 2.5),
            (-9.0, 5.0, 2.5), (-9.0, -5.0, 2.5), (-5.0, -9.0, 2.5), (5.0, -9.0, 2.5),
            (9.0, -5.0, 2.5), (9.0, 5.0, 2.5), (5.0, 9.0, 2.5), (-5.0, 9.0, 2.5),
            (-9.0, 5.0, 2.5), (-9.0, 5.0, 5.0), (-9.0, 5.0, 5.0), (-9.0, -5.0, 5.0),
            (-5.0, -9.0, 5.0), (5.0, -9.0, 5.0), (9.0, -5.0, 5.0), (9.0, 5.0, 5.0),
            (5.0, 9.0, 5.0), (-5.0, 9.0, 5.0), (-9.0, 5.0, 5.0), (-9.0, 5.0, -10.0))
        called_points = tuple(
            (round(call.args[0], 2), round(call.args[1], 2), round(call.args[2], 2))
            for call in cartesian_point.call_args_list)
        self.assertSequenceEqual(expected_points, called_points)
        gcode = profile.to_gcode().split('\n')
        self.assertSequenceEqual(gcode,
                                 ['G0X-9Y-5Z5', 'F50G1Z-7.5', 'F300G1Y5', 'G2X-5Y9I1.454J2.546', 'F300G1X5',
                                  'G2X9Y5I2.546J-1.454', 'F300G1Y-5', 'G2X5Y-9I-1.454J-2.546',
                                  'F300G1X-5', 'G2X-9Y-5I-2.546J1.454', 'F50G1Z-10', 'F300G1Y5',
                                  'G2X-5Y9I1.454J2.546', 'F300G1X5', 'G2X9Y5I2.546J-1.454', 'F300G1Y-5',
                                  'G2X5Y-9I-1.454J-2.546', 'F300G1X-5', 'G2X-9Y-5I-2.546J1.454', 'G0Z5'])
