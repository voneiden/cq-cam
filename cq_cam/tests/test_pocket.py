import unittest
import cadquery as cq

from cq_cam import JobV2
from cq_cam.operations.pocket import pocket


class TestPocket(unittest.TestCase):
    def setUp(self):
        self.job_wp = cq.Workplane()
        self.job = JobV2(self.job_wp.plane, 300, tool_diameter=1)

    def path_to_wire(self, vx):
        edges = []
        for v1, v2 in zip(vx, vx[1:]):
            edge = cq.Edge.makeLine(v1, v2)
            edges.append(edge)

        wire = cq.Wire.assembleEdges(edges)
        return wire

    def test_simple_pocket(self):
        face = cq.Face.makePlane(5, 5, (0, 0, -1))
        result = pocket(self.job, [face])
        cc = result[0][0]
        sc1 = cc.sub_chains[0]
        sc2 = sc1.sub_chains[0]

        self.assertEqual(len(cc.sub_chains), 1)
        self.assertEqual(len(sc1.sub_chains), 1)
        self.assertEqual(len(sc2.sub_chains), 0)

        self.assertEqual(face.Wires()[0].Length(), 20)

        cc_wire = self.path_to_wire(cc.path)
        sc1_wire = self.path_to_wire(sc1.path)
        sc2_wire = self.path_to_wire(sc2.path)

        self.assertEqual(16, cc_wire.Length())
        self.assertEqual(10, sc1_wire.Length())
        self.assertEqual(4, sc2_wire.Length())

