import unittest
from typing import List

import cadquery as cq

from cq_cam.utils import utils


class ProjectFaceTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_face_with_hole(self):
        # This should create a projected face that is 2x4 (XY)
        box = (
            cq.Workplane('XZ')
            .lineTo(2, 0)
            .lineTo(2, 6)
            .close()
            .extrude(4)
            .faces('<Z')
            .workplane()
            .moveTo(1, 2)
            .rect(1, 1)
            .cutThruAll()
        )

        face_wp = cq.Workplane(obj=box.faces().objects[1])
        plane = face_wp.workplane().plane

        # Make sure we picked the right face
        self.assertEqual(plane.xDir, cq.Vector(0.0, -1.0, 0.0))
        self.assertEqual(plane.yDir, cq.Vector(0.316227766016838, 0.0, 0.9486832980505139))
        self.assertEqual(plane.zDir, cq.Vector(-0.9486832980505139, -0.0, 0.316227766016838))

        result = utils.project_face(face_wp.objects[0])

        class TestVector(cq.Vector):
            def __eq__(self, other):
                if getattr(other, 'wrapped', None):
                    return super().__eq__(other)
                return False

        expected_outer_wire = [
            TestVector(2, 0, 0),
            TestVector(2, -4, 0),
            TestVector(0, -4, 0),
            TestVector(0, 0, 0)
        ]
        expected_inner_wire = [
            TestVector(0.5, -1.5, 0),
            TestVector(0.5, -2.5, 0),
            TestVector(1.5, -1.5, 0),
            TestVector(1.5, -2.5, 0)
        ]

        def wire_to_vectors(wire: cq.Wire) -> List[cq.Vector]:
            return [to_vector(vertex) for vertex in wire.Vertices()]

        def to_vector(vertex: cq.Vertex) -> cq.Vector:
            return TestVector(vertex.toTuple())

        self.assertCountEqual(wire_to_vectors(result.outerWire()), expected_outer_wire)

        inner_wires = result.innerWires()
        self.assertEqual(len(inner_wires), 1)
        self.assertCountEqual(wire_to_vectors(inner_wires[0]), expected_inner_wire)
