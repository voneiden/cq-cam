import unittest
from unittest.mock import patch

from cadquery import cq

from cq_cam.job import Job
from cq_cam.operations.base_operation import Operation


class TaskTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_combine_faces(self):
        box = cq.Workplane().box(10, 20, 30).faces('>X').workplane().circle(2.5).cutBlind(-2)

        job_plane = box.faces('>X').workplane()
        op_faces = box.faces('>X[1]').objects

        with patch('cq_cam.job.Job') as job_cls:
            job: Job = job_cls.return_value
            job.workplane = job_plane

            task = Operation(job=job, clearance_height=10, top_height=5)
            transformed_faces = task.transform_shapes_to_global(op_faces)
            result_plane = cq.Workplane().add(transformed_faces).workplane()
            self.assertEqual(result_plane.plane.origin, cq.Vector(0, 0, -2))
            self.assertEqual(result_plane.plane.xDir, cq.Vector(1, 0, 0))
            self.assertEqual(result_plane.plane.yDir, cq.Vector(0, 1, 0))
            self.assertEqual(result_plane.plane.zDir, cq.Vector(0, 0, 1))
