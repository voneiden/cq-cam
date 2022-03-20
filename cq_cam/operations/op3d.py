from dataclasses import dataclass
from typing import Tuple, List

import cadquery as cq
import numpy as np
import ocl
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Pnt

from cq_cam.commands.base_command import Unit
from cq_cam.commands.command import Rapid, Plunge, Cut
from cq_cam.job import Job
from cq_cam.operations.base_operation import FaceBaseOperation
from cq_cam.operations.strategy import ZigZagStrategy
from cq_cam.utils import utils
from cq_cam.utils.utils import flatten_list
from cq_cam.visualize import visualize_task


@dataclass
class Surface3D(FaceBaseOperation):
    tool: ocl.MillingCutter = ocl.CylCutter(3.175, 15)
    interpolation_step: float = 0.5

    @property
    def _tool_diameter(self) -> float:
        return self.tool.getDiameter()

    def __post_init__(self):
        """
        The 3D job can work very similar to 2D pocket:

        1) Collect all the faces to get the work boundary
        2) Generate cut sequences
        3) Split cut sequences into sufficiently small sections
        3) Use OpenCAMLib to calculate the depth for each section
        4) Proceed generating multiple depths taking section depths into consideration
        :return:
        """
        super().__post_init__()
        faces = self.transform_shapes_to_global(self._faces)
        compound = cq.Workplane().add(faces).combine().objects[0]
        bb = compound.BoundingBox()

        projected_faces = [utils.project_face(face) for face in faces]
        combine_result = cq.Workplane().add(projected_faces).combine().objects[0]
        if isinstance(combine_result, cq.Compound):
            base_boundaries = self.break_compound_to_faces(combine_result)
        else:
            base_boundaries = [combine_result]

        op_boundaries = flatten_list([self.offset_boundary(boundary) for boundary in base_boundaries])

        for op_boundary in op_boundaries:
            outer_boundary = op_boundary.outerWire()
            inner_boundaries = op_boundary.innerWires()  # TODO is this needed?

            cut_sequences = ZigZagStrategy.process(self, [outer_boundary], [])

            def interpolate_cut_sequence(cut_sequence):
                interpolated = [cut_sequence[0]]
                v1 = cq.Vector(cut_sequence[0])
                for p2 in cut_sequence[1:]:
                    v2 = cq.Vector(p2)
                    v = v2 - v1
                    u = v.normalized()
                    l = v.Length
                    for step in np.arange(0, l, self.interpolation_step):
                        step_v = v1 + u * step
                        interpolated.append((step_v.x, step_v.y))
                    interpolated.append(p2)
                    v1 = v2
                return interpolated

            # Note, this interpolation doesn't consider depth at all
            # Ideally we'd have a point at every depth boundary  (TODO?)
            # Should be kinda easy?
            interpolated_cut_sequences = [interpolate_cut_sequence(cut_sequence) for cut_sequence in cut_sequences]
            cl_points = []
            for cut_sequence in interpolated_cut_sequences:
                for point in cut_sequence:
                    cl_point = ocl.CLPoint(point[0], point[1], bb.zmin)
                    cl_points.append(cl_point)

            triangles = self.shape_to_triangles(compound)
            stl_surf = ocl.STLSurf()
            for triangle in triangles:
                points = (ocl.Point(*vertex) for vertex in triangle)
                tri = ocl.Triangle(*points)
                stl_surf.addTriangle(tri)

            # TODO add post optimization for the moves (detect linear sequences)

            op = ocl.BatchDropCutter()
            op.setCutter(self.tool)
            for cl_point in cl_points:
                op.appendPoint(cl_point)
            op.setSTL(stl_surf)
            op.run()

            # Merge bottom height data
            result_points = [cl_point_to_tuple(point) for point in op.getCLPoints()]
            i = 0
            for cut_sequence in interpolated_cut_sequences:
                for j, point in enumerate(cut_sequence):
                    cut_sequence[j] = (*point, result_points[i][2])
                    i += 1

            bottom_height = bb.zmin
            if self.stepdown:
                depths = list(np.arange(self.top_height + self.stepdown, bottom_height, self.stepdown))
                if depths[-1] != bottom_height:
                    depths.append(bottom_height)
            else:
                depths = [bottom_height]

            for i, depth in enumerate(depths):

                # We want to include i-2 - otherwise we get gaps between depths
                last_last_depth = depths[i - 2] if i > 1 else 0
                depth_cut_sequences = self._chop_sequences_by_depth(interpolated_cut_sequences, last_last_depth)
                # TODO optimize order of cut sequences to minimize rapid distances
                # note to self: in zigzag, I guess maintaining the order is the best bet
                # TODO if there is a new cut sequence within radius of max_step then use it without retracting
                for cut_sequence in depth_cut_sequences:
                    cut_start = cut_sequence[0]
                    self.commands.append(Rapid(x=None, y=None, z=self.clearance_height))
                    self.commands.append(Rapid(x=cut_start[0], y=cut_start[1], z=None))
                    self.commands.append(Rapid(x=None, y=None, z=self.top_height))  # TODO plunge or rapid?
                    self.commands.append(Plunge(z=cut_start[2]))
                    for cut in cut_sequence[1:]:
                        self.commands.append(Cut(x=cut[0], y=cut[1], z=max(depth, cut[2])))

        # for i, base_boundary in enumerate(base_boundaries):
        #    show_object(base_boundary, f'base_boundary-{i}')
        #
        # for i, op_boundary in enumerate(op_boundaries):
        #    show_object(op_boundary, f'op_boundary-{i}')
        #
        # show_object(faces, 'depth_boundary')

    @classmethod
    def shape_to_triangles(cls,
                           shape: cq.Shape,
                           tolerance: float = 1e-3,
                           angular_tolerance: float = 0.1) -> List[Tuple[Tuple[float, float, float], ...]]:

        # BRepMesh_IncrementalMesh gets mad if you try to pass a Compound to it
        if isinstance(shape, cq.Compound):
            faces = cls.break_compound_to_faces(shape)
            results = []
            for face in faces:
                results.append(cls.shape_to_triangles(face, tolerance, angular_tolerance))
            return flatten_list(results)

        mesh = BRepMesh_IncrementalMesh(shape.wrapped, tolerance, True, angular_tolerance)
        mesh.Perform()

        explorer = TopExp_Explorer(shape.wrapped, TopAbs_FACE)
        triangles = []
        location = TopLoc_Location()
        brep_tool = BRep_Tool()
        while explorer.More():
            shape_face = explorer.Current()
            triangulation = brep_tool.Triangulation_s(shape.wrapped, location)
            for i in range(triangulation.NbTriangles()):
                triangle = triangulation.Triangle(i + 1)
                face_triangles = tuple(point_to_tuple(triangulation.Node(node)) for node in triangle.Get())
                triangles.append(face_triangles)

            explorer.Next()

        return triangles

    @staticmethod
    def _chop_sequences_by_depth(sequences: List[List[Tuple[float, float, float]]], last_depth: float):
        new_sequences = []
        for sequence in sequences:
            new_sequence = []
            for point in sequence:
                if point[2] > last_depth:
                    if new_sequence:
                        new_sequences.append(new_sequence)
                        new_sequence = []
                else:
                    new_sequence.append(point)
            if new_sequence:
                new_sequences.append(new_sequence)
        return new_sequences


def point_to_tuple(point: gp_Pnt) -> Tuple[float, float, float]:
    return point.X(), point.Y(), point.Z()


def shape_to_triangles(shape: cq.Shape,
                       tolerance: float = 1e-3,
                       angular_tolerance: float = 0.1) -> List[Tuple[Tuple[float, float, float], ...]]:
    mesh = BRepMesh_IncrementalMesh(shape.wrapped, tolerance, True, angular_tolerance)
    mesh.Perform()

    explorer = TopExp_Explorer(shape.wrapped, TopAbs_FACE)
    triangles = []
    location = TopLoc_Location()
    brep_tool = BRep_Tool()
    while explorer.More():
        shape_face = explorer.Current()
        triangulation = brep_tool.Triangulation_s(shape.wrapped, location)
        for i in range(triangulation.NbTriangles()):
            triangle = triangulation.Triangle(i + 1)
            face_triangles = tuple(point_to_tuple(triangulation.Node(node)) for node in triangle.Get())
            triangles.append(face_triangles)

        explorer.Next()

    return triangles


def cl_point_to_tuple(point: ocl.CLPoint) -> Tuple[float, float, float]:
    return point.x, point.y, point.z


def demo():
    wp = cq.Workplane('XZ').lineTo(100, 0).lineTo(100, 120).lineTo(80, 120).lineTo(0, 0).close().extrude(50)
    job = Job(workplane=wp.faces('>Z').workplane(),
              feed=300,
              plunge_feed=100,
              unit=Unit.METRIC,
              rapid_height=10)

    faces = wp.faces('(not +X) and (not -X) and (not -Y) and (not +Y) and (not -Z)')
    op = Surface3D(job=job, clearance_height=2, top_height=0, o=faces, tool=ocl.CylCutter(3.175, 10),
                   avoid=None, stepdown=-5)

    toolpath = visualize_task(job, op)
    show_object(wp, 'part')
    show_object(toolpath, 'toolpath')


def demo2():
    result = (
        cq.Workplane('XY').rect(30, 30).extrude(20)
            .faces('>Z').workplane().rect(20, 20).cutBlind(-5)
            .faces('>Z[1]').workplane().rect(10, 10).extrude(3)
            .faces('>Z[1]').fillet(1)
            .faces('>Z[2]').fillet(1)
            .faces('>Z')
    )
    result.objects = result.objects[0].innerWires()
    result = result.fillet(1)
    job = Job(workplane=result.faces('>Z').workplane(),
              feed=300,
              plunge_feed=100,
              unit=Unit.METRIC,
              rapid_height=10)
    op = Surface3D(job=job, clearance_height=2, top_height=0, o=result.faces(), tool=ocl.CylCutter(3.175, 10),
                   interpolation_step=0.1, outer_boundary_offset=0)
    toolpath = visualize_task(job, op, as_edges=False)

    show_object(result)
    show_object(toolpath)


if 'show_object' in locals() or __name__ == '__main__':
    demo2()
