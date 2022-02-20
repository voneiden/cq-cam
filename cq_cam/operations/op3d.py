from dataclasses import dataclass
from typing import Tuple, List

import cadquery as cq
import numpy as np
# Fiber(const Point &p1, const Point &p2)
# MillingCutter:  ocl::BallCutter, ocl::BullCutter, ocl::CompositeCutter, ocl::ConeCutter, ocl::CylCutter
# STLSurf
# * addTriangle
# Triangle(Point p1, Point p2, Point p3)
# BatchPushCutter: use for batch jobs
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
from cq_cam.utils import utils
from cq_cam.utils.utils import flatten_list, WireClipper
from cq_cam.visualize import visualize_task


@dataclass
class Surface3D(FaceBaseOperation):
    tool: ocl.MillingCutter = None

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

        faces = self.transform_shapes_to_global(self.faces)
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
            inner_boundaries = op_boundary.innerWires()
            clipper = WireClipper(cq.Workplane().plane)
            outer_polygon = clipper.add_clip_wire(outer_boundary)
            inner_polygons = []
            for inner_boundary in inner_boundaries:
                polygon = clipper.add_clip_wire(inner_boundary)
                inner_polygons.append(polygon)

            max_bounds = clipper.max_bounds()

            # Generate ZigZag scanlines
            y_scanpoints = list(np.arange(max_bounds['bottom'], max_bounds['top'], self.tool_diameter * self.stepover))
            scanline_templates = [((max_bounds['left'], y), (max_bounds['right'], y)) for y in y_scanpoints]

            for scanline_template in scanline_templates:
                clipper.add_subject_polygon(scanline_template)

            scanlines = clipper.execute()

            scanpoint_to_scanline, scanpoints = self._scanline_end_map(scanlines)

            linked_polygons, scanpoint_to_linked_polygon = self._link_scanpoints_to_boundaries(
                scanpoints, [outer_polygon] + inner_polygons)

            cut_sequences = self._route_zig_zag(linked_polygons,
                                                scanlines,
                                                scanpoint_to_linked_polygon,
                                                scanpoint_to_scanline)

            # Get depths for
            max_step = 1

            def interpolate_cut_sequence(cut_sequence):
                interpolated = [cut_sequence[0]]
                v1 = cq.Vector(cut_sequence[0])
                for p2 in cut_sequence[1:]:
                    v2 = cq.Vector(p2)
                    v = v2 - v1
                    u = v.normalized()
                    l = v.Length
                    for step in np.arange(0, l, max_step):
                        step_v = v1 + u * step
                        interpolated.append((step_v.x, step_v.y))
                    interpolated.append(p2)
                    v1 = v2
                return interpolated

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

            cutter = ocl.CylCutter(self.tool_diameter, 100)

            op = ocl.BatchDropCutter()
            op.setCutter(cutter)
            for cl_point in cl_points:
                op.appendPoint(cl_point)
            op.setSTL(stl_surf)
            op.run()

            result_points = [cl_point_to_tuple(point) for point in op.getCLPoints()]
            tmp_i = 0
            for i, depth in enumerate([bb.zmin]):
                for cut_sequence in interpolated_cut_sequences:
                    cut_start = cut_sequence[0]
                    self.commands.append(Rapid(None, None, self.clearance_height))
                    self.commands.append(Rapid(*cut_start, result_points[tmp_i][2]))
                    self.commands.append(Rapid(None, None, self.top_height))  # TODO plunge or rapid?
                    self.commands.append(Plunge(depth))
                    tmp_i += 1
                    for cut in cut_sequence[1:]:
                        self.commands.append(Cut(*cut, result_points[tmp_i][2]))
                        tmp_i += 1

        for i, base_boundary in enumerate(base_boundaries):
            show_object(base_boundary, f'base_boundary-{i}')

        for i, op_boundary in enumerate(op_boundaries):
            show_object(op_boundary, f'op_boundary-{i}')

        show_object(faces, 'depth_boundary')

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


box = cq.Workplane().box(10, 10, 10)
face: cq.Face = box.faces('>Z').objects[0]

triangles = shape_to_triangles(face)
# print(triangles)
# CL point should be below surface!
clpoint = ocl.CLPoint(1, 1, -20)
stl_surf = ocl.STLSurf()
for triangle in triangles:
    # print(triangle)
    points = (ocl.Point(*vertex) for vertex in triangle)
    tri = ocl.Triangle(*points)
    stl_surf.addTriangle(tri)

cutter = ocl.CylCutter(2, 10)

op = ocl.BatchDropCutter()
op.setCutter(cutter)
op.appendPoint(clpoint)
op.setSTL(stl_surf)


# op.run()

def cl_point_to_tuple(point: ocl.CLPoint) -> Tuple[float, float, float]:
    return point.x, point.y, point.z


print("points", [cl_point_to_tuple(point) for point in op.getCLPoints()])


def demo():
    wp = cq.Workplane('XZ').lineTo(100, 0).lineTo(100, 120).lineTo(80, 120).lineTo(0, 0).close().extrude(50)
    job = Job(workplane=wp.faces('>Z').workplane(),
              feed=300,
              plunge_feed=100,
              unit=Unit.METRIC,
              rapid_height=10)

    faces = wp.faces('(not +X) and (not -X) and (not -Y) and (not +Y) and (not -Z)')
    op = Surface3D(job=job, clearance_height=2, top_height=0, faces=faces.objects, tool=ocl.CylCutter(3.175, 10),
                   avoid=[], tool_diameter=3.175)

    toolpath = visualize_task(job, op)
    show_object(wp, 'part')
    show_object(toolpath, 'toolpath')
    # show_object(wp)
    # show_object(faces)

    # jobplane = wp.faces('>X').workplane()
    ##boink = wp.objects[0].transformShape(jobplane.plane.fG)
    # x = cq.Workplane(obj=wp.objects[0])
    # x.plane = jobplane.plane.rotated((180, 0 ,0))
    # show_object(wp, 'wp')
    # f = wp.faces('>X')
    # show_object(f, 'f')
    # show_object(boink, 'boink')
    # show_object(x, 'x')


if 'show_object' in locals() or __name__ == '__main__':
    demo()
