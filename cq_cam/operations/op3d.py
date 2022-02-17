from dataclasses import dataclass
from typing import Tuple, List

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Pnt
from cadquery import cq

# Fiber(const Point &p1, const Point &p2)
# MillingCutter:  ocl::BallCutter, ocl::BullCutter, ocl::CompositeCutter, ocl::ConeCutter, ocl::CylCutter
# STLSurf
# * addTriangle
# Triangle(Point p1, Point p2, Point p3)
# BatchPushCutter: use for batch jobs
import ocl

from cq_cam.operations.base_operation import Task

"""
mesh = BRepMesh_IncrementalMesh(self.wrapped, tolerance, True, angularTolerance)
mesh.Perform()

BRep_Tool.Triangulations
writer = StlAPI_Writer()

        return writer.Write(self.wrapped, fileName)
"""

@dataclass
class Surface3D(Task):
    faces: List[cq.Face]

    def __post_init__(self):
        pass



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
#print(triangles)
# CL point should be below surface!
clpoint = ocl.CLPoint(1, 1, -20)
stl_surf = ocl.STLSurf()
for triangle in triangles:
    #print(triangle)
    points = (ocl.Point(*vertex) for vertex in triangle)
    tri = ocl.Triangle(*points)
    stl_surf.addTriangle(tri)

cutter = ocl.CylCutter(2, 10)

op = ocl.BatchDropCutter()
op.setCutter(cutter)
op.appendPoint(clpoint)
op.setSTL(stl_surf)
op.run()

def cl_point_to_tuple(point: ocl.CLPoint) -> Tuple[float, float, float]:
    return point.x, point.y, point.z

print("points", [cl_point_to_tuple(point) for point in op.getCLPoints()])
