from typing import List

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType
from cadquery import cq

from cq_cam.operations.base_operation import OperationError, Job
from cq_cam.utils.utils import is_parallel_plane


class PlaneValidationMixin:
    @staticmethod
    def validate_plane(job: Job, source_workplane: cq.Workplane):
        face_workplane = source_workplane.workplane()
        if not is_parallel_plane(job.workplane.plane, face_workplane.plane):
            raise OperationError('Face plane is not parallel with job plane')
        return face_workplane, source_workplane

    @staticmethod
    def validate_coplanar(source_workplanes: List[cq.Workplane]):
        # TODO
        objs = [obj for source_workplane in source_workplanes for obj in source_workplane.objects]
        try:
            cq.Workplane().add(objs).workplane()
        except ValueError as ex:
            raise OperationError(*ex.args)

    @staticmethod
    def validate_face_plane(face: cq.Face):
        adaptor = BRepAdaptor_Surface(face.wrapped)
        if adaptor.GetType() != GeomAbs_SurfaceType.GeomAbs_Plane:
            raise OperationError("2D operations can only work on flat faces. Given face is not of GeomAbs_Plane")

    @staticmethod
    def validate_face_planar(face: cq.Face):
        if not face.geomType() in ("PLANE", "CIRCLE"):
            raise OperationError(
                "Given face is not planar"
            )

class ObjectsValidationMixin:
    @staticmethod
    def _validate_count(source_workplane: cq.Workplane, count=None):
        if not source_workplane.objects:
            raise OperationError("Empty source workplane")

        if count is not None and len(source_workplane.objects) != count:
            raise OperationError(
                f"Workplane contains incorrect amount of faces (expected {count}, actual {len(source_workplane.objects)}"
            )

    @staticmethod
    def _validate_class(source_workplane: cq.Workplane, cls):
        for workplane_object in source_workplane.objects:
            if not isinstance(workplane_object, cls):
                raise OperationError(f"Workplane has non-{cls.__name__} object(s)")

    def validate_faces(self, source_workplane: cq.Workplane, count=None):
        self._validate_count(source_workplane, count)
        self._validate_class(source_workplane, cq.Face)

    def validate_wires(self, source_workplane: cq.Workplane, count=None):
        self._validate_count(source_workplane, count)
        self._validate_class(source_workplane, cq.Wire)

