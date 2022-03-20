from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Union, Optional, Tuple

from OCP.BRepFeat import BRepFeat
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from cadquery import cq

from cq_cam.commands.base_command import Command
from cq_cam.job import Job
from cq_cam.utils.utils import flatten_list


class OperationError(Exception):
    pass


@dataclass
class Operation(ABC):
    job: Job
    """ The `Job` which this task belongs to.
    """

    commands: List[Command] = field(init=False, default_factory=list)
    """List of commands that this task wants to perform.
    """

    clearance_height: float = field(default=20, kw_only=True)
    """ Safe height for rapids inside the task (relative to `Job` surface).
    Note: A task may perform rapids still at lower depths if it deems safe to do so. 
    """

    top_height: float = field(default=0, kw_only=True)
    """ Height of cut layer (relative to `Job` surface).
    """

    def __post_init__(self):
        self.job.tasks.append(self)

    def to_gcode(self):
        start = cq.Vector(10, 10, 10)

        def command_gcode_generator(start):
            previous_command = None
            for command in self.commands:
                gcode, start = command.to_gcode(previous_command, start, self.job)
                yield gcode
                previous_command = command

        return "\n".join(command_gcode_generator(start))

    @staticmethod
    def break_compound_to_faces(compound: Union[cq.Compound, cq.Face]) -> List[cq.Face]:
        faces = []
        explorer = TopExp_Explorer(compound.wrapped, TopAbs_FACE)
        while explorer.More():
            face = explorer.Current()
            faces.append(cq.Face(face))
            explorer.Next()
        return faces

    @staticmethod
    def combine_faces(faces: List[cq.Face]) -> List[Union[cq.Compound, cq.Face]]:
        return [compound for compound in cq.Workplane().add(faces).combine().objects if
                isinstance(compound, cq.Compound) or isinstance(compound, cq.Face)]

    @classmethod
    def combine_faces_and_break(cls, faces: List[cq.Face]) -> List[List[cq.Face]]:
        """
        Combine a list of faces into compounds and break the compounds back into faces
        """
        combined_items = cls.combine_faces(faces)
        results = []
        for combined in combined_items:
            if isinstance(combined, cq.Compound):
                results.append(cls.break_compound_to_faces(combined))
            else:
                results.append(combined)
        return results

    def transform_shapes_to_global(self, faces: List[cq.Shape]) -> List[cq.Shape]:
        matrix = self.job.workplane.plane.fG
        return [face.transformShape(matrix) for face in faces]

    def _o_objects(self, o):
        if isinstance(o, cq.Workplane):
            return o.objects
        elif isinstance(o, list):
            return o
        else:
            return [o]


@dataclass
class FaceBaseOperation(Operation, ABC):
    """ Base class for any operation that operates primarily on face(s)
    """

    _op_o_shapes = Union[cq.Wire, cq.Face]

    o: Union[cq.Workplane, List[_op_o_shapes], _op_o_shapes] = None
    """ The cadquery Workplane containing faces and/or
    wires that the profile will operate on. 
    """

    avoid: Optional[Union[cq.Workplane, List[_op_o_shapes], _op_o_shapes]] = None
    """ [INOP] List of faces that the tool may not enter. This option
    can be relevant when using an `outer_boundary_offset` that
    would otherwise cause the tool to enter features you do
    not want to cut."""

    stepover: float = 0.8
    """ Stepover (cut width) as a fraction of tool diameter (0..1]. 
    For example a value of 0.5 means the operation tries to use 
    50% of the tool width."""

    outer_boundary_offset: Union[float, Tuple[float, float]] = -1
    """ Offset is in multiples of tool diameter
      * -1 for closed pockets and inside profiles
      * 0 for open pockets
      * 1 for outside profiles
      
    This offset is applied to the outerWire of the operation boundary
    
    When doing open 2.5D pockets, see `avoid`.
    """

    inner_boundary_offset: Optional[Union[float, Tuple[float, float]]] = 1
    """ See `outer_boundary_offset`  """

    boundary_final_pass_stepover: Union[float, None] = None
    """ Stepover for a final boundary (profile) pass.
    """

    stepdown: Union[float, None] = None
    """ Maximum distance to step down on each pass 
    """

    def __post_init__(self):
        if isinstance(self.outer_boundary_offset, (float, int)):
            self.outer_boundary_offset = (self.outer_boundary_offset, 0)
        if isinstance(self.inner_boundary_offset, (float, int)):
            self.inner_boundary_offset = (self.inner_boundary_offset, 0)

    @property
    @abstractmethod
    def _tool_diameter(self) -> float:
        pass

    def _wp_to_faces(self, name, o):
        faces: List[cq.Face] = []
        for obj in self._o_objects(o):
            if isinstance(obj, cq.Face):
                faces.append(obj)
            elif isinstance(obj, cq.Wire):
                faces.append(cq.Face.makeFromWires(obj))
            else:
                raise OperationError(f'Object type "{type(obj)}" not supported by a face operation')

        if not faces:
            raise OperationError(f'{name} selection must contain at least one face or wire')

        return faces

    @property
    def _faces(self):
        return self._wp_to_faces('o', self.o)

    @property
    def _avoid(self):
        return self._wp_to_faces('avoid', self.avoid)

    def offset_boundary(self, boundary: cq.Face) -> List[cq.Face]:
        assert boundary.geomType() in ("PLANE", "CIRCLE")

        outer_offset = self._tool_diameter * self.outer_boundary_offset[0] + self.outer_boundary_offset[1]
        inner_offset = self._tool_diameter * self.inner_boundary_offset[0] + self.inner_boundary_offset[1]

        outer_wires = boundary.outerWire().offset2D(outer_offset)
        inner_wires = [] if inner_offset is None else flatten_list(
            [inner.offset2D(inner_offset) for inner in boundary.innerWires()])

        outer_faces = [cq.Face.makeFromWires(wire) for wire in outer_wires]
        inner_faces = [cq.Face.makeFromWires(wire) for wire in inner_wires]
        feat = BRepFeat()
        boundaries = []
        for outer_face in outer_faces:
            inner = []
            for inner_face in inner_faces[:]:
                if feat.IsInside_s(inner_face.wrapped, outer_face.wrapped):
                    inner.append(inner_face.outerWire())
                    inner_faces.remove(inner_face)

            boundaries.append(cq.Face.makeFromWires(outer_face.outerWire(), inner))

        return boundaries
