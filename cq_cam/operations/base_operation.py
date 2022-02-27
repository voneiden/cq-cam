from abc import ABC, abstractproperty, abstractmethod
from dataclasses import dataclass, field
from typing import List, Union, Optional, Tuple, Dict

from OCP.BRepFeat import BRepFeat
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from cadquery import cq

from cq_cam.commands.base_command import Command
from cq_cam.job import Job
from cq_cam.utils.linked_polygon import LinkedPolygon
from cq_cam.utils.utils import flatten_list, pairwise, dist_to_segment_squared


class OperationError(Exception):
    pass


Scanpoint = Tuple[float, float]
Scanline = List[Scanpoint]


@dataclass
class Task(ABC):
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


@dataclass
class FaceBaseOperation(Task, ABC):
    """ Base class for any operation that operates primarily on face(s)
    """

    wp: cq.Workplane = None
    """ The cadquery Workplane containing faces and/or
    wires that the profile will operate on. 
    """

    avoid: Optional[cq.Workplane] = None
    """ [INOP] List of faces that the tool may not enter. This option
    can be relevant when using an `outer_boundary_offset` that
    would otherwise cause the tool to enter features you do
    not want to cut."""

    stepover: float = 0.8
    """ Stepover (cut width) as a fraction of tool diameter (0..1]. 
    For example a value of 0.5 means the operation tries to use 
    50% of the tool width."""

    outer_boundary_offset: float = -1
    """ Offset is in multiples of tool diameter
      * -1 for closed pockets and inside profiles
      * 0 for open pockets
      * 1 for outside profiles
      
    This offset is applied to the outerWire of the operation boundary
    
    When doing open 2.5D pockets, see `avoid`.
    """

    inner_boundary_offset: Optional[float] = 1
    """ See `outer_boundary_offset`  """

    boundary_final_pass_stepover: Union[float, None] = None
    """ Stepover for a final boundary (profile) pass.
    """

    stepdown: Union[float, None] = None
    """ Maximum distance to step down on each pass 
    """

    @property
    @abstractmethod
    def _tool_diameter(self) -> float:
        pass

    def _wp_to_faces(self, name, wp):
        faces: List[cq.Face] = []
        for obj in self.wp.objects:
            if isinstance(obj, cq.Face):
                faces.append(obj)
            elif isinstance(obj, cq.Wire):
                faces.append(cq.Face.makeFromWires(obj))
            else:
                raise OperationError(f'Object type "{type(obj)}" not supported by a face operation')

        if not faces:
            raise OperationError(f'{name} selection must contain at least one face or wire')

    @property
    def _faces(self):
        return self._wp_to_faces('wp', self.wp)

    @property
    def _avoid(self):
        return self._wp_to_faces('avoid', self.avoid)

    def offset_boundary(self, boundary: cq.Face) -> List[cq.Face]:
        assert boundary.geomType() in ("PLANE", "CIRCLE")

        outer_offset = self._tool_diameter * self.outer_boundary_offset
        inner_offset = self._tool_diameter * self.inner_boundary_offset

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

    @staticmethod
    def _scanline_end_map(scanlines: List[Scanline]):
        scanline_end_map = {}
        scanpoints = []
        for scanline in scanlines:
            scanline_start, scanline_end = scanline[0], scanline[-1]
            scanline_end_map[scanline_start] = scanline
            scanline_end_map[scanline_end] = scanline
            scanpoints.append(scanline_start)
            scanpoints.append(scanline_end)
        return scanline_end_map, scanpoints

    @staticmethod
    def _link_scanpoints_to_boundaries(scanpoints: List[Scanpoint],
                                       boundaries: List[List[Tuple[float, float]]]):
        remaining_scanpoints = scanpoints[:]
        scanpoint_to_linked_polygon = {}
        linked_polygons = []
        for polygon in boundaries:
            linked_polygon = LinkedPolygon(polygon[:])
            linked_polygons.append(linked_polygon)
            for p1, p2 in pairwise(polygon):
                for scanpoint in remaining_scanpoints[:]:
                    d = dist_to_segment_squared(scanpoint, p1, p2)
                    # Todo pick a good number. Tests show values between 1.83e-19 and 1.38e-21
                    if d < 0.0000001:
                        remaining_scanpoints.remove(scanpoint)
                        linked_polygon.link_point(scanpoint, p1, p2)
                        scanpoint_to_linked_polygon[scanpoint] = linked_polygon

        assert not remaining_scanpoints
        return linked_polygons, scanpoint_to_linked_polygon

    @staticmethod
    def _route_zig_zag(linked_polygons: List[LinkedPolygon],
                       scanlines: List[Tuple[float, float]],
                       scanpoint_to_linked_polygon: Dict[Tuple[float, float], LinkedPolygon],
                       scanpoint_to_scanline: Dict[Tuple[float, float], List[Tuple[float, float]]]):

        # Prepare to route the zigzag
        for linked_polygon in linked_polygons:
            linked_polygon.reset()

        scanlines = list(scanlines)

        # Pick a starting position. Clipper makes no guarantees about the orientation
        # of polylines it returns, so figure the top left scanpoint as the
        # starting position.
        starting_scanline = scanlines.pop(0)
        start_position, cut_position = starting_scanline
        if start_position[0] > cut_position[0]:
            start_position, cut_position = cut_position, start_position

        scanpoint_to_linked_polygon[start_position].drop(start_position)
        cut_sequence = [start_position, cut_position]
        cut_sequences = []

        # Primary routing loop
        while scanlines:
            linked_polygon = scanpoint_to_linked_polygon[cut_position]
            path = linked_polygon.nearest_linked(cut_position)
            if path is None:
                cut_sequences.append(cut_sequence)
                # TODO some optimization potential in picking the nearest scanpoint
                start_position, cut_position = scanlines.pop(0)
                # TODO some optimization potential in picking a direction
                cut_sequence = [start_position, cut_position]
                continue

            cut_sequence += path
            scanline = scanpoint_to_scanline[path[-1]]
            cut_sequence.append(scanline[1] if scanline[0] == path[-1] else scanline[0])
            cut_position = cut_sequence[-1]
            scanlines.remove(scanline)

        cut_sequences.append(cut_sequence)
        return cut_sequences
