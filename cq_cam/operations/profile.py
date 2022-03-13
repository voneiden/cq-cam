import logging
from dataclasses import dataclass, field
from typing import Union, List, Optional, Tuple

import numpy as np
from cadquery import cq

from cq_cam.commands.command import Rapid, Plunge
from cq_cam.commands.util_command import wire_to_command_sequence2

from cq_cam.operations.base_operation import Task, OperationError
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin
from cq_cam.operations.tabs import Tabs, NoTabs, WireTabs
from cq_cam.utils.utils import (
    plane_offset_distance,
    cut_clockwise
)
from cq_cam.visualize import visualize_task

logger = logging.getLogger(__name__)

_op_o_shapes = Union[cq.Wire, cq.Face]


@dataclass
class Profile(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """
    Create a profiles based on selected wires and faces in a Workplane.
    """

    o: Union[cq.Workplane, List[_op_o_shapes], _op_o_shapes] = None
    """
    Operation target. This may be one of
    
    * `cq.Workplane` with faces and/or wires selected
    *  List of or a single `cq.Face` and/or `cq.Wire`
    """

    stepdown: Union[float, None] = None
    """ 
    Z stepdown. A value of None means straight to bottom. 
    """

    tool_diameter: float = 3.175

    face_offset_outer: Optional[Union[float, Tuple[float, float]]] = 1
    """ Offset is in multiples of tool diameter
      * -1 for closed pockets and inside profiles
      * 0 for open pockets
      * 1 for outside profiles
      
    It is possible to also give a tuple of the form (multiple, relative_offset)
    in which case first the multiple is calculated and the relative offset is added 
    to the final result. For example (1, 0.5) in metric would mean an outer profile with
    0.5 mm stock left.
      
    This offset is applied to any wires and the outer wires of any faces 
    defined in `o`.
    """

    face_offset_inner: Optional[Union[float, Tuple[float, float]]] = None
    """ 
    See `face_offset_outer`  
    """

    wire_offset: Optional[Union[float, Tuple[float, float]]] = 1
    """
    See `face_offset_outer` - this value applies for any wires in `o`
    """

    tabs: Optional[Tabs] = NoTabs()

    def __post_init__(self):
        super().__post_init__()

        if not self.o:
            raise OperationError("o must be defined")

        if isinstance(self.face_offset_outer, (float, int)):
            self.face_offset_outer = (self.face_offset_outer, 0)
        if isinstance(self.face_offset_inner, (float, int)):
            self.face_offset_inner = (self.face_offset_inner, 0)
        if isinstance(self.wire_offset, (float, int)):
            self.wire_offset = (self.wire_offset, 0)

        self.stepdown = abs(self.stepdown) if self.stepdown is not None else None

        faces: List[cq.Face] = []
        wires: List[cq.Wire] = []
        objects = self._o_objects()
        for obj in objects:
            if isinstance(obj, cq.Face):
                faces.append(obj)
            elif isinstance(obj, cq.Wire):
                wires.append(obj)
            else:
                raise OperationError(f'Object type "{type(obj)}" not supported by Profile operation')

        if not faces and not wires:
            raise OperationError('wp selection must contain at least one face or wire')

        for face in faces:
            if self.face_offset_outer is None and self.face_offset_inner is None:
                raise OperationError("Define at least one of 'face_offset_outer' and 'face_offset_inner'")

            if self.face_offset_outer is not None:
                self.profile(face.outerWire(),
                             self.face_offset_outer[0] * self.tool_diameter + self.face_offset_outer[1])
            if self.face_offset_inner is not None:
                inner_wires = face.innerWires()
                if not inner_wires:
                    logger.warning("Face had no innerWires but inner_boundary_offset was set")
                for wire in inner_wires:
                    self.profile(wire, self.face_offset_inner[0] * self.tool_diameter + self.face_offset_inner[1])

        for wire in wires:
            if self.wire_offset is None:
                raise OperationError("'wire_offset' must be defined when profiling wires")
            self.profile(wire, self.wire_offset[0] * self.tool_diameter + self.wire_offset[1])

    def _o_objects(self):
        if isinstance(self.o, cq.Workplane):
            return self.o.objects
        elif isinstance(self.o, list):
            return self.o
        else:
            return [self.o]

    def profile(self, wire: cq.Wire, offset: float):

        face = cq.Workplane(cq.Face.makeFromWires(wire))
        workplane, _ = self.validate_plane(self.job, face)

        # Originally I intended to use clipper to do the offset, however
        # I realized later that CadQuery/OpenCASCADE can do 2D offsets for
        # wires as well. This has the benefit that it conserves arcs
        # which can then be effortlessly be converted to G2/G3 commands.
        offset_wires = wire.offset2D(offset, 'arc')
        assert len(offset_wires) == 1

        command_sequence = wire_to_command_sequence2(offset_wires[0], self.tabs)

        if command_sequence.is_clockwise() != cut_clockwise(True, True, True):
            command_sequence.reverse()
            pass

        start = command_sequence.start
        end = command_sequence.end

        # Rapid to clearance height
        self.commands.append(Rapid(x=start.x, y=start.y, z=self.clearance_height))
        bottom_height = plane_offset_distance(self.job.workplane.plane, workplane.plane)
        if self.stepdown:
            depths = list(np.arange(self.top_height - self.stepdown, bottom_height, -self.stepdown))
            if depths[-1] != bottom_height:
                depths.append(bottom_height)
        else:
            depths = [bottom_height]

        tab_height = self.tabs.height + bottom_height
        for i, depth in enumerate(depths):
            # self.commands.append(profile[0])
            if getattr(command_sequence.commands[0], 'tab', False):
                self.commands.append(Plunge(tab_height))
            else:
                self.commands.append(Plunge(depth))
            # TODO apply tabs
            self.commands += command_sequence.duplicate(depth, tab_z=tab_height).commands

        self.commands.append(Rapid(x=end.x, y=end.y, z=self.clearance_height))

        self.job.tasks.append(self)


def demo():
    from cq_cam.job import Job
    from cq_cam.commands.base_command import Unit
    wp = cq.Workplane().box(10, 20, 10)
    job = Job(wp.faces('>Z').workplane(), 300, 50, Unit.METRIC, 5)
    profile = Profile(job=job, o=wp.wires('<Z'), stepdown=-1, clearance_height=2, top_height=0)
    print("OK")
    viz = visualize_task(job, profile)
    show_object(wp, 'wp')
    show_object(viz, 'viz')

def demo2():
    from cq_cam import Job, Profile, Unit, visualize_task
    result = cq.Workplane("front").box(20.0, 20.0, 5)

    job_plane = result.faces('>Z').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Profile(job=job, o=result.wires('<Z'), tabs=WireTabs(count=4, width=2, height=2))
    toolpath = visualize_task(job, op, as_edges=False)
    #result.objects += toolpath
    show_object(result)
    show_object(toolpath)


if 'show_object' in locals() or __name__ == '__main__':
    demo2()
