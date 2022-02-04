from dataclasses import dataclass

from cq_cam.commands.command import Rapid, Plunge
from cq_cam.commands.util_command import type_vector_to_command
from cq_cam.operations.base_operation import Task
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin

from cq_cam.operations.base_operation import OperationError, Job

from dataclasses import dataclass
from typing import List, Union

import numpy as np
import pyclipper
from OCP.BRepTools import BRepTools_WireExplorer
from cadquery import cq, Edge, Face

from cq_cam.utils import is_parallel_plane, flatten_edges, plane_offset_distance, dot, wire_to_ordered_edges, \
    wire_to_type_vectors, is_tvs_clockwise, cut_clockwise, reverse_type_vectors, TypeVector, LineTypeVector, \
    CWArcTypeVector, CCWArcTypeVector
from cq_cam.visualize import visualize_task

@dataclass
class Profile(PlaneValidationMixin, ObjectsValidationMixin, Task):
    """
    Create a profile around the outer wire of a given face
    """
    face: cq.Workplane
    offset: float
    stepdown: Union[float, None]

    def __post_init__(self):

        wires = self.face.wires()
        self.validate_wires(wires, 1)
        workplane, _ = self.validate_plane(self.job, self.face)

        # Originally I intended to use clipper to do the offset, however
        # I realized later that CadQuery/OpenCASCADE can do 2D offsets for
        # wires as well. This has the benefit that it conserves arcs
        # which can then be effortlessly be converted to G2/G3 commands.

        wire: cq.Wire = wires.objects[0]
        offset_wires = wire.offset2D(self.offset, 'arc')
        assert len(offset_wires) == 1

        # Convert the offset wire into 2D TypeVector
        tvs = wire_to_type_vectors(self.job.workplane.plane, offset_wires[0])

        if is_tvs_clockwise(tvs) != cut_clockwise(True, True, True):
            reverse_type_vectors(tvs)

        # Grap the outer wire edges
        # edges = wire_to_ordered_edges(wires.objects[0])
        # vectors = flatten_edges(edges)

        # Test offset2d

        # show_object(offset_wire, 'offset_wire')
        # TODO here we need to pick the correct X/Y coordinates according to our workplane!
        # vector.x, vector.y is correct only when we are doing top to bottom profiling
        # scaled_points = pyclipper.scale_to_clipper(tuple((vector.x, vector.y) for vector in vectors))
        ##scaled_points = pyclipper.scale_to_clipper(
        ##    tuple(
        ##        (
        ##            job.workplane.plane.xDir.dot(vector),
        ##            job.workplane.plane.yDir.dot(vector)
        ##        ) for vector in vectors)
        ##)
        ##
        ##pco = pyclipper.PyclipperOffset()
        ##pco.AddPath(scaled_points, pyclipper.JT_SQUARE, pyclipper.ET_CLOSEDLINE)
        ##
        ##points = pyclipper.scale_from_clipper(pco.Execute(pyclipper.scale_to_clipper(self.offset)))[0]

        # Render your stuff man!

        # TODO collect layers?
        # Generate automatically the motions when moving between layers
        # Also layer entry points etc.
        # TODO
        profile = [type_vector_to_command(tv) for tv in tvs]

        start = tvs[0].start

        # Start with a rapid to starting position
        self.commands.append(Rapid(start[0], start[1], None))
        self.commands.append(Rapid(None, None, self.clearance_height))
        bottom_height = plane_offset_distance(self.job.workplane.plane, workplane.plane)
        if self.stepdown:
            depths = list(np.arange(self.top_height + self.stepdown, bottom_height, self.stepdown))
            if depths[-1] != bottom_height:
                depths.append(bottom_height)

            for i, depth in enumerate(depths):
                #self.commands.append(profile[0])
                self.commands.append(Plunge(depth))
                self.commands += profile
            #self.commands.append(profile[0])
        else:
            self.commands.append(profile[0])
            self.commands.append(Plunge(bottom_height))
            self.commands = profile[1:]
            self.commands.append(profile[0])

        self.commands.append(Rapid(None, None, self.clearance_height))
        # Construct profile polygons

        # Generate operation layers

        self.job.tasks.append(self)
