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

class Pocket(PlaneValidationMixin, ObjectsValidationMixin, Task):
    def __init__(self, job: Job, obj: Union[cq.Workplane, List[cq.Workplane]]):
        source_workplanes: List[cq.Workplane] = obj if isinstance(obj, list) else [obj]

        # Validate source workplane object stack
        for source_workplane in source_workplanes:
            self.validate_faces(source_workplane)

        # Create face-source workplane pairs
        pairs = [self.validate_plane(job, wp) for wp in source_workplanes]

        # TODO validate faces are co-blah

        # Determine face depths

        # Construct profile polygons

        # Generate operation layers