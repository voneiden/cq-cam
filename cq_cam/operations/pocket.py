from typing import List, Union

from cadquery import cq

from cq_cam.operations.base_operation import Job
from cq_cam.operations.base_operation import Task
from cq_cam.operations.mixin_operation import PlaneValidationMixin, ObjectsValidationMixin


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
