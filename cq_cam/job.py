from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING

from cadquery import cq

if TYPE_CHECKING:
    from cq_cam.commands.base_command import Unit
    from cq_cam.operations.base_operation import Task


@dataclass
class Job:
    workplane: cq.Workplane
    tasks: List[Task] = field(init=False, default_factory=list)
    feed: float
    plunge_feed: float
    unit: Unit
    rapid_height: float
    gcode_precision: int = 3

    def to_gcode(self):
        task_break = "\n\n\n"
        return f"{self.unit.to_gcode()}\n{task_break.join(task.to_gcode() for task in self.tasks)}"
