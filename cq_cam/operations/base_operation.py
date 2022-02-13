from abc import ABC
from dataclasses import dataclass, field
from typing import List

from cadquery import cq

from cq_cam.commands.base_command import Command
from cq_cam.job import Job


class OperationError(Exception):
    pass


@dataclass
class Task(ABC):
    job: Job
    """ The `Job` which this task belongs to.
    """

    commands: List[Command] = field(init=False, default_factory=list)
    """List of commands that this task wants to perform.
    """

    clearance_height: float
    """ Safe height for rapids inside the task (relative to `Job` surface).
    Note: A task may perform rapids still at lower depths if it deems safe to do so. 
    """

    top_height: float
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
