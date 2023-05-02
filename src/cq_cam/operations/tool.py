"""
The Tool class is to be used by operations that require a tool or configuration change.
In a roughing pass it is common to work with higher diameter tool or using different feeds or speeds.
In a subsequent operation you can change the tools or setting of the machine for a better finish.

The most common settings to change between operation are the feed and speeds, and the tools.
To do that successfully you need to update the configuration of the Job class and issue the necessary Commands (ToolChange and StartSequence)
"""
from typing import Optional
from cq_cam.fluent import Job
from cq_cam.command import ToolChange, StartSequence

class Tool:
    def __init__(self, job: Job, feed: Optional[float], speed: Optional[int], tool_diameter: Optional[float], tool_number: Optional[int]):
        """Feed does not to issue any commands. It just needs to be updated so that subsequent operations can use it with G1, G2, and G3 commands"""
        job.feed = feed

        """If the tool number changes a ToolChange command needs to be issued as well. There is no need to check for tool_diameter changes as each tool number is a unique reference to each tool. If two or more tool_diameter match to the same tool number it is an error"""
        if job.tool_number != tool_number:
            job.tool_diameter = tool_diameter
            job.tool_number = tool_number
            job._add_operation(ToolChange(tool_number, speed, job.coolant))
        elif job.speed != speed:
            job.speed = speed
            job._add_operation(StartSequence(speed, job.coolant))