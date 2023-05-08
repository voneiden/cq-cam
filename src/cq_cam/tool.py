from typing import Optional


class Tool:
    def __init__(
        self,
        tool_diameter: Optional[float] = None,
        tool_number: Optional[int] = None,
        feed: Optional[float] = None,
        speed: Optional[int] = None,
    ):
        self.tool_diameter = tool_diameter
        self.tool_number = tool_number
        self.feed = feed
        self.speed = speed
