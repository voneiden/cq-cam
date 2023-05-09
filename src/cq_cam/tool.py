class Tool:
    def __init__(
        self,
        tool_diameter: float | None = None,
        tool_number: int | None = None,
        feed: float | None = None,
        speed: int | None = None,
    ):
        self.tool_diameter = tool_diameter
        self.tool_number = tool_number
        self.feed = feed
        self.speed = speed
