from cq_cam.commands.command import Cut, CircularCW, CircularCCW
from cq_cam.utils import TypeVector, LineTypeVector, CWArcTypeVector, CCWArcTypeVector


def type_vector_to_command(tv: TypeVector):
    start = tv.start
    end = tv.end
    if isinstance(tv, LineTypeVector):
        return Cut(end[0], end[1], None)

    elif isinstance(tv, CWArcTypeVector):
        return CircularCW(start[0], start[1], end[0], end[1], tv.center, tv.radius)

    elif isinstance(tv, CCWArcTypeVector):
        return CircularCCW(start[0], start[1], end[0], end[1], tv.center, tv.radius)

    else:
        raise NotImplemented(f'Unknown type vector "{tv.__class__}"')
