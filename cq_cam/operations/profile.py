import itertools
from typing import List, Optional, TYPE_CHECKING

import cadquery as cq

from cq_cam.routers import route
from cq_cam.utils.utils import compound_to_edges

if TYPE_CHECKING:
    from cq_cam.fluent import JobV2


def profile(job: 'JobV2',
            outer_wires: List[cq.Wire] = None,
            inner_wires: List[cq.Wire] = None,
            outer_offset: float = 1,
            inner_offset: float = -1,
            stepdown: Optional[int] = None,
            ):
    # Transform to relative coordinates
    outers = [outer.transformShape(job.top.fG) for outer in outer_wires]
    inners = [inner.transformShape(job.top.fG) for inner in inner_wires]

    # Generate base features
    base_features = []
    for outer in outers:
        base_features += outer.offset2D(outer_offset * job.tool_diameter)

    for inner in inners:
        base_features += inner.offset2D(inner_offset * job.tool_diameter)

    if stepdown:
        toolpaths = []
        for base_feature in base_features:
            step = cq.Vector(0, 0, 1) * stepdown
            layers: List[cq.Wire] = [base_feature]

            for i in itertools.count():
                if i > job.max_stepdown_count:
                    raise RuntimeError('Profile max_stepdown_count exceeded')

                i_op: cq.Wire = base_feature.moved(cq.Location(step * (i + 1)))
                if i_op.BoundingBox().zmin >= 0:
                    break

                edges = compound_to_edges(i_op.cut(job.top_plane_face))
                edges = [edge for edge in edges if edge.Center().z < 0]
                wires = cq.Wire.combine(edges)
                if not wires:
                    break

                layers.append(i_op)

            layers.reverse()
            toolpaths += layers

    else:
        toolpaths = base_features

    commands = route(job, toolpaths)
    return commands
