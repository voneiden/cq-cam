from typing import List

import cadquery as cq


def circle_bug_workaround(source_wire: cq.Wire, target_wires: List[cq.Wire]):
    """
    FreeCAD style workaround for
    https://github.com/CadQuery/cadquery/issues/896

    :param source_wire:
    :param target_wires:
    :return:
    """
    if len(source_wire.Edges()) == 1:
        edge = source_wire.Edges()[0]
        if edge.startPoint() == edge.endPoint():
            # OCCT bug with offsetting circles!
            for target in target_wires:
                target.wrapped.Location(source_wire.wrapped.Location().Inverted())
