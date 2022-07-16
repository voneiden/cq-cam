import logging
from math import isclose
from typing import List, Tuple, Optional, TYPE_CHECKING, Literal

import pyclipper
from cadquery import cq

from cq_cam.routers import route
from cq_cam.utils.utils import flatten_list, compound_to_faces, interpolate_wire_with_unstable_edges, \
    wire_to_ordered_edges, edge_end_point, edge_start_point, interpolate_edge, interpolate_edge_as_2d_path, \
    get_underlying_geom_type

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from cq_cam.fluent import JobV2

DEBUG = []


def sort_by_depth(faces: List[cq.Face]) -> List[Tuple[float, cq.Face]]:
    faces = [(face.Center().z, face) for face in faces]
    faces.sort(key=lambda x: x[0], reverse=True)
    return faces


def offset_face(face: cq.Face, outer_offset, inner_offset, log=True):
    outers = interpolate_wire_with_unstable_edges(face.outerWire()).offset2D(outer_offset)

    inners = []
    if inner_offset:
        for inner in face.innerWires():
            try:
                # TODO interpolate wire with unstable edges?
                for _inner in inner.offset2D(inner_offset):
                    inners.append(_inner)
            except ValueError:
                if log:
                    logger.warning('Failed to offset inner boundary')
                continue
    else:
        inners = face.innerWires()
    inner_faces = [cq.Face.makeFromWires(inner) for inner in inners]
    results = []
    for outer in outers:
        result = cq.Face.makeFromWires(outer)
        if inner_faces:
            compound = result.cut(*inner_faces)
            compound_faces = compound_to_faces(compound)
            if len(compound_faces) == 0:
                continue
            for face in compound_faces:
                results.append(face)
        else:
            results.append(result)

    if not results:
        raise ValueError('Empty result')

    return results


def pocket(job: 'JobV2',
           faces: List[cq.Face],
           avoid: Optional[List[cq.Face]] = None,
           stepover: float = 0.75,
           boundary_offset: float = -1,
           stepdown: Optional[float] = None,
           strategy: Literal['contour'] = 'contour'
           ):
    for face in faces:
        if face.geomType() != 'PLANE' and face.normalAt(face.wrapped.Location()) != job.top.zDir:
            raise ValueError('Face is not planar with job plane')

    if stepdown is not None and stepdown <= 0:
        raise ValueError('Stepdown must be more than zero')

    if strategy == 'contour':
        # offset 3.2 seconds and cuts 3.1 seconds
        # strategy_f = lambda boundary: perform_contour(boundary, stepover_distance=stepover * job.tool_diameter)
        strategy_f = lambda boundary: perform_pyclipper_contour(boundary,
                                                                stepover_distance=stepover * job.tool_diameter)
    else:
        raise ValueError(f'Invalid strategy "{strategy}"')

    faces = [face.transformShape(job.top.fG) for face in faces]
    faces = sort_by_depth(faces)

    boundaries = []
    boundary_outer_offset = boundary_offset * job.tool_radius
    boundary_inner_offset = -boundary_outer_offset
    for depth, face in faces:
        try:
            for boundary in offset_face(face, boundary_outer_offset, boundary_inner_offset):
                boundaries.append((depth, boundary))
        except ValueError:
            logger.error('Failed to offset initial boundary. Tool too big?')
            raise

    # TODO implement avoid

    if stepdown is None:
        toolpaths = flatten_list([strategy_f(boundary[1]) for boundary in boundaries])
    else:
        toolpaths = []
    global DEBUG
    DEBUG = toolpaths
    return toolpaths


def perform_contour(boundary: cq.Face, stepover_distance: float) -> List[cq.Wire]:
    return perform_shrinking_contour(boundary, -stepover_distance)


def perform_shrinking_contour(boundary: cq.Face, stepover_distance):
    assert stepover_distance < 0
    stepover_distance = -abs(stepover_distance)

    outer = boundary.outerWire()
    inner_faces = [cq.Face.makeFromWires(inner) for inner in boundary.innerWires()]
    toolpaths = [outer]

    todo = [outer]
    while todo:
        contour = todo.pop()
        try:
            sub_contours = contour.offset2D(stepover_distance)
        except ValueError:
            continue

        for sub_contour in sub_contours:
            if inner_faces:
                try:
                    sub_compound = cq.Face.makeFromWires(sub_contour).cut(*inner_faces)
                    sub_faces = compound_to_faces(sub_compound)
                    sub_contour_wires = [sub_face.outerWire() for sub_face in sub_faces]
                except ValueError:
                    continue
            else:
                sub_contour_wires = [sub_contour]

            for sub_contour_wire in sub_contour_wires:
                # TODO can we have some smartness here that converts a near zero area into  just a line?
                if isclose(cq.Face.makeFromWires(sub_contour_wire).Area(), 0, abs_tol=0.001):
                    toolpaths.append(sub_contour_wire)
                    continue

                toolpaths.append(sub_contour_wire)
                todo.append(sub_contour_wire)

    toolpaths += boundary.innerWires()
    return toolpaths


def perform_contour_both(boundary: cq.Face, stepover_distance: float) -> List[cq.Wire]:
    assert stepover_distance > 0
    outer_contours, inner_contours = perform_contour_helper(boundary, stepover_distance)
    return outer_contours + inner_contours


def perform_contour_helper(contour: cq.Face, stepover_distance: float):
    outer_toolpaths = [contour.outerWire()]
    inner_toolpaths = contour.innerWires()

    try:
        sub_contours = offset_face(contour, -stepover_distance, stepover_distance)
    except ValueError:
        try:
            sub_contours = offset_face(contour, -stepover_distance, 0)
            outer_toolpaths += [sub_contour.outerWire() for sub_contour in sub_contours]
            return outer_toolpaths, inner_toolpaths
        except ValueError:
            return [], []

    for sub_contour in sub_contours:
        if sub_contour.Area() == 0:
            sub_edges = sub_contour.Edges()
            if len(sub_edges) == 2:
                outer_toolpaths.append(sub_edges[0])
        else:
            sub_outer_toolpaths, sub_inner_toolpaths = perform_contour_helper(sub_contour, stepover_distance)
            outer_toolpaths += sub_outer_toolpaths
            inner_toolpaths = sub_inner_toolpaths + inner_toolpaths

    return outer_toolpaths, inner_toolpaths


def perform_shrinking_contour_step(contour: cq.Wire, inner_contours: List[cq.Wire], stepover_distance: float):
    try:
        sub_contours = contour.offset2D(-stepover_distance)
    except ValueError:
        return []

    all_contours = []
    for sub_contour in sub_contours:
        if inner_contours:
            sub_face = cq.Face.makeFromWires(sub_contour)

        if sub_contour.Area() == 0:
            sub_edges = sub_contour.Edges()
            if len(sub_edges) == 2:
                all_contours.append(sub_edges[0])
            else:
                logger.warning(f'Encountered a strange contour with {len(sub_edges)} edges')
        else:
            all_contours += perform_shrinking_contour()


def wire_to_path(wire: cq.Wire):
    edges = wire_to_ordered_edges(wire)
    sp = edge_start_point(edges[0])
    path = [[sp.x, sp.y]]
    for edge in edges:
        geom_type = edge.geomType()
        if geom_type == 'OFFSET':
            geom_type = get_underlying_geom_type(edge)
        ep = edge_end_point(edge)
        if geom_type == 'LINE':
            path.append([ep.x, ep.y])

        elif geom_type in ['ARC', 'CIRCLE', 'BSPLINE', 'BEZIER']:
            path += interpolate_edge_as_2d_path(edge)[1:]

        else:
            raise RuntimeError(f'Unsupported geom type: {geom_type}')
    return path


def perform_pyclipper_contour(boundary: cq.Face, stepover_distance: float):
    assert stepover_distance > 0
    stepover_distance = pyclipper.scale_to_clipper(-abs(stepover_distance))

    clipper = pyclipper.Pyclipper()
    clipper_offset = pyclipper.PyclipperOffset()

    outer_boundary_path = wire_to_path(boundary.outerWire())
    inner_boundary_paths = [wire_to_path(inner) for inner in boundary.innerWires()]

    scaled_outer_boundary_path = pyclipper.scale_to_clipper(outer_boundary_path)
    scaled_inner_boundary_paths = [pyclipper.scale_to_clipper(inner) for inner in inner_boundary_paths]

    todo = [scaled_outer_boundary_path]
    results = []
    while todo:
        parent_contour = todo.pop()
        #clipper_offset.Clear()
        clipper_offset = pyclipper.PyclipperOffset(arc_tolerance=pyclipper.scale_to_clipper(0.1))
        clipper_offset.AddPath(path=parent_contour, join_type=pyclipper.JT_ROUND, end_type=pyclipper.ET_CLOSEDPOLYGON)
        sub_contours = clipper_offset.Execute(stepover_distance)
        for sub_contour in sub_contours:
            if scaled_inner_boundary_paths:
                clipper.Clear()
                clipper.AddPaths(paths=scaled_inner_boundary_paths, poly_type=pyclipper.PT_CLIP, closed=True)
                clipper.AddPath(path=sub_contour, poly_type=pyclipper.PT_SUBJECT, closed=True)
                split_sub_contours = clipper.Execute(pyclipper.CT_DIFFERENCE)
                for split_sub_contour in split_sub_contours:
                    results.append(split_sub_contour)
                    todo.append(split_sub_contour)
            else:
                results.append(sub_contour)
                todo.append(sub_contour)
        #break
    results = [pyclipper.scale_from_clipper(result) for result in results]
    z = boundary.Center().z
    # Clipper returns path without last vertex
    for result in results:
        if result[0] != result[-1]:
            result.append(result[0][:])

    results = [outer_boundary_path] + results + inner_boundary_paths

    for result in results:
        for point in result:
            point.append(z)
    return results
    # Reconstruct wires

    result_wires = []
    for result in results:
        print("Makeline start")
        first = result[0]
        last = result[-1]
        result.append(first)
        edges = [cq.Edge.makeLine(
            cq.Vector(p1[0], p1[1], z),
            cq.Vector(p2[0], p2[1], z)) for p1, p2 in zip(result, result[1:])]
        print(f"{len(edges)} edges")
        print("Makeline end, assemble start")

        print("Assemble end")
        # check geomtype
        try:
            [edge.geomType() for edge in edges]
        except:
            print(result)
            #for i,e in enumerate(edges):
            #    print("edge i", i, e.geomType())
            continue
        result_wires.append(cq.Wire.assembleEdges(edges))

    result_wires = [boundary.outerWire()] + result_wires + boundary.innerWires()
    return result_wires
