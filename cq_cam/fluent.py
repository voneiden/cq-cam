from __future__ import annotations

import itertools
from copy import copy
from typing import List

from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCP.BRepLib import BRepLib_FindSurface
from OCP.ShapeFix import ShapeFix_Face, ShapeFix_Shape
from OCP.TopoDS import TopoDS
from cadquery import cq, Compound, Wire

#from cq_cam import Profile
from cq_cam.commands.util_command import wire_to_command_sequence2
from cq_cam.common import Unit
from cq_cam.utils.utils import extract_wires, compound_to_edges, filter_edges_below_plane, wire_to_ordered_edges



class JobV2:
    def __init__(self,
                 top: cq.Plane,
                 feed: float,
                 tool_diameter: float,
                 plunge_feed: float = None,
                 rapid_height: float = None,
                 op_safe_height: float = None,
                 gcode_precision: int = 3,
                 unit: Unit = Unit.METRIC):
        self.top = top
        self.top_plane_face = cq.Face.makePlane(None, None, top.origin, top.zDir)
        self.feed = feed
        self.tool_diameter = tool_diameter
        self.plunge_feed = feed if plunge_feed is None else plunge_feed
        self.rapid_height = self._default_rapid_height(unit) if rapid_height is None else rapid_height
        self.op_safe_height = self._default_op_safe_height(unit) if op_safe_height is None else op_safe_height
        self.gcode_precision = gcode_precision
        self.unit = unit

        self.max_stepdown_count = 100

        self.operations = []

    @staticmethod
    def _default_rapid_height(unit: Unit):
        if unit == Unit.METRIC:
            return 10
        return 0.4

    @staticmethod
    def _default_op_safe_height(unit: Unit):
        if unit == Unit.METRIC:
            return 1
        return 0.04

    def to_gcode(self):
        task_break = "\n\n\n"
        return f"{self.unit.to_gcode()}\n{task_break.join(task.to_gcode() for task in self.operations)}"

    def _add_operation(self, operation):
        job = copy(self)
        job.operations = [*self.operations, operation]
        return job

    def profile(self, shape, offset=1, offset_inner=None, stepdown=None):
        if offset_inner is None:
            offset_inner = -offset
        outers, inners = extract_wires(shape)

        # Transform to relative coordinates
        outers = [outer.transformShape(self.top.fG) for outer in outers]
        inners = [inner.transformShape(self.top.fG) for inner in inners]

        self.debug = outers + inners

        # Generate base features
        base_features = []
        for outer in outers:
            base_features += outer.offset2D(offset * self.tool_diameter)

        for inner in inners:
            base_features += inner.offset2D(offset_inner * self.tool_diameter)

        if stepdown:
            for base_feature in base_features:
                step = cq.Vector(0, 0, 1) * stepdown
                layers: List[cq.Wire] = [base_feature]
                for i in itertools.count():
                    if i > self.max_stepdown_count:
                        raise RuntimeError('Job.max_stepdown_count exceeded')

                    i_op: cq.Wire = base_feature.moved(cq.Location(step * (i + 1)))
                    if i_op.BoundingBox().zmin >= 0:
                        break
                    # TODO Do cutting last?
                    #edges = compound_to_edges(i_op.cut(self.top_plane_face))
                    #edges = [edge for edge in edges if edge.Center().z < 0]
                    #wires = cq.Wire.combine(edges)
                    #if not wires:
                    #    break
                    self.debug.append(i_op)
                    layers.append(i_op)

                layers.reverse()

                # How to ramp
                # 1) Calculate horizontal length of ramp
                # 2) Grab sufficient amount of edges to cover the length from both layers
                # 3) Create a face from said edges
                # 4) Create a face that passes through the ramp start and ramp end,
                #    and the perpendicular axis is horizontal
                # 5) Intersect (section?) these two and you have a wire that describes the ramp toolpath

                stepdown_angle = 0.5
                stepdown_distance = stepdown
                l1 = wire_to_ordered_edges(layers[0])
                l2 = wire_to_ordered_edges(layers[1])
                ext = cq.Solid.extrudeLinear(layers[0], [], cq.Vector(0,0, -stepdown))
                #self.debug += ext.Faces()
                i = 4
                start = l1[i].startPoint()
                end = l2[1].endPoint()

                target = end - start


                #self.debug += [cut_plane]
                #self.debug.append(cq.Edge.makeLine(start, end))



                x = target.x
                y = target.y
                z = target.z
                target.z = 0
                target.x = -y
                target.y = x
                #self.debug.append(cq.Edge.makeLine(start, start + target))

                normal = (end-start).cross(target)

                #self.debug.append(cq.Edge.makeLine(start, start + normal))


                cut_plane = cq.Face.makePlane(None, None, start, normal)
                faces = cq.Workplane(ext).faces('(not <Z) and (not >Z)').objects
                self.debug2 = [*faces, cut_plane]
                for face in faces:
                    section = BRepAlgoAPI_Section(cut_plane.wrapped, face.wrapped, True)
                    sedges = section.SectionEdges()
                    for e in sedges:
                        self.debug2.append(cq.Edge(e))

                # TODO create VerticalChain and HorizontalChain
                # for turning wire sequences into commands
                """
                commands = []
                previous_sequence = None
                for layer in layers:
                    sequence = wire_to_command_sequence2(layer)
                    #cq.Face.
                """

        else:
            toolpaths = base_features

        # Sort toolpaths highest first
        #toolpaths.sort(key=lambda tp: tp.BoundingBox().zmax, reverse=True)

        #self.debug += toolpaths
        return self._add_operation([])

if __name__ == 'temp':
    wp = cq.Workplane().box(15,10,5)
    top = wp.faces('>X').workplane()
    bottom = wp.faces('<X')
    cam = JobV2(top.plane, 100, 3.175).profile(bottom, stepdown=3)
    show_object(wp)
    show_object(cam.debug, 'stuff')
    show_object(cam.debug2, 'stuff2')
