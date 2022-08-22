# -*- coding: utf-8 -*-
# 
# Copyright (C) 2022 Frank David Martinez M. <https://github.com/mnesarco/>
# 
# This file is part of FCScript.
# 
# FCScript is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# Utils is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with FCScript.  If not, see <http://www.gnu.org/licenses/>.
# 

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] Common Builtin Imports
#  ┌────────────────────────────────────────────────────────────────────────────┐

from cmath import exp
from contextlib import contextmanager
from enum import Enum
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union
import math
import sys
import threading

try:
    from typing import Protocol, runtime_checkable
except:
    from typing_extensions import Protocol, runtime_checkable


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] FreeCAD Imports
#  ┌────────────────────────────────────────────────────────────────────────────┐

from FreeCAD import Base
import FreeCAD as App
import Part
import Sketcher
from ProfileLib import RegularPolygon
import FreeCADGui as Gui


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Lang] Typing
#  ┌────────────────────────────────────────────────────────────────────────────┐

@runtime_checkable
class ObjectWithOrigin(Protocol):
    @property
    def Origin(self) -> App.DocumentObject:
        ...


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [FreeCAD] Aliases
#  ┌────────────────────────────────────────────────────────────────────────────┐

#: Point/Vector alias
Pnt = Base.Vector

#: Vector alias
Vec = Base.Vector

#: App.Rotation alias
Rotation = App.Rotation

#: Quantity converter from string
Quantity = App.Units.Quantity

#: Commands
command = Gui.runCommand

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Constants] Common constants
#  ┌────────────────────────────────────────────────────────────────────────────┐

#: Geometry start point (except circles or ellipses)
GeomStart = 1

#: Geometry end point (except circles or ellipses)
GeomEnd = 2

#: Geometry center point (only circles and ellipses)
GeomCenter = 3

#: X Axis Geometry index in Sketch
XAxisIndex = -1

#: Y Axis Geometry index in Sketch
YAxisIndex = -2


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Util] Common utilities
#  ┌────────────────────────────────────────────────────────────────────────────┐

def to_vec(input : Any) -> Vec:
    """Convert tuple/list/vector to Vec."""
    if isinstance(input, Vec):
        return input
    if isinstance(input, (tuple, list)):
        if len(input) == 3:
            return Vec(*input)
        if len(input) == 2:
            return Vec(*input, 0)
        if len(input) == 1:
            return Vec(*input, 0, 0)
    if hasattr(input, "X"):        
        if hasattr(input, "Y"):
            if hasattr(input, "Z"):
                return Vec(input.X, input.Y, input.Z)
            else:
                return Vec(input.X, input.Y, 0)
        else:
            return Vec(input.X, 0, 0)
    if isinstance(input, (float, int)):
        return Vec(input, 0, 0)
    raise RuntimeError(f"Invalid input, {type(input)} is not convertible to Vec")


def to_vecs(*input : Any) -> Tuple[Vec, ...]:
    """Convert arguments into Vectors. See to_vec."""
    return tuple(to_vec(i) for i in input)


def find_obj_origin_geo_feature(obj: ObjectWithOrigin, prefix: str) -> App.GeoFeature:
    """Extract Axes or Planes from object's Origin"""
    for geo in obj.Origin.OutList:
        if geo.Name.startswith(prefix):
            return geo


def find_obj_origin_axis(obj: ObjectWithOrigin, prefix: str) -> App.GeoFeature:
    """Extract Axes from object's Origin"""
    return find_obj_origin_geo_feature(obj, f'{prefix.upper()}_Axis')


def find_obj_origin_plane(obj: ObjectWithOrigin, prefix: str) -> App.GeoFeature:
    """Extract Planes from object's Origin"""
    return find_obj_origin_geo_feature(obj, f'{prefix.upper()}_Plane')


def recompute():
    """Recompute the whole active document"""
    App.ActiveDocument.recompute()


def Dx(v: Union[float, Vec]):
    """Vector in x direction"""
    if isinstance(v, (float, int)):
        return Vec(v, 0, 0)
    return Vec(v.X, 0, 0)


def Dy(v: Union[float, Vec]):
    """Vector in y direction"""
    if isinstance(v, (float, int)):
        return Vec(0, v, 0)
    return Vec(0, v.Y, 0)


def Dz(v: Union[float, Vec]):
    """Vector in z direction"""
    if isinstance(v, (float, int)):
        return Vec(0, 0, v)
    return Vec(0, 0, v.Z)

#: Decorator
def set_function(target, attribute):
    """Decorator: Set function to an attribute of target."""
    def deco(fn):
        setattr(target, attribute, fn)
        return fn
    return deco


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Expression] Expression Engine Utilities
#  ┌────────────────────────────────────────────────────────────────────────────┐
# 
#   Example 1 (Eval):
#    e1 = Expr(".Length * 3 + <<PartX>>.Radius")
#    result = e1(App.ActiveDocument.Box)
#
#   Example 2 (Set):
#    e1.set_to(App.ActiveDocument.Pad002, 'Length')
#
#   Example 3 (Set in place)
#       bind_expr(App.ActiveDocument.Pad002, 'Length', ".Length * 3 + <<PartX>>.Radius")
#

class Expr:
    """Callable expression (FreeCAD's Expression Engine)"""

    def __init__(self, value: str, context: Any = None):
        self.value = value
        self.context = context

    def __call__(self, context: Any = None) -> Any:
        return (context or self.context).evalExpression(self.value)

    def set_to(self, obj, property: str, auto_recompute=False):
        obj.setExpression(property, self.value)
        if auto_recompute:
            recompute()

def bind_expr(obj: Any, property: str, expr: Union[str, Expr], auto_recompute: bool = False):
    if not isinstance(expr, Expr):
        expr = Expr(expr, obj)
    expr.set_to(obj, property, auto_recompute)


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Sketch] Geometry
#  ┌────────────────────────────────────────────────────────────────────────────┐

class SketchGeom:
    """Geometry info inside a sketch"""
    def __init__(self, index, obj, name=None):
        self.index = index
        self.obj = obj
        self.name = name


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Sketch] Solver
#  ┌────────────────────────────────────────────────────────────────────────────┐

class Solver(Enum):
    BFGSSolver = 0
    LevenbergMarquardtSolver = 1
    DogLegSolver = 2


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Sketch] Geometric Quantities
#  ┌────────────────────────────────────────────────────────────────────────────┐

class gq:

    class Base:
        value: float
        def __init__(self, value: Union[float,str]):
            if isinstance(value, str):
                self.value = Quantity(str)
            else:
                self.value = value

    class Radius(Base):
        def __init__(self, value: Union[float,str]):
            super().__init__(value)

    class Diameter(Base):
        def __init__(self, value: Union[float,str]):
            super().__init__(value)

    class Length(Base):
        def __init__(self, value: Union[float,str]):
            super().__init__(value)

    class Angle(Base):
        def __init__(self, value: Union[float,str]):
            super().__init__(value)

    class DeltaVec(Base):
        def __init__(self, value: any):
            self.value = to_vec(value)

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Sketch] Sketch
#  ┌────────────────────────────────────────────────────────────────────────────┐

class XSketch:
    """Sketch Wrapper"""

    DEFAULT_SOLVER : Solver = Solver.DogLegSolver

    @staticmethod
    def select_default_solver(solver: Solver):
        App.ParamGet('User parameter:BaseApp/Preferences/Mod/Sketcher').SetBool('ShowSolverAdvancedWidget', True)
        App.ParamGet('User parameter:BaseApp/Preferences/Mod/Sketcher/SolverAdvanced').SetInt('DefaultSolver', solver.value)


    def __init__(self, name: str = 'XSketch', parent: Any = None, clean: bool = True):
        XSketch.select_default_solver(XSketch.DEFAULT_SOLVER)        
        if parent:
            name = f"{parent.Name}_{name}"
        self.obj = App.ActiveDocument.getObject(name)
        if self.obj:
            if clean:
                self.obj.deleteAllGeometry()
        else:
            if parent:
                self.obj = parent.newObject("Sketcher::SketchObject", name)
            else:
                self.obj = App.ActiveDocument.addObject("Sketcher::SketchObject", name)

        self.named_geom = {}
        self._ref_mode = False
        self.parent = parent
        

    @contextmanager
    def ref_mode(self, mode : bool = True):
        saved = self._ref_mode
        if saved == mode:
            yield self
        else:
            self._ref_mode = mode
            try:
                yield self
            finally:
                self._ref_mode = saved


    def set_geom_mode(self):
        self._ref_mode = False

    #  └────────────────────────────────────────────────────────────────────────────┘
    #    [SECTION] [Sketch/Geometry]
    #  ┌────────────────────────────────────────────────────────────────────────────┐

    def g_line(self, start, end):
        """Draw a line segment from start to end"""
        p0, p1 = to_vec(start), to_vec(end)
        line = Part.LineSegment(p0, p1)
        index = self.obj.addGeometry(line, self._ref_mode)
        return SketchGeom(index, line)


    def g_regular_polygon(self, p1, p2, edges):
        """Draw a regular polygon (center, vertex, edges)."""
        RegularPolygon.makeRegularPolygon(self.obj, edges, to_vec(p1), to_vec(p2), self._ref_mode)
        index = len(self.obj.Geometry) - 1
        return SketchGeom(index, self.obj.Geometry[index])


    def g_bspline(self, poles, mults=None, knots=None, periodic=False, degree=3, weights=None, check_rational=False):
        """Draw a bspline"""
        vec_poles = [to_vec(p) for p in poles]
        bspline = Part.BSplineCurve(vec_poles, mults, knots, periodic, degree, weights, check_rational)
        index = self.obj.addGeometry(bspline, self._ref_mode)
        return SketchGeom(index, bspline)


    def g_circle_center_radius(self, cnt, rad):
        c = to_vec(cnt)
        circle = Part.Circle(c, Vec(0,0,1), rad)
        index = self.obj.addGeometry(circle, self._ref_mode)
        return SketchGeom(index, circle)
    

    def g_circle_3points(self, p1, p2, p3):
        v1, v2, v3 = to_vecs(p1, p2, p3)
        circle = Part.Circle(v1, v2, v3)
        index = self.obj.addGeometry(circle, self._ref_mode)
        return SketchGeom(index, circle)
    

    def g_arc_3points(self, p1, p2, p3):
        v1, v2, v3 = to_vecs(p1, p2, p3)
        arc = Part.Arc(v1, v2, v3)
        index = self.obj.addGeometry(arc, self._ref_mode)
        return SketchGeom(index, arc)
    

    def g_arc_center_radius(self, cnt, rad, start=0, end=math.radians(180)):
        c = to_vec(cnt)
        circle = Part.Circle(c, Vec(0,0,1), rad)
        arc = Part.ArcOfCircle(circle, start, end)
        index = self.obj.addGeometry(arc, self._ref_mode)
        return SketchGeom(index, arc)   


    def g_point(self, p):
        pnt = Part.Point(p)
        index = self.obj.addGeometry(pnt, self._ref_mode)
        return SketchGeom(index, pnt)   

    #  └────────────────────────────────────────────────────────────────────────────┘
    #    [SECTION] [Sketch/Constraints]
    #  ┌────────────────────────────────────────────────────────────────────────────┐

    def rename_constraint(self, constraint, name):
        if name:
            self.obj.renameConstraint(constraint, name)


    def c_coincident(self, g1, g1c, g2, g2c, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("Coincident", g1, g1c, g2, g2c))
        self.rename_constraint(c, name)
        return c


    def c_vertical(self, index, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("Vertical", index))
        self.rename_constraint(c, name)
        return c


    def c_horizontal(self, index, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("Horizontal", index))
        self.rename_constraint(c, name)
        return c


    def c_coincident_end_start(self, g1, g2, name=None):
        g_prev = self.obj.Geometry[g1]
        g_current = self.obj.Geometry[g2]
        c_end = GeomEnd if hasattr(g_prev, "EndPoint") else GeomCenter 
        c_start = GeomStart if hasattr(g_current, "StartPoint") else GeomCenter 
        c = self.c_coincident(g1, c_end, g2, c_start)
        self.rename_constraint(c, name)
        return c


    def c_x_angle(self, index: int, angle: Union[float, Expr], name=None):
        return self.c_angle(XAxisIndex, GeomStart, index, GeomStart, angle, name=name)


    def c_y_angle(self, index, angle: Union[float, Expr], name=None):
        return self.c_angle(YAxisIndex, GeomStart, index, GeomStart, angle, name=name)


    def c_angle(self, g1, g1c, g2, g2c, angle: Union[float, Expr], name=None):
        if isinstance(angle, Expr):
            value = angle(self.obj)
            c = self.obj.addConstraint(Sketcher.Constraint("Angle", g1, g1c, g2, g2c, value))
            angle.set_to(self.obj, f'.Constraints[{c}]')
        else:
            c = self.obj.addConstraint(Sketcher.Constraint("Angle", g1, g1c, g2, g2c, angle))
        self.rename_constraint(c, name)
        return c


    def c_length(self, index, length: Union[float, Expr], name=None):
        if isinstance(length, Expr):
            value = length(self.obj)
            c = self.obj.addConstraint(Sketcher.Constraint("Distance", index, value))
            length.set_to(self.obj, f'.Constraints[{c}]')
        else:
            c = self.obj.addConstraint(Sketcher.Constraint("Distance", index, length))
        self.rename_constraint(c, name)
        return c


    def c_distance(self, g1, g1p, g2, g2p, dist: Union[float, Expr], name=None):
        if isinstance(dist, Expr):
            value = dist(self.obj)
            c = self.obj.addConstraint(Sketcher.Constraint("Distance", g1, g1p, g2, g2p, value))
            dist.set_to(self.obj, f'.Constraints[{c}]')
        else:
            c = self.obj.addConstraint(Sketcher.Constraint("Distance", g1, g1p, g2, g2p, dist))
        self.rename_constraint(c, name)
        return c


    def c_perpendicular(self, g1, g2, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("Perpendicular", g1, g2))
        self.rename_constraint(c, name)
        return c


    def c_parallel(self, g1, g2, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("Parallel", g1, g2))
        self.rename_constraint(c, name)
        return c


    def _c_xy(self, g, distance, type, anchor=GeomStart, name=None):
        if isinstance(distance, Expr):
            value = distance(self.obj)
            c = self.obj.addConstraint(Sketcher.Constraint(f"Distance{type}", g, anchor, value))
            distance.set_to(self.obj, f'.Constraints[{c}]')
        else:
            c = self.obj.addConstraint(Sketcher.Constraint(f"Distance{type}", g, anchor, distance))
        self.rename_constraint(c, name)
        return c


    def c_x(self, g, distance, anchor=GeomStart, name=None):
        return self._c_xy(g, distance, 'X', anchor, name)


    def c_y(self, g, distance, anchor=GeomStart, name=None):
        return self._c_xy(g, distance, 'Y', anchor, name)


    def c_xy(self, g, x, y, anchor=GeomStart, name=None):
        c1 = self.c_x(g, x, anchor=anchor, name=None if name is None else f'{name}_x')
        c2 = self.c_y(g, y, anchor=anchor, name=None if name is None else f'{name}_y')
        return c1, c2


    def c_point_on_object(self, pnt, obj, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("PointOnObject", pnt, GeomStart, obj))
        self.rename_constraint(c, name)
        return c


    def c_equal(self, g1, g2, name=None):
        c = self.obj.addConstraint(Sketcher.Constraint("Equal", g1, g2))
        self.rename_constraint(c, name)
        return c


    def c_diameter(self, g, diameter, name=None):
        if isinstance(diameter, Expr):
            value = diameter(self.obj)
            c = self.obj.addConstraint(Sketcher.Constraint("Diameter", g, value))
            diameter.set_to(self.obj, f'.Constraints[{c}]')
        else:
            c = self.obj.addConstraint(Sketcher.Constraint("Diameter", g, diameter))
        self.rename_constraint(c, name)
        return c


    def c_radius(self, g, radius, name=None):
        if isinstance(radius, Expr):
            value = radius(self.obj)
            c = self.obj.addConstraint(Sketcher.Constraint("Radius", g, value))
            radius.set_to(self.obj, f'.Constraints[{c}]')
        else:
            c = self.obj.addConstraint(Sketcher.Constraint("Radius", g, radius))
        self.rename_constraint(c, name)
        return c


    def c_auto_coincident(self, name=None):
        g_len = len(self.obj.Geometry)
        if g_len > 1:
            return self.c_coincident_end_start(g_len-2, g_len-1, name=name)


    def c_bspline_control_point(self, g, g_c, bspl, bspl_c, weight=1.0):
        with self.ref_mode():
            geom = self.obj.Geometry[g]
            pnt = None
            if g_c == GeomStart:
                if hasattr(geom, "StartPoint"):
                    pnt = geom.StartPoint
                elif hasattr(geom, "X"):
                    pnt = Pnt(geom.X, geom.Y, geom.Z)
            elif g_c == GeomEnd:
                if hasattr(geom, "EndPoint"):
                    pnt = geom.EndPoint
                elif hasattr(geom, "X"):
                    pnt = Pnt(geom.X, geom.Y, geom.Z)
            elif g_c == GeomCenter:
                if hasattr(geom, "Center"):
                    pnt = geom.Center
                elif hasattr(geom, "X"):
                    pnt = Pnt(geom.X, geom.Y, geom.Z)
            if pnt is None:
                raise RuntimeError(f"Unsupported constraint {g}.{g_c}")

            cpc = self.g_circle_center_radius(pnt, 10)
            self.obj.addConstraint(Sketcher.Constraint('Weight', cpc.index, weight)) 
            self.obj.addConstraint(Sketcher.Constraint('Coincident', cpc.index, GeomCenter, g, g_c))
            self.obj.addConstraint(Sketcher.Constraint('InternalAlignment:Sketcher::BSplineControlPoint', cpc.index, GeomCenter, bspl, bspl_c))

    #  └────────────────────────────────────────────────────────────────────────────┘
    #    [SECTION] [Sketch/Lookup]
    #  ┌────────────────────────────────────────────────────────────────────────────┐

    def g_by_name(self, name):
        index = self.named_geom.get(name, None)
        if index:
            return SketchGeom(index, self.obj.Geometry[index], name)

    def g_by_index(self, index):
        return SketchGeom(index, self.obj.Geometry[index])

    def set_name(self, name, geom):
        if name in self.named_geom:
            raise RuntimeError("Duplicated name")
        if isinstance(geom, SketchGeom):
            self.named_geom[name] = geom.index
        else:
            self.named_geom[name] = geom

    #  └────────────────────────────────────────────────────────────────────────────┘
    #    [SECTION] [Sketch/Builders]
    #  ┌────────────────────────────────────────────────────────────────────────────┐

    def create_group(self, auto_coincident=True):
        return XSketchGroup(self, auto_coincident)

    #  └────────────────────────────────────────────────────────────────────────────┘
    #    [SECTION] [Sketch/Placement]
    #  ┌────────────────────────────────────────────────────────────────────────────┐

    def rotate(self, angle):
        self.obj.AttachmentOffset = App.Placement(
            self.obj.AttachmentOffset.Base, 
            App.Rotation(self.obj.AttachmentOffset.Rotation.Axis, angle)
        )

    def move(self, pos):
        self.obj.AttachmentOffset = App.Placement(
            to_vec(pos), 
            self.obj.AttachmentOffset.Rotation
        )

    def attach(self, pos, axis, angle):
        self.obj.AttachmentOffset = App.Placement(
            to_vec(pos), 
            App.Rotation(to_vec(axis), angle)
        )

    #  └────────────────────────────────────────────────────────────────────────────┘
    #    [SECTION] [Sketch/PartDesign]
    #  ┌────────────────────────────────────────────────────────────────────────────┐

    def pad(self, value, name='Pad', direction=None):
        name = f"{self.obj.Name}_{name}"
        if self.parent:
            feature = self.parent.getObject(name)
            if not feature:
                feature = self.parent.newObject('PartDesign::Pad', name)
            else:
                if feature.TypeId != 'PartDesign::Pad':
                    raise RuntimeError(f'The named object "{name}" already exists but it is not a PartDesign::Pad')
            feature.Profile = self.obj
            feature.Length = value
            if direction:
                feature.UseCustomVector = True
                feature.Direction = direction
            return feature


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [Sketch] Group
#  ┌────────────────────────────────────────────────────────────────────────────┐

class XSketchGroup:

    def __init__(self, sketch: XSketch, auto_coincident: bool=True):
        self.sketch = sketch
        self.begin = None
        self.auto_coincident = auto_coincident
        self.pos = Pnt(0,0,0)

    def _concat(self, pnt, g, name):
        if self.begin is not None and g.index > 0 and self.auto_coincident:
            self.sketch.c_auto_coincident()
        self.pos = pnt
        if self.begin is None:
            self.begin = g.index
        if name:
            self.sketch.set_name(name, g)

    def line_to(self, pnt_, name=None):
        pnt = to_vec(pnt_)
        g = self.sketch.g_line(self.pos, pnt)
        self._concat(pnt, g, name)
        return g

    def line(self, dx=0, dy=0, name=None):
        return self.line_to(self.pos + Vec(dx, dy, 0), name=name)

    def close(self, keep=False):
        if self.begin is not None:
            begin = self.sketch.g_by_index(self.begin)
            g = self.line_to(begin.obj.StartPoint)
            if self.auto_coincident:
                self.sketch.c_coincident_end_start(g.index, self.begin)
            if not keep:
                self.begin = None
            return g

    def by_name(self, name):
        return self.sketch.g_by_name(name)

    def move_to(self, pnt):
        self.pos = to_vec(pnt)
        self.begin = None

    def move(self, dx=0, dy=0):
        self.pos = self.pos + to_vec((dx, dy))
        self.begin = None

    def bspline_to(self, *pnts, name=None):
        pnt = to_vec(pnts[-1])
        g = self.sketch.g_bspline([self.pos, *pnts])
        self._concat(pnt, g, name)
        return g
        
    def bspline(self, *pnts, name=None):
        start = self.pos
        apnts = []
        for pnt in pnts:
            start = start + pnt
            apnts.append(start)
        pnt = to_vec(apnts[-1])
        g = self.sketch.g_bspline([self.pos, *apnts])
        self._concat(pnt, g, name)
        return g
        
    def circle(self, radius, name=None):
        g = self.sketch.g_circle_center_radius(self.pos, radius)
        self._concat(self.pos, g, name)
        return g      

    def point(self, name=None):
        g = self.sketch.g_point(self.pos)
        self.pos = to_vec(g.obj)
        if name:
            self.sketch.set_name(name, g)            
        return g

    def circle_3points(self, p2, p3, name=None):
        g = self.sketch.g_circle_3points(self.pos, p2, p3)
        self._concat(to_vec(p3), g, name)
        return g      

    def rect_to(self, pnt):
        orig = self.pos
        self.move_to(orig)
        g = self.line_to((orig[0], pnt[1]))
        self.sketch.c_vertical(g.index)
        g = self.line_to(pnt)
        self.sketch.c_horizontal(g.index)
        g = self.line_to((pnt[0], orig[1]))
        self.sketch.c_vertical(g.index)
        g = self.close()
        self.sketch.c_horizontal(g.index)
        return g

    def rect(self, w, h, angle=Quantity('0.0 deg')):
        if w <= 0:
            raise(RuntimeError(f'Invalid w={w} too small'))
        if h <= 0:
            raise(RuntimeError(f'Invalid h={h} too small'))
        orig = self.pos
        self.move_to(orig)
        g1 = self.line(dx=w)
        self.sketch.c_length(g1.index, w)
        g2 = self.line(dy=h)
        self.sketch.c_length(g2.index, h)
        self.sketch.c_angle(g1.index, GeomEnd, g2.index, GeomStart, Quantity('270 deg'))
        g3 = self.line(dx=-w)
        self.sketch.c_length(g3.index, w)
        self.sketch.c_angle(g2.index, GeomEnd, g3.index, GeomStart, Quantity('270 deg'))
        g4 = self.close()
        self.sketch.c_x_angle(g1.index, angle)
        self.sketch.c_xy(g1.index, orig[0], orig[1])
        return g4

    def regular_polygon(self, param: any, edges: int, angle=None, constrain_pos: Optional[bool]=None, constrain_size: bool=True):
        orig = self.pos
        prev_index = len(self.sketch.obj.Geometry) - 1
        if isinstance(param, gq.Length):
            r = param.value / (2 * math.sin(math.pi/edges))
            p2 = orig + Vec(0, r, 0)
        elif isinstance(param, gq.Radius):
            r = param.value
            p2 = orig + Vec(0, r, 0)
        elif isinstance(param, gq.Diameter):
            r = param.value/2.0
            p2 = orig + Vec(0, r, 0)
        elif isinstance(param, gq.DeltaVec):
            r = param.value.Length
            p2 = orig + param.value
        elif isinstance(param, (float,int)):  # Implies Diameter
            r = param/2.0
            p2 = orig + Vec(0, r, 0)
        else: # Absolute point
            p2 = to_vec(param)
            r = (orig - p2).Length
        g = self.sketch.g_regular_polygon(orig, p2, edges)

        if constrain_pos is True:
            self.sketch.c_xy(g.index, orig[0], orig[1], anchor=GeomCenter)
        elif constrain_pos is None and self.auto_coincident and prev_index > -1 and self.begin:
            self.sketch.c_coincident_end_start(prev_index, g.index)

        if constrain_size:
            if isinstance(param, gq.Length):
                self.sketch.c_length(g.index-1, param.value)                
            elif isinstance(param, gq.Radius):
                self.sketch.c_radius(g.index, param.value)
            elif isinstance(param, gq.Diameter):
                self.sketch.c_diameter(g.index, param.value)
            elif isinstance(param, gq.DeltaVec):
                self.sketch.c_radius(g.index, r)
            else:
                length = r * (2 * math.sin(math.pi/edges))
                self.sketch.c_length(g.index-1, length)           

        if angle is not None:
            with self.sketch.ref_mode():
                seg = self.sketch.g_line(orig, p2)
                self.sketch.c_coincident(g.index, GeomCenter, seg.index, GeomStart)
                self.sketch.c_coincident(prev_index+1, GeomStart, seg.index, GeomEnd)
                self.sketch.c_y_angle(seg.index, angle)

        return g


    def line_with_length(self, dx=0, dy=0, ref=False, constraint_name=None):
        with self.sketch.ref_mode(ref):
            if dx == 0 and dy != 0:
                length = abs(dy)
            elif dx != 0 and dy == 0:
                length = abs(dx)
            elif dx != 0 and dy != 0:
                length = math.sqrt(dx*dx + dy*dy)
            else:
                raise(RuntimeError(f"Invalid (dx, dy) = ({dx}, {dy})"))
            seg = self.line(dx, dy)
            self.sketch.c_length(seg.index, length, name=constraint_name)
            return seg


    def rect_rounded(self, w, h, r, angle=Quantity('0.0 deg')):
        """
        r: int | float => for all corners
           (rw, rh) => width and height radious
           (tl, tr, br, bl) => custom radius for each corner
        """
        if w <= 0:
            raise(RuntimeError(f'Invalid w={w} too small'))
        if h <= 0:
            raise(RuntimeError(f'Invalid h={h} too small'))

        BL, BR, TR, TL = 0, 1, 2, 3 # Corners
        RW, RH = 0, 1 # Radiuses
        
        # transform r into ( (blw, blh), (brw, brh), (trw, trh), (tlw, tlh) )
        if isinstance(r, (float, int)):
            r = ((r,r),)*4
        elif len(r) == 1:
            r = ((r,r),)*4
        elif len(r) == 2:
            r = ((r[0],r[1]),)*4
        elif len(r) == 4:
            r = ((r[0],r[0]),(r[1],r[1]),(r[2],r[2]),(r[3],r[3]))
        else:
            raise(RuntimeError(f'Invalid r={r}'))

        if any((x[RW] <= 0 or x[RH] <= 0 for x in r)):
            raise(RuntimeError(f'Invalid r={r} too small values'))

        if (   (r[BL][RW] + r[BR][RW] > w) 
            or (r[TR][RW] + r[TL][RW] > w) 
            or (r[BL][RH] + r[TL][RH] > h) 
            or (r[BR][RH] + r[TR][RH] > h)):
            raise(RuntimeError(f'Invalid r={r} too large values'))
 
        orig = self.pos

        self.move_to(orig)

        angle180 = Quantity('180 deg')
        angle270 = Quantity('270 deg')

        # Bottom
        bl = self.line_with_length(dx=r[BL][RW], ref=True)
        bottom = self.line_with_length(dx=w - r[BL][RW] - r[BR][RW])
        br = self.line_with_length(dx=r[BR][RW], ref=True)

        self.sketch.c_angle(bl.index, GeomEnd, bottom.index, GeomStart, angle180)
        self.sketch.c_angle(bottom.index, GeomEnd, br.index, GeomStart, angle180)

        # Right
        rb = self.line_with_length(dy=r[BR][RH], ref=True)
        right = self.line_with_length(dy=h - r[BR][RH] - r[TR][RH])
        rt = self.line_with_length(dy=-r[TR][RH], ref=True)

        self.sketch.c_angle(br.index, GeomEnd, rb.index, GeomStart, angle270)
        self.sketch.c_angle(rb.index, GeomEnd, right.index, GeomStart, angle180)
        self.sketch.c_angle(right.index, GeomEnd, rt.index, GeomStart, angle180)

        # Top
        tr = self.line_with_length(dx=-r[TR][RW], ref=True)
        top = self.line_with_length(dx=-w + r[TR][RW] + r[TL][RW])
        tl = self.line_with_length(dx=-r[TL][RW], ref=True)

        self.sketch.c_angle(rt.index, GeomEnd, tr.index, GeomStart, angle270)
        self.sketch.c_angle(tr.index, GeomEnd, top.index, GeomStart, angle180)
        self.sketch.c_angle(top.index, GeomEnd, tl.index, GeomStart, angle180)

        # Left
        lt = self.line_with_length(dy=-r[TL][RH], ref=True)
        left = self.line_with_length(dy=-h + r[BL][RH] + r[TL][RH])
        lb = self.line_with_length(dy=r[BL][RH], ref=True)

        self.sketch.c_angle(tl.index, GeomEnd, lt.index, GeomStart, angle270)
        self.sketch.c_angle(lt.index, GeomEnd, left.index, GeomStart, angle180)
        self.sketch.c_angle(left.index, GeomEnd, lb.index, GeomStart, angle180)

        # Corner: BL
        self.move_to(left.obj.EndPoint)
        blc = self.bspline_to(lb.obj.EndPoint, bottom.obj.StartPoint)
        self.sketch.c_bspline_control_point(left.index, GeomEnd, blc.index, 0)
        self.sketch.c_bspline_control_point(lb.index, GeomEnd, blc.index, 1)
        self.sketch.c_bspline_control_point(bottom.index, GeomStart, blc.index, 2)

        # Corner: BR
        self.move_to(bottom.obj.EndPoint)
        blc = self.bspline_to(br.obj.EndPoint, rb.obj.StartPoint)
        self.sketch.c_bspline_control_point(bottom.index, GeomEnd, blc.index, 0)
        self.sketch.c_bspline_control_point(br.index, GeomEnd, blc.index, 1)
        self.sketch.c_bspline_control_point(right.index, GeomStart, blc.index, 2)

        # Corner: TR
        self.move_to(right.obj.EndPoint)
        blc = self.bspline_to(rt.obj.EndPoint, top.obj.StartPoint)
        self.sketch.c_bspline_control_point(right.index, GeomEnd, blc.index, 0)
        self.sketch.c_bspline_control_point(rt.index, GeomEnd, blc.index, 1)
        self.sketch.c_bspline_control_point(top.index, GeomStart, blc.index, 2)

        # Corner: TL
        self.move_to(top.obj.EndPoint)
        blc = self.bspline_to(tl.obj.EndPoint, left.obj.StartPoint)
        self.sketch.c_bspline_control_point(top.index, GeomEnd, blc.index, 0)
        self.sketch.c_bspline_control_point(tl.index, GeomEnd, blc.index, 1)
        self.sketch.c_bspline_control_point(left.index, GeomStart, blc.index, 2)
      
        # Pos
        self.sketch.c_xy(bl.index, orig[0], orig[1])
        
        # Angle
        self.sketch.c_x_angle(bottom.index, angle)

        return blc


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [PartDesign] Body
#  ┌────────────────────────────────────────────────────────────────────────────┐


class XBody:
    """PartDesign Body builder"""

    def __init__(self, name='XBody'):
        """Create or reuse a PartDesign Body"""        
        self.obj = App.ActiveDocument.getObject(name)
        if not self.obj:
            self.obj = App.ActiveDocument.addObject('PartDesign::Body', name)
        else:
            if self.obj.TypeId != 'PartDesign::Body':
                raise RuntimeError(f'The named object "{name}" already exists but it is not a PartDesign::Body')


    @property
    def name(self):
        return self.obj.Name


    def sketch(self, name:str='XSketch', plane:str='XY', reversed:bool=False, pos:Vec=Vec(0,0,0), rot:Rotation=Rotation(0,0,0)):
        """Sketch builder."""
        support = find_obj_origin_plane(self.obj, plane)
        return self.sketch_on([(support, '')], name=name, reversed=reversed, pos=pos, rot=rot)


    def sketch_on(self, support, mode='FlatFace', parameter=0.0, name:str='XSketch', reversed:bool=False, pos:Vec=Vec(0,0,0), rot:Rotation=Rotation(0,0,0)):
        """Sketch builder."""
        xsketch = XSketch(name, parent=self.obj)
        sketch = xsketch.obj
        sketch.AttachmentOffset = App.Placement(pos, rot)
        sketch.MapReversed = reversed
        sketch.Support = support
        sketch.MapPathParameter = parameter
        sketch.MapMode = mode
        return xsketch


    def fillet(self, edges: list, name='Fillet'):
        name = f"{self.name}_{name}"
        fillet = App.ActiveDocument.getObject(name)
        if not fillet:
            fillet = App.ActiveDocument.addObject('Part::Fillet', name)
        fillet.Base = self.obj
        fillet.Edges = [*edges]
        Gui.ActiveDocument.getObject(self.name).Visibility = False


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [FeaturePython] Data Object
#  ┌────────────────────────────────────────────────────────────────────────────┐

class DataObject:
    """Document Object with properties"""

    def __init__(self, name="Data"):
        obj = App.ActiveDocument.getObject(name)
        if not obj:
            obj = App.ActiveDocument.addObject('App::FeaturePython', name)
        else:
            if obj.TypeId != 'App::FeaturePython':
                raise RuntimeError(f'The named object "{name}" already exists but it is not of type App::FeaturePython')
        super().__setattr__('obj', obj)
    
    def add_property(self, property_type, name, section="Properties", docs="", mode=0):
        try:
            self.obj.addProperty(property_type, name, section, docs, mode)
        except:
            pass # Ignore if already exists

    def __setattr__(self, __name: str, __value: Any) -> None:
        if hasattr(self, __name):
            super().__setattr__(__name, __value)
        else:
            if isinstance(__value, Expr):
                self.obj.__setattr__(__name, __value(self.obj))
                __value.set_to(self.obj, __name)
            else:
                self.obj.setExpression(__name, None)
                self.obj.__setattr__(__name, __value)

    def __getattr__(self, __name: str) -> Any:
        return self.obj.__getattr(__name)

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Imports
#  ┌────────────────────────────────────────────────────────────────────────────┐

from PySide import QtCore, QtGui

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Aliases
#  ┌────────────────────────────────────────────────────────────────────────────┐

Icon = QtGui.QIcon
Image = QtGui.QPixmap

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Globals
#  ┌────────────────────────────────────────────────────────────────────────────┐

ThreadLocalGuiVars = threading.local()  # Store GUI State per thread


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Utils
#  ┌────────────────────────────────────────────────────────────────────────────┐

def set_qt_attrs(qt_object, **kwargs):
    """Call setters on QT objects by argument names."""
    for name, value in kwargs.items():
        if value is not None:
            setter = getattr(qt_object, f'set{name[0].upper()}{name[1:]}', None)
            if setter:
                if isinstance(value, tuple):
                    setter(*value)
                else:
                    setter(value)


def setup_layout(layout, add=True, **kwargs):
    """Setup layouts adding wrapper widget if required."""
    set_qt_attrs(layout, **kwargs)
    parent = build_context().current()
    if parent.layout() is not None or add is False:
        w = QtGui.QWidget()
        w.setLayout(layout)
        if add:
            parent.layout().addWidget(w)
        with build_context().stack(w):
            yield w
    else:
        parent.setLayout(layout)
        yield parent


def place_widget(ed, label=None, stretch=0, alignment=QtCore.Qt.Alignment()):
    """Place widget in layout."""
    layout = build_context().current().layout()
    if layout is None:
        layout = QtGui.QVBoxLayout()
        build_context().current().setLayout(layout)   
    if label is None:
        build_context().current().layout().addWidget(ed, stretch, alignment)
    else:
        w = QtGui.QWidget()
        parent = QtGui.QHBoxLayout()
        parent.addWidget(QtGui.QLabel(label))
        parent.addWidget(ed)
        w.setLayout(parent)        
        build_context().current().layout().addWidget(w, stretch, alignment)


class PySignal:
    """Imitate Qt Signals for non QObject objects"""

    def __init__(self):
        self._listeners = []

    def connect(self, listener):
        self._listeners.append(listener)

    def trigger(self, *args, **kwargs):
        for listener in self._listeners:
            listener(*args, **kwargs)


#: Decorator
def on_event(target, event=None):
    """Decorator: Event binder"""
    if event is None:
        def deco(fn):
            target.connect(fn)
            return fn
    else:
        def deco(fn):
            getattr(target, event).connect(fn)
            return fn
    return deco


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Selection
#  ┌────────────────────────────────────────────────────────────────────────────┐

class SelectedObject:
    """Store Selection information"""
    
    def __init__(self, doc, obj, sub=None, pnt=None):
        self.doc = doc
        self.obj = obj
        self.sub = sub
        self.pnt = pnt
    
    def __iter__(self):
        yield App.getDocument(self.doc).getObject(self.obj)
        yield self.sub
        yield self.pnt

    def __repr__(self) -> str:
        return f"{self.doc}#{self.obj}.{self.sub}"

    def __hash__(self) -> int:
        return hash((self.doc, self.obj, self.sub))

    def __eq__(self, __o: object) -> bool:
        return hash(self) == hash(__o)

    def __ne__(self, __o: object) -> bool:
        return not self.__eq__(__o)


def register_select_observer(owner: QtGui.QWidget, observer):
    """Add observer with auto remove on owner destroyed"""
    Gui.Selection.addObserver(observer)
    def destroyed(_):
        Gui.Selection.removeObserver(observer)
    owner.destroyed.connect(destroyed)


@contextmanager
def selection(*names, clean=True):
    sel = Gui.Selection
    try:
        doc = App.ActiveDocument.Name
        if len(names) == 0:
            yield sel.getSelection(doc)
        else:
            sel.clearSelection()
            for name in names:
                if isinstance(name, (tuple, list)):
                    sel.addSelection(doc, *name)
                elif isinstance(name, SelectedObject):
                    sel.addSelection(name.doc, name.obj, name.sub)
                else:
                    sel.addSelection(doc, name)
            yield sel.getSelection(doc)
    finally:
        if clean:
            sel.clearSelection()


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Build Context
#  ┌────────────────────────────────────────────────────────────────────────────┐

def build_context():
    bc = getattr(ThreadLocalGuiVars, 'BuildContext', None)
    if bc is None:
        ThreadLocalGuiVars.BuildContext = _BuildContext()
        return ThreadLocalGuiVars.BuildContext
    else:
        return bc

class _BuildContext:
    def __init__(self):
        self._stack = []

    def push(self, widget):
        self._stack.append(widget)

    def pop(self):
        self._stack.pop()

    @contextmanager
    def stack(self, widget):
        self.push(widget)
        try:
            yield widget
        finally:
            self.pop()

    @contextmanager
    def parent(self):
        if len(self._stack) > 1:
            current = self._stack[-1]
            self._stack.pop()
            parent = self._stack[-1]
            try:
                yield parent
            finally:
                self._stack.append(current)

    def current(self):
        return self._stack[-1]

    def dump(self):
        print(f"BuildContext: {self._stack}")    

@contextmanager
def Parent():
    """Put parent on top of BuildContext"""
    with build_context().parent() as p:
        yield p


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] Dialog
#  ┌────────────────────────────────────────────────────────────────────────────┐

class Dialogs:
    _list = []

    @classmethod
    def dump(cls):
        print(f"Dialogs: {cls._list}")

    @classmethod
    def register(cls, dialog):
        cls._list.append(dialog)
        dialog.closeEvent = lambda e: cls.destroy_dialog(dialog)

    @classmethod
    def destroy_dialog(cls, dlg):
        cls._list.remove(dlg)
        dlg.deleteLater()


@contextmanager
def Dialog(title=None, size=None, show=True, parent=None):
    if parent is None:
        w = QtGui.QDialog(parent=Gui.getMainWindow())
    else:
        w = QtGui.QWidget(parent=parent)
    if title is not None:
        w.setWindowTitle(title)
    with build_context().stack(w):
        yield w
        if show:
            Dialogs.register(w)
            w.show()
            if isinstance(size, (tuple,list)):
                w.resize(size[0], size[1])
            else:
                w.adjustSize()


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] GroupBox
#  ┌────────────────────────────────────────────────────────────────────────────┐

@contextmanager
def GroupBox(title=None):
    w = QtGui.QGroupBox()
    if title:
        w.setTitle(title)
    place_widget(w)
    with build_context().stack(w):
        yield w


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] Stretch
#  ┌────────────────────────────────────────────────────────────────────────────┐

def Stretch(stretch=0):
    """Add Layout spacer"""
    layout = build_context().current().layout()
    if layout:
        layout.addStretch(stretch)


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] TabContainer
#  ┌────────────────────────────────────────────────────────────────────────────┐

@contextmanager
def TabContainer(**kwargs):
    w = QtGui.QTabWidget()
    set_qt_attrs(w, **kwargs)
    place_widget(w)
    with build_context().stack(w):
        yield w


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] Tab
#  ┌────────────────────────────────────────────────────────────────────────────┐

@contextmanager
def Tab(title:str, icon=None):
    w = QtGui.QWidget()
    with build_context().stack(w):
        yield w
    if icon:
        build_context().current().addTab(w, title, icon)
    else:
        build_context().current().addTab(w, title)


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Layout] Col
#  ┌────────────────────────────────────────────────────────────────────────────┐

@contextmanager 
def Col(add=True, **kwargs):
    """Vertical Layout"""
    yield from setup_layout(QtGui.QVBoxLayout(), add=add, **kwargs)


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Layout] Row
#  ┌────────────────────────────────────────────────────────────────────────────┐

@contextmanager 
def Row(add=True, **kwargs):
    """Horizontal Layout"""
    yield from setup_layout(QtGui.QHBoxLayout(), add=add, **kwargs)


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] TextLabel
#  ┌────────────────────────────────────────────────────────────────────────────┐

def TextLabel(text="", stretch=0, alignment=QtCore.Qt.Alignment(), **kwargs):
    label = QtGui.QLabel(text)
    set_qt_attrs(label, **kwargs)
    place_widget(label, stretch=stretch, alignment=alignment)
    return label


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputFloat
#  ┌────────────────────────────────────────────────────────────────────────────┐

def InputFloat(name=None, min=0.0, max=sys.float_info.max, decimals=6, 
               step=0.01, label=None, value=0.0, stretch=0, 
               alignment=QtCore.Qt.Alignment(), **kwargs):
    editor = QtGui.QDoubleSpinBox()
    editor.setMinimum(min)
    editor.setMaximum(max)
    editor.setSingleStep(step)
    editor.setDecimals(decimals)
    editor.setValue(value)
    set_qt_attrs(editor, **kwargs)
    if name:
        editor.setObjectName(name)
    place_widget(editor, label=label, stretch=stretch, alignment=alignment)
    return editor


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputInt
#  ┌────────────────────────────────────────────────────────────────────────────┐

class InputTextWidget(QtGui.QLineEdit):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    def value(self):
        return self.text()
    def setValue(self, value):
        self.setText(str(value))

def InputText(name=None, label=None, value="", 
             stretch=0, alignment=QtCore.Qt.Alignment(), **kwargs):
    editor = InputTextWidget()
    editor.setText(value)
    set_qt_attrs(editor, **kwargs)
    if name:
        editor.setObjectName(name)
    place_widget(editor, label=label, stretch=stretch, alignment=alignment)
    return editor


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputInt
#  ┌────────────────────────────────────────────────────────────────────────────┐

def InputInt(name=None, min=0, max=2^31, step=1, label=None, value=0, 
             stretch=0, alignment=QtCore.Qt.Alignment(), **kwargs):
    editor = QtGui.QSpinBox()
    editor.setMinimum(min)
    editor.setMaximum(max)
    editor.setSingleStep(step)
    editor.setValue(value)
    set_qt_attrs(editor, **kwargs)
    if name:
        editor.setObjectName(name)
    place_widget(editor, label=label, stretch=stretch, alignment=alignment)
    return editor


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputBoolean
#  ┌────────────────────────────────────────────────────────────────────────────┐

class QCheckBoxExt(QtGui.QCheckBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def value(self) -> bool:
        return self.checkState() == QtCore.Qt.Checked

    def setValue(self, value : bool):
        self.setCheckState(QtCore.Qt.Checked if value else QtCore.Qt.Unchecked)


def InputBoolean(name=None, label=None, value=False, stretch=0, 
                 alignment=QtCore.Qt.Alignment(), **kwargs):
    editor = QCheckBoxExt()
    editor.setValue(value)
    set_qt_attrs(editor, **kwargs)
    if name:
        editor.setObjectName(name)
    place_widget(editor, label=label, stretch=stretch, alignment=alignment)
    return editor

#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputVector
#  ┌────────────────────────────────────────────────────────────────────────────┐

class InputVectorWrapper:
    def __init__(self, g, x, y, z):
        self.group = g
        self.x = x
        self.y = y
        self.z = z
    
    def value(self) -> Vec:
        return Vec(self.x.value(), self.y.value(), self.z.value())

    def setValue(self, value):
        v = to_vec(value)
        self.x.setValue(v.x)
        self.y.setValue(v.y)
        self.z.setValue(v.z)

def InputVector(label=None, value=(0.0,0.0,0.0)):
    with GroupBox(title=label) as g:
        with Col():
            x = InputFloat(label="X:")
            y = InputFloat(label="Y:")
            z = InputFloat(label="Z:")
            widget = InputVectorWrapper(g, x, y, z)
            widget.setValue(value)
            return widget


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputOptions
#  ┌────────────────────────────────────────────────────────────────────────────┐

class InputOptionsWrapper:
    def __init__(self, combobox:QtGui.QComboBox, data: Dict[str,Any]):
        self.combobox = combobox
        self.index = dict()
        self.lookup = dict()
        i = 0
        for label, value in data.items():
            self.index[i] = value
            self.lookup[value] = i
            i += 1
            combobox.addItem(label)

    def value(self):
        return self.index.get(self.combobox.currentIndex(), None)

    def setValue(self, value):
        index = self.lookup.get(value, None)
        if index is not None:
            self.combobox.setCurrentIndex(index)

def InputOptions(options, value=None, label=None, name=None, stretch=0, 
                 alignment=QtCore.Qt.Alignment(), **kwargs):
    widget = QtGui.QComboBox()
    set_qt_attrs(widget, **kwargs)
    editor = InputOptionsWrapper(widget, options)
    if value is not None:
        editor.setValue(value)
    if name:
        editor.combobox.setObjectName(name)
    place_widget(editor.combobox, label=label, stretch=stretch, alignment=alignment)
    return editor


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputSelectOne
#  ┌────────────────────────────────────────────────────────────────────────────┐

class InputSelectOne:

    def __init__(self, label=None, name=None, active=False, auto_deactivate=True):
        self._value = None
        self._pre = None
        self._auto_deactivate = auto_deactivate
        self.selected = PySignal()
        with Row(add=False, spacing=0, margin=0, contentsMargins=(0,0,0,0)) as ctl:
            
            @button(
                text="Select...", 
                tool=True,
                checkable=True, 
                styleSheet="QToolButton:checked{background-color: #FF0000; color:#FFFFFF;}",
                focusPolicy=QtCore.Qt.FocusPolicy.NoFocus,
                objectName=name,
                checked=active)
            def select(): pass

            @button(
                tool=True,
                focusPolicy=QtCore.Qt.FocusPolicy.NoFocus,
                icon=Icon(':icons/edit-cleartext.svg'))
            def clear(): self.setValue(None)

            display = QtGui.QLineEdit()
            display.setReadOnly(True)
            place_widget(display)

            self.display = display
            self.button = select
            register_select_observer(select, self)

            with Parent():
                place_widget(ctl, label=label)

    @property
    def active(self) -> bool:
        return self.button.isChecked()

    def value(self) -> Optional[SelectedObject]:
        return self._value

    def pre(self) -> Optional[SelectedObject]:
        return self._pre

    def setValue(self, value: Optional[SelectedObject]) -> None:
        self._value = value
        if value:
            self.display.setText(f"{value.doc}#{value.obj}.{value.sub}")
            if self._auto_deactivate:
                self.button.setChecked(False)
            self.selected.trigger(self._value)
        else:
            self.display.setText(f"")

    def setPreselection(self, doc, obj, sub):
        if self.button.isChecked():
            self._pre = SelectedObject(doc, obj, sub)

    def addSelection(self, doc, obj, sub, pnt):
        if self.button.isChecked():
            self.setValue(SelectedObject(doc, obj, sub, pnt))

    def removeSelection(self, doc, obj, sub):
        if self.button.isChecked():
            if self._value:
                v = self._value
                if (v.doc, v.obj) == (doc, obj):
                    self.setValue(None)

    def setSelection(self, doc):   
        if self.button.isChecked():
            self.setValue(SelectedObject(doc, Gui.Selection.getSelection()[-1].Name))

    def clearSelection(self, doc):
        pass


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] InputSelectMany
#  ┌────────────────────────────────────────────────────────────────────────────┐

class InputSelectMany:

    ValueDataRole = QtCore.Qt.UserRole

    def __init__(self, label=None, name=None, active=False):
        self._value = set()
        self.selected = PySignal()
        with Col(add=False, spacing=0, margin=0, contentsMargins=(0,0,0,0)) as ctl:
            with Row(spacing=0, margin=0, contentsMargins=(0,0,0,0)):
                @button(
                    text="Add", 
                    alignment=QtCore.Qt.AlignLeft,
                    tool=True,
                    checkable=True, 
                    styleSheet="QToolButton:checked{background-color: #FF0000; color:#FFFFFF;}",
                    focusPolicy=QtCore.Qt.FocusPolicy.NoFocus,
                    objectName=name,
                    checked=active)
                def select(): pass

                @button(
                    text="Remove", 
                    tool=True,
                    alignment=QtCore.Qt.AlignLeft,
                    focusPolicy=QtCore.Qt.FocusPolicy.NoFocus)
                def remove():
                    selected = self.display.selectedItems()
                    for item in selected:
                        value = item.data(0, InputSelectMany.ValueDataRole)
                        self._value.remove(value)
                        self.display.takeTopLevelItem(self.display.indexOfTopLevelItem(item))

                @button(
                    text="Clean",
                    tool=True,
                    alignment=QtCore.Qt.AlignLeft,
                    focusPolicy=QtCore.Qt.FocusPolicy.NoFocus,
                    icon=Icon(':icons/edit-cleartext.svg'))
                def clear(): 
                    self._value.clear()
                    self.display.clear()

                Stretch()

            display = QtGui.QTreeWidget()
            display.setColumnCount(2)
            display.setHeaderLabels(['Object', 'SubObject'])
            place_widget(display)

            self.display = display
            self.button = select
            register_select_observer(select, self)

            with Parent():
                with GroupBox(title=label):
                    place_widget(ctl)

    @property
    def active(self) -> bool:
        return self.button.isChecked()

    def value(self) -> List[SelectedObject]:
        return self._value

    def addValue(self, value: SelectedObject) -> None:
        if value not in self._value:
            item = QtGui.QTreeWidgetItem([value.obj, value.sub])
            item.setData(0, InputSelectMany.ValueDataRole, value)
            self.display.addTopLevelItem(item)
            self._value.add(value)
            self.selected.trigger(value)

    def setPreselection(self, doc, obj, sub):
        pass

    def addSelection(self, doc, obj, sub, pnt):
        if self.button.isChecked():
            self.addValue(SelectedObject(doc, obj, sub, pnt))

    def removeSelection(self, doc, obj, sub):
        pass

    def setSelection(self, doc):   
        if self.button.isChecked():
            self.addValue(SelectedObject(doc, Gui.Selection.getSelection()[-1].Name))

    def clearSelection(self, doc):
        pass


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] [Widget] button
#  ┌────────────────────────────────────────────────────────────────────────────┐

def button(label=None, add:bool=True, tool:bool=False, stretch=0, alignment=QtCore.Qt.Alignment(), **kwargs):
    if tool:
        btn = QtGui.QToolButton()
    else:
        btn = QtGui.QPushButton()
    set_qt_attrs(btn, **kwargs)
    if label: 
        btn.setText(label)
    elif 'text' not in kwargs:
        btn.setText("Button")
    if add:
        place_widget(btn, stretch=stretch, alignment=alignment)
    def wrapper(handler):
        btn.clicked.connect(handler)
        return btn
    return wrapper


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] progress_indicator
#  ┌────────────────────────────────────────────────────────────────────────────┐

@contextmanager
def progress_indicator(message: str = "Working...", steps: int = 0):
    bar = Base.ProgressIndicator()
    bar.start(message, steps)
    try:
        yield bar
    finally:
        bar.stop()
        del bar


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [GUI] Message Boxes
#  ┌────────────────────────────────────────────────────────────────────────────┐

def show_msgbox(message, title="Information", std_icon=QtGui.QMessageBox.Information, std_buttons=QtGui.QMessageBox.NoButton, parent=None):
    diag = QtGui.QMessageBox(std_icon, title, message, std_buttons, parent)
    diag.setWindowModality(QtCore.Qt.ApplicationModal)
    diag.exec_()


def show_warning(message, title="Warning", std_icon=QtGui.QMessageBox.Warning, std_buttons=QtGui.QMessageBox.NoButton, parent=None):
    show_msgbox(message, title, std_icon, std_buttons, parent)


def show_error(message, title="Error", std_icon=QtGui.QMessageBox.Critical, std_buttons=QtGui.QMessageBox.NoButton, parent=None):
    show_msgbox(message, title, std_icon, std_buttons, parent)


show_info = show_msgbox


#  └────────────────────────────────────────────────────────────────────────────┘
#    [SECTION] [TEST] 
#  ┌────────────────────────────────────────────────────────────────────────────┐

