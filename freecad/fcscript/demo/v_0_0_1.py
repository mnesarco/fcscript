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

from freecad.fcscript.v_0_0_1 import (
    InputVector, XBody, XSketch, Vec, Pnt, Quantity, recompute, Dx, Dy, Dz, Expr,
    Dialog, InputFloat, InputInt, InputSelectMany, InputSelectOne, 
    InputBoolean, Icon, Row, Col, TabContainer, Tab,
    button, on_event, gq, progress_indicator, selection, 
    show_error, show_info, show_msgbox, show_warning,
    DataObject, App
)

def test1():
    s = XSketch("test1")

    # Triangle
    grp = s.create_group()
    grp.line_to((30, 30)) 
    grp.line_to((100, 0))
    grp.close()

    # Triangle
    grp.move_to((200,0))
    grp.line_to((250,250))
    grp.line_to((0,250))
    grp.close()

    # SpLine
    g2 = s.create_group()
    g2.move_to((300,0))
    g2.line_to((0, 50))
    g2.bspline_to((50,50), (50,60), (60,50), (50,70), (70,50), (50,80), (80,50))
    g2.close()

    # SpLine
    g3 = s.create_group()
    g3.move_to((400,0))
    g3.bspline_to((200,100), (300,200), (300,300))
    g3.move_to((0,0))
    g3.circle(45)
    g3.rect_to((20,20))
    g3.move_to((-100,-100))
    g3.rect(50, 100, angle=Quantity('30 deg'))
    g3.rect(50, 100)

    recompute()

def test2_partial(body, plane):
    rows, cols = 5, 3
    w, h = 25, 10
    sx, sy = 15, 5
    s = body.sketch(plane=plane, name=f'test2_{plane}')
    g = s.create_group()

    x, y = g.pos[0], g.pos[1]
    x0 = x
    for row in range(rows):
        for col in range(cols):
            g.move_to((x,y))
            g.rect(w, h)
            x = x + w + sx
        x = x0
        y = y + h + sy

    s.rotate(45)
    s.pad(10, name=f'Pad_{plane}')
    recompute()


def test2():
    with progress_indicator():
        body = XBody(name="test2")
        test2_partial(body, 'XY')
        test2_partial(body, 'XZ')


def test3():   
    with progress_indicator():
        hole = 5
        side = 100

        body = XBody(name='test3')   
        sketch = body.sketch(plane='XY', name='test3_s1')

        path = sketch.create_group()
        path.rect_to(Pnt(side, side))
        h1 = Pnt(hole, hole) * 2
        hole_dist = side - 4*hole
        path.move_to(h1)
        path.circle(hole)
        path.move_to(h1 + Vec(hole_dist, 0, 0))
        path.circle(hole)
        path.move_to(h1 + Vec(0, hole_dist, 0))
        path.circle(hole)
        path.move_to(h1 + Vec(hole_dist, hole_dist, 0))
        path.circle(hole)

        sketch.pad(10)
        recompute()

def test4():
    with progress_indicator():    
        body = XBody(name='test4')   
        sketch = body.sketch(plane='XY', name='test4_s2')
        path = sketch.create_group()

        path.line_to(Pnt(10,0))
        path.bspline_to(Pnt(15, 0), Pnt(15, 5))
        path.line_to(Pnt(15,20))
        path.bspline_to(Pnt(15, 25), Pnt(10, 25))
        path.line_to(Pnt(0,25))
        path.close()

        path = sketch.create_group()
        path.move_to(Pnt(7.5, 7.5))
        path.rect(7, 7, Quantity('45 deg'))

        sketch.pad(30, direction=Vec(0,0,0) + Dy(-1) + Dz(-1))
        recompute()


def test5():
    with progress_indicator():    
        body = XBody(name='test5')
        sketch = body.sketch(plane='XY', name='test5_s1')
        path = sketch.create_group()
        path.rect_rounded(w=7, h=14, r=2, angle=Quantity('45 deg'))   
        path.move_to((50,0))
        path.rect_rounded(w=7, h=14, r=2, angle=Expr("0.5 rad"))
        sketch.pad(3)
        recompute()


def test6():
    with progress_indicator():    
        body = XBody(name='test6')
        sketch = body.sketch(plane='XY', name='test6_s1')
        path = sketch.create_group()
        path.rect_rounded(w=10, h=10, r=(1,2,1,2), angle=Expr("30 deg"))
        path.move_to((50,0))
        path.rect_rounded(w=10, h=10, r=(1,2,1,2), angle=Expr("30 deg"))
        sketch.pad(2)
        recompute()


def test_7_polygons():
    with progress_indicator():    
        body = XBody(name='test7')
        sketch = body.sketch(plane='XY', name='test7_poly')
        path = sketch.create_group()

        def auto(fn, *args, **kwargs):
            fn(*args, **kwargs)

        def constrain_pos(fn, constrain, *args, **kwargs):
            fn(*args, **kwargs, constrain_pos=constrain)

        path.line(dx=10)
        auto(path.regular_polygon, 50, 3)
        path.move(dx=50)
        constrain_pos(path.regular_polygon, True, 50, 4)
        path.move(dx=50)
        constrain_pos(path.regular_polygon, False, 50, 5)
        path.move(dx=50)
        constrain_pos(path.regular_polygon, True, 50, 6)

        path.move(dx=50)
        constrain_pos(path.regular_polygon, True, 50, 6, angle=Quantity('15 deg'))
        recompute()


def test_8_hive():
    with progress_indicator():    
        body = XBody(name='test8')
        sketch = body.sketch(plane='XY', name='test8_s1')
        path = sketch.create_group()
        size = 10
        sep = 1,-1
        rows = 6
        cols = 4
        for row in range(rows):
            shift = -0.5 if row % 2 else 0.5
            for col in range(cols):
                path.move(dx=size+sep[0])
                path.regular_polygon(gq.Radius(size/2.0), 6, constrain_pos=True)
            path.move(dx=-(cols - shift)*(size+sep[0]), dy=size+sep[1])
        path.move_to((0, -size/2 -sep[1] -5))
        path.rect_rounded(w=(size+sep[0])*cols+30, h=(size+sep[1])*rows+10, r=3)
        sketch.pad(2)
        recompute()


def test9_diag1():
    with Dialog(title="1st Dialog"):
        with Row():
            with Col():
                rad = InputFloat(label="Radius:")
                sides = InputInt(label="Sides:")
            with Col():
                it = InputInt(label="Iteration:")
                conv = InputFloat(label="Convolution:")

def test10_diag2():
    with Dialog(title="2nd Dialog"):
        with Col():
            rad = InputFloat(label="Radius:")
            sides = InputInt(label="Sides:")
            it = InputInt(label="Iteration:")
            conv = InputFloat(label="Convolution:")
            active = InputBoolean(label="Active:")
            pos = InputVector(label="Pos (Vector):", value=(10,20,30))
            @button(text="Dump")
            def btn():
                print(f"rad={rad.value()}, sides={sides.value()}, it={it.value()}, conv={conv.value()}, active={active.value()}")
                print(f"Pos={pos.value()}")


def test11_diag3():
    with Dialog(title="3rd Dialog"):
        rad = InputFloat(label="Radius:")
        sides = InputInt(label="Sides:")
        it = InputInt(label="Iteration:")
        conv = InputFloat(label="Convolution:")
        active = InputBoolean(label="Active:")


def test12_diag4():
    with Dialog(title="4th Dialog") as dialog:
        with Col():
            count = InputInt(label="Count:", value=2)
            width = InputFloat(label="Width:", value=4)
            height = InputFloat(label="Height:", value=6)            
            radius = InputFloat(label="Radius:", value=1)     
            @button(text="Execute")
            def execute():
                with progress_indicator("Working..."):
                    body = XBody(name='test9')
                    sketch = body.sketch(plane='XY', name='main9')
                    path = sketch.create_group()
                    for _ in range(count.value()):
                        path.rect_rounded(width.value(), height.value(), radius.value())   
                        path.move(dx=width.value()*1.2)
                    sketch.pad(3)
                    recompute()
                    dialog.close()


def test13_diag5():
    with Dialog(title="5th Dialog / Single Select") as dialog:
        with Col():
            a = InputSelectOne(label="Part A:")
            b = InputSelectOne(label="Part B:")

            @on_event(a.selected)
            def example_listener(sel):
                print(f"Part A was selected: {sel}")

            @on_event(b.selected)
            def example_listener2(sel):
                print(f"Part B was selected: {sel}")

            @button(text="Do Something", icon=Icon(':/icons/edit_OK.svg'))
            def do_something():
                with selection(a.value(), b.value()):
                    print(f"Parts: {a.value()} and {b.value()}")


def test14_diag6():
    with Dialog(title="6th Dialog / Multi Select") as dialog:
        with Col():
            c = InputSelectMany(label="Many Parts:")

            @on_event(c.selected)
            def example_listener(sel):
                print(f"One Part was selected: {sel}")

            @button(text="Do Something", icon=Icon(':/icons/edit_OK.svg'))
            def do_something():
                print("-" * 80)
                for sel in c.value():
                    print(f"Selected: {sel}")


def test15_diag7():
    with Dialog(title="7th Dialog / Tabs") as dialog:
        with Col():
            with TabContainer():

                with Tab('First Tab'):
                    with Col():
                        InputFloat(label="Sample input1:")
                        InputFloat(label="Sample input2:")
                        InputFloat(label="Sample input3:")
                        InputFloat(label="Sample input4:")
                
                with Tab('Second Tab'):
                    with Col():
                        InputSelectMany(label="Sample select many")
                
                with Tab('3th Tab'):
                    with Row():
                        @button(text="Btn1")
                        def do_nothing():
                            pass
                        @button(text="Btn2")
                        def do_nothing():
                            pass


def test16_msgboxes():
    show_info("test info")
    show_warning("test warn")
    show_error("test err")

    show_info("test info", title="aaaa")
    show_warning("test warn", title="bbbb")
    show_error("test err", title="cccc")


def test17_parametric():
    dataObject = DataObject("test17")
    dataObject.add_property("App::PropertyFloat", "Radius")
    dataObject.add_property("App::PropertyFloat", "Extent")
    dataObject.add_property("App::PropertyFloat", "Whatever")
    dataObject.Radius = 55
    dataObject.Extent = Expr(".Radius*2")
    dataObject.Whatever = Expr(".Extent*2.5 + 10")


def test18_rounded_rect():
    with Dialog("Test18: Simple Rounded Rect"):
        with Col():
            width = InputFloat(label="Width:", value=50)
            length = InputFloat(label="Length:", value=50)
            height = InputFloat(label="Height:", value=5)
            radius = InputFloat(label="Border radius:", value=3)
            @button(text="Create")
            def create():
                body = XBody(name='test18')
                sketch = body.sketch(plane='XY', name='test18_sketch')
                path = sketch.create_group()
                path.rect_rounded(w=width.value(), h=length.value(), r=radius.value())
                sketch.pad(height.value())
                recompute()


with Dialog(title="FCScript Demo") as dialog:
    if not App.ActiveDocument:
        App.newDocument()
    with Row():
        with Col():
            @button(text="Test1: Basic Sketch")
            def run_test1(): test1()

            @button(text="Test2: Planes")
            def run_test2(): test2()

            @button(text="Test3: Plate-Holes")
            def run_test3(): test3()

            @button(text="Test4: Extrusion")
            def run_test4(): test4()

        with Col():
            @button(text="Test5: Rounded Rect")
            def run_test5(): test5()

            @button(text="Test6: Asymmetric Rounded Rect")
            def run_test6(): test6()

            @button(text="Test7: Polygons")
            def run_test7(): test_7_polygons()

            @button(text="Test8: Hive")
            def run_test8(): test_8_hive()

        with Col():
            @button(text="Test11: Dialogs Basic")
            def run_test11(): test11_diag3()

            @button(text="Test10: Dialogs Col")
            def run_test10(): test10_diag2()

            @button(text="Test9: Dialogs Row/Col")
            def run_test9(): test9_diag1()

            @button(text="Test12: Dialogs Button")
            def run_test12(): test12_diag4()

        with Col():
            @button(text="Test13: Dialogs Single Select")
            def run_test13(): test13_diag5()

            @button(text="Test14: Dialogs Multi Select")
            def run_test14(): test14_diag6()

            @button(text="Test15: Dialogs Tabs")
            def run_test15(): test15_diag7()

            @button(text="Test16: Message Boxes")
            def run_test16(): test16_msgboxes()

        with Col():
            @button(text="Test17: Data Object")
            def run_test17(): test17_parametric()

            @button(text="Test18: Rounded Rect")
            def run_test18(): test18_rounded_rect()
