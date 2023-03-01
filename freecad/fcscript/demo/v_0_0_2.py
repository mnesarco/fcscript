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

from freecad.fcscript.v_0_0_2 import *

def test1_wire3D():
    radius = 5
    start = 0,0,radius
    w = Wire3D(start)
    w.add_segment((0,0,100-radius))
    w.add_round_corner((0,100,100), radius)
    w.add_round_corner((100,100,100), radius)
    w.add_round_corner((0,0,radius), radius)
    w.set_to('test1_wire_3d')


with Dialog(title="FCScript Demo 0.0.2") as dialog:
    if not App.ActiveDocument:
        App.newDocument()
    with Row():
        with Col():
            @button(text="Test1: Basic Wire in 3D Space")
            def run_test1(): test1_wire3D()
