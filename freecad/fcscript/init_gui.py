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

from freecad.fcscript import __version__ as fc_version

import pathlib, shutil

import FreeCAD as App

auto_update_macro = True

if auto_update_macro:
    DEMO_MACRO = pathlib.Path(App.getUserMacroDir(), "FCScript_Demo.FCMacro")
    ver = f"v_{fc_version.replace('.', '_')}.py"
    src = pathlib.Path(pathlib.Path(__file__).parent, "demo", ver)
    print(f"FCScript Demo Macro init: {src} -> {DEMO_MACRO}")
    if src.exists():
        shutil.copy2(src, DEMO_MACRO)
        