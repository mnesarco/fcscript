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

from freecad.fcscript import __version__ as fs_version
import pathlib, shutil, os, re
import FreeCAD as App

auto_update_macro = True

if auto_update_macro:
    user_macro_dir = pathlib.Path(App.getUserMacroDir())
    demo_macro_dir = pathlib.Path(pathlib.Path(__file__).parent, "demo")
    pattern = re.compile(r'^v_(\d+)_(\d+)_(\d+).py$')
    for file in os.listdir(demo_macro_dir):
        if pattern.match(file):
            src = pathlib.Path(demo_macro_dir, file)
            target = pathlib.Path(user_macro_dir, f"FCScript_Demo_{file}")
            print(f"FCScript Demo Macro init: {src} -> {target}")
            shutil.copy2(src, target)
        