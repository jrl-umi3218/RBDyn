#
#    Copyright 2010 CNRS
#
#    Author: Florent Lamiraux
#
# This file is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# this file. If not, see <http://www.gnu.org/licenses/>.

import sys, DLFCN
import dynamic_graph as dg
flags = sys.getdlopenflags()
# Import C++ symbols in a global scope
# This is necessary for signal compiled in different modules to be compatible
sys.setdlopenflags(DLFCN.RTLD_NOW|DLFCN.RTLD_GLOBAL)
import wrap
# Recover previous flags
sys.setdlopenflags(flags)

dg.entity.updateEntityClasses(globals())

${ENTITY_CLASS_LIST}
