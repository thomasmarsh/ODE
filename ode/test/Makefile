#########################################################################
#                                                                       #
# Open Dynamics Engine, Copyright (C) 2001 Russell L. Smith.            #
#   Email: russ@q12.org   Web: www.q12.org                              #
#                                                                       #
# This library is free software; you can redistribute it and/or         #
# modify it under the terms of the GNU Lesser General Public            #
# License as published by the Free Software Foundation; either          #
# version 2.1 of the License, or (at your option) any later version.    #
#                                                                       #
# This library is distributed in the hope that it will be useful,       #
# but WITHOUT ANY WARRANTY; without even the implied warranty of        #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU      #
# Lesser General Public License for more details.                       #
#                                                                       #
# You should have received a copy of the GNU Lesser General Public      #
# License along with this library (see the file LICENSE.TXT); if not,   #
# write to the Free Software Foundation, Inc., 59 Temple Place,         #
# Suite 330, Boston, MA 02111-1307 USA.                                 #
#                                                                       #
#########################################################################

MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

APPNAMES = $(call fEXENAME,test_ode) $(call fEXENAME,test_chain1) $(call fEXENAME,test_chain2) $(call fEXENAME,test_hinge) $(call fEXENAME,test_slider) $(call fEXENAME,test_collision) $(call fEXENAME,test_boxstack) $(call fEXENAME,test_buggy) $(call fEXENAME,test_joints) $(call fEXENAME,test_space) $(call fEXENAME,test_I) $(call fEXENAME,test_step) $(call fEXENAME,test_friction)

INCLUDE_PATHS = ../../include
LIB_PATHS = ../../lib
EXTRA_CLEAN = $(APPNAMES)
DEFINES=d$(DREAL)

ifeq ($(PLATFORM_IS_WINDOWS),1)
  RSRC=../../lib/drawstuff.res
endif

all: $(APPNAMES)

$(call fEXENAME,test_ode): SUBSYSTEM=CONSOLE
$(call fEXENAME,test_ode): $(call fTARGETS,test_ode.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_ode.cpp) $(call fLIB,ode) $(LINKWIN) $(LINKMATH))

$(call fEXENAME,test_chain1): $(call fTARGETS,test_chain1.c)
	$(call fLINK,$@,$(call fTARGETS,test_chain1.c) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_chain2): $(call fTARGETS,test_chain2.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_chain2.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_hinge): $(call fTARGETS,test_hinge.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_hinge.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_slider): $(call fTARGETS,test_slider.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_slider.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_collision): $(call fTARGETS,test_collision.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_collision.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_boxstack): $(call fTARGETS,test_boxstack.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_boxstack.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_buggy): $(call fTARGETS,test_buggy.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_buggy.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_joints): $(call fTARGETS,test_joints.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_joints.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_space): $(call fTARGETS,test_space.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_space.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_I): $(call fTARGETS,test_I.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_I.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_step): $(call fTARGETS,test_step.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_step.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))

$(call fEXENAME,test_friction): $(call fTARGETS,test_friction.cpp)
	$(call fLINK,$@,$(call fTARGETS,test_friction.cpp) $(call fLIB,ode) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH) $(RSRC))
