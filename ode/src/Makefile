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

# possible members of DEFINES
#   dSINGLE or dDOUBLE for single or double precision arithmetic
#   dNODEBUG to disable debugging
#   PENTIUM if the chip is a pentium
#   SSE1 if the chip supports Intel SSE1 instructions
#   NOW3D if the chip supports AMD 3D-Now instructions


MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

INCLUDE_PATHS = ../../include
DEFINES=d$(DREAL)

ifeq ($(BUILD_MODE),release)
  CPP_OPT=2
  FAST=1
  DEFINES += dNODEBUG
  # CPP_FLAGS=-malign-double -mpentiumpro -march=pentiumpro
else
  DEFINES += dDEBUG_ALLOC
endif

ifeq ($(PLATFORM),linux)
  DEFINES += PENTIUM
endif

LIB=../../lib/$(call fLIBNAME,ode)
PREGEN_SRC = fastldlt.c fastlsolve.c fastltsolve.c fastdot.c
SRC = array.cpp error.cpp memory.cpp stack.cpp odemath.cpp matrix.cpp \
  misc.cpp rotation.cpp mass.cpp ode.cpp step.cpp lcp.cpp joint.cpp space.cpp \
  geom.cpp timer.cpp mat.cpp $(PREGEN_SRC) testing.cpp

# pregenerated sources must be compiled with -O1 optimization
$(call fTARGETS,$(PREGEN_SRC)): CPP_OPT=1


all: $(LIB)

$(LIB): $(call fTARGETS,$(SRC))
