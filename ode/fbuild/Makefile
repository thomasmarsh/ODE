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

# currently this only works under linux, and it's a bit of a mess!

MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

INCLUDE_PATHS=../../include
LIB_PATHS = ../../lib
DEFINES=dDOUBLE

SOURCES_CPP=test_ldlt.cpp
SOURCES_C=fastldlt.c fastlsolve.c fastltsolve.c
APPS=$(call fEXENAME,test_ldlt) $(call fEXENAME,test_dot) $(call fEXENAME,test_multidot)
EXTRA_CLEAN=test_ldlt test_dot test_multidot fastldlt.c fastlsolve.c fastltsolve.c fastdot.c fastmultidot.c


all: $(APPS)

$(call fEXENAME,test_ldlt): $(call fTARGETS,$(SOURCES_CPP) $(SOURCES_C))
	gcc -o $@ $^ -L $(LIB_PATHS) $(call fLIB,ode) -lm

$(call fEXENAME,test_dot): test_dot.o fastdot.o
	gcc -o $@ test_dot.o fastdot.o -L $(LIB_PATHS) $(call fLIB,ode) -lm

$(call fEXENAME,test_multidot): test_multidot.o fastmultidot.o
	gcc -o $@ test_multidot.o fastmultidot.o -L $(LIB_PATHS) $(call fLIB,ode) -lm

fastldlt.o: fastldlt.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastlsolve.o: fastlsolve.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastltsolve.o: fastltsolve.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastdot.o: fastdot.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastmultidot.o: fastmultidot.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastldlt.c: BuildLDLT BuildUtil ParametersF
	./BuildLDLT ParametersF

fastlsolve.c: BuildLDLT BuildUtil ParametersS
	./BuildLDLT ParametersS

fastltsolve.c: BuildLDLT BuildUtil ParametersT
	./BuildLDLT ParametersT

fastdot.c: BuildDot BuildUtil ParametersD
	./BuildDot ParametersD

fastmultidot.c: BuildMultidot BuildUtil ParametersM
	./BuildMultidot ParametersM
