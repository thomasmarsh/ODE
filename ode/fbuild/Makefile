# currently this only works under linux, and it's a bit of a mess!

MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

INCLUDE_PATHS=../../include
LIB_PATHS = ../../lib
DEFINES=dDOUBLE

SOURCES_CPP=test_ldlt.cpp
SOURCES_C=fastldlt.c fastlsolve.c
APPS=$(call fEXENAME,test_ldlt) $(call fEXENAME,test_dot)
EXTRA_CLEAN=test_ldlt test_dot fastldlt.c fastlsolve.c fastdot.c


all: $(APPS)

$(call fEXENAME,test_ldlt): $(call fTARGETS,$(SOURCES_CPP) $(SOURCES_C))
	gcc -o $@ $^ -L $(LIB_PATHS) $(call fLIB,ode) -lm

$(call fEXENAME,test_dot): test_dot.o fastdot.o fastlsolve.o
	gcc -o $@ test_dot.o fastdot.o fastlsolve.o -L $(LIB_PATHS) $(call fLIB,ode) -lm

fastldlt.o: fastldlt.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastlsolve.o: fastlsolve.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastdot.o: fastdot.c
	gcc -O1 -I$(INCLUDE_PATHS) -ffast-math -fomit-frame-pointer -c -D$(DEFINES) $<

fastldlt.c: BuildLDLT BuildUtil ParametersF
	./BuildLDLT ParametersF

fastlsolve.c: BuildLDLT BuildUtil ParametersS
	./BuildLDLT ParametersS

fastdot.c: BuildDot BuildUtil ParametersD
	./BuildDot ParametersD
