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
SRC = array.cpp error.cpp memory.cpp stack.cpp odemath.cpp matrix.cpp \
  misc.cpp rotation.cpp mass.cpp ode.cpp step.cpp lcp.cpp joint.cpp space.cpp \
  geom.cpp timer.cpp mat.cpp fastldlt.c fastlsolve.c fastdot.c

all: $(LIB)

$(LIB): $(call fTARGETS,$(SRC))
