
MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

INCLUDE_PATHS = ../../include

LIB=../../lib/$(call fLIBNAME,drawstuff)
SRC = drawstuff.cpp

ifeq ($(PLATFORM_IS_WINDOWS),1)
  SRC += windows.cpp resources.rc
else
  SRC += x11.cpp
endif

all: $(LIB)
ifeq ($(PLATFORM_IS_WINDOWS),1)
	copy resources.RES ..\..\lib\drawstuff.res
endif

$(LIB): $(call fTARGETS,$(SRC))
