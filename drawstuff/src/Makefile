
MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

INCLUDE_PATHS = ../../include

LIB=../../lib/$(call fLIBNAME,drawstuff)
SRC = drawstuff.cpp

ifeq ($(PLATFORM),win32)
  SRC += windows.cpp resources.rc
else
  SRC += x11.cpp
endif

all: $(LIB)
ifeq ($(PLATFORM),win32)
	copy resources.res ..\..\lib\drawstuff.res
endif

$(LIB): $(call fTARGETS,$(SRC))
