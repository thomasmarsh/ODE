
MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

APPNAME = $(call fEXENAME,dstest)
SRC = dstest.cpp

INCLUDE_PATHS = ../../include
LIB_PATHS = ../../lib
EXTRA_CLEAN = $(APPNAME)

all: $(APPNAME)

$(APPNAME): $(call fTARGETS,$(SRC))
	$(call fLINK,$(APPNAME),$(call fTARGETS,$(SRC)) $(call fLIB,drawstuff) $(LINKOPENGL) $(LINKWIN) $(LINKMATH))
