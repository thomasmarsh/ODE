
MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

APPNAMES = $(call fEXENAME,test_ode) $(call fEXENAME,test_chain1) $(call fEXENAME,test_chain2) $(call fEXENAME,test_hinge) $(call fEXENAME,test_slider) $(call fEXENAME,test_collision) $(call fEXENAME,test_boxstack) $(call fEXENAME,test_buggy)

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
