
SUBDIRS=drawstuff ode lib

MAKEFILE_INC=build/Makefile.inc
include $(MAKEFILE_INC)

ode: drawstuff
