
SUBDIRS=reuse drawstuff zdriver zwindows ode lib

MAKEFILE_INC=build/Makefile.inc
include $(MAKEFILE_INC)

zdriver: reuse
zwindows: zdriver
ode: drawstuff
