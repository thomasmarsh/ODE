
SUBDIRS=reuse drawstuff zdriver zwindows lib

MAKEFILE_INC=build/Makefile.inc
include $(MAKEFILE_INC)

zdriver: reuse
zwindows: zdriver
