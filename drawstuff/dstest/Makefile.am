AM_CXXFLAGS = @CXXFLAGS@ -I@TOPDIR@/include
dstest_LDFLAGS = -ldrawstuff @LDFLAGS@ @GL_LIBS@ @LIBS@ -L@TOPDIR@/drawstuff/src

noinst_PROGRAMS= dstest

dstest_SOURCES= dstest.cpp

if WIN32
resources.o: ../src/resources.rc ../src/resource.h
	@WINDRES@ ../src/resources.rc -o resources.o
dstest_LDADD= resources.o
endif
