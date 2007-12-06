noinst_PROGRAMS= dstest
dstest_CPPFLAGS = -I@TOPDIR@/drawstuff/src -I@TOPDIR@/include -I@BUILDDIR@

dstest_SOURCES= dstest.cpp
dstest_DEPENDENCIES=@BUILDDIR@/drawstuff/src/libdrawstuff.la
dstest_LDFLAGS = -L@BUILDDIR@/drawstuff/src
dstest_LDADD=-ldrawstuff @LDFLAGS@ @GL_LIBS@ @LIBS@
if WIN32
dstest_DEPENDENCIES+=resources.o
resources.o: @TOPDIR@/drawstuff/src/resources.rc @TOPDIR@/drawstuff/src/resource.h
	@WINDRES@ @TOPDIR@/drawstuff/src/resources.rc -o resources.o
dstest_LDADD+= resources.o
endif
