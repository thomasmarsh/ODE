noinst_PROGRAMS= dstest
dstest_CXXFLAGS = -I@TOPDIR@/include
dstest_SOURCES= dstest.cpp
dstest_DEPENDENCIES=@TOPDIR@/drawstuff/src/libdrawstuff.a
dstest_LDFLAGS = @LDFLAGS@ @GL_LIBS@ @LIBS@ -L@TOPDIR@/drawstuff/src
dstest_LDADD=-ldrawstuff
if WIN32
resources.o: ../src/resources.rc ../src/resource.h
	@WINDRES@ ../src/resources.rc -o resources.o
dstest_LDADD+= resources.o
endif
