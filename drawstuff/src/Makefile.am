# Drawstuff is meant as an aid for testing and not as a full
# rendering library.

noinst_LTLIBRARIES = libdrawstuff.la
libdrawstuff_la_SOURCES = drawstuff.cpp internal.h
libdrawstuff_la_CPPFLAGS = -I@TOPDIR@/include -I@BUILDDIR@/include
libdrawstuff_la_LIBADD = @LIBWINMM@ @LIBGDI32@

if WIN32
libdrawstuff_la_SOURCES+= windows.cpp
endif
if X11
libdrawstuff_la_SOURCES+= x11.cpp
endif
if OSX
libdrawstuff_la_SOURCES+= osx.cpp
endif

