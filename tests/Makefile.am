SUBDIRS = UnitTest++

AM_CPPFLAGS = -I $(srcdir)/UnitTest++/src \
              -I $(top_srcdir)/include \
              -I $(top_srcdir)/ode/src

LDADD = $(builddir)/UnitTest++/src/libunittestpp.la \
        $(top_builddir)/ode/src/libode.la

check_PROGRAMS = tests

TESTS = tests

tests_SOURCES = main.cpp joint.cpp odemath.cpp \
                joints/fixed.cpp \
	        joints/hinge.cpp \
	        joints/pu.cpp \
	        joints/slider.cpp 

