SUBDIRS = UnitTest++

AM_CPPFLAGS = -I $(srcdir)/UnitTest++/src -I $(top_srcdir)/include

check_PROGRAMS = tests

TESTS = tests

tests_SOURCES = main.cpp joint.cpp odemath.cpp joints/fixed.cpp joints/hinge.cpp joints/slider.cpp

tests_LDFLAGS = $(builddir)/UnitTest++/src/libunittestpp.la \
            $(top_builddir)/ode/src/libode.la

