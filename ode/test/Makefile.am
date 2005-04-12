AM_CXXFLAGS = @CXXFLAGS@ -I@TOPDIR@/include
AM_CFLAGS = @CXXFLAGS@ -I@TOPDIR@/include
AM_LDFLAGS = 	-L@TOPDIR@/drawstuff/src \
		-L../../ode/src -ldrawstuff -lode @LDFLAGS@ \
		@GL_LIBS@ @LIBS@ 
noinst_PROGRAMS=test_collision \
		test_slider \
		test_crash \
		test_space \
		test_I \
		test_friction \
		test_space_stress \
		test_boxstack test_hinge \
		test_step \
		test_buggy \
		test_joints \
		test_chain1 \
		test_chain2 \
		test_ode
if OPCODE
noinst_PROGRAMS+= test_trimesh test_moving_trimesh
endif
test_collision_SOURCES= test_collision.cpp
test_slider_SOURCES= test_slider.cpp 
test_crash_SOURCES= test_crash.cpp
test_space_SOURCES= test_space.cpp
test_I_SOURCES= test_I.cpp
test_friction_SOURCES= test_friction.cpp
test_space_stress_SOURCES= test_space_stress.cpp
test_boxstack_SOURCES= test_boxstack.cpp
test_hinge_SOURCES= test_hinge.cpp
test_step_SOURCES= test_step.cpp
test_buggy_SOURCES= test_buggy.cpp
test_joints_SOURCES= test_joints.cpp
test_chain1_SOURCES= test_chain1.c
test_chain2_SOURCES= test_chain2.cpp
test_ode_SOURCES= test_ode.cpp
if OPCODE
test_trimesh_SOURCES= test_trimesh.cpp
test_moving_trimesh_SOURCES= test_moving_trimesh.cpp
endif

if WIN32
resources.o: ../../drawstuff/src/resources.rc ../../drawstuff/src/resource.h
	@WINDRES@ ../../drawstuff/src/resources.rc -o resources.o
test_ode_LDADD= resources.o
test_chain2_LDADD= resources.o
test_chain1_LDADD= resources.o
test_joints_LDADD= resources.o
test_buggy_LDADD= resources.o
test_step_LDADD= resources.o
test_hinge_LDADD= resources.o
test_boxstack_LDADD= resources.o
test_space_stress_LDADD= resources.o
test_friction_LDADD= resources.o
test_I_LDADD= resources.o
test_space_LDADD= resources.o
test_crash_LDADD= resources.o
test_slider_LDADD= resources.o
test_collision_LDADD= resources.o
if OPCODE
test_trimesh_LDADD= resources.o
test_moving_trimesh_LDADD= resources.o
endif
endif
