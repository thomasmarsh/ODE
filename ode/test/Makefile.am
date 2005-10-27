AM_CXXFLAGS = @CXXFLAGS@ -I@TOPDIR@/include
AM_CFLAGS = @CXXFLAGS@ -I@TOPDIR@/include

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
test_collision_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_slider_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@  
test_crash_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_space_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_I_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_friction_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_space_stress_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_boxstack_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_hinge_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_step_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_buggy_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_joints_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_chain1_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_chain2_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 
test_ode_LDFLAGS= -L@TOPDIR@/drawstuff/src \
		-L@TOPDIR@/ode/src @LDFLAGS@ 

test_collision_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_slider_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_crash_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_space_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_I_DEPENDENCIES=		@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_friction_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_space_stress_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_boxstack_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_hinge_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_step_DEPENDENCIES=		@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_buggy_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_joints_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_chain1_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_chain2_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a
test_ode_DEPENDENCIES=		@TOPDIR@/ode/src/libode.a \
				@TOPDIR@/drawstuff/src/libdrawstuff.a

if OPCODE
test_trimesh_SOURCES= 			test_trimesh.cpp
test_moving_trimesh_SOURCES= 		test_moving_trimesh.cpp
test_trimesh_LDFLAGS= 			-L@TOPDIR@/drawstuff/src \
					-L@TOPDIR@/ode/src @LDFLAGS@ \
					@GL_LIBS@ @LIBS@
test_moving_trimesh_LDFLAGS= 		-L@TOPDIR@/drawstuff/src \
					-L@TOPDIR@/ode/src @LDFLAGS@ \
					@GL_LIBS@ @LIBS@
test_trimesh_DEPENDENCIES=		@TOPDIR@/ode/src/libode.a \
					@TOPDIR@/drawstuff/src/libdrawstuff.a
test_moving_trimesh_DEPENDENCIES=	@TOPDIR@/ode/src/libode.a \
					@TOPDIR@/drawstuff/src/libdrawstuff.a

endif

test_ode_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_chain2_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_chain1_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_joints_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_buggy_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_step_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_hinge_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_boxstack_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_space_stress_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_friction_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_I_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_space_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_crash_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_slider_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_collision_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
if OPCODE
test_trimesh_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
test_moving_trimesh_LDADD= -ldrawstuff -lode @GL_LIBS@ @LIBS@
endif


if WIN32
resources.o: ../../drawstuff/src/resources.rc ../../drawstuff/src/resource.h
	@WINDRES@ ../../drawstuff/src/resources.rc -o resources.o
test_ode_LDADD+= resources.o
test_chain2_LDADD+= resources.o
test_chain1_LDADD+= resources.o
test_joints_LDADD+= resources.o
test_buggy_LDADD+= resources.o
test_step_LDADD+= resources.o
test_hinge_LDADD+= resources.o
test_boxstack_LDADD+= resources.o
test_space_stress_LDADD+= resources.o
test_friction_LDADD+= resources.o
test_I_LDADD+= resources.o
test_space_LDADD+= resources.o
test_crash_LDADD+= resources.o
test_slider_LDADD+= resources.o
test_collision_LDADD+= resources.o
test_ode_DEPENDENCIES+= resources.o
test_chain2_DEPENDENCIES+= resources.o
test_chain1_DEPENDENCIES+= resources.o
test_joints_DEPENDENCIES+= resources.o
test_buggy_DEPENDENCIES+= resources.o
test_step_DEPENDENCIES+= resources.o
test_hinge_DEPENDENCIES+= resources.o
test_boxstack_DEPENDENCIES+= resources.o
test_space_stress_DEPENDENCIES+= resources.o
test_friction_DEPENDENCIES+= resources.o
test_I_DEPENDENCIES+= resources.o
test_space_DEPENDENCIES+= resources.o
test_crash_DEPENDENCIES+= resources.o
test_slider_DEPENDENCIES+= resources.o
test_collision_DEPENDENCIES+= resources.o
if OPCODE
test_trimesh_LDADD+= resources.o
test_moving_trimesh_LDADD+= resources.o
test_trimesh_DEPENDENCIES+= resources.o
test_moving_trimesh_DEPENDENCIES+= resources.o
endif
endif
