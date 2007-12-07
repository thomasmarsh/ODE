# @XINCLUDES@ 
AM_CPPFLAGS = -I$(abs_top_srcdir)/include -I$(abs_top_builddir)/include \
        -DDRAWSTUFF_TEXTURE_PATH="\"$(abs_top_srcdir)/drawstuff/textures\""

LIBS = @GL_LIBS@ @XLIBRARIES@ @LIBSTDCXX@
LDADD = $(top_builddir)/drawstuff/src/libdrawstuff.la $(top_builddir)/ode/src/libode.la
DEPENDENCIES = $(top_builddir)/drawstuff/src/libdrawstuff.la $(top_builddir)/ode/src/libode.la


noinst_HEADERS = basket_geom.h texturepath.h world_geom3.h

noinst_PROGRAMS=demo_collision \
		demo_slider \
		demo_feedback \
		demo_crash \
		demo_space \
		demo_I \
		demo_friction \
		demo_space_stress \
		demo_boxstack demo_hinge \
		demo_step \
		demo_buggy \
		demo_joints \
		demo_motor \
		demo_piston \
		demo_chain1 \
		demo_chain2 \
                demo_cylvssphere \
		demo_ode \
		demo_plane2d \
		demo_heightfield \
		demo_convex_cd \
		demo_jointPR
if TRIMESH
noinst_PROGRAMS+= demo_trimesh demo_moving_trimesh demo_basket demo_cyl
endif

demo_collision_SOURCES= demo_collision.cpp
demo_slider_SOURCES= demo_slider.cpp 
demo_feedback_SOURCES= demo_feedback.cpp 
demo_crash_SOURCES= demo_crash.cpp
demo_space_SOURCES= demo_space.cpp
demo_I_SOURCES= demo_I.cpp
demo_friction_SOURCES= demo_friction.cpp
demo_space_stress_SOURCES= demo_space_stress.cpp
demo_boxstack_SOURCES= demo_boxstack.cpp
demo_hinge_SOURCES= demo_hinge.cpp
demo_step_SOURCES= demo_step.cpp
demo_buggy_SOURCES= demo_buggy.cpp
demo_cyl_SOURCES= demo_cyl.cpp world_geom3.h
demo_cylvssphere_SOURCES= demo_cylvssphere.cpp
demo_joints_SOURCES= demo_joints.cpp
demo_jointPR_SOURCES= demo_jointPR.cpp
demo_motor_SOURCES= demo_motor.cpp
demo_piston_SOURCES = demo_piston.cpp
demo_chain1_SOURCES= demo_chain1.c
demo_chain2_SOURCES= demo_chain2.cpp
demo_ode_SOURCES= demo_ode.cpp
demo_plane2d_SOURCES= demo_plane2d.cpp
demo_heightfield_SOURCES= demo_heightfield.cpp
demo_convex_cd_SOURCES= demo_convex_cd.cpp


if TRIMESH
demo_trimesh_SOURCES= 			demo_trimesh.cpp
demo_moving_trimesh_SOURCES= 		demo_moving_trimesh.cpp
demo_basket_SOURCES= 			demo_basket.cpp
endif


if WIN32
resources.o: $(top_srcdir)/drawstuff/src/resources.rc $(top_srcdir)/drawstuff/src/resource.h
	@WINDRES@ $(top_srcdir)/drawstuff/src/resources.rc -o resources.o
LDADD += resources.o
endif

