AM_CPPFLAGS = -I$(abs_top_srcdir)/include \
        -DDRAWSTUFF_TEXTURE_PATH="\"$(abs_top_srcdir)/drawstuff/textures\""

if X11
AM_LDFLAGS = $(X_PRE_LIBS) $(X_LIBS) $(X_EXTRA_LIBS)
endif

# On Windows, GL_LIBS must go after libdrawstuff.la.
LDADD = $(top_builddir)/drawstuff/src/libdrawstuff.la \
        $(top_builddir)/ode/src/libode.la @GL_LIBS@

noinst_HEADERS = basket_geom.h texturepath.h world_geom3.h bunny_geom.h convex_bunny_geom.h \
                 icosahedron_geom.h

noinst_PROGRAMS = \
                demo_boxstack \
                demo_buggy \
                demo_cards \
                demo_chain1 \
                demo_chain2 \
                demo_collision \
                demo_convex_cd \
                demo_crash \
                demo_cylvssphere \
                demo_feedback \
                demo_friction \
                demo_gyroscopic \
                demo_heightfield \
                demo_hinge \
                demo_I \
                demo_jointPR \
                demo_joints \
                demo_jointPU \
                demo_kinematic \
                demo_motion \
                demo_motor \
                demo_ode \
                demo_piston \
                demo_plane2d \
                demo_slider \
                demo_space \
                demo_space_stress \
                demo_step \
                demo_tracks

demo_boxstack_SOURCES = demo_boxstack.cpp
demo_buggy_SOURCES = demo_buggy.cpp
demo_cards_SOURCES = demo_cards.cpp
demo_chain1_SOURCES = demo_chain1.c
demo_chain1_LDADD = $(LDADD) -lstdc++ 
demo_chain2_SOURCES = demo_chain2.cpp
demo_collision_SOURCES = demo_collision.cpp
demo_convex_cd_SOURCES = demo_convex_cd.cpp
demo_convex_cd_DEPENDENCIES = $(top_builddir)/ode/src/libode.la
demo_crash_SOURCES = demo_crash.cpp
demo_cylvssphere_SOURCES = demo_cylvssphere.cpp
demo_feedback_SOURCES = demo_feedback.cpp
demo_friction_SOURCES = demo_friction.cpp
demo_gyroscopic_SOURCES = demo_gyroscopic.cpp
demo_heightfield_SOURCES = demo_heightfield.cpp
demo_hinge_SOURCES = demo_hinge.cpp
demo_I_SOURCES = demo_I.cpp
demo_jointPR_SOURCES = demo_jointPR.cpp
demo_joints_SOURCES = demo_joints.cpp
demo_jointPU_SOURCES = demo_jointPU.cpp
demo_kinematic_SOURCES = demo_kinematic.cpp
demo_motion_SOURCES = demo_motion.cpp
demo_motor_SOURCES = demo_motor.cpp
demo_ode_SOURCES = demo_ode.cpp
demo_piston_SOURCES = demo_piston.cpp
demo_plane2d_SOURCES = demo_plane2d.cpp
demo_slider_SOURCES = demo_slider.cpp
demo_space_SOURCES = demo_space.cpp
demo_space_stress_SOURCES = demo_space_stress.cpp
demo_step_SOURCES = demo_step.cpp
demo_tracks_SOURCES = demo_tracks.cpp


if TRIMESH
noinst_PROGRAMS += \
                demo_basket \
                demo_cyl \
                demo_moving_trimesh \
                demo_moving_convex \
                demo_trimesh

demo_basket_SOURCES = demo_basket.cpp
demo_cyl_SOURCES = demo_cyl.cpp
demo_moving_trimesh_SOURCES = demo_moving_trimesh.cpp
demo_moving_convex_SOURCES = demo_moving_convex.cpp
demo_trimesh_SOURCES = demo_trimesh.cpp

AM_CPPFLAGS += -DdTRIMESH_ENABLED
endif



if WIN32
resources.o: $(top_srcdir)/drawstuff/src/resources.rc $(top_srcdir)/drawstuff/src/resource.h
	@WINDRES@ $(top_srcdir)/drawstuff/src/resources.rc -o resources.o
LDADD += resources.o
endif
