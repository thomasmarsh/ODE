AM_CPPFLAGS = -I$(abs_top_srcdir)/include \
        -DDRAWSTUFF_TEXTURE_PATH="\"$(abs_top_srcdir)/drawstuff/textures\""

AM_LDFLAGS = \
        $(top_builddir)/drawstuff/src/libdrawstuff.la \
        $(top_builddir)/ode/src/libode.la \
        @GL_LIBS@
if X11
AM_LDFLAGS+= $(X_PRE_LIBS) $(X_LIBS) $(X_EXTRA_LIBS)
endif

noinst_HEADERS = basket_geom.h texturepath.h world_geom3.h bunny_geom.h

noinst_PROGRAMS = \
                demo_boxstack \
                demo_buggy \
                demo_chain1 \
                demo_chain2 \
                demo_collision \
                demo_convex_cd \
                demo_crash \
                demo_cylvssphere \
                demo_feedback \
                demo_friction \
                demo_hinge \
                demo_I \
                demo_jointPR \
                demo_joints \
                demo_motor \
                demo_ode \
                demo_piston \
                demo_plane2d \
                demo_slider \
                demo_space \
                demo_space_stress \
                demo_step

demo_boxstack_SOURCES = demo_boxstack.cpp
demo_buggy_SOURCES = demo_buggy.cpp
demo_chain1_SOURCES = demo_chain1.c
demo_chain1_LDADD = $(LDADD) -lstdc++ 
demo_chain2_SOURCES = demo_chain2.cpp
demo_collision_SOURCES = demo_collision.cpp
demo_convex_cd_SOURCES = demo_convex_cd.cpp
demo_crash_SOURCES = demo_crash.cpp
demo_cylvssphere_SOURCES = demo_cylvssphere.cpp
demo_feedback_SOURCES = demo_feedback.cpp
demo_friction_SOURCES = demo_friction.cpp
demo_hinge_SOURCES = demo_hinge.cpp
demo_I_SOURCES = demo_I.cpp
demo_jointPR_SOURCES = demo_jointPR.cpp
demo_joints_SOURCES = demo_joints.cpp
demo_motor_SOURCES = demo_motor.cpp
demo_ode_SOURCES = demo_ode.cpp
demo_piston_SOURCES = demo_piston.cpp
demo_plane2d_SOURCES = demo_plane2d.cpp
demo_slider_SOURCES = demo_slider.cpp
demo_space_SOURCES = demo_space.cpp
demo_space_stress_SOURCES = demo_space_stress.cpp
demo_step_SOURCES = demo_step.cpp




if TRIMESH
noinst_PROGRAMS += \
                demo_basket \
                demo_cyl \
                demo_moving_trimesh \
                demo_trimesh \
                demo_heightfield

demo_basket_SOURCES = demo_basket.cpp
demo_cyl_SOURCES = demo_cyl.cpp
demo_moving_trimesh_SOURCES = demo_moving_trimesh.cpp
demo_trimesh_SOURCES = demo_trimesh.cpp
demo_heightfield_SOURCES = demo_heightfield.cpp

AM_CPPFLAGS += -DdTRIMESH_ENABLED
endif



if WIN32
resources.o: $(top_srcdir)/drawstuff/src/resources.rc $(top_srcdir)/drawstuff/src/resource.h
	@WINDRES@ $(top_srcdir)/drawstuff/src/resources.rc -o resources.o
LDADD = resources.o
endif


