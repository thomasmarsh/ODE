AM_CPPFLAGS = @XINCLUDES@ -I$(top_srcdir)/include -I$(top_builddir)/include

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
demo_collision_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_slider_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@  
demo_feedback_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@  
demo_crash_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_space_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_I_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_friction_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_space_stress_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_boxstack_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_hinge_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_step_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_buggy_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_cyl_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_cylvssphere_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_joints_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_jointPR_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_motor_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_piston_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_chain1_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_chain2_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_ode_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_plane2d_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_heightfield_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 
demo_convex_cd_LDFLAGS= -L$(top_builddir)/drawstuff/src \
		-L$(top_builddir)/ode/src @LDFLAGS@ 

demo_collision_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_slider_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_feedback_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_crash_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_space_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_I_DEPENDENCIES=		$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_friction_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_space_stress_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_boxstack_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_hinge_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_step_DEPENDENCIES=		$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_buggy_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_cyl_DEPENDENCIES=		$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_cylvssphere_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_joints_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_jointPR_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_motor_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_piston_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_chain1_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_chain2_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_ode_DEPENDENCIES=		$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_plane2d_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_heightfield_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_convex_cd_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
				$(top_builddir)/drawstuff/src/libdrawstuff.la

if TRIMESH
demo_trimesh_SOURCES= 			demo_trimesh.cpp
demo_moving_trimesh_SOURCES= 		demo_moving_trimesh.cpp
demo_basket_SOURCES= 			demo_basket.cpp
demo_trimesh_LDFLAGS= 			-L$(top_builddir)/drawstuff/src \
					-L$(top_builddir)/ode/src @LDFLAGS@ \
					@GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_moving_trimesh_LDFLAGS= 		-L$(top_builddir)/drawstuff/src \
					-L$(top_builddir)/ode/src @LDFLAGS@ \
					@GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_basket_LDFLAGS=          		-L$(top_builddir)/drawstuff/src \
                                        -L$(top_builddir)/ode/src @LDFLAGS@ \
                                        @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_trimesh_DEPENDENCIES=		$(top_builddir)/ode/src/libode.la \
					$(top_builddir)/drawstuff/src/libdrawstuff.la
demo_moving_trimesh_DEPENDENCIES=	$(top_builddir)/ode/src/libode.la \
					$(top_builddir)/drawstuff/src/libdrawstuff.la

demo_basket_DEPENDENCIES=		$(top_builddir)/ode/src/libode.la \
					$(top_builddir)/drawstuff/src/libdrawstuff.la

endif

demo_ode_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_plane2d_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_heightfield_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_chain2_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_chain1_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_joints_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_jointPR_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_motor_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_piston_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_buggy_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_cyl_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_cylvssphere_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_step_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_hinge_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_boxstack_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_space_stress_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_friction_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_I_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_space_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_crash_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_slider_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_feedback_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_collision_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_convex_cd_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@


if TRIMESH
demo_trimesh_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_moving_trimesh_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@  @XLIBRARIES@ @LIBSTDCXX@
demo_basket_LDADD= -ldrawstuff $(top_builddir)/ode/src/libode.la @GL_LIBS@ @XLIBRARIES@ @LIBSTDCXX@
endif


if WIN32
resources.o: ../../drawstuff/src/resources.rc ../../drawstuff/src/resource.h
	@WINDRES@ ../../drawstuff/src/resources.rc -o resources.o
demo_ode_LDADD+= resources.o
demo_heightfield_LDADD+= resources.o
demo_chain2_LDADD+= resources.o
demo_chain1_LDADD+= resources.o
demo_joints_LDADD+= resources.o
demo_jointPR_LDADD+= resources.o
demo_motor_LDADD+= resources.o
demo_piston_LDADD+= resources.o
demo_buggy_LDADD+= resources.o
demo_cyl_LDADD+= resources.o
demo_cylvssphere_LDADD+= resources.o
demo_step_LDADD+= resources.o
demo_hinge_LDADD+= resources.o
demo_boxstack_LDADD+= resources.o
demo_space_stress_LDADD+= resources.o
demo_friction_LDADD+= resources.o
demo_I_LDADD+= resources.o
demo_space_LDADD+= resources.o
demo_crash_LDADD+= resources.o
demo_slider_LDADD+= resources.o
demo_feedback_LDADD+= resources.o
demo_collision_LDADD+= resources.o
demo_convex_cd_LDADD+= resources.o
demo_ode_DEPENDENCIES+= resources.o
demo_chain2_DEPENDENCIES+= resources.o
demo_chain1_DEPENDENCIES+= resources.o
demo_joints_DEPENDENCIES+= resources.o
demo_jointPR_DEPENDENCIES+= resources.o
demo_motor_DEPENDENCIES+= resources.o
demo_piston_DEPENDENCIES+= resources.o
demo_buggy_DEPENDENCIES+= resources.o
demo_cyl_DEPENDENCIES+= resources.o
demo_cylvssphere_DEPENDENCIES+= resources.o
demo_step_DEPENDENCIES+= resources.o
demo_hinge_DEPENDENCIES+= resources.o
demo_boxstack_DEPENDENCIES+= resources.o
demo_space_stress_DEPENDENCIES+= resources.o
demo_friction_DEPENDENCIES+= resources.o
demo_I_DEPENDENCIES+= resources.o
demo_space_DEPENDENCIES+= resources.o
demo_crash_DEPENDENCIES+= resources.o
demo_slider_DEPENDENCIES+= resources.o
demo_feedback_DEPENDENCIES+= resources.o
demo_collision_DEPENDENCIES+= resources.o
demo_convex_cd_DEPENDENCIES+= resources.o

if TRIMESH
demo_trimesh_LDADD+= resources.o
demo_moving_trimesh_LDADD+= resources.o
demo_trimesh_DEPENDENCIES+= resources.o
demo_moving_trimesh_DEPENDENCIES+= resources.o
demo_basket_DEPENDENCIES+= resources.o
endif
endif
