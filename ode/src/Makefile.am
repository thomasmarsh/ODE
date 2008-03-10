# convenience library to simulate per object cflags
noinst_LTLIBRARIES = libfast.la

libfast_la_CPPFLAGS = -I$(top_srcdir)/include
libfast_la_CFLAGS = -O1 -Wall
libfast_la_SOURCES = fastldlt.c fastltsolve.c fastdot.c fastlsolve.c



lib_LTLIBRARIES = libode.la

libode_la_CPPFLAGS = -I$(top_srcdir)/include
libode_la_LDFLAGS = @EXTRA_LIBTOOL_LDFLAGS@ -release @LIBTOOL_RELEASE@
libode_la_LIBADD = libfast.la



libode_la_SOURCES =     objects.h \
                        obstack.cpp \
                        collision_util.cpp \
                        obstack.h \
                        array.cpp \
                        collision_util.h \
                        ode.cpp \
                        array.h \
                        error.cpp \
                        odemath.cpp \
                        collision_kernel.cpp \
                        export-dif.cpp \
                        quickstep.cpp \
                        collision_kernel.h \
                        quickstep.h \
                        collision_quadtreespace.cpp \
                        collision_sapspace.cpp \
                        rotation.cpp \
                        collision_space.cpp \
                        collision_space_internal.h \
                        collision_cylinder_box.cpp \
                        collision_cylinder_sphere.cpp \
                        collision_cylinder_plane.cpp \
                        sphere.cpp \
                        box.cpp \
                        capsule.cpp \
                        plane.cpp \
                        ray.cpp \
                        cylinder.cpp \
                        convex.cpp \
                        joint.cpp \
                        stack.h \
                        collision_std.h \
                        joint.h \
                        step.cpp \
                        collision_transform.cpp \
                        lcp.cpp \
                        step.h \
                        collision_transform.h \
                        lcp.h \
                        stepfast.cpp \
                        mass.cpp \
                        testing.cpp \
                        mat.cpp \
                        testing.h \
                        mat.h \
                        timer.cpp \
                        matrix.cpp \
                        util.cpp \
                        memory.cpp \
                        util.h \
                        misc.cpp \
                        heightfield.cpp \
                        heightfield.h


###################################
#   G I M P A C T    S T U F F
###################################


if GIMPACT
libode_la_SOURCES+= collision_trimesh_gimpact.cpp

libode_la_LIBADD += $(top_builddir)/GIMPACT/src/libGIMPACT.la
libode_la_CPPFLAGS += -DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT -I$(top_srcdir)/GIMPACT/include
libode_la_SOURCES +=    collision_trimesh_trimesh.cpp \
                        collision_trimesh_sphere.cpp \
                        collision_trimesh_ray.cpp \
                        collision_trimesh_opcode.cpp \
                        collision_trimesh_box.cpp \
                        collision_trimesh_ccylinder.cpp \
                        collision_trimesh_distance.cpp \
                        collision_trimesh_internal.h \
                        collision_cylinder_trimesh.cpp \
                        collision_trimesh_plane.cpp
endif



#################################
#   O P C O D E    S T U F F
#################################


if OPCODE

libode_la_LIBADD += $(top_builddir)/OPCODE/libOPCODE.la \
                    $(top_builddir)/OPCODE/Ice/libIce.la

libode_la_CPPFLAGS += -I$(top_srcdir)/OPCODE -I$(top_srcdir)/OPCODE/Ice -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE
libode_la_SOURCES+=     collision_trimesh_trimesh.cpp \
                        collision_trimesh_sphere.cpp \
                        collision_trimesh_ray.cpp \
                        collision_trimesh_opcode.cpp \
                        collision_trimesh_box.cpp \
                        collision_trimesh_ccylinder.cpp \
                        collision_trimesh_distance.cpp \
                        collision_trimesh_internal.h \
                        collision_cylinder_trimesh.cpp \
                        collision_trimesh_plane.cpp
endif



