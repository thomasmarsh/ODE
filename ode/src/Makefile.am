SUBDIRS = joints

AM_CPPFLAGS = -I$(top_srcdir)/include

# convenience library to simulate per object cflags
noinst_LTLIBRARIES = libfast.la
libfast_la_CFLAGS = -O1 -Wall
libfast_la_SOURCES = fastldlt.c fastltsolve.c fastdot.c fastlsolve.c



lib_LTLIBRARIES = libode.la

libode_la_LDFLAGS = @EXTRA_LIBTOOL_LDFLAGS@ @ODE_VERSION_INFO@
libode_la_LIBADD = libfast.la joints/libjoints.la


# please, let's keep the filenames sorted
libode_la_SOURCES =     array.cpp array.h \
                        box.cpp \
                        capsule.cpp \
                        collision_cylinder_box.cpp \
                        collision_cylinder_plane.cpp \
                        collision_cylinder_sphere.cpp \
                        collision_kernel.cpp collision_kernel.h \
                        collision_quadtreespace.cpp \
                        collision_sapspace.cpp \
                        collision_space.cpp \
                        collision_space_internal.h \
                        collision_std.h \
                        collision_transform.cpp collision_transform.h \
                        collision_trimesh_colliders.h \
                        collision_trimesh_disabled.cpp \
                        collision_trimesh_internal.h \
                        collision_util.cpp collision_util.h \
                        convex.cpp \
                        cylinder.cpp \
                        error.cpp \
                        export-dif.cpp \
                        heightfield.cpp heightfield.h \
                        lcp.cpp lcp.h \
                        mass.cpp \
                        mat.cpp mat.h \
                        matrix.cpp \
                        memory.cpp \
                        misc.cpp \
                        objects.h \
                        obstack.cpp obstack.h \
                        ode.cpp \
                        odeinit.cpp \
                        odemath.cpp \
                        odeou.h \
                        odetls.h \
                        plane.cpp \
                        quickstep.cpp quickstep.h \
                        ray.cpp \
                        rotation.cpp \
                        sphere.cpp \
                        step.cpp step.h \
                        stepfast.cpp \
                        testing.cpp testing.h \
                        timer.cpp \
                        util.cpp util.h


###################################
#       O U    S T U F F
###################################


if ENABLE_OU

AM_CPPFLAGS += -I$(top_srcdir)/ou/include
libode_la_LIBADD += $(top_builddir)/ou/src/ou/libou.la
libode_la_SOURCES +=    odetls.cpp odetls.h \
                        odeou.cpp odeou.h

endif


###################################
#   G I M P A C T    S T U F F
###################################


if GIMPACT
AM_CPPFLAGS += -DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT -I$(top_srcdir)/GIMPACT/include
libode_la_SOURCES+= collision_trimesh_gimpact.cpp

libode_la_LIBADD += $(top_builddir)/GIMPACT/src/libGIMPACT.la
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
AM_CPPFLAGS += -I$(top_srcdir)/OPCODE -I$(top_srcdir)/OPCODE/Ice -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE
libode_la_LIBADD += $(top_builddir)/OPCODE/libOPCODE.la \
                    $(top_builddir)/OPCODE/Ice/libIce.la

libode_la_SOURCES+=     collision_trimesh_trimesh.cpp \
                        collision_trimesh_trimesh_new.cpp \
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



