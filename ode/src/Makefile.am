AM_CXXFLAGS = -I@TOPDIR@/include
AM_CPPFLAGS = -I@TOPDIR@/include
AM_CFLAGS = -I@TOPDIR@/include
lib_LTLIBRARIES = libode.la
libode_la_LDFLAGS = -no-undefined -version-info @ODE_CURRENT@:@ODE_REVISION@:@ODE_AGE@ -release @ODE_RELEASE@
libode_la_CPPFLAGS = -O2

# convenience library to simulate per object cflags
noinst_LTLIBRARIES= libfast.la
libfast_la_CFLAGS= -O1
libfast_la_SOURCES= fastldlt.c fastltsolve.c fastdot.c fastlsolve.c

libode_la_DEPENDENCIES = libfast.la
libode_la_LIBADD= libfast.la

libode_la_SOURCES = 	objects.h \
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
			rotation.cpp \
			collision_space.cpp \
			collision_space_internal.h \
			collision_std.cpp \
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
			misc.cpp

if OPCODE
libode_la_DEPENDENCIES+=@TOPDIR@/OPCODE/libOPCODE.la
libode_la_LIBADD+= @TOPDIR@/OPCODE/libOPCODE.la
AM_CXXFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED
AM_CFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED
libode_la_SOURCES+=	collision_trimesh_trimesh.cpp \
			collision_trimesh_sphere.cpp \
			collision_trimesh_ray.cpp \
			collision_trimesh.cpp \
			collision_trimesh_box.cpp \
			collision_trimesh_ccylinder.cpp \
			collision_trimesh_distance.cpp \
			collision_trimesh_internal.h
endif
if FREESOLID
# we just asume the fresolid-config script to exist here
libode_la_LDFLAGS+= `freesolid-config --libs`
AM_CXXFLAGS += `freesolid-config --cflags` -DdTRIMESH_ENABLED -DWITH_FREESOLID
AM_CFLAGS +=  `freesolid-config --cflags` -DdTRIMESH_ENABLED -DWITH_FREESOLID
libode_la_SOURCES+=	collision_trimesh_trimesh.cpp \
			collision_trimesh_sphere.cpp \
			collision_trimesh_ray.cpp \
			collision_trimesh.cpp \
			collision_trimesh_box.cpp \
			collision_trimesh_ccylinder.cpp \
			collision_trimesh_distance.cpp \
			collision_trimesh_internal.h
endif


