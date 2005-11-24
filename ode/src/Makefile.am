AM_CXXFLAGS = @ARCHFLAGS@ -I$(top_srcdir)/include -I$(top_builddir)/include
AM_CPPFLAGS = @ARCHFLAGS@ -I$(top_srcdir)/include -I$(top_builddir)/include
AM_CFLAGS = @ARCHFLAGS@ -I$(topsrcdir)/include -I$(top_builddir)/include
lib_LIBRARIES = libode.a
libode_a_CPPFLAGS = -O2

# Fake an executable in order to get a shared library
EXEEXT=@so_ext@
bin_PROGRAMS=libode
libode_DEPENDENCIES = libfast.a libode.a
libode_LDFLAGS=-shared
libode_SOURCES=$(libode_a_SOURCES) $(libfast_a_SOURCES)
if OPCODE
libode_DEPENDENCIES+= libOPCODE.a
libode_SOURCES+=$(libOPCODE_a_SOURCES)
endif

# convenience library to simulate per object cflags
noinst_LIBRARIES= libfast.a
libfast_a_CFLAGS= -O1
libfast_a_SOURCES= fastldlt.c fastltsolve.c fastdot.c fastlsolve.c

libode_a_DEPENDENCIES = libfast.a
libode_a_LIBADD= $(libfast_a_OBJECTS)

libode_a_SOURCES = 	objects.h \
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
noinst_LIBRARIES+= libOPCODE.a
libOPCODE_a_CPPFLAGS= -O1
libOPCODE_a_SOURCES=	@TOPDIR@/OPCODE/OPC_AABBCollider.cpp \
			@TOPDIR@/OPCODE/OPC_AABBTree.cpp \
			@TOPDIR@/OPCODE/OPC_BaseModel.cpp \
			@TOPDIR@/OPCODE/OPC_BoxPruning.cpp \
			@TOPDIR@/OPCODE/OPC_Collider.cpp \
			@TOPDIR@/OPCODE/OPC_Common.cpp \
			@TOPDIR@/OPCODE/OPC_HybridModel.cpp \
			@TOPDIR@/OPCODE/OPC_LSSCollider.cpp \
			@TOPDIR@/OPCODE/OPC_MeshInterface.cpp \
			@TOPDIR@/OPCODE/OPC_Model.cpp \
			@TOPDIR@/OPCODE/OPC_OBBCollider.cpp \
			@TOPDIR@/OPCODE/Opcode.cpp \
			@TOPDIR@/OPCODE/OPC_OptimizedTree.cpp \
			@TOPDIR@/OPCODE/OPC_Picking.cpp \
			@TOPDIR@/OPCODE/OPC_PlanesCollider.cpp \
			@TOPDIR@/OPCODE/OPC_RayCollider.cpp \
			@TOPDIR@/OPCODE/OPC_SphereCollider.cpp \
			@TOPDIR@/OPCODE/OPC_SweepAndPrune.cpp \
			@TOPDIR@/OPCODE/OPC_TreeBuilders.cpp \
			@TOPDIR@/OPCODE/OPC_TreeCollider.cpp \
			@TOPDIR@/OPCODE/OPC_VolumeCollider.cpp \
			@TOPDIR@/OPCODE/Ice/IceAABB.cpp \
			@TOPDIR@/OPCODE/Ice/IceContainer.cpp \
			@TOPDIR@/OPCODE/Ice/IceHPoint.cpp \
			@TOPDIR@/OPCODE/Ice/IceIndexedTriangle.cpp \
			@TOPDIR@/OPCODE/Ice/IceMatrix3x3.cpp \
			@TOPDIR@/OPCODE/Ice/IceMatrix4x4.cpp \
			@TOPDIR@/OPCODE/Ice/IceOBB.cpp \
			@TOPDIR@/OPCODE/Ice/IcePlane.cpp \
			@TOPDIR@/OPCODE/Ice/IcePoint.cpp \
			@TOPDIR@/OPCODE/Ice/IceRandom.cpp \
			@TOPDIR@/OPCODE/Ice/IceRay.cpp \
			@TOPDIR@/OPCODE/Ice/IceRevisitedRadix.cpp \
			@TOPDIR@/OPCODE/Ice/IceSegment.cpp \
			@TOPDIR@/OPCODE/Ice/IceTriangle.cpp \
			@TOPDIR@/OPCODE/Ice/IceUtils.cpp
libode_a_DEPENDENCIES+=libOPCODE.a

libode_a_LIBADD+= $(libOPCODE_a_OBJECTS)
AM_CXXFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED
AM_CFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED
libode_a_SOURCES+=	collision_trimesh_trimesh.cpp \
			collision_trimesh_sphere.cpp \
			collision_trimesh_ray.cpp \
			collision_trimesh.cpp \
			collision_trimesh_box.cpp \
			collision_trimesh_ccylinder.cpp \
			collision_trimesh_distance.cpp \
			collision_trimesh_internal.h
endif

if CYLINDER
AM_CXXFLAGS += -DdCYLINDER_ENABLED
libode_a_SOURCES+=      collision_cylinder_box.cpp \
                        collision_cylinder_sphere.cpp \
                        collision_cylinder_trimesh.cpp
endif

