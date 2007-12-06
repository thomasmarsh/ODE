lib_LTLIBRARIES = libode.la
libode_la_CPPFLAGS = -I$(top_srcdir)/include -I$(top_builddir)/include
libode_la_LDFLAGS=@EXTRA_LIBTOOL_LDFLAGS@ -release @LIBTOOL_RELEASE@
libode_la_DEPENDENCIES = libfast.la
libode_la_LIBADD= $(libfast_la_OBJECTS)

if OPCODE
libode_la_DEPENDENCIES+= libOPCODE.la
libode_la_LIBADD+=$(libOPCODE_la_OBJECTS)
endif


if GIMPACT
libode_la_DEPENDENCIES+= libGIMPACT.la
libode_la_LIBADD+=$(libGIMPACT_la_OBJECTS)
endif


# convenience library to simulate per object cflags
noinst_LTLIBRARIES= libfast.la
libfast_la_CPPFLAGS= -I$(top_srcdir)/include -I$(top_builddir)/include
libfast_la_CFLAGS= -O1
libfast_la_SOURCES= fastldlt.c fastltsolve.c fastdot.c fastlsolve.c
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
noinst_LTLIBRARIES+= libGIMPACT.la
libGIMPACT_la_CPPFLAGS= -O2 -fno-strict-aliasing

libode_la_SOURCES+= collision_trimesh_gimpact.cpp

libGIMPACT_la_SOURCES = \
  @TOPDIR@/GIMPACT/src/gim_boxpruning.cpp \
  @TOPDIR@/GIMPACT/src/gim_contact.cpp \
  @TOPDIR@/GIMPACT/src/gim_math.cpp \
  @TOPDIR@/GIMPACT/src/gim_memory.cpp \
  @TOPDIR@/GIMPACT/src/gim_tri_tri_overlap.cpp \
  @TOPDIR@/GIMPACT/src/gim_trimesh.cpp \
  @TOPDIR@/GIMPACT/src/gim_trimesh_capsule_collision.cpp \
  @TOPDIR@/GIMPACT/src/gim_trimesh_ray_collision.cpp \
  @TOPDIR@/GIMPACT/src/gim_trimesh_sphere_collision.cpp \
  @TOPDIR@/GIMPACT/src/gim_trimesh_trimesh_collision.cpp \
  @TOPDIR@/GIMPACT/src/gimpact.cpp

libode_la_DEPENDENCIES+=libGIMPACT.la
libode_la_LIBADD+= $(libGIMPACT_la_OBJECTS)
libode_la_CPPFLAGS+= -I@TOPDIR@/GIMPACT/include -DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT
libode_la_SOURCES+=	collision_trimesh_trimesh.cpp \
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
noinst_LTLIBRARIES+= libOPCODE.la
libOPCODE_la_CPPFLAGS= -O2 -fno-strict-aliasing


libOPCODE_la_SOURCES=	@TOPDIR@/OPCODE/OPC_AABBCollider.cpp \
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
libode_la_DEPENDENCIES+=libOPCODE.la

libode_la_LIBADD+= $(libOPCODE_la_OBJECTS)
libode_la_CPPFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE
libode_la_SOURCES+=	collision_trimesh_trimesh.cpp \
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


