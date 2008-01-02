lib_LTLIBRARIES = libode.la
libode_la_CPPFLAGS = -I$(top_srcdir)/include -I$(top_builddir)/include
libode_la_LDFLAGS=@EXTRA_LIBTOOL_LDFLAGS@ -release @LIBTOOL_RELEASE@
libode_la_DEPENDENCIES = libfast.la
libode_la_LIBADD= $(libfast_la_OBJECTS)


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
libGIMPACT_la_CPPFLAGS= -O2 -fno-strict-aliasing -I$(top_builddir)/include -I$(top_srcdir)/GIMPACT/include

libGIMPACT_la_SOURCES = \
  $(top_srcdir)/GIMPACT/src/gim_boxpruning.cpp \
  $(top_srcdir)/GIMPACT/src/gim_contact.cpp \
  $(top_srcdir)/GIMPACT/src/gim_math.cpp \
  $(top_srcdir)/GIMPACT/src/gim_memory.cpp \
  $(top_srcdir)/GIMPACT/src/gim_tri_tri_overlap.cpp \
  $(top_srcdir)/GIMPACT/src/gim_trimesh.cpp \
  $(top_srcdir)/GIMPACT/src/gim_trimesh_capsule_collision.cpp \
  $(top_srcdir)/GIMPACT/src/gim_trimesh_ray_collision.cpp \
  $(top_srcdir)/GIMPACT/src/gim_trimesh_sphere_collision.cpp \
  $(top_srcdir)/GIMPACT/src/gim_trimesh_trimesh_collision.cpp \
  $(top_srcdir)/GIMPACT/src/gimpact.cpp

libode_la_DEPENDENCIES+=libGIMPACT.la
libode_la_LIBADD+= $(libGIMPACT_la_OBJECTS)
libode_la_CPPFLAGS+= -I$(top_srcdir)/GIMPACT/include
libode_la_SOURCES+=	collision_trimesh_trimesh.cpp \
			collision_trimesh_sphere.cpp \
			collision_trimesh_ray.cpp \
			collision_trimesh_gimpact.cpp \
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
libOPCODE_la_CPPFLAGS= -O2 -fno-strict-aliasing -I$(top_builddir)/include -I$(top_srcdir)/include -I$(top_srcdir)/OPCODE

libOPCODE_la_SOURCES=	$(top_srcdir)/OPCODE/OPC_AABBCollider.cpp \
			$(top_srcdir)/OPCODE/OPC_AABBTree.cpp \
			$(top_srcdir)/OPCODE/OPC_BaseModel.cpp \
			$(top_srcdir)/OPCODE/OPC_BoxPruning.cpp \
			$(top_srcdir)/OPCODE/OPC_Collider.cpp \
			$(top_srcdir)/OPCODE/OPC_Common.cpp \
			$(top_srcdir)/OPCODE/OPC_HybridModel.cpp \
			$(top_srcdir)/OPCODE/OPC_LSSCollider.cpp \
			$(top_srcdir)/OPCODE/OPC_MeshInterface.cpp \
			$(top_srcdir)/OPCODE/OPC_Model.cpp \
			$(top_srcdir)/OPCODE/OPC_OBBCollider.cpp \
			$(top_srcdir)/OPCODE/Opcode.cpp \
			$(top_srcdir)/OPCODE/OPC_OptimizedTree.cpp \
			$(top_srcdir)/OPCODE/OPC_Picking.cpp \
			$(top_srcdir)/OPCODE/OPC_PlanesCollider.cpp \
			$(top_srcdir)/OPCODE/OPC_RayCollider.cpp \
			$(top_srcdir)/OPCODE/OPC_SphereCollider.cpp \
			$(top_srcdir)/OPCODE/OPC_SweepAndPrune.cpp \
			$(top_srcdir)/OPCODE/OPC_TreeBuilders.cpp \
			$(top_srcdir)/OPCODE/OPC_TreeCollider.cpp \
			$(top_srcdir)/OPCODE/OPC_VolumeCollider.cpp \
			$(top_srcdir)/OPCODE/Ice/IceAABB.cpp \
			$(top_srcdir)/OPCODE/Ice/IceContainer.cpp \
			$(top_srcdir)/OPCODE/Ice/IceHPoint.cpp \
			$(top_srcdir)/OPCODE/Ice/IceIndexedTriangle.cpp \
			$(top_srcdir)/OPCODE/Ice/IceMatrix3x3.cpp \
			$(top_srcdir)/OPCODE/Ice/IceMatrix4x4.cpp \
			$(top_srcdir)/OPCODE/Ice/IceOBB.cpp \
			$(top_srcdir)/OPCODE/Ice/IcePlane.cpp \
			$(top_srcdir)/OPCODE/Ice/IcePoint.cpp \
			$(top_srcdir)/OPCODE/Ice/IceRandom.cpp \
			$(top_srcdir)/OPCODE/Ice/IceRay.cpp \
			$(top_srcdir)/OPCODE/Ice/IceRevisitedRadix.cpp \
			$(top_srcdir)/OPCODE/Ice/IceSegment.cpp \
			$(top_srcdir)/OPCODE/Ice/IceTriangle.cpp \
			$(top_srcdir)/OPCODE/Ice/IceUtils.cpp \
			$(top_srcdir)/OPCODE/OPC_AABBCollider.h \
			$(top_srcdir)/OPCODE/OPC_MeshInterface.h \
			$(top_srcdir)/OPCODE/OPC_Settings.h \
			$(top_srcdir)/OPCODE/OPC_AABBTree.h \
			$(top_srcdir)/OPCODE/OPC_Model.h \
			$(top_srcdir)/OPCODE/OPC_SphereAABBOverlap.h \
			$(top_srcdir)/OPCODE/OPC_BaseModel.h \
			$(top_srcdir)/OPCODE/OPC_OBBCollider.h \
			$(top_srcdir)/OPCODE/OPC_SphereCollider.h \
			$(top_srcdir)/OPCODE/OPC_BoxBoxOverlap.h \
			$(top_srcdir)/OPCODE/Opcode.h \
			$(top_srcdir)/OPCODE/OPC_SphereTriOverlap.h \
			$(top_srcdir)/OPCODE/OPC_BoxPruning.h \
			$(top_srcdir)/OPCODE/OPC_OptimizedTree.h \
			$(top_srcdir)/OPCODE/OPC_SweepAndPrune.h \
			$(top_srcdir)/OPCODE/OPC_Collider.h \
			$(top_srcdir)/OPCODE/OPC_Picking.h \
			$(top_srcdir)/OPCODE/OPC_TreeBuilders.h \
			$(top_srcdir)/OPCODE/OPC_Common.h \
			$(top_srcdir)/OPCODE/OPC_PlanesAABBOverlap.h \
			$(top_srcdir)/OPCODE/OPC_TreeCollider.h \
			$(top_srcdir)/OPCODE/OPC_HybridModel.h \
			$(top_srcdir)/OPCODE/OPC_PlanesCollider.h \
			$(top_srcdir)/OPCODE/OPC_TriBoxOverlap.h \
			$(top_srcdir)/OPCODE/OPC_IceHook.h \
			$(top_srcdir)/OPCODE/OPC_PlanesTriOverlap.h \
			$(top_srcdir)/OPCODE/OPC_TriTriOverlap.h \
			$(top_srcdir)/OPCODE/OPC_LSSAABBOverlap.h \
			$(top_srcdir)/OPCODE/OPC_RayAABBOverlap.h \
			$(top_srcdir)/OPCODE/OPC_VolumeCollider.h \
			$(top_srcdir)/OPCODE/OPC_LSSCollider.h \
			$(top_srcdir)/OPCODE/OPC_RayCollider.h \
			$(top_srcdir)/OPCODE/Stdafx.h \
			$(top_srcdir)/OPCODE/OPC_LSSTriOverlap.h \
			$(top_srcdir)/OPCODE/OPC_RayTriOverlap.h \
			$(top_srcdir)/OPCODE/Ice/IceAABB.h \
			$(top_srcdir)/OPCODE/Ice/IceMatrix3x3.h \
			$(top_srcdir)/OPCODE/Ice/IceRandom.h \
			$(top_srcdir)/OPCODE/Ice/IceAxes.h \
			$(top_srcdir)/OPCODE/Ice/IceMatrix4x4.h \
			$(top_srcdir)/OPCODE/Ice/IceRay.h \
			$(top_srcdir)/OPCODE/Ice/IceBoundingSphere.h \
			$(top_srcdir)/OPCODE/Ice/IceMemoryMacros.h \
			$(top_srcdir)/OPCODE/Ice/IceRevisitedRadix.h \
			$(top_srcdir)/OPCODE/Ice/IceContainer.h \
			$(top_srcdir)/OPCODE/Ice/IceOBB.h \
			$(top_srcdir)/OPCODE/Ice/IceSegment.h \
			$(top_srcdir)/OPCODE/Ice/IceFPU.h \
			$(top_srcdir)/OPCODE/Ice/IcePairs.h \
			$(top_srcdir)/OPCODE/Ice/IceTriangle.h \
			$(top_srcdir)/OPCODE/Ice/IceHPoint.h \
			$(top_srcdir)/OPCODE/Ice/IcePlane.h \
			$(top_srcdir)/OPCODE/Ice/IceTriList.h \
			$(top_srcdir)/OPCODE/Ice/IceIndexedTriangle.h \
			$(top_srcdir)/OPCODE/Ice/IcePoint.h \
			$(top_srcdir)/OPCODE/Ice/IceTypes.h

libode_la_DEPENDENCIES+=libOPCODE.la

libode_la_LIBADD+= $(libOPCODE_la_OBJECTS)
libode_la_CPPFLAGS += -I$(top_srcdir)/OPCODE -I$(top_srcdir)/OPCODE/Ice
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


