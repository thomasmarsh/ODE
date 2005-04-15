AM_CXXFLAGS = -I@TOPDIR@/include
AM_CPPFLAGS = -I@TOPDIR@/include
AM_CFLAGS = -I@TOPDIR@/include
lib_LTLIBRARIES = libode.la
libode_la_LDFLAGS = -no-undefined -version-info @ODE_CURRENT@:@ODE_REVISION@:@ODE_AGE@ -release @ODE_RELEASE@
libode_la_CPPFLAGS = -O2

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
			fastdot.c \
			quickstep.h \
			collision_quadtreespace.cpp \
			fastldlt.c \
			rotation.cpp \
			collision_space.cpp \
			fastlsolve.c \
			collision_space_internal.h \
			fastltsolve.c \
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

#Hack to get around automake inability to set per-file CFLAGS

libode_la-fastldlt.lo: fastldlt.c
	if $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS) -MT libode_la-fastldlt.lo -MD -MP -MF "$(DEPDIR)/libode_la-fastldlt.Tpo" -c -o libode_la-fastldlt.lo `test -f 'fastldlt.c' || echo '$(srcdir)/'`fastldlt.c; \
	then mv -f "$(DEPDIR)/libode_la-fastldlt.Tpo" "$(DEPDIR)/libode_la-fastldlt.Plo"; else rm -f "$(DEPDIR)/libode_la-fastldlt.Tpo"; exit 1; fi

libode_la-fastlsolve.lo: fastlsolve.c
	if $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS) -MT libode_la-fastlsolve.lo -MD -MP -MF "$(DEPDIR)/libode_la-fastlsolve.Tpo" -c -o libode_la-fastlsolve.lo `test -f 'fastlsolve.c' || echo '$(srcdir)/'`fastlsolve.c; \
	then mv -f "$(DEPDIR)/libode_la-fastlsolve.Tpo" "$(DEPDIR)/libode_la-fastlsolve.Plo"; else rm -f "$(DEPDIR)/libode_la-fastlsolve.Tpo"; exit 1; fi

libode_la-fastltsolve.lo: fastltsolve.c
	if $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS) -MT libode_la-fastltsolve.lo -MD -MP -MF "$(DEPDIR)/libode_la-fastltsolve.Tpo" -c -o libode_la-fastltsolve.lo `test -f 'fastltsolve.c' || echo '$(srcdir)/'`fastltsolve.c; \
	then mv -f "$(DEPDIR)/libode_la-fastltsolve.Tpo" "$(DEPDIR)/libode_la-fastltsolve.Plo"; else rm -f "$(DEPDIR)/libode_la-fastltsolve.Tpo"; exit 1; fi

libode_la-fastdot.lo: fastdot.c
	if $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS) -MT libode_la-fastdot.lo -MD -MP -MF "$(DEPDIR)/libode_la-fastdot.Tpo" -c -o libode_la-fastdot.lo `test -f 'fastdot.c' || echo '$(srcdir)/'`fastdot.c; \
	then mv -f "$(DEPDIR)/libode_la-fastdot.Tpo" "$(DEPDIR)/libode_la-fastdot.Plo"; else rm -f "$(DEPDIR)/libode_la-fastdot.Tpo"; exit 1; fi

if OPCODE

AM_CXXFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED
AM_CFLAGS += -I@TOPDIR@/OPCODE -I@TOPDIR@/OPCODE/Ice -DdTRIMESH_ENABLED
libode_la_SOURCES+=	collision_trimesh_trimesh.cpp \
			collision_trimesh_sphere.cpp \
			collision_trimesh_ray.cpp \
			collision_trimesh.cpp \
			collision_trimesh_box.cpp \
			collision_trimesh_ccylinder.cpp \
			collision_trimesh_distance.cpp \
			collision_trimesh_internal.h \
			@TOPDIR@/OPCODE/OPC_AABBCollider.cpp \
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

#Hack to get around automake inability to set per-file CFLAGS

libode_la-OPC_AABBCollider.lo: @TOPDIR@/OPCODE/OPC_AABBCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_AABBCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_AABBCollider.Tpo" -c -o libode_la-OPC_AABBCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_AABBCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_AABBCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_AABBCollider.Tpo" "$(DEPDIR)/libode_la-OPC_AABBCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_AABBCollider.Tpo"; exit 1; fi

libode_la-OPC_AABBTree.lo: @TOPDIR@/OPCODE/OPC_AABBTree.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_AABBTree.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_AABBTree.Tpo" -c -o libode_la-OPC_AABBTree.lo `test -f '@TOPDIR@/OPCODE/OPC_AABBTree.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_AABBTree.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_AABBTree.Tpo" "$(DEPDIR)/libode_la-OPC_AABBTree.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_AABBTree.Tpo"; exit 1; fi

libode_la-OPC_BaseModel.lo: @TOPDIR@/OPCODE/OPC_BaseModel.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_BaseModel.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_BaseModel.Tpo" -c -o libode_la-OPC_BaseModel.lo `test -f '@TOPDIR@/OPCODE/OPC_BaseModel.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_BaseModel.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_BaseModel.Tpo" "$(DEPDIR)/libode_la-OPC_BaseModel.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_BaseModel.Tpo"; exit 1; fi

libode_la-OPC_BoxPruning.lo: @TOPDIR@/OPCODE/OPC_BoxPruning.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_BoxPruning.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_BoxPruning.Tpo" -c -o libode_la-OPC_BoxPruning.lo `test -f '@TOPDIR@/OPCODE/OPC_BoxPruning.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_BoxPruning.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_BoxPruning.Tpo" "$(DEPDIR)/libode_la-OPC_BoxPruning.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_BoxPruning.Tpo"; exit 1; fi

libode_la-OPC_Collider.lo: @TOPDIR@/OPCODE/OPC_Collider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_Collider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_Collider.Tpo" -c -o libode_la-OPC_Collider.lo `test -f '@TOPDIR@/OPCODE/OPC_Collider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_Collider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_Collider.Tpo" "$(DEPDIR)/libode_la-OPC_Collider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_Collider.Tpo"; exit 1; fi


libode_la-OPC_Common.lo: @TOPDIR@/OPCODE/OPC_Common.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_Common.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_Common.Tpo" -c -o libode_la-OPC_Common.lo `test -f '@TOPDIR@/OPCODE/OPC_Common.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_Common.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_Common.Tpo" "$(DEPDIR)/libode_la-OPC_Common.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_Common.Tpo"; exit 1; fi


libode_la-OPC_HybridModel.lo: @TOPDIR@/OPCODE/OPC_HybridModel.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_HybridModel.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_HybridModel.Tpo" -c -o libode_la-OPC_HybridModel.lo `test -f '@TOPDIR@/OPCODE/OPC_HybridModel.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_HybridModel.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_HybridModel.Tpo" "$(DEPDIR)/libode_la-OPC_HybridModel.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_HybridModel.Tpo"; exit 1; fi


libode_la-OPC_LSSCollider.lo: @TOPDIR@/OPCODE/OPC_LSSCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_LSSCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_LSSCollider.Tpo" -c -o libode_la-OPC_LSSCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_LSSCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_LSSCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_LSSCollider.Tpo" "$(DEPDIR)/libode_la-OPC_LSSCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_LSSCollider.Tpo"; exit 1; fi


libode_la-OPC_MeshInterface.lo: @TOPDIR@/OPCODE/OPC_MeshInterface.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_MeshInterface.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_MeshInterface.Tpo" -c -o libode_la-OPC_MeshInterface.lo `test -f '@TOPDIR@/OPCODE/OPC_MeshInterface.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_MeshInterface.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_MeshInterface.Tpo" "$(DEPDIR)/libode_la-OPC_MeshInterface.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_MeshInterface.Tpo"; exit 1; fi

libode_la-OPC_Model.lo: @TOPDIR@/OPCODE/OPC_Model.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_Model.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_Model.Tpo" -c -o libode_la-OPC_Model.lo `test -f '@TOPDIR@/OPCODE/OPC_Model.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_Model.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_Model.Tpo" "$(DEPDIR)/libode_la-OPC_Model.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_Model.Tpo"; exit 1; fi


libode_la-OPC_OBBCollider.lo: @TOPDIR@/OPCODE/OPC_OBBCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_OBBCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_OBBCollider.Tpo" -c -o libode_la-OPC_OBBCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_OBBCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_OBBCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_OBBCollider.Tpo" "$(DEPDIR)/libode_la-OPC_OBBCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_OBBCollider.Tpo"; exit 1; fi


libode_la-OPC_OptimizedTree.lo: @TOPDIR@/OPCODE/OPC_OptimizedTree.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_OptimizedTree.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_OptimizedTree.Tpo" -c -o libode_la-OPC_OptimizedTree.lo `test -f '@TOPDIR@/OPCODE/OPC_OptimizedTree.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_OptimizedTree.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_OptimizedTree.Tpo" "$(DEPDIR)/libode_la-OPC_OptimizedTree.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_OptimizedTree.Tpo"; exit 1; fi


libode_la-OPC_Picking.lo: @TOPDIR@/OPCODE/OPC_Picking.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_Picking.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_Picking.Tpo" -c -o libode_la-OPC_Picking.lo `test -f '@TOPDIR@/OPCODE/OPC_Picking.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_Picking.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_Picking.Tpo" "$(DEPDIR)/libode_la-OPC_Picking.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_Picking.Tpo"; exit 1; fi


libode_la-OPC_PlanesCollider.lo: @TOPDIR@/OPCODE/OPC_PlanesCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_PlanesCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_PlanesCollider.Tpo" -c -o libode_la-OPC_PlanesCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_PlanesCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_PlanesCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_PlanesCollider.Tpo" "$(DEPDIR)/libode_la-OPC_PlanesCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_PlanesCollider.Tpo"; exit 1; fi


libode_la-OPC_RayCollider.lo: @TOPDIR@/OPCODE/OPC_RayCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_RayCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_RayCollider.Tpo" -c -o libode_la-OPC_RayCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_RayCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_RayCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_RayCollider.Tpo" "$(DEPDIR)/libode_la-OPC_RayCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_RayCollider.Tpo"; exit 1; fi

libode_la-OPC_SphereCollider.lo: @TOPDIR@/OPCODE/OPC_SphereCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_SphereCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_SphereCollider.Tpo" -c -o libode_la-OPC_SphereCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_SphereCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_SphereCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_SphereCollider.Tpo" "$(DEPDIR)/libode_la-OPC_SphereCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_SphereCollider.Tpo"; exit 1; fi


libode_la-OPC_SweepAndPrune.lo: @TOPDIR@/OPCODE/OPC_SweepAndPrune.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_SweepAndPrune.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_SweepAndPrune.Tpo" -c -o libode_la-OPC_SweepAndPrune.lo `test -f '@TOPDIR@/OPCODE/OPC_SweepAndPrune.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_SweepAndPrune.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_SweepAndPrune.Tpo" "$(DEPDIR)/libode_la-OPC_SweepAndPrune.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_SweepAndPrune.Tpo"; exit 1; fi

libode_la-OPC_TreeBuilders.lo: @TOPDIR@/OPCODE/OPC_TreeBuilders.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_TreeBuilders.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_TreeBuilders.Tpo" -c -o libode_la-OPC_TreeBuilders.lo `test -f '@TOPDIR@/OPCODE/OPC_TreeBuilders.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_TreeBuilders.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_TreeBuilders.Tpo" "$(DEPDIR)/libode_la-OPC_TreeBuilders.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_TreeBuilders.Tpo"; exit 1; fi

libode_la-OPC_TreeCollider.lo: @TOPDIR@/OPCODE/OPC_TreeCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_TreeCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_TreeCollider.Tpo" -c -o libode_la-OPC_TreeCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_TreeCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_TreeCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_TreeCollider.Tpo" "$(DEPDIR)/libode_la-OPC_TreeCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_TreeCollider.Tpo"; exit 1; fi

libode_la-OPC_VolumeCollider.lo: @TOPDIR@/OPCODE/OPC_VolumeCollider.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-OPC_VolumeCollider.lo -MD -MP -MF "$(DEPDIR)/libode_la-OPC_VolumeCollider.Tpo" -c -o libode_la-OPC_VolumeCollider.lo `test -f '@TOPDIR@/OPCODE/OPC_VolumeCollider.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/OPC_VolumeCollider.cpp; \
	then mv -f "$(DEPDIR)/libode_la-OPC_VolumeCollider.Tpo" "$(DEPDIR)/libode_la-OPC_VolumeCollider.Plo"; else rm -f "$(DEPDIR)/libode_la-OPC_VolumeCollider.Tpo"; exit 1; fi

libode_la-IceAABB.lo: @TOPDIR@/OPCODE/Ice/IceAABB.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceAABB.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceAABB.Tpo" -c -o libode_la-IceAABB.lo `test -f '@TOPDIR@/OPCODE/Ice/IceAABB.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceAABB.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceAABB.Tpo" "$(DEPDIR)/libode_la-IceAABB.Plo"; else rm -f "$(DEPDIR)/libode_la-IceAABB.Tpo"; exit 1; fi

libode_la-IceContainer.lo: @TOPDIR@/OPCODE/Ice/IceContainer.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceContainer.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceContainer.Tpo" -c -o libode_la-IceContainer.lo `test -f '@TOPDIR@/OPCODE/Ice/IceContainer.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceContainer.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceContainer.Tpo" "$(DEPDIR)/libode_la-IceContainer.Plo"; else rm -f "$(DEPDIR)/libode_la-IceContainer.Tpo"; exit 1; fi


libode_la-IceHPoint.lo: @TOPDIR@/OPCODE/Ice/IceHPoint.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceHPoint.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceHPoint.Tpo" -c -o libode_la-IceHPoint.lo `test -f '@TOPDIR@/OPCODE/Ice/IceHPoint.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceHPoint.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceHPoint.Tpo" "$(DEPDIR)/libode_la-IceHPoint.Plo"; else rm -f "$(DEPDIR)/libode_la-IceHPoint.Tpo"; exit 1; fi


libode_la-IceIndexedTriangle.lo: @TOPDIR@/OPCODE/Ice/IceIndexedTriangle.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceIndexedTriangle.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceIndexedTriangle.Tpo" -c -o libode_la-IceIndexedTriangle.lo `test -f '@TOPDIR@/OPCODE/Ice/IceIndexedTriangle.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceIndexedTriangle.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceIndexedTriangle.Tpo" "$(DEPDIR)/libode_la-IceIndexedTriangle.Plo"; else rm -f "$(DEPDIR)/libode_la-IceIndexedTriangle.Tpo"; exit 1; fi


libode_la-IceMatrix3x3.lo: @TOPDIR@/OPCODE/Ice/IceMatrix3x3.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceMatrix3x3.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceMatrix3x3.Tpo" -c -o libode_la-IceMatrix3x3.lo `test -f '@TOPDIR@/OPCODE/Ice/IceMatrix3x3.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceMatrix3x3.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceMatrix3x3.Tpo" "$(DEPDIR)/libode_la-IceMatrix3x3.Plo"; else rm -f "$(DEPDIR)/libode_la-IceMatrix3x3.Tpo"; exit 1; fi


libode_la-IceMatrix4x4.lo: @TOPDIR@/OPCODE/Ice/IceMatrix4x4.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceMatrix4x4.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceMatrix4x4.Tpo" -c -o libode_la-IceMatrix4x4.lo `test -f '@TOPDIR@/OPCODE/Ice/IceMatrix4x4.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceMatrix4x4.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceMatrix4x4.Tpo" "$(DEPDIR)/libode_la-IceMatrix4x4.Plo"; else rm -f "$(DEPDIR)/libode_la-IceMatrix4x4.Tpo"; exit 1; fi

libode_la-IceOBB.lo: @TOPDIR@/OPCODE/Ice/IceOBB.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceOBB.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceOBB.Tpo" -c -o libode_la-IceOBB.lo `test -f '@TOPDIR@/OPCODE/Ice/IceOBB.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceOBB.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceOBB.Tpo" "$(DEPDIR)/libode_la-IceOBB.Plo"; else rm -f "$(DEPDIR)/libode_la-IceOBB.Tpo"; exit 1; fi


libode_la-IcePlane.lo: @TOPDIR@/OPCODE/Ice/IcePlane.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IcePlane.lo -MD -MP -MF "$(DEPDIR)/libode_la-IcePlane.Tpo" -c -o libode_la-IcePlane.lo `test -f '@TOPDIR@/OPCODE/Ice/IcePlane.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IcePlane.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IcePlane.Tpo" "$(DEPDIR)/libode_la-IcePlane.Plo"; else rm -f "$(DEPDIR)/libode_la-IcePlane.Tpo"; exit 1; fi


libode_la-IcePoint.lo: @TOPDIR@/OPCODE/Ice/IcePoint.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IcePoint.lo -MD -MP -MF "$(DEPDIR)/libode_la-IcePoint.Tpo" -c -o libode_la-IcePoint.lo `test -f '@TOPDIR@/OPCODE/Ice/IcePoint.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IcePoint.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IcePoint.Tpo" "$(DEPDIR)/libode_la-IcePoint.Plo"; else rm -f "$(DEPDIR)/libode_la-IcePoint.Tpo"; exit 1; fi


libode_la-IceRandom.lo: @TOPDIR@/OPCODE/Ice/IceRandom.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceRandom.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceRandom.Tpo" -c -o libode_la-IceRandom.lo `test -f '@TOPDIR@/OPCODE/Ice/IceRandom.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceRandom.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceRandom.Tpo" "$(DEPDIR)/libode_la-IceRandom.Plo"; else rm -f "$(DEPDIR)/libode_la-IceRandom.Tpo"; exit 1; fi

libode_la-IceRay.lo: @TOPDIR@/OPCODE/Ice/IceRay.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceRay.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceRay.Tpo" -c -o libode_la-IceRay.lo `test -f '@TOPDIR@/OPCODE/Ice/IceRay.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceRay.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceRay.Tpo" "$(DEPDIR)/libode_la-IceRay.Plo"; else rm -f "$(DEPDIR)/libode_la-IceRay.Tpo"; exit 1; fi

libode_la-IceRevisitedRadix.lo: @TOPDIR@/OPCODE/Ice/IceRevisitedRadix.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceRevisitedRadix.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceRevisitedRadix.Tpo" -c -o libode_la-IceRevisitedRadix.lo `test -f '@TOPDIR@/OPCODE/Ice/IceRevisitedRadix.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceRevisitedRadix.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceRevisitedRadix.Tpo" "$(DEPDIR)/libode_la-IceRevisitedRadix.Plo"; else rm -f "$(DEPDIR)/libode_la-IceRevisitedRadix.Tpo"; exit 1; fi

libode_la-IceSegment.lo: @TOPDIR@/OPCODE/Ice/IceSegment.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceSegment.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceSegment.Tpo" -c -o libode_la-IceSegment.lo `test -f '@TOPDIR@/OPCODE/Ice/IceSegment.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceSegment.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceSegment.Tpo" "$(DEPDIR)/libode_la-IceSegment.Plo"; else rm -f "$(DEPDIR)/libode_la-IceSegment.Tpo"; exit 1; fi

libode_la-IceTriangle.lo: @TOPDIR@/OPCODE/Ice/IceTriangle.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceTriangle.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceTriangle.Tpo" -c -o libode_la-IceTriangle.lo `test -f '@TOPDIR@/OPCODE/Ice/IceTriangle.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceTriangle.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceTriangle.Tpo" "$(DEPDIR)/libode_la-IceTriangle.Plo"; else rm -f "$(DEPDIR)/libode_la-IceTriangle.Tpo"; exit 1; fi

libode_la-IceUtils.lo: @TOPDIR@/OPCODE/Ice/IceUtils.cpp
	if $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) -O1 $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS) -MT libode_la-IceUtils.lo -MD -MP -MF "$(DEPDIR)/libode_la-IceUtils.Tpo" -c -o libode_la-IceUtils.lo `test -f '@TOPDIR@/OPCODE/Ice/IceUtils.cpp' || echo '$(srcdir)/'`@TOPDIR@/OPCODE/Ice/IceUtils.cpp; \
	then mv -f "$(DEPDIR)/libode_la-IceUtils.Tpo" "$(DEPDIR)/libode_la-IceUtils.Plo"; else rm -f "$(DEPDIR)/libode_la-IceUtils.Tpo"; exit 1; fi
endif


