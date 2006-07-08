# Microsoft Developer Studio Project File - Name="ode" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=ode - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "ode.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "ode.mak" CFG="ode - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "ode - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "ode - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "ode - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/release"
# PROP BASE Intermediate_Dir "obj/ode/Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/release"
# PROP Intermediate_Dir "obj/ode/Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GR /GX /O2 /Oy /I "../../include" /I "../../OPCODE" /D "ODE_DLL" /D "WIN32" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GR /GX /O2 /Oy /I "../../include" /I "../../OPCODE" /D "ODE_DLL" /D "WIN32" /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib /nologo /dll /machine:I386 /implib:"../ode.lib" /out:"../../lib/release/ode.dll" /libpath:".."
# ADD LINK32 user32.lib /nologo /dll /machine:I386 /implib:"../ode.lib" /out:"../../lib/release/ode.dll" /libpath:".."

!ELSEIF  "$(CFG)" == "ode - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "../../lib/debug"
# PROP BASE Intermediate_Dir "obj/ode/Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "../../lib/debug"
# PROP Intermediate_Dir "obj/ode/Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /I "../../OPCODE" /D "ODE_DLL" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /I "../../OPCODE" /D "ODE_DLL" /D "WIN32" /YX /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib /nologo /dll /incremental:yes /debug /machine:I386 /implib:"../../lib/debug/ode.lib" /out:"../../lib/debug/ode.dll" /pdbtype:sept /libpath:"../../lib/debug"
# ADD LINK32 user32.lib /nologo /dll /incremental:yes /debug /machine:I386 /implib:"../../lib/debug/ode.lib" /out:"../../lib/debug/ode.dll" /pdbtype:sept /libpath:"../../lib/debug"

!ENDIF

# Begin Target

# Name "ode - Win32 Release"
# Name "ode - Win32 Debug"
# Begin Group "include"

# PROP Default_Filter ""
# Begin Group "ode"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../include/ode/collision.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/collision_space.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/collision_trimesh.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/common.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/compatibility.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/config.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/contact.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/error.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/export-dif.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/mass.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/matrix.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/memory.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/misc.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/objects.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/ode.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/odecpp.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/odecpp_collision.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/odecpp_old.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/odemath.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/rotation.h
# End Source File
# Begin Source File

SOURCE=../../include/ode/timer.h
# End Source File
# End Group
# End Group
# Begin Group "ode"

# PROP Default_Filter ""
# Begin Group "src"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../ode/src/array.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_kernel.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_space_internal.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_std.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_transform.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_internal.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_util.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/heightfield.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/joint.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/lcp.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/mat.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/objects.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/obstack.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/quickstep.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/stack.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/step.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/testing.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/util.h
# End Source File
# Begin Source File

SOURCE=../../ode/src/fastdot.c
# End Source File
# Begin Source File

SOURCE=../../ode/src/fastldlt.c
# End Source File
# Begin Source File

SOURCE=../../ode/src/fastlsolve.c
# End Source File
# Begin Source File

SOURCE=../../ode/src/fastltsolve.c
# End Source File
# Begin Source File

SOURCE=../../ode/src/array.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/box.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/capsule.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_cylinder_box.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_cylinder_plane.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_cylinder_sphere.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_cylinder_trimesh.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_kernel.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_quadtreespace.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_space.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_transform.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_box.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_ccylinder.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_distance.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_plane.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_ray.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_sphere.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_trimesh_trimesh.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/collision_util.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/convex.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/cylinder.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/error.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/export-dif.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/heightfield.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/joint.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/lcp.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/mass.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/mat.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/matrix.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/memory.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/misc.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/obstack.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/ode.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/odemath.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/plane.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/quickstep.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/ray.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/rotation.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/sphere.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/step.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/stepfast.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/testing.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/timer.cpp
# End Source File
# Begin Source File

SOURCE=../../ode/src/util.cpp
# End Source File
# End Group
# End Group
# Begin Group "OPCODE"

# PROP Default_Filter ""
# Begin Group "Ice"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../OPCODE/Ice/IceAABB.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceAxes.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceBoundingSphere.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceContainer.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceFPU.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceHPoint.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceIndexedTriangle.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceLSS.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceMatrix3x3.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceMatrix4x4.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceMemoryMacros.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceOBB.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IcePairs.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IcePlane.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IcePoint.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IcePreprocessor.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceRandom.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceRay.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceRevisitedRadix.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceSegment.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceTriangle.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceTriList.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceTypes.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceUtils.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceAABB.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceContainer.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceHPoint.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceIndexedTriangle.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceMatrix3x3.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceMatrix4x4.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceOBB.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IcePlane.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IcePoint.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceRandom.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceRay.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceRevisitedRadix.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceSegment.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceTriangle.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Ice/IceUtils.cpp
# End Source File
# End Group
# Begin Source File

SOURCE=../../OPCODE/Opcode.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_AABBCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_AABBTree.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_BaseModel.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_BoxBoxOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_BoxPruning.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Collider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Common.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_HybridModel.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_IceHook.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_LSSAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_LSSCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_LSSTriOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_MeshInterface.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Model.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_OBBCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_OptimizedTree.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Picking.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_PlanesAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_PlanesCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_PlanesTriOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_RayAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_RayCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_RayTriOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Settings.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_SphereAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_SphereCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_SphereTriOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_SweepAndPrune.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_TreeBuilders.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_TreeCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_TriBoxOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_TriTriOverlap.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_VolumeCollider.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Stdafx.h
# End Source File
# Begin Source File

SOURCE=../../OPCODE/Opcode.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_AABBCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_AABBTree.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_BaseModel.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_BoxPruning.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Collider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Common.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_HybridModel.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_LSSCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_MeshInterface.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Model.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_OBBCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_OptimizedTree.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_Picking.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_PlanesCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_RayCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_SphereCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_SweepAndPrune.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_TreeBuilders.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_TreeCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/OPC_VolumeCollider.cpp
# End Source File
# Begin Source File

SOURCE=../../OPCODE/StdAfx.cpp
# End Source File
# End Group
# End Target
# End Project
