# Microsoft Developer Studio Project File - Name="drawstuff" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=drawstuff - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "drawstuff.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "drawstuff.mak" CFG="drawstuff - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "drawstuff - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "drawstuff - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "drawstuff - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/release"
# PROP BASE Intermediate_Dir "obj/drawstuff/Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/release"
# PROP Intermediate_Dir "obj/drawstuff/Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GR /GX /O2 /I "../../include" /D "DS_DLL" /D "USRDLL" /D "WIN32" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GR /GX /O2 /I "../../include" /D "DS_DLL" /D "USRDLL" /D "WIN32" /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib opengl32.lib glu32.lib winmm.lib gdi32.lib /nologo /dll /machine:I386 /implib:"../drawstuff.lib" /out:"../../lib/release/drawstuff.dll" /libpath:".."
# ADD LINK32 user32.lib opengl32.lib glu32.lib winmm.lib gdi32.lib /nologo /dll /machine:I386 /implib:"../drawstuff.lib" /out:"../../lib/release/drawstuff.dll" /libpath:".."

!ELSEIF  "$(CFG)" == "drawstuff - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "../../lib/debug"
# PROP BASE Intermediate_Dir "obj/drawstuff/Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "../../lib/debug"
# PROP Intermediate_Dir "obj/drawstuff/Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "DS_DLL" /D "USRDLL" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "DS_DLL" /D "USRDLL" /D "WIN32" /YX /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib opengl32.lib glu32.lib winmm.lib gdi32.lib /nologo /dll /incremental:yes /debug /machine:I386 /implib:"../../lib/debug/drawstuff.lib" /out:"../../lib/debug/drawstuff.dll" /pdbtype:sept /libpath:"../../lib/debug"
# ADD LINK32 user32.lib opengl32.lib glu32.lib winmm.lib gdi32.lib /nologo /dll /incremental:yes /debug /machine:I386 /implib:"../../lib/debug/drawstuff.lib" /out:"../../lib/debug/drawstuff.dll" /pdbtype:sept /libpath:"../../lib/debug"

!ENDIF

# Begin Target

# Name "drawstuff - Win32 Release"
# Name "drawstuff - Win32 Debug"
# Begin Group "include"

# PROP Default_Filter ""
# Begin Group "drawstuff"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../include/drawstuff/drawstuff.h
# End Source File
# Begin Source File

SOURCE=../../include/drawstuff/version.h
# End Source File
# End Group
# End Group
# Begin Group "drawstuff"

# PROP Default_Filter ""
# Begin Group "src"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../drawstuff/src/internal.h
# End Source File
# Begin Source File

SOURCE=../../drawstuff/src/drawstuff.cpp
# End Source File
# Begin Source File

SOURCE=../../drawstuff/src/resource.h
# End Source File
# Begin Source File

SOURCE=../../drawstuff/src/resources.rc
# End Source File
# Begin Source File

SOURCE=../../drawstuff/src/windows.cpp
# End Source File
# End Group
# End Group
# End Target
# End Project
