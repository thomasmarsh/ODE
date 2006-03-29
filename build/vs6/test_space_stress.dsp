# Microsoft Developer Studio Project File - Name="test_space_stress" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=test_space_stress - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "test_space_stress.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "test_space_stress.mak" CFG="test_space_stress - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "test_space_stress - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "test_space_stress - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "test_space_stress - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/release"
# PROP BASE Intermediate_Dir "obj/space_stress/Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/release"
# PROP Intermediate_Dir "obj/space_stress/Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GR /GX /O2 /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GR /GX /O2 /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /machine:I386 /out:"../../lib/release/test_space_stress.exe" /libpath:".."
# ADD LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /machine:I386 /out:"../../lib/release/test_space_stress.exe" /libpath:".."

!ELSEIF  "$(CFG)" == "test_space_stress - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "../../lib/debug"
# PROP BASE Intermediate_Dir "obj/space_stress/Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "../../lib/debug"
# PROP Intermediate_Dir "obj/space_stress/Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/debug/test_space_stress.exe" /pdbtype:sept /libpath:"../../lib/debug"
# ADD LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/debug/test_space_stress.exe" /pdbtype:sept /libpath:"../../lib/debug"

!ENDIF

# Begin Target

# Name "test_space_stress - Win32 Release"
# Name "test_space_stress - Win32 Debug"
# Begin Group "ode"

# PROP Default_Filter ""
# Begin Group "test"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../ode/test/test_space_stress.cpp
# End Source File
# End Group
# End Group
# Begin Group "drawstuff"

# PROP Default_Filter ""
# Begin Group "src"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../drawstuff/src/resources.rc
# End Source File
# End Group
# End Group
# End Target
# End Project
