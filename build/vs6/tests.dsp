# Microsoft Developer Studio Project File - Name="tests" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=tests - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "tests.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "tests.mak" CFG="tests - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "tests - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "tests - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "tests - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/release"
# PROP BASE Intermediate_Dir "obj/tests/Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/release"
# PROP Intermediate_Dir "obj/tests/Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GR /GX /O2 /I "../../include" /I "../../tests/CppTestHarness" /D "_CRT_SECURE_NO_DEPRECATE" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GR /GX /O2 /I "../../include" /I "../../tests/CppTestHarness" /D "_CRT_SECURE_NO_DEPRECATE" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /entry:"mainCRTStartup" /subsystem:console /machine:I386 /out:"../../lib/release/tests.exe" /libpath:".."
# ADD LINK32 /nologo /entry:"mainCRTStartup" /subsystem:console /machine:I386 /out:"../../lib/release/tests.exe" /libpath:".."

!ELSEIF  "$(CFG)" == "tests - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "../../lib/debug"
# PROP BASE Intermediate_Dir "obj/tests/Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "../../lib/debug"
# PROP Intermediate_Dir "obj/tests/Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /I "../../tests/CppTestHarness" /D "_CRT_SECURE_NO_DEPRECATE" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /I "../../tests/CppTestHarness" /D "_CRT_SECURE_NO_DEPRECATE" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/debug/tests.exe" /pdbtype:sept /libpath:"../../lib/debug"
# ADD LINK32 /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/debug/tests.exe" /pdbtype:sept /libpath:"../../lib/debug"

!ENDIF

# Begin Target

# Name "tests - Win32 Release"
# Name "tests - Win32 Debug"
# Begin Group "tests"

# PROP Default_Filter ""
# Begin Group "CppTestHarness"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../tests/CppTestHarness/Checks.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestLauncher.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestResults.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestReporter.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/CppTestHarness.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TypedTestLauncher.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestRunner.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/Test.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/CheckMacros.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestMacros.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/PrintfTestReporter.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/SignalTranslator.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/HTMLTestReporter.h
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestLauncher.cpp
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestResults.cpp
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestReporter.cpp
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/TestRunner.cpp
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/Test.cpp
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/PrintfTestReporter.cpp
# End Source File
# Begin Source File

SOURCE=../../tests/CppTestHarness/HTMLTestReporter.cpp
# End Source File
# End Group
# Begin Group "colliders"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../tests/colliders/box_sphere.cpp
# End Source File
# End Group
# Begin Source File

SOURCE=../../tests/main.cpp
# End Source File
# End Group
# End Target
# End Project
