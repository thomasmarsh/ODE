-------------------------------------------
 MS VisualC++ 6 workspace for ODE (v0.9).
-------------------------------------------

This directory contains these VC6 workspaces:

1) "ode.dsw"                  - for building ODE as a static library
2) "odeDLL.dsw"               - for building ODE as a DLL
3) "Samples/MakeAllTests.dsw" - for building the ODE samples
                                (also builds the ODE lib).

Before you can use these, you must:

 a) configure the ODE build for either single or double floating point, and
with/without Trimesh support.  Simply double-click one of the included batch
files (you can do this at any time) and then perform a build.

 b) add the ODE and Opcode (if using trimesh) include and library paths to your
    app's project settings - there are two ways to do this:

        1) If you intend to use different versions of ODE with different 
           applications, you can set them up for each project independently:

           "Project Settings->C/C++->Preprocessor->Additional include directories"
              add:  <ode dir>\Include;<ode dir>\OPCODE

           "Project Settings->C/C++->Link->Input->Additional library path" 
              add:  <ode dir>\lib

        2) Otherwise you can just globally set them via:

           "Tools->Options->Directories"
             "Include files", add:  <ode dir>\Include
                                    <ode dir>\Opcode
             "Library files", add:  <ode dir>\lib

Notes:

 All libs will be built in the <ode_dir>/lib directory.
 All samples will be copied to the <ode_dir>/test directory.
 Opcode is only required for Trimesh support - it can be removed from the
  workspaces otherwise.

Important:
 The workspaces are set to use the '(Debug) Multithreaded DLL' C runtime.  If 
  your app is compiled with a different type, you must change them so they all
  use the same (which type is up to you) - otherwise you will get all kinds of
  linker errors.  This is a standard VC issue, and not related to ODE or these
  workspaces.

Thanks to Frank Compagner for his original VC6 workspace and config presets.

Enjoy.
--
gl