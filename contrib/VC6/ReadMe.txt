
MS VisualC++ 6 workspace for ODE v0.8.

This directory contains these VC6 workspaces:

1) "ode.dsw"                  - for building ODE as a static library
2) "odeDLL.dsw"               - for building ODE as a DLL
3) "Samples/MakeAllTests.dsw" - for building the ODE samples
                                (also builds the ODE lib).

Before you can use these, you must:

 a) add the following directories to your VC include dirs:

	<ode dir>\Include
	<ode dir>\OPCODE  (needed only for Trimesh support).

 b) add the following directory to your VC library dirs:

	<ode dir>\lib

 c) configure the ODE build for either single or double floating point, and
with/without trimesh support.  To do this, simply double-click one of the
included batch files (you can do this at any time) and then perform a build.

Notes:

 All libs will be built in the <ode_dir>/lib directory.
 All samples will be copied to the <ode_dir>/test directory.
 Opcode is only required for Trimesh support - it can be removed from the ODE
  workspaces otherwise.

Thanks to Frank Compagner for his original VC6 workspace and config presets.

Enjoy.
--
gl


-----------------------------------------------------------------------------
0.8 Changes:

MS VC6 Workspace:
> created new 'VC6' directory in <ode>/ode/src
> created 'ode'    workspace (builds libs in <ode>/lib)
> created 'odeDLL' workspace (builds DLLs in <ode>/lib).
> created build config presets (based on Frank Compagner's files) for
   single/double with/without trimesh support.
> created batch files to auto-configure the VC6 build (they copy the correct
   config file to <ode>/include/config.h and clear the Debug and Release dirs
   to force a rebuild).

> created 'Samples' subdir that contains all the samples and drawstuff.
> added trimesh samples.
> note: changed drawstuff to build 'drawstuffd.lib' under debug.

Opcode:
Changed Opcode workspace to create libs in <ode>/lib

ODE src:
> collision_std.cpp: wrapped some double values in REAL() to avoid
   conversions/warnings in single mode
> collision_trimesh_sphere.cpp: "
