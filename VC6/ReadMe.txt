
-------------------------------------------
 MS VisualC++ 6 workspace for ODE (v0.82).
-------------------------------------------

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
with/without Trimesh support.  To do this, simply double-click one of the
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

