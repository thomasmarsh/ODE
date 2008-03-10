project.name = "ode"


-- We no longer support VC++ 6.0; too many incompatibilities

  if (options["target"] == "vs6") then
    error("Visual Studio 6 is no longer supported; please upgrade to Visual Studio 2005 C++ Express.")
  end
  

-- Define the build configurations. You can also use the flags 
-- `--enable-shared-only` and `--enable-static-only` if you want to 
-- call these packages from within your own Premake-enabled project.

  if (not options["enable-shared-only"] and not options["enable-static-only"]) then
    project.configs = { "DebugSingleDLL", "ReleaseSingleDLL", "DebugSingleLib", "ReleaseSingleLib", "DebugDoubleDLL", "ReleaseDoubleDLL", "DebugDoubleLib", "ReleaseDoubleLib" }
  end


-- Project options

--  addoption("with-doubles",  "Use double instead of float as base numeric type")
  addoption("with-demos",    "Builds the demo applications and DrawStuff library")
  addoption("with-tests",    "Builds the unit test application")
  addoption("with-gimpact",  "Use GIMPACT for trimesh collisions (experimental)")
  addoption("no-dif",        "Exclude DIF (Dynamics Interchange Format) exports")
  addoption("no-trimesh",    "Exclude trimesh collision geometry")
  addoption("no-alloca",     "Use heap memory instead of the stack (experimental)")


-- If the `--usetargetpath` flag is specified, each set of generated files
-- be placed in a directory named for the target toolset. This flag is
-- used by the `--makeall` command (see below).

  if (options["usetargetpath"]) then
    project.path = options["target"]
  end


-- Set the output directories

  if (options["enable-shared-only"] or options["enable-static-only"]) then
    project.config["DebugSingle"].bindir   = "../lib/debug"
    project.config["DebugSingle"].libdir   = "../lib/debug"
    project.config["ReleaseSingle"].bindir = "../lib/release"
    project.config["ReleaseSingle"].libdir = "../lib/release"
    project.config["DebugDouble"].bindir   = "../lib/debug"
    project.config["DebugDouble"].libdir   = "../lib/debug"
    project.config["ReleaseDouble"].bindir = "../lib/release"
    project.config["ReleaseDouble"].libdir = "../lib/release"
  else
    project.config["DebugSingleDLL"].bindir   = "../lib/DebugSingleDLL"
    project.config["DebugSingleDLL"].libdir   = "../lib/DebugSingleDLL"
    project.config["ReleaseSingleDLL"].bindir = "../lib/ReleaseSingleDLL"
    project.config["ReleaseSingleDLL"].libdir = "../lib/ReleaseSingleDLL"
    project.config["DebugSingleLib"].bindir   = "../lib/DebugSingleLib"
    project.config["DebugSingleLib"].libdir   = "../lib/DebugSingleLib"
    project.config["ReleaseSingleLib"].bindir = "../lib/ReleaseSingleLib"
    project.config["ReleaseSingleLib"].libdir = "../lib/ReleaseSingleLib"

    project.config["DebugDoubleDLL"].bindir   = "../lib/DebugDoubleDLL"
    project.config["DebugDoubleDLL"].libdir   = "../lib/DebugDoubleDLL"
    project.config["ReleaseDoubleDLL"].bindir = "../lib/ReleaseDoubleDLL"
    project.config["ReleaseDoubleDLL"].libdir = "../lib/ReleaseDoubleDLL"
    project.config["DebugDoubleLib"].bindir   = "../lib/DebugDoubleLib"
    project.config["DebugDoubleLib"].libdir   = "../lib/DebugDoubleLib"
    project.config["ReleaseDoubleLib"].bindir = "../lib/ReleaseDoubleLib"
    project.config["ReleaseDoubleLib"].libdir = "../lib/ReleaseDoubleLib"
  end


-- Build packages

  if (options["with-demos"]) then
    dopackage("demos.lua")
    dopackage("drawstuff.lua")
  end

  if (options["with-tests"]) then
    dopackage("tests.lua")
  end

  dopackage("ode.lua")
  

-- Remove all intermediate files

  function doclean(cmd, arg)
    docommand(cmd, arg)
    if (options["target"] == "") then
      os.remove("../ode/src/config.h")
    end
    os.rmdir("custom")
    os.rmdir("../lib/debug")
    os.rmdir("../lib/release")
    os.rmdir("../lib/DebugSingleDLL")
    os.rmdir("../lib/DebugSingleLib")
    os.rmdir("../lib/ReleaseSingleDLL")
    os.rmdir("../lib/ReleaseSingleLib")
    os.rmdir("../lib/DebugDoubleDLL")
    os.rmdir("../lib/DebugDoubleLib")
    os.rmdir("../lib/ReleaseDoubleDLL")
    os.rmdir("../lib/ReleaseDoubleLib")
    os.rmdir("gnu/obj")
    os.rmdir("vs2002/obj")
    os.rmdir("vs2003/obj")
    os.rmdir("vs2005/obj")
  end


-- Generate all toolsets in one go

  function domakeall(cmd, arg)
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target vs2002")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target vs2003")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target vs2005")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target gnu")
  end
