project.name = "ode"

-- Project options

  addoption("dll",          "Build ODE as a DLL instead of a static library")
  addoption("no-cylinder",  "Disable cylinder collision geometry")
  addoption("no-double",    "Use single precision math instead of double")
  addoption("no-dif",       "Exclude DIF (Dynamics Interchange Format) exports")
  addoption("no-trimesh",   "Exclude trimesh support")
  addoption("with-all",     "Include all samples and tests")
  addoption("with-samples", "Build the sample applications")
  addoption("with-tests",   "Build the library unit tests")
  
  
-- Set the output directories. Just taking a guess here.

  project.config["Debug"].bindir = "../../lib/debug"
  project.config["Debug"].libdir = "../../lib/debug"

  project.config["Release"].bindir = "../../lib/release"
  project.config["Release"].libdir = "../../lib/release"
  
 
-- Copy my custom config.h to the include directory. I am trying
-- to come up with a generic version, but if necessary I could
-- build one on the fly. For now I just copy.

  os.copyfile("config.h", "../../include/ode/config.h")


-- Helper function to set some common build settings

  dofile("common_settings.lua")
    
    
-- Normally, each module would have a separate premake.lua file
-- in the same directory as the source code (like Makefile.am).
-- Because this is a contrib, I have to keep everything local.

  if (options["with-all"]) then
    options["with-samples"] = 1
    options["with-tests"] = 1
  end
  
  if (options["with-samples"]) then
    dofile("samples.lua")
    dopackage("drawstuff")
  end
  
  if (options["with-tests"]) then
    dopackage("tests")
  end  

  dopackage("ode")
  

