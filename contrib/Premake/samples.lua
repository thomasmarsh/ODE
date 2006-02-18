-- Helper function to build the test packages

  function makepackage(name)
    package = newpackage()
	
    -- package information
    package.name = "test_" .. name
    package.path = "../../ode/test"
    package.kind = "exe"
    package.language = "c++"
  
    -- This function applies the settings that are common across all
    -- packages, such as defined symbols and include directories
    applyCommonSettings()
    
    -- build settings
    package.includepaths = { "../../include" }
	
    -- libraries
    package.links = { "ode", "drawstuff" }
    if (windows) then
      table.insert(package.links, { "gdi32", "opengl32", "glu32" })
    else
      table.insert(package.links, { "GL", "GLU" })
    end
  
    -- files (hack for the one .c file)
    if (name == "chain1") then
      package.files = { "test_" .. name .. ".c" }
    else
      package.files = { "test_" .. name .. ".cpp" }
    end

    if (windows) then
      table.insert(package.files, "../../drawstuff/src/resources.rc")
    end
  end


-- Samples

  makepackage("boxstack")
  makepackage("buggy")
  makepackage("chain1")
  makepackage("chain2")
  makepackage("collision")
  makepackage("crash")
  makepackage("friction")
  makepackage("hinge")
  makepackage("I")
  makepackage("joints")
  makepackage("ode")
  makepackage("slider")
  makepackage("space")
  makepackage("space_stress")
  makepackage("step")

  if (not options["no-cylinder"]) then
    makepackage("cyl")
  end
    
  if (not options["no-trimesh"]) then
    makepackage("moving_trimesh")
    makepackage("trimesh")
  end

