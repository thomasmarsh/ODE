-- Separate distribution files into toolset subdirectories

  if (options["usetargetpath"]) then
    packagepath = options["target"]
  else
    packagepath = "custom"
  end


-- DrawStuff library

  package.name = "drawstuff"
  package.kind = "lib"
  package.language = "c++"
  package.path = packagepath

  package.includepaths =
  {
    "../../include"
  }
  
  package.defines = { "_CRT_SECURE_NO_DEPRECATE" }

  package.files = 
  {
    matchfiles("../../include/drawstuff/*.h"),
    "../../drawstuff/src/internal.h",
    "../../drawstuff/src/drawstuff.cpp"
  }

  if (windows) then
    table.insert(package.defines, "WIN32")
    table.insert(package.files, "../../drawstuff/src/resource.h")
    table.insert(package.files, "../../drawstuff/src/resources.rc")
    table.insert(package.files, "../../drawstuff/src/windows.cpp")
  else
    table.insert(package.files, "../../drawstuff/src/x11.cpp")
  end
  

-- Factory function for test packages

  function maketest(name)
    package = newpackage()
    package.name = "test_" .. name
    package.kind = "exe"
    package.language = "c++"
    package.path = packagepath
    package.objdir = "obj/"..name
  
    package.includepaths = { "../../include" }
    package.defines = { "_CRT_SECURE_NO_DEPRECATE" }
	
    package.links = { "ode", "drawstuff" }
    if (windows) then
      table.insert(package.links, { "user32", "gdi32", "opengl32", "glu32" })
    else
      table.insert(package.links, { "GL", "GLU" })
    end
  
    if (name == "chain1") then
      package.files = { "../../ode/test/test_" .. name .. ".c" }
    else
      package.files = { "../../ode/test/test_" .. name .. ".cpp" }
    end

    if (windows) then
      table.insert(package.defines, "WIN32")
      table.insert(package.files, "../../drawstuff/src/resources.rc")
    end
  end

  maketest("boxstack")
  maketest("buggy")
  maketest("chain1")
  maketest("chain2")
  maketest("collision")
  maketest("crash")
  maketest("friction")
  maketest("hinge")
  maketest("I")
  maketest("joints")
  maketest("ode")
  maketest("slider")
  maketest("space")
  maketest("space_stress")
  maketest("step")

  if (not options["no-cylinder"]) then
    maketest("cyl")
  end
    
  if (not options["no-trimesh"]) then
    maketest("moving_trimesh")
    maketest("trimesh")
  end


-- Unit tests

  package = newpackage()
  package.name = "tests"
  package.kind = "exe"
  package.language = "c++"
  package.path = packagepath
  package.objdir = "obj/tests"
  
  package.includepaths =
  {
    "../../include",
    "../../tests/CppTestHarness"
  }

  package.defines = 
  { 
    "_CRT_SECURE_NO_DEPRECATE" 
  }
  
  package.links =
  {
    "ode"
  }
  
  package.files =
  {
    matchrecursive("../../tests/*.h", "../../tests/*.cpp")
  }
  