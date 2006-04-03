-- Here are the lists of tests to build. Add/remove new 
-- tests here and everything else should just work

  local tests =
  {
    "basket",
    "boxstack",
    "buggy",
    "chain1",
    "chain2",
    "collision",
    "crash",
    "friction",
    "hinge",
    "I",
    "joints",
    "ode",
    "slider",
    "space",
    "space_stress",
    "step"
  }

  if (not options["no-cylinder"]) then
    table.insert(tests, "cyl")
  end
    
  if (not options["no-trimesh"]) then
    table.insert(tests, "moving_trimesh")
    table.insert(tests, "trimesh")
  end


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

  function maketest(index, name)
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
      table.insert(package.links, { "user32", "winmm", "gdi32", "opengl32", "glu32" })
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

  table.foreach(tests, maketest)


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
  