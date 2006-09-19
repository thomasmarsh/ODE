project.name = "Ode.NET"

-- Project options

  addoption("with-doubles",  "Use double instead of float as base numeric type")
  addoption("with-tests",    "Builds the test applications and DrawStuff library")
  addoption("no-trimesh",    "Exclude trimesh collision geometry")


-- Build settings

	project.config["Debug"].bindir = "bin/Debug"
	project.config["Release"].bindir = "bin/Release"
	

-- Test Application

	if (options["with-tests"]) then
		package = newpackage()
		package.name = "TestBoxStack"
		package.kind = "exe"
		package.language = "c#"
		package.files = { "TestBoxStack.cs" }
		package.links = { "System", "Drawstuff.NET", "Ode.NET" }

		if (options["with-doubles"]) then
			package.defines = { "dDOUBLE" }
		else
			package.defines = { "dSINGLE" }
		end
	end
	
	
-- Drawstuff

	if (options["with-tests"]) then
		package = newpackage()
		package.name = "Drawstuff.NET"
		package.kind = "dll"
		package.language = "c#"
		package.files = { "Drawstuff.cs" }
		package.links = { "System", "Ode.NET" }

		if (options["with-doubles"]) then
			package.defines = { "dDOUBLE" }
		else
			package.defines = { "dSINGLE" }
		end
	end
	
	
-- ODE

	package = newpackage()
	package.name = "Ode.NET"
	package.kind = "dll"
	package.language = "c#"
	package.files = { "Ode.cs", "AssemblyInfo.cs" }
	package.links = { "System" }
	
	if (options["with-doubles"]) then
		package.defines = { "dDOUBLE" }
	else
		package.defines = { "dSINGLE" }
	end
	