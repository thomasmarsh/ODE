-- Premake build scripts for ODE unit tests

package.name = "tests"
package.kind = "exe"
package.language = "c++"
package.path = "custom"
package.objdir = "obj/tests"

package.includepaths = 
{
  "../../include",
  "../../tests/UnitTest++/src"
}

package.defines = 
{
  "_CRT_SECURE_NO_DEPRECATE"
}

package.files =
{
  matchfiles("../../tests/*.cpp"),
  matchfiles("../../tests/UnitTest++/src/*")
}

if (windows) then
  table.insert(package.files, matchfiles("../../tests/UnitTest++/src/Win32/*"))
else
  table.insert(package.files, matchfiles("../../tests/UnitTest++/src/Posix/*"))
end

package.links =
{
  "ode"
}


-- Separate distribution files into toolset subdirectories

  if (options["usetargetpath"]) then
    package.path = options["target"]
  end

