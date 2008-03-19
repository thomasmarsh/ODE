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


  -- Output is placed in a directory named for the target toolset.
  package.path = options["target"]


if (not options["enable-static-only"]) then
  table.insert(package.config["DebugDoubleDLL"].defines, "dDOUBLE")
  table.insert(package.config["ReleaseDoubleDLL"].defines, "dDOUBLE")
end
if (not options["enable-shared-only"]) then
  table.insert(package.config["DebugDoubleLib"].defines, "dDOUBLE")
  table.insert(package.config["ReleaseDoubleLib"].defines, "dDOUBLE")
end
