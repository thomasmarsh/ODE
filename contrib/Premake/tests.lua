package.name = "tests"
package.path = "../../tests"
package.kind = "exe"
package.language = "c++"

-- This function applies the settings that are common across all
-- packages, such as defined symbols and include directories
  
  applyCommonSettings()


-- Build Settings

  package.includepaths =
  {
    "../include",
    "CppTestHarness"
  }


-- Libraries

  package.links =
  {
    "ode"
  }
  

-- Files

  package.files =
  {
    matchrecursive("*.h", "*.cpp")
  }
  