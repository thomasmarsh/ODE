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
    "main.cpp",
    
    -- Collision primitives
    "colliders/box_sphere.cpp",
    
    -- CppTestHarness testing framework (http://cnicholson.net/content.php?id=52)
    matchfiles("CppTestHarness/*.h", "CppTestHarness/*.cpp")
  }
  