package.name = "drawstuff"
package.path = "../../drawstuff"
package.kind = "lib"
package.language = "c++"

-- This function applies the settings that are common across all
-- packages, such as defined symbols and include directories
  
  applyCommonSettings()


-- Build Settings

  package.includepaths =
  {
    "../include"     
  }
  
  
-- Files

  package.files = 
  {
    "../include/drawstuff/drawstuff.h",
    "../include/drawstuff/version.h",
    "src/drawstuff.cpp",
    "src/internal.h"
  }
  
  if (windows) then
    table.insert(package.files, "src/resource.h")
    table.insert(package.files, "src/resources.rc")
    table.insert(package.files, "src/windows.cpp")
  else
    table.insert(package.files, "src/x11.cpp")
  end
  
    