package.name = "ode"
package.path = "../../ode"
package.language = "c++"

-- If the `--dll` option was specified, build a DLL. Otherwise
-- build a static library

  if (options["dll"]) then
    package.kind = "dll"
  else
    package.kind = "lib"
  end
  
  
-- This function applies the settings that are common across all
-- packages, such as defined symbols and include directories
  
  applyCommonSettings()
  
    
-- Build Settings

  package.includepaths =
  {
    "../include",
    "../OPCODE"
  }
 
  
-- Files

  package.files = 
  {
    matchfiles("../include/ode/*.h"),
    matchfiles ("src/*.h", "src/*.c", "src/*.cpp")
  }
  
  package.excludes =
  {
    "src/scrapbook.cpp",
    "src/stack.cpp"
  }

  trimesh_files =
  {
    "src/collision_trimesh_internal.h",
    "src/collision_trimesh.cpp",
    "src/collision_trimesh_box.cpp",
    "src/collision_trimesh_ccylinder.cpp",
    "src/collision_cylinder_trimesh.cpp",
    "src/collision_trimesh_distance.cpp",
    "src/collision_trimesh_ray.cpp",
    "src/collision_trimesh_sphere.cpp",
    "src/collision_trimesh_trimesh.cpp"
  }
  
  cylinder_files =
  {
    "src/collision_cylinder_box.cpp",
    "src/collision_cylinder_sphere.cpp",
    "src/collision_cylinder_plane.cpp",
    "src/collision_cylinder_trimesh.cpp",
  }
   
  opcode_files =
  {
    matchrecursive("../OPCODE/*.h", "../OPCODE/*.cpp")
  }
  
  dif_files = 
  {
    "src/export-dif.cpp"
  }


-- Exclude files from the build based on user options

  if (options["no-dif"]) then
    table.insert(package.excludes, dif_files)
  end

  if (options["no-cylinder"]) then
    table.insert(package.excludes, cylinder_files)
  end
  
  if (options["no-trimesh"]) then
    table.insert(package.excludes, trimesh_files)
  else
    table.insert(package.files, opcode_files)
  end
   
    
-- Dynamically build an exports file for Win32 DLLs. I create a list of
-- all of the *_exports.def files that I need and then append them all
-- together into a master exports.def file

  def_files = { }
    
  if (not options["no-cylinder"]) then
    table.insert(def_files, "cylinder.def")
  end

  if (not options["no-dif"]) then
    table.insert(def_files, "dif.def")
  end
    
  if (not options["no-trimesh"]) then
    table.insert(def_files, "trimesh.def")
  end
  

  if (options["clean"]) then
    os.remove(package.path .. "/exports.def")
  end      

  if (windows and options["dll"]) then
    exports_file = package.path .. "/exports.def"
    
    os.copyfile("exports/core.def", exports_file)
    for i = 1, table.getn(def_files) do
      os.appendfile("exports/" .. def_files[i], exports_file)
    end
    
    table.insert(package.files, "exports.def")
  end
  
