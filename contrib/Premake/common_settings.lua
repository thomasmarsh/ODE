-- This function applies the settings that are common across all packages

function applyCommonSettings()

  -- Turn off 64-bit compatibility checks until I've had a change to
  -- go through the code and fix some of the warnings (there are a lot)
  
    package.buildflags = 
    { 
      "no-64bit-checks" 
    }
  
  -- Defined symbols
     
    package.defines =
    {
      "_CRT_SECURE_NO_DEPRECATE"  -- disable VS2005 CRT security warnings
    }
    
    if (options["no-double"]) then
      table.insert(package.defines, "dSINGLE")
    else
      table.insert(package.defines, "dDOUBLE")
    end

    if (not options["no-cylinder"]) then
      table.insert(package.defines, "dCYLINDER_ENABLED")
    end
  
    if (not options["no-trimesh"]) then
      table.insert(package.defines, "dTRIMESH_ENABLED")
    end

    if (windows) then
      table.insert(package.defines, "WIN32")
    end

end

