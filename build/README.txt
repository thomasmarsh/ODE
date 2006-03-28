Premake-based Windows Build System
Contributed by Jason Perkins (jason379@users.sourceforge.net)
-------------------------------------------------------------------

 SVN USERS: Before using these project files you must copy
 config-default.h to ode/include/ode/config.h (or run Premake to
 generate a new set of project files, see below). 


 ABOUT THESE FILES

 This is my first take on a replacement set of Visual Studio project
 files. They work fairly well, but have not been thorougly tested 
 yet. Once I'm convinced the system is reasonably bulletproof I will 
 write up additional documentation, including how to generate custom 
 project files, and merge everything into the project trunk.

 These project files are automatically generated using a tool called
 Premake, available from http://premake.sourceforge.net/. The scripts
 used to build them have the ".lua" file extension. To regenerate the
 stock project files for inclusion in a new release, type:

   premake --makeall


 CREATING CUSTOM PROJECT FILES

 To create a set of custom project files, first type `premake --help`
 to see the options that you have available. Then generate the new
 project files using the form:

   premake [options] --target [toolset]

 For instance:

   premake --no-trimesh --target vs2005

 If you ever decide that you want to remove your custom project, you
 can just type:

   premake --clean

 Feel free to direct any questions or comments to myself or the ODE
 mailing list.


