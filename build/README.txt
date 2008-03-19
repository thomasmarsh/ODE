Premake-based Windows Build System
Contributed by Jason Perkins (jason379@users.sourceforge.net)
-------------------------------------------------------------------


ABOUT THESE FILES

 This folder contains an automatic project generation tool called
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

 To build the demo applications and Drawstuff library, use the form:
 
   premake --with-demos --target [toolset]
   
 If you ever decide that you want to remove your custom project, you
 can just type:

   premake --with-demos --clean

 Feel free to direct any questions or comments to myself or the ODE
 mailing list.


