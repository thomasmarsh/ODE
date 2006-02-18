ODE PREMAKE-BASED BUILD SYSTEM
Contributed by Jason Perkins (starkos@gmail.com)

 This is an alternative build system that uses Premake to create
 the input files for several different compilers and IDEs. For
 more information about Premake, see the website at:
 
   http://premake.sourceforge.net/
   
 In a nutshell, Premake is a small command-line utility that 
 reads a generic text description of a software project and
 uses it to create GNU makefiles, Visual Studio solutions,
 and more. Premake also embeds Lua, a fully-functional 
 scripting language, so it can handle all kinds of setup
 tasks.

 Note that this build approach does not use the configurator 
 tool. I am currently trying to get by with a combination of 
 Premake features and a stock config.h file. This works on the 
 platforms that I have available for testing, but will probably 
 need additional effort to work everywhere.


HOW TO USE IT

 To get started, download Premake v3.1 or later from the
 project website shown above and place it on the system path
 or anywhere convenient. Then open a command prompt and cd
 to this directory.
 
 The basic form of a Premake command is:
 
   premake [--option ...] --target toolset
   
 You can see a full list of options and supported toolsets by
 this command while in the ODE project directory:
 
   premake --help
   
 By default, these scripts will get you a static library using
 double-precision math and trimesh support. It will not build
 DrawStuff, the unit tests, or any of the sample applications. 
 If this default configuration is okay with you, you can run 
 a command like:
 
   premake --target vs2005
 
 This will generate a solution and project files for Visual
 Studio 2005. Again, see `premake --help` for a full list of
 supported toolsets. Once the files are generated, you can
 load up Visual Studio (or whichever tool you chose) and build 
 the library. The compiled binaries will be placed into the
 ode/lib/debug and ode/lib/release directories.
  
 A full list of options is below, but here are a few samples
 to get you started. If you wanted to exclude trimeshes and 
 build with Visual C++ 6:
 
   premake --no-trimesh --target vs6
 
 If you want to include the sample applications and unit tests:
 
   premake --with-samples --with-tests --target gnu

 If you ever want to get rid of everything and start over:
 
   premake --with-all --clean

 
AVAILABLE OPTIONS
     
 Here's a list of options currently supported by the scripts:
 
 --dll
   Build ODE as a DLL instead of a static library.
   
 --no-dif
   Don't include support for DIF (Dynamics Interchange Format)
   exports. This will make the library a little bit smaller,
   but DIF support is required by the sample applications.
   
 --no-double
   Use single precision math instead of double.
   
 --no-trimesh
   Exclude all support for trimeshes, including the OPCODE
   library and any trimesh sample applications.
   
 --with-samples
   Build the sample applications and DrawStuff library. The
   samples have some additional system requirements, such as
   OpenGL and X11, that you might have to install.
   
 --with-tests
   Include the new unit test framework in the build. This will
   create an executable called `tests` which will run all of
   the contributed tests and display the results.
   
 

LEFT TO DO
----------------------------------------------
* Test on Linux
* Try turning off no-64bit-checks and fixing warnings
* Add flag to build documentation?
* MacOS X support and testing
* 64-bit support and testing

