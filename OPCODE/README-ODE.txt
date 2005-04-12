
This is a copy of the OPCODE collision detection library by Pierre Terdiman.
See http://www.codercorner.com/Opcode.htm for more information, and read
the ReadMe.txt in this directory.

If you want to use the TriList (triangle mesh) geometry class in ODE, the
OPCODE library must be compiled. To do so, simply uncomment the
OPCODE_DIRECTORY variable in ODE's user-settings file, and ODE's default build
system will compile OPCODE for you.

If you are using the experimental autotools support to compile ODE, you just
have to specify --with-opcode when calling ./configure.

This code was originally written for and compiled on windows, but it has been
ported so that it should compile under unix/gcc too. Your mileage may vary.

Russ Smith, April 12 2005.
