ODE with Erwin de Vries' triangle collider extension

Copy the source and header files from the contrib/tri-collider directory into 
the main ODE source directory.  Also put the OPCODE directory into MSVC7's 
include directory list (Tools | Options | Projects | VC++ Directories) and
tweak geom.cpp (see Erwin's notes about that).

Commented out #define GENERATEBODIES in dcTriListCollider.cpp, line 21, as it 
caused compiler errors later on.

--

I couldn't get configure.exe to work.  If you can't either, copy the config.h 
from this directory to [ode]/include/ode/config.h and try that.  It works for
me, but I make no promises.

changes were made to:
	msvcdefs.def (added a few missing functions)
	misc.cpp (added dInfinityValue)
	added Win32.cpp (not really necessary)