
/* functions supplied and used by the platform specific code */

#ifndef __DS_INTERNAL_H
#define __DS_INTERNAL_H

#include "drawstuff/drawstuff.h"


// supplied by platform specific code

void dsPlatformSimLoop (int window_width, int window_height,
			dsFunctions *fn);


// used by platform specific code

void dsStartGraphics (int width, int height, dsFunctions *fn);
void dsDrawFrame (int width, int height, dsFunctions *fn, int pause);
void dsStopGraphics();
void dsMotion (int mode, int deltax, int deltay);

int dsGetShadows();
void dsSetShadows (int a);

int dsGetTextures();
void dsSetTextures (int a);

#endif
