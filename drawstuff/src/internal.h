/*************************************************************************
 *                                                                       *
 * DrawStuff Library, Copyright (C) 2001 Russell L. Smith.               *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of the GNU Lesser General Public            *
 * License as published by the Free Software Foundation; either          *
 * version 2.1 of the License, or (at your option) any later version.    *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU      *
 * Lesser General Public License for more details.                       *
 *                                                                       *
 * You should have received a copy of the GNU Lesser General Public      *
 * License along with this library (see the file LICENSE.TXT); if not,   *
 * write to the Free Software Foundation, Inc., 59 Temple Place,         *
 * Suite 330, Boston, MA 02111-1307 USA.                                 *
 *                                                                       *
 *************************************************************************/

/* functions supplied and used by the platform specific code */

#ifndef __DS_INTERNAL_H
#define __DS_INTERNAL_H

#include "drawstuff/drawstuff.h"


// supplied by platform specific code

void dsPlatformSimLoop (int window_width, int window_height,
			dsFunctions *fn, int initial_pause);


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
