/*

Draw Stuff
----------

this is a library for rendering simple 3D objects in a virtual environment.

NOTES
-----

in the virtual world, the z axis is "up" and z=0 is the floor.

the user is able to click+drag in the main window to move the camera:
  * left button - pan and tilt.
  * right button - forward and sideways.
  * left + right button (or middle button) - sideways and up.

*/


#ifndef __DRAWSTUFF_H__
#define __DRAWSTUFF_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "drawstuff/version.h"


/* texture numbers */
#define DS_NONE   0	/* uses the current color instead of a texture */
#define DS_WOOD   1


struct dsFunctions {
  int version;			/* put DS_VERSION here */
  /* version 1 data */
  void (*start)();		/* called before sim loop starts */
  void (*step) (int pause);	/* called before every frame */
  void (*command) (int cmd);	/* called if a command key is pressed */
  void (*stop)();		/* called after sim loop exits */
  /* version 2 data */
  char *path_to_textures;	/* if nonzero, path to texture files */
};


/* the main() function should fill in the dsFunctions structure then
 * call this.
 */
void dsSimulationLoop (int argc, char **argv,
		       int window_width, int window_height,
		       dsFunctions *fn);

/* these functions display an error message then exit. they take arguments
 * in the same way as printf(), except you do not have to add a terminating
 * '\n'. Debug() tries to dump core or start the debugger.
 */
void dsError (char *msg, ...);
void dsDebug (char *msg, ...);

/* dsPrint() prints out a message. it takes arguments in the same way as
 * printf() (i.e. you must add a '\n' at the end of every line).
 */
void dsPrint (char *msg, ...);

/* set and get the camera position. xyz is the cameria position (x,y,z).
 * hpr contains heading, pitch and roll numbers in degrees. heading=0
 * points along the x axis, pitch=0 is looking towards the horizon, and
 * roll 0 is "unrotated".
 */
void dsSetViewpoint (float xyz[3], float hpr[3]);
void dsGetViewpoint (float xyz[3], float hpr[3]);

/* change the way objects are drawn. these changes will apply to all further
 * dsDrawXXX() functions. the texture number must be a DS_xxx texture
 * constant. the red, green, and blue number are between 0 and 1.
 * the current texture is colored according to the current color.
 * at the start of each frame, the texture is reset to none and the color is
 * reset to white.
 */
void dsSetTexture (int texture_number);
void dsSetColor (float red, float green, float blue);

/* draw objects.
 *   - pos[] is the x,y,z of the center of the object.
 *   - R[] is a 3x3 rotation matrix for the object, stored by row like this:
 *        [ R11 R12 R13 0 ]
 *        [ R21 R22 R23 0 ]
 *        [ R31 R32 R33 0 ]
 *   - sides[] is an array of x,y,z side lengths.
 *   - all cylinders are aligned along the z axis.
 */
void dsDrawBox (float pos[3], float R[12], float sides[3]);
void dsDrawSphere (float pos[3], float R[12], float radius);
void dsDrawCylinder (float pos[3], float R[12], float length, float radius);
void dsDrawCappedCylinder (float pos[3], float R[12],
			   float length, float radius);

/* these drawing functions are identical to the ones above, except they take
 * double arrays for `pos' and `R'.
 */
void dsDrawBoxD (double pos[3], double R[12], float sides[3]);
void dsDrawSphereD (double pos[3], double R[12], float radius);
void dsDrawCylinderD (double pos[3], double R[12], float length, float radius);
void dsDrawCappedCylinderD (double pos[3], double R[12],
			    float length, float radius);


/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif
