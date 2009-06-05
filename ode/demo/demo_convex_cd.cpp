/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <stdio.h>
#include <math.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

//<---- Convex Object
dReal planes[]= // planes for a cube
  {
    1.0f ,0.0f ,0.0f ,0.25f,
    0.0f ,1.0f ,0.0f ,0.25f,
    0.0f ,0.0f ,1.0f ,0.25f,
    -1.0f,0.0f ,0.0f ,0.25f,
    0.0f ,-1.0f,0.0f ,0.25f,
    0.0f ,0.0f ,-1.0f,0.25f
    /*
    1.0f ,0.0f ,0.0f ,2.0f,
    0.0f ,1.0f ,0.0f ,1.0f,
    0.0f ,0.0f ,1.0f ,1.0f,
    0.0f ,0.0f ,-1.0f,1.0f,
    0.0f ,-1.0f,0.0f ,1.0f,
    -1.0f,0.0f ,0.0f ,0.0f
    */
  };
const unsigned int planecount=6;

dReal points[]= // points for a cube
  {
    0.25f,0.25f,0.25f,  //  point 0
    -0.25f,0.25f,0.25f, //  point 1

    0.25f,-0.25f,0.25f, //  point 2
    -0.25f,-0.25f,0.25f,//  point 3

    0.25f,0.25f,-0.25f, //  point 4
    -0.25f,0.25f,-0.25f,//  point 5

    0.25f,-0.25f,-0.25f,//  point 6
    -0.25f,-0.25f,-0.25f,// point 7
  };
const unsigned int pointcount=8;
unsigned int polygons[] = //Polygons for a cube (6 squares)
  {
    4,0,2,6,4, // positive X Side 0
    4,1,0,4,5, // positive Y Side 1
    4,0,1,3,2, // positive Z Side 2
    4,3,1,5,7, // negative X Side 3
    4,2,3,7,6, // negative Y Side 4
    4,5,4,6,7, // negative Z Side 5
  };
//----> Convex Object

#ifdef dDOUBLE
#define dsDrawConvex dsDrawConvexD
#define dsDrawBox dsDrawBoxD
#endif

dGeomID* geoms;
dGeomID boxes[2];
dGeomID convex[2];
dSpaceID space;
dWorldID world;
dJointGroupID contactgroup;
/*
glRotate Matrix:
( xx(1-c)+c	xy(1-c)-zs  xz(1-c)+ys	 0  )
|					    |
| yx(1-c)+zs	yy(1-c)+c   yz(1-c)-xs	 0  |
| xz(1-c)-ys	yz(1-c)+xs  zz(1-c)+c	 0  |
|					    |
(	 0	     0		 0	 1  )
Where	c = cos(angle),	s = sine(angle), and ||( x,y,z )|| = 1
	  (if not, the GL will normalize this vector).
*/

dVector3 geom1pos={0.0,0.250,0.50};
dQuaternion geom1quat={1,0,0,0};
dQuaternion geom0quat={0.7071,0,0.7071,0};

bool DumpInfo=true;
int drawmode = DS_WIREFRAME;

#if 0
const dReal fixed_pos_0[]={0.703704,-0.748281,0.249495};
const dReal fixed_rot_0[]={0.996994,-0.001009,-0.077468,0.000000,
                          -0.077468,-0.000117,-0.996995,0.000000,
                           0.000996, 1.000000,-0.000195,0.000000};

const dReal fixed_pos_1[]={0.894169,-0.372081,0.249432};
const dReal fixed_rot_1[]={-0.999461, 0.032777,0.001829,0.000000,
                           -0.032777,-0.999463,0.000033,0.000000,
                            0.001829,-0.000027,0.999998,0.000000};

#else
// for EDGE-EDGE test
const dReal fixed_pos_0[]={0.0,0.0,0.25};
const dReal fixed_rot_0[]={ 1,0,0,0,0,1,0,0,0,0,1,0 };
const dReal fixed_pos_1[]={0.000000,0.450000,0.600000};
const dReal fixed_rot_1[]={0.708311,-0.705472,-0.000000,0.000000,
                           0.516939,0.519297,-0.679785,0.000000,
                           0.480067,0.481293,0.733034,0.000000};
#endif
void start()
{
  // adjust the starting viewpoint a bit
  float xyz[3],hpr[3];
  dsGetViewpoint (xyz,hpr);
  hpr[0] += 7;
  dsSetViewpoint (xyz,hpr);
  convex[0]=dCreateConvex (space,
			  planes,
			  planecount,
			  points,
			  pointcount,
			  polygons);
  convex[1]=dCreateConvex (space,
			  planes,
			  planecount,
			  points,
			  pointcount,
			  polygons);
  boxes[0]=dCreateBox(space,0.5,0.5,0.5);
  boxes[1]=dCreateBox(space,0.5,0.5,0.5);
  geoms=convex;

#if 0
  dMatrix3 m1 = { 1,0,0,0,0,1,0,0,0,0,1,0 };
  dMatrix3 m2 = { 1,0,0,0,0,1,0,0,0,0,1,0 };
  dGeomSetPosition (convex[0],
		    0.0,
		    0.0,
		    0.25);
  dGeomSetPosition (convex[1],
		    geom1pos[0],
		    geom1pos[1],
		    geom1pos[2]);
  dQtoR (geom0quat, m1);
  dGeomSetRotation (convex[0],m1);
  dQtoR (geom1quat, m2);
  dGeomSetRotation (convex[1],m2);

  dGeomSetPosition (boxes[0],
		    0.0,
		    0.0,
		    0.25);
  dGeomSetPosition (boxes[1],
		    geom1pos[0],
		    geom1pos[1],
		    geom1pos[2]);
  dQtoR (geom0quat, m1);
  dGeomSetRotation (boxes[0],m1);
  dQtoR (geom1quat, m2);
  dGeomSetRotation (boxes[1],m2);
#else
  dGeomSetPosition (convex[0],
		    fixed_pos_0[0],
		    fixed_pos_0[1],
		    fixed_pos_0[2]);
  dGeomSetPosition (convex[1],
		    fixed_pos_1[0],
		    fixed_pos_1[1],
		    fixed_pos_1[2]);
  dGeomSetRotation (convex[0],fixed_rot_0);
  dGeomSetRotation (convex[1],fixed_rot_1);
  dGeomSetPosition (boxes[0],
		    fixed_pos_0[0],
		    fixed_pos_0[1],
		    fixed_pos_0[2]);
  dGeomSetPosition (boxes[1],
		    fixed_pos_1[0],
		    fixed_pos_1[1],
		    fixed_pos_1[2]);
  dGeomSetRotation (boxes[0],fixed_rot_0);
  dGeomSetRotation (boxes[1],fixed_rot_1);
#endif

}

int dCollideConvexConvex (dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip);
int dCollideBoxBox (dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip);

void simLoop (int pause)
{
  int contactcount;
  const dReal ss[3] = {0.02,0.02,0.02};
  dContactGeom contacts[8];
if(geoms==convex)
  contactcount = dCollideConvexConvex(geoms[0],geoms[1],8,contacts,sizeof(dContactGeom));
else
  contactcount = dCollideBoxBox(geoms[0],geoms[1],8,contacts,sizeof(dContactGeom));

  //fprintf(stdout,"Contact Count %d\n",contactcount);
  const dReal* pos;
  const dReal* R;
  dsSetTexture (DS_WOOD);
  pos = dGeomGetPosition (geoms[0]);
  R = dGeomGetRotation (geoms[0]);
  dsSetColor (0.6f,0.6f,1);
  dsSetDrawMode(drawmode);
  dsDrawConvex(pos,R,planes,
	       planecount,
	       points,
	       pointcount,
	       polygons);
  dsSetDrawMode(DS_POLYFILL);
  pos = dGeomGetPosition (geoms[1]);
  R = dGeomGetRotation (geoms[1]);
  dsSetColor (0.4f,1,1);
  dsSetDrawMode(drawmode);
  dsDrawConvex(pos,R,planes,
	       planecount,
	       points,
	       pointcount,
	       polygons);
    dsSetDrawMode(DS_POLYFILL);
  /*if (show_contacts) */
  dMatrix3 RI;
  dRSetIdentity (RI);
  dsSetColor (1.0f,0,0);
  for(int i=0;i<contactcount;++i)
    {
      if(DumpInfo)
	{
	  //DumpInfo=false;
	  fprintf(stdout,"Contact %d Normal %f,%f,%f Depth %f Pos %f %f %f ",
		  i,
		  contacts[i].normal[0],
		  contacts[i].normal[1],
		  contacts[i].normal[2],
		  contacts[i].depth,
		  contacts[i].pos[0],
		  contacts[i].pos[1],
		  contacts[i].pos[2]);
	  if(contacts[i].g1==geoms[0])
	    {
	      fprintf(stdout,"Geoms 1 2\n");
	    }
	  else
	    {
	      fprintf(stdout,"Geoms 2 1\n");
	    }
	}
      dsDrawBox (contacts[i].pos,RI,ss);
    }
  if(DumpInfo)
    DumpInfo=false;

}


void command (int cmd)
{
  // note: 0.0174532925 radians = 1 degree
  dQuaternion q;
  //dMatrix3 m;
  bool changed = false;
  switch(cmd)
    {
    case 'w':
      geom1pos[0]+=0.05;
      changed = true;
      break;
    case 'a':
      geom1pos[1]-=0.05;
      changed = true;
      break;
    case 's':
      geom1pos[0]-=0.05;
      changed = true;
      break;
    case 'd':
      geom1pos[1]+=0.05;
      changed = true;
      break;
    case 'e':
      geom1pos[2]-=0.05;
      changed = true;
      break;
    case 'q':
      geom1pos[2]+=0.05;
      changed = true;
      break;
    case 'i':
      dQFromAxisAndAngle (q, 0, 0, 1,0.0174532925);
      dQMultiply0(geom1quat,geom1quat,q);
      changed = true;
      break;
    case 'j':
      dQFromAxisAndAngle (q, 1, 0, 0,0.0174532925);
      dQMultiply0(geom1quat,geom1quat,q);
      changed = true;
      break;
    case 'k':
      dQFromAxisAndAngle (q, 0, 0, 1,-0.0174532925);
      dQMultiply0(geom1quat,geom1quat,q);
      changed = true;
      break;
    case 'l':
      dQFromAxisAndAngle (q, 1, 0, 0,-0.0174532925);
      dQMultiply0(geom1quat,geom1quat,q);
      changed = true;
      break;
    case 'm':
		(drawmode!=DS_POLYFILL)? drawmode=DS_POLYFILL:drawmode=DS_WIREFRAME;
      break;
    case 'n':
		(geoms!=convex)? geoms=convex:geoms=boxes;
		if(geoms==convex)
		{
		    printf("CONVEX------------------------------------------------------>\n");
		}
		else
		{
		    printf("BOX--------------------------------------------------------->\n");
		}
      break;
    default:
      dsPrint ("received command %d (`%c')\n",cmd,cmd);
    }
#if 0
            dGeomSetPosition (geoms[1],
		    geom1pos[0],
		    geom1pos[1],
		    geom1pos[2]);
            dQtoR (geom1quat, m);
            dGeomSetRotation(geoms[1],m);
    if(changed)
    {

            printf("POS: %f,%f,%f\n",geom1pos[0],geom1pos[1],geom1pos[2]);
            printf("ROT:\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
            m[0],m[1],m[2],m[3],
            m[4],m[5],m[6],m[7],
            m[8],m[9],m[10],m[11]);
    }
#endif
  DumpInfo=true;
}


int main (int argc, char **argv)
{
  // setup pointers to callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;	// uses default
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);

  // run simulation
  dsSimulationLoop (argc,argv,400,400,&fn);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);

  return 0;
}
