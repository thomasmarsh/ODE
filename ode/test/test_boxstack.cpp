/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001 Russell L. Smith.            *
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

#include <stdio.h>
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"


// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif


// some constants

#define NUM 10			// max number of objects
#define MASS (1.0)		// mass of a box / sphere


// dynamics and collision objects

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static dBodyID body[NUM];
static dJointGroupID contactgroup;
static dGeomID geom[NUM];
static int objtype[NUM];	// 0=box, 1=sphere
static dReal sides[NUM][3];


// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  if (o1->body && o2->body && dAreConnected (o1->body,o2->body)) return;

  dContact contact[3];			// up to 3 contacts per box
  for (i=0; i<3; i++) {
    contact[i].surface.mode = 0; //dContactMu2;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
  }
  if (int numc = dCollide (o1,o2,3,&contact[0].geom,sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};

    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,o1->body,o2->body);
      dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press b or s to drop another box or sphere.\n");
}


// called when a key pressed

static void command (int cmd)
{
  if (cmd == 'B') cmd = 'b';
  if (cmd == 'S') cmd = 's';
  if (cmd == 'b' || cmd == 's') {
    int i;
    if (num < NUM) {
      i = num;
      num++;
    }
    else {
      i = nextobj;
      nextobj++;
      if (nextobj >= num) nextobj = 0;
      dBodyDestroy (body[i]);
      dDestroyGeom (space,geom[i]);
    }
    body[i] = dBodyCreate (world);
    for (int j=0; j<3; j++) sides[i][j] = dRandReal()*0.5+0.1;
    body[i] = dBodyCreate (world);
    dBodySetPosition (body[i],dRandReal()*2-1,dRandReal()*2-1,dRandReal()+1);
    dMatrix3 R;
    dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    dBodySetRotation (body[i],R);
    dMass m;
    dMassSetBox (&m,1,sides[i][0],sides[i][1],sides[i][2]);
    dMassAdjust (&m,MASS);
    dBodySetMass (body[i],&m);
    dBodySetData (body[i],(void*) i);

    if (cmd == 'b') {
      geom[i] = dCreateBox (space,sides[i][0],sides[i][1],sides[i][2]);
      objtype[i] = 0;
    }
    else {
      sides[i][0] *= 0.5;
      geom[i] = dCreateSphere (space,sides[i][0]);
      objtype[i] = 1;
    }
    geom[i]->body = body[i];
    geom[i]->pos = const_cast<dReal*> (dBodyGetPosition(body[i]));
    geom[i]->R = const_cast<dReal*> (dBodyGetRotation(body[i]));
  }
}


// simulation loop

static void simLoop (int pause)
{
  dsSetColor (0,0,2);
  dSpaceCollide (space,0,&nearCallback);
  if (!pause) dWorldStep (world,0.05);

  // remove all contact joints
  dJointGroupEmpty (contactgroup);

  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  for (int i=0; i<num; i++) {
    if (objtype[i]==0)
      dsDrawBox (dBodyGetPosition(body[i]),dBodyGetRotation(body[i]),sides[i]);
    else
      dsDrawSphere (dBodyGetPosition(body[i]),dBodyGetRotation(body[i]),
		    sides[i][0]);
  }
}


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = "../../drawstuff/textures";

  // create world

  world = dWorldCreate();
  space = dSpaceCreate();
  contactgroup = dJointGroupCreate (1000000);
  dWorldSetGravity (world,0,0,-0.5);
  dCreatePlane (space,0,0,1,0);

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);

  return 0;
}
