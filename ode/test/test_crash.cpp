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

// This is a demo of the StepFast code, by David Whittaker.

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef MSVC
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif


// some constants

#define LENGTH 3.5		// chassis length
#define WIDTH 2.5		// chassis width
#define HEIGHT 1.0		// chassis height
#define RADIUS 0.5		// wheel radius
#define STARTZ 1.0		// starting height of chassis
#define CMASS 1			// chassis mass
#define WMASS 1			// wheel mass
#define COMOFFSET -5		// center of mass offset
#define WALLMASS 1		// wall box mass
#define BALLMASS 1		// ball mass
#define FMAX 25			// car engine fmax
#define ROWS 1			// rows of cars
#define COLS 1			// columns of cars
#define ITERS 10		// number of iterations
#define WBOXSIZE 1.0		// size of wall boxes
#define WALLWIDTH 20		// width of wall
#define WALLHEIGHT 10		// height of wall
#define DISABLE_THRESHOLD 0.008	// maximum velocity (squared) a body can have and be disabled
#define DISABLE_STEPS 10	// number of steps a box has to have been disable-able before it will be disabled

//#define BOX
#define CARS
#define WALL
//#define BALLS
//#define BALLSTACK
//#define ONEBALL
//#define CENTIPEDE

// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[10000];
static int bodies;
static dJointID joint[100000];
static int joints;
static dJointGroupID contactgroup;
static dGeomID ground;
static dGeomID box[10000];
static int boxes;
static dGeomID sphere[10000];
static int spheres;
static dGeomID wall_boxes[10000];
static dBodyID wall_bodies[10000];
static int wb_stepsdis[10000];
static int wb;
static bool doFast;
static dBodyID b;
static dMass m;


// things that the user controls

static dReal turn = 0, speed = 0;	// user commands



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	int i,n;
	
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2))
		return;
	
	const int N = 10;
	dContact contact[N];
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n > 0) {
		for (i=0; i<n; i++) {
			contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
			if (dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass)
				contact[i].surface.mu = 20;
			else
				contact[i].surface.mu = 0.5;
			contact[i].surface.slip1 = 0.0;
			contact[i].surface.slip2 = 0.0;
			contact[i].surface.soft_erp = 0.8;
			contact[i].surface.soft_cfm = 0.01;
			dJointID c = dJointCreateContact (world,contactgroup,contact+i);
			dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
		}
	}
}


// start simulation - set viewpoint

static void start()
{
	static float xyz[3] = {10.0f,0.0f,2.000f};
	static float hpr[3] = {180.0f,0.0f,0.25f};
	dsSetViewpoint (xyz,hpr);
	printf ("Press:\t'a' to increase speed.\n"
			"\t'z' to decrease speed.\n"
			"\t',' to steer left.\n"
			"\t'.' to steer right.\n"
			"\t' ' to reset speed and steering.\n"
			"\t'f' to toggle fast step mode.\n"
			"\t'+' to increase AutoEnableDepth.\n"
			"\t'-' to decrease AutoEnableDepth.\n"
			"\t'r' to reset simulation.\n");
}


void makeCar(dReal x, dReal y, int &bodyI, int &jointI, int &boxI, int &sphereI)
{
	int i;
	dMass m;
	
	// chassis body
	body[bodyI] = dBodyCreate (world);
	dBodySetPosition (body[bodyI],x,y,STARTZ);
	dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
	dMassAdjust (&m,CMASS/2.0);
	dBodySetMass (body[bodyI],&m);
	box[boxI] = dCreateBox (space,LENGTH,WIDTH,HEIGHT);
	dGeomSetBody (box[boxI],body[bodyI]);
	
	// wheel bodies
	for (i=1; i<=4; i++) {
		body[bodyI+i] = dBodyCreate (world);
		dQuaternion q;
		dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
		dBodySetQuaternion (body[bodyI+i],q);
		dMassSetSphere (&m,1,RADIUS);
		dMassAdjust (&m,WMASS);
		dBodySetMass (body[bodyI+i],&m);
		sphere[sphereI+i-1] = dCreateSphere (space,RADIUS);
		dGeomSetBody (sphere[sphereI+i-1],body[bodyI+i]);
	}
	dBodySetPosition (body[bodyI+1],x+0.4*LENGTH-0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
	dBodySetPosition (body[bodyI+2],x+0.4*LENGTH-0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
	dBodySetPosition (body[bodyI+3],x-0.4*LENGTH+0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
	dBodySetPosition (body[bodyI+4],x-0.4*LENGTH+0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
	
	// front and back wheel hinges
	for (i=0; i<4; i++) {
		joint[jointI+i] = dJointCreateHinge2 (world,0);
		dJointAttach (joint[jointI+i],body[bodyI],body[bodyI+i+1]);
		const dReal *a = dBodyGetPosition (body[bodyI+i+1]);
		dJointSetHinge2Anchor (joint[jointI+i],a[0],a[1],a[2]);
		dJointSetHinge2Axis1 (joint[jointI+i],0,0,(i<2 ? 1 : -1));
		dJointSetHinge2Axis2 (joint[jointI+i],0,1,0);
		dJointSetHinge2Param (joint[jointI+i],dParamSuspensionERP,0.8);
		dJointSetHinge2Param (joint[jointI+i],dParamSuspensionCFM,1e-5);
		dJointSetHinge2Param (joint[jointI+i],dParamVel2,0);
		dJointSetHinge2Param (joint[jointI+i],dParamFMax2,FMAX);
	}
	
	//center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
	dBodyID b = dBodyCreate (world);
	dBodySetPosition (b,x,y,STARTZ+COMOFFSET);
	dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
	dMassAdjust (&m,CMASS/2.0);
	dBodySetMass (b,&m);
	dJointID j = dJointCreateFixed(world, 0);
	dJointAttach(j, body[bodyI], b);
	dJointSetFixed(j);
	//box[boxI+1] = dCreateBox(space,LENGTH,WIDTH,HEIGHT);
	//dGeomSetBody (box[boxI+1],b);
	
	bodyI	+= 5;
	jointI	+= 4;
	boxI	+= 1;
	sphereI	+= 4;
}


void resetSimulation()
{
	int i;
	i = 0;
	// destroy world if it exists
	if (bodies)
	{
		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
	}
	
	for (i = 0; i < 1000; i++)
		wb_stepsdis[i] = 0;

	// recreate world
	
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-1.5);
	dWorldSetCFM (world, 1e-5);
	dWorldSetERP (world, 0.8);
	ground = dCreatePlane (space,0,0,1,0);
	
	bodies = 0;
	joints = 0;
	boxes = 0;
	spheres = 0;
	wb = 0;
	
#ifdef CARS
	for (dReal x = 0.0; x < COLS*(LENGTH+RADIUS); x += LENGTH+RADIUS)
		for (dReal y = -((ROWS-1)*(WIDTH/2+RADIUS)); y <= ((ROWS-1)*(WIDTH/2+RADIUS)); y += WIDTH+RADIUS*2)
			makeCar(x, y, bodies, joints, boxes, spheres);
#endif
#ifdef WALL
	bool offset = false;
	for (dReal z = WBOXSIZE/2.0; z <= WALLHEIGHT; z+=WBOXSIZE)
	{
		offset = !offset;
		for (dReal y = (-WALLWIDTH+z)/2; y <= (WALLWIDTH-z)/2; y+=WBOXSIZE)
		{
			wall_bodies[wb] = dBodyCreate (world);
			dBodySetPosition (wall_bodies[wb],-20,y,z);
			dMassSetBox (&m,1,WBOXSIZE,WBOXSIZE,WBOXSIZE);
			dMassAdjust (&m, WALLMASS);
			dBodySetMass (wall_bodies[wb],&m);
			wall_boxes[wb] = dCreateBox (space,WBOXSIZE,WBOXSIZE,WBOXSIZE);
			dGeomSetBody (wall_boxes[wb],wall_bodies[wb]);
			//dBodyDisable(wall_bodies[wb++]);
			wb++;
		}
	}
	dMessage(0,"wall boxes: %i", wb);
#endif
#ifdef BALLS
	for (dReal x = -7; x <= -4; x+=1)
		for (dReal y = -1.5; y <= 1.5; y+=1)
			for (dReal z = 1; z <= 4; z+=1)
			{
				b = dBodyCreate (world);
				dBodySetPosition (b,x*RADIUS*2,y*RADIUS*2,z*RADIUS*2);
				dMassSetSphere (&m,1,RADIUS);
				dMassAdjust (&m, BALLMASS);
				dBodySetMass (b,&m);
				sphere[spheres] = dCreateSphere (space,RADIUS);
				dGeomSetBody (sphere[spheres++],b);
			}
#endif
#ifdef ONEBALL
	b = dBodyCreate (world);
	dBodySetPosition (b,0,0,2);
	dMassSetSphere (&m,1,RADIUS);
	dMassAdjust (&m, 1);
	dBodySetMass (b,&m);
	sphere[spheres] = dCreateSphere (space,RADIUS);
	dGeomSetBody (sphere[spheres++],b);
#endif
#ifdef BALLSTACK
	for (dReal z = 1; z <= 6; z+=1)
	{
		b = dBodyCreate (world);
		dBodySetPosition (b,0,0,z*RADIUS*2);
		dMassSetSphere (&m,1,RADIUS);
		dMassAdjust (&m, 0.1);
		dBodySetMass (b,&m);
		sphere[spheres] = dCreateSphere (space,RADIUS);
		dGeomSetBody (sphere[spheres++],b);
	}
#endif
#ifdef CENTIPEDE
	dBodyID lastb = 0;
	for (dReal y = 0; y < 10*LENGTH; y+=LENGTH+0.1)
	{
		// chassis body
		
		b = body[bodies] = dBodyCreate (world);
		dBodySetPosition (body[bodies],-15,y,STARTZ);
		dMassSetBox (&m,1,WIDTH,LENGTH,HEIGHT);
		dMassAdjust (&m,CMASS);
		dBodySetMass (body[bodies],&m);
		box[boxes] = dCreateBox (space,WIDTH,LENGTH,HEIGHT);
		dGeomSetBody (box[boxes++],body[bodies++]);
		
		for (dReal x = -17; x > -20; x-=RADIUS*2)
		{
			body[bodies] = dBodyCreate (world);
			dBodySetPosition(body[bodies], x, y, STARTZ);
			dMassSetSphere(&m, 1, RADIUS);
			dMassAdjust(&m, WMASS);
			dBodySetMass(body[bodies], &m);
			sphere[spheres] = dCreateSphere (space, RADIUS);
			dGeomSetBody (sphere[spheres++], body[bodies]);
			
			joint[joints] = dJointCreateHinge2 (world,0);
			if (x == -17)
				dJointAttach (joint[joints],b,body[bodies]);
			else
				dJointAttach (joint[joints],body[bodies-2],body[bodies]);
			const dReal *a = dBodyGetPosition (body[bodies++]);
			dJointSetHinge2Anchor (joint[joints],a[0],a[1],a[2]);
			dJointSetHinge2Axis1 (joint[joints],0,0,1);
			dJointSetHinge2Axis2 (joint[joints],1,0,0);
			dJointSetHinge2Param (joint[joints],dParamSuspensionERP,1.0);
			dJointSetHinge2Param (joint[joints],dParamSuspensionCFM,1e-5);
			dJointSetHinge2Param (joint[joints],dParamLoStop,0);
			dJointSetHinge2Param (joint[joints],dParamHiStop,0);
			dJointSetHinge2Param (joint[joints],dParamVel2,-10.0);
			dJointSetHinge2Param (joint[joints++],dParamFMax2,FMAX);

			body[bodies] = dBodyCreate (world);
			dBodySetPosition(body[bodies], -30 - x, y, STARTZ);
			dMassSetSphere(&m, 1, RADIUS);
			dMassAdjust(&m, WMASS);
			dBodySetMass(body[bodies], &m);
			sphere[spheres] = dCreateSphere (space, RADIUS);
			dGeomSetBody (sphere[spheres++], body[bodies]);
			
			joint[joints] = dJointCreateHinge2 (world,0);
			if (x == -17)
				dJointAttach (joint[joints],b,body[bodies]);
			else
				dJointAttach (joint[joints],body[bodies-2],body[bodies]);
			const dReal *b = dBodyGetPosition (body[bodies++]);
			dJointSetHinge2Anchor (joint[joints],b[0],b[1],b[2]);
			dJointSetHinge2Axis1 (joint[joints],0,0,1);
			dJointSetHinge2Axis2 (joint[joints],1,0,0);
			dJointSetHinge2Param (joint[joints],dParamSuspensionERP,1.0);
			dJointSetHinge2Param (joint[joints],dParamSuspensionCFM,1e-5);
			dJointSetHinge2Param (joint[joints],dParamLoStop,0);
			dJointSetHinge2Param (joint[joints],dParamHiStop,0);
			dJointSetHinge2Param (joint[joints],dParamVel2,10.0);
			dJointSetHinge2Param (joint[joints++],dParamFMax2,FMAX);
		}
		if (lastb)
		{
			dJointID j = dJointCreateFixed(world,0);
			dJointAttach (j, b, lastb);
			dJointSetFixed(j);
		}
		lastb = b;
	}
#endif
#ifdef BOX
	body[bodies] = dBodyCreate (world);
	dBodySetPosition (body[bodies],0,0,HEIGHT/2);
	dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
	dMassAdjust (&m, 1);
	dBodySetMass (body[bodies],&m);
	box[boxes] = dCreateBox (space,LENGTH,WIDTH,HEIGHT);
	dGeomSetBody (box[boxes++],body[bodies++]);	
#endif
}

// called when a key pressed

static void command (int cmd)
{
	switch (cmd) {
	case 'a': case 'A':
		speed += 0.3;
	break;
	case 'z': case 'Z':
		speed -= 0.3;
	break;
	case ',':
		turn += 0.1;
		if (turn > 0.3)
			turn = 0.3;
	break;
	case '.':
		turn -= 0.1;
		if (turn < -0.3)
			turn = -0.3;
	break;
	case ' ':
		speed = 0;
		turn = 0;
	break;
	case 'f': case 'F':
		doFast = !doFast;
	break;
	case '+':
		dWorldSetAutoEnableDepthSF1 (world, dWorldGetAutoEnableDepthSF1 (world) + 1);
	break;
	case '-':
		dWorldSetAutoEnableDepthSF1 (world, dWorldGetAutoEnableDepthSF1 (world) - 1);
	break;
	case 'r': case 'R':
		resetSimulation();
	break;
	}
}


// simulation loop

static void simLoop (int pause)
{
	int i, j;
		
	dsSetTexture (DS_WOOD);

	if (!pause) {
#ifdef BOX
		dBodyAddForce(body[bodies-1],lspeed,0,0);
#endif
		for (j = 0; j < joints; j++)
		{
			dReal curturn = dJointGetHinge2Angle1 (joint[j]);
			//dMessage (0,"curturn %e, turn %e, vel %e", curturn, turn, (turn-curturn)*1.0);
			dJointSetHinge2Param(joint[j],dParamVel,(turn-curturn)*1.0);
			dJointSetHinge2Param(joint[j],dParamFMax,dInfinity);
			dJointSetHinge2Param(joint[j],dParamVel2,speed);
			dJointSetHinge2Param(joint[j],dParamFMax2,FMAX);
			dBodyEnable(dJointGetBody(joint[j],0));
			dBodyEnable(dJointGetBody(joint[j],1));
		}		
		if (doFast)
		{
			dSpaceCollide (space,0,&nearCallback);
			dWorldStepFast1 (world,0.05,ITERS);
			dJointGroupEmpty (contactgroup);
		}
		else
		{
			dSpaceCollide (space,0,&nearCallback);
			dWorldStep (world,0.05);
			dJointGroupEmpty (contactgroup);
		}
		
		for (i = 0; i < wb; i++)
		{
			b = dGeomGetBody(wall_boxes[i]);
			if (dBodyIsEnabled(b)) 
			{
				bool disable = true;
				const dReal *lvel = dBodyGetLinearVel(b);
				dReal lspeed = lvel[0]*lvel[0]+lvel[1]*lvel[1]+lvel[2]*lvel[2];
				if (lspeed > DISABLE_THRESHOLD)
					disable = false;
				const dReal *avel = dBodyGetAngularVel(b);
				dReal aspeed = avel[0]*avel[0]+avel[1]*avel[1]+avel[2]*avel[2];
				if (aspeed > DISABLE_THRESHOLD)
					disable = false;
				
				if (disable)
					wb_stepsdis[i]++;
				else
					wb_stepsdis[i] = 0;
				
				if (wb_stepsdis[i] > DISABLE_STEPS)
				{
					dBodyDisable(b);
					dsSetColor(0.5,0.5,1);
				}
				else
					dsSetColor(1,1,1);

			}
			else
				dsSetColor(0.4,0.4,0.4);
			dVector3 ss;
			dGeomBoxGetLengths (wall_boxes[i], ss);
			dsDrawBox(dGeomGetPosition(wall_boxes[i]), dGeomGetRotation(wall_boxes[i]), ss);
		}
	}
	else
	{
		for (i = 0; i < wb; i++)
		{
			b = dGeomGetBody(wall_boxes[i]);
			if (dBodyIsEnabled(b))
				dsSetColor(1,1,1);
			else
				dsSetColor(0.4,0.4,0.4);
			dVector3 ss;
			dGeomBoxGetLengths (wall_boxes[i], ss);
			dsDrawBox(dGeomGetPosition(wall_boxes[i]), dGeomGetRotation(wall_boxes[i]), ss);
		}
	}
	
	if (doFast)
		dsSetColor(0,1,0);
	else
		dsSetColor(1,0,0);
	dReal pos[3] = {-10,0,10};
	dReal R[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
	dsDrawSphere(pos, R, 2);
	
	dsSetColor (0,1,1);
	dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
	for (i = 0; i < boxes; i++)
		dsDrawBox (dGeomGetPosition(box[i]),dGeomGetRotation(box[i]),sides);
	dsSetColor (1,1,1);
	for (i=0; i< spheres; i++) dsDrawSphere (dGeomGetPosition(sphere[i]),
				   dGeomGetRotation(sphere[i]),RADIUS);
	
	
}

int main (int argc, char **argv)
{
	doFast = true;
	
	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "../../drawstuff/textures";
	
	bodies = 0;
	joints = 0;
	boxes = 0;
	spheres = 0;
	
	resetSimulation();
	
	// run simulation
	dsSimulationLoop (argc,argv,352,288,&fn);
	
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

	return 0;
}
