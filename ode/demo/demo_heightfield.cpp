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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "bunny_geom.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#define	DEGTORAD			0.01745329251994329577f				//!< PI / 180.0, convert degrees to radians

int g_allow_trimesh;

// Our heightfield geom
dGeomID gheight;



// Heightfield dimensions

#define HFIELD_WSTEP			15			// Vertex count along edge >= 2
#define HFIELD_DSTEP			31

#define HFIELD_WIDTH			REAL( 4.0 )
#define HFIELD_DEPTH			REAL( 8.0 )

#define HFIELD_WSAMP			( HFIELD_WIDTH / ( HFIELD_WSTEP-1 ) )
#define HFIELD_DSAMP			( HFIELD_DEPTH / ( HFIELD_DSTEP-1 ) )



//<---- Convex Object
dReal planes[]= // planes for a cube
  {
    1.0f ,0.0f ,0.0f ,0.25f,
    0.0f ,1.0f ,0.0f ,0.25f,
    0.0f ,0.0f ,1.0f ,0.25f,
    0.0f ,0.0f ,-1.0f,0.25f,
    0.0f ,-1.0f,0.0f ,0.25f,
    -1.0f,0.0f ,0.0f ,0.25f
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
    4,0,2,6,4, // positive X
    4,1,0,4,5, // positive Y
    4,0,1,3,2, // positive Z
    4,3,1,5,7, // negative X
    4,2,3,7,6, // negative Y
    4,5,4,6,7, // negative Z
  };
//----> Convex Object

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#endif


// some constants

#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 64		// maximum number of contact points per body


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body

  // Trimesh only - double buffered matrices for 'last transform' setup
  dReal matrix_dblbuff[ 16 * 2 ];
  int last_matrix_index;
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static int write_world = 0;




//============================

dGeomID TriMesh1;
dGeomID TriMesh2;
//static dTriMeshDataID TriData1, TriData2;  // reusable static trimesh data

//============================


dReal heightfield_callback( void* pUserData, int x, int z )
{
	dReal fx = ( ((dReal)x) - ( HFIELD_WSTEP-1 )/2 ) / (dReal)( HFIELD_WSTEP-1 );
	dReal fz = ( ((dReal)z) - ( HFIELD_DSTEP-1 )/2 ) / (dReal)( HFIELD_DSTEP-1 );

	// Create an interesting 'hump' shape
	dReal h = REAL( 1.0 ) + ( REAL( -16.0 ) * ( fx*fx*fx + fz*fz*fz ) );

	return h;
}




// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   b for box.\n");
  printf ("   s for sphere.\n");
  printf ("   c for capsule.\n");
  printf ("   y for cylinder.\n");
  printf ("   v for a convex object.\n");
  printf ("   x for a composite object.\n");
  if ( g_allow_trimesh )
	printf ("   m for a trimesh.\n");
  printf ("To select an object, press space.\n");
  printf ("To disable the selected object, press d.\n");
  printf ("To enable the selected object, press e.\n");
  printf ("To toggle showing the geom AABBs, press a.\n");
  printf ("To toggle showing the contact points, press t.\n");
  printf ("To toggle dropping from random position/orientation, press r.\n");
  printf ("To save the current state to 'state.dif', press 1.\n");
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed

static void command (int cmd)
{
	size_t i;
	int j,k;
	dReal sides[3];
	dMass m;

	cmd = locase (cmd);


	//
	// Geom Creation
	//

	if ( cmd == 'b' || cmd == 's' || cmd == 'c' || ( cmd == 'm' && g_allow_trimesh ) ||
		 cmd == 'x' || cmd == 'y' || cmd == 'v' )
	{
		if ( num < NUM )
		{
			i = num;
			num++;
		}
		else
		{
			i = nextobj;
			nextobj++;
			if (nextobj >= num) nextobj = 0;

			// destroy the body and geoms for slot i
			dBodyDestroy (obj[i].body);
			for (k=0; k < GPB; k++)
			{
				if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
			}
			memset (&obj[i],0,sizeof(obj[i]));
		}

		obj[i].body = dBodyCreate (world);
		for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

		dMatrix3 R;
		if (random_pos) {
			dBodySetPosition (obj[i].body,
				(dRandReal()-0.5)*HFIELD_WIDTH*0.75,
				(dRandReal()-0.5)*HFIELD_DEPTH*0.75,
				dRandReal() + 2 );
			dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
				dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
		}
		else {
			dReal maxheight = 0;
			for (k=0; k<num; k++) {
				const dReal *pos = dBodyGetPosition (obj[k].body);
				if (pos[2] > maxheight) maxheight = pos[2];
			}
			dBodySetPosition (obj[i].body, 0,maxheight+1,0);
			dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
		}
		dBodySetRotation (obj[i].body,R);
		dBodySetData (obj[i].body,(void*) i);

		if (cmd == 'b')
		{
			dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
			obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
		}
		else if (cmd == 'c')
		{
			sides[0] *= 0.5;
			dMassSetCapsule (&m,DENSITY,3,sides[0],sides[1]);
			obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
		}
		//<---- Convex Object
		else if (cmd == 'v')
		{
			dMassSetBox (&m,DENSITY,0.25,0.25,0.25);
			obj[i].geom[0] = dCreateConvex (space,
				planes,
				planecount,
				points,
				pointcount,
				polygons);
		}
		//----> Convex Object
		else if (cmd == 'y')
		{
			dMassSetCylinder (&m,DENSITY,3,sides[0],sides[1]);
			obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
		}
		else if (cmd == 's')
		{
			sides[0] *= 0.5;
			dMassSetSphere (&m,DENSITY,sides[0]);
			obj[i].geom[0] = dCreateSphere (space,sides[0]);
		}
		else if (cmd == 'm' && g_allow_trimesh)
		{
			dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
			dGeomTriMeshDataBuildSingle(new_tmdata, &Vertices[0], 3 * sizeof(float), VertexCount, 
				&Indices[0], IndexCount, 3 * sizeof(dTriIndex));

			obj[i].geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);

			dMassSetTrimesh( &m, DENSITY, obj[i].geom[0] );
			printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
			dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
		}
		else if (cmd == 'x')
		{
			dGeomID g2[GPB];		// encapsulated geometries
			dReal dpos[GPB][3];	// delta-positions for encapsulated geometries

			// start accumulating masses for the encapsulated geometries
			dMass m2;
			dMassSetZero (&m);

			// set random delta positions
			for (j=0; j<GPB; j++) {
				for (k=0; k<3; k++) dpos[j][k] = dRandReal()*0.3-0.15;
			}

			for (k=0; k<GPB; k++) {
				obj[i].geom[k] = dCreateGeomTransform (space);
				dGeomTransformSetCleanup (obj[i].geom[k],1);
				if (k==0) {
					dReal radius = dRandReal()*0.25+0.05;
					g2[k] = dCreateSphere (0,radius);
					dMassSetSphere (&m2,DENSITY,radius);
				}
				else if (k==1) {
					g2[k] = dCreateBox (0,sides[0],sides[1],sides[2]);
					dMassSetBox (&m2,DENSITY,sides[0],sides[1],sides[2]);
				}
				else {
					dReal radius = dRandReal()*0.1+0.05;
					dReal length = dRandReal()*1.0+0.1;
					g2[k] = dCreateCapsule (0,radius,length);
					dMassSetCapsule (&m2,DENSITY,3,radius,length);
				}
				dGeomTransformSetGeom (obj[i].geom[k],g2[k]);

				// set the transformation (adjust the mass too)
				dGeomSetPosition (g2[k],dpos[k][0],dpos[k][1],dpos[k][2]);
				dMassTranslate (&m2,dpos[k][0],dpos[k][1],dpos[k][2]);
				dMatrix3 Rtx;
				dRFromAxisAndAngle (Rtx,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
					dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
				dGeomSetRotation (g2[k],Rtx);
				dMassRotate (&m2,Rtx);

				// add to the total mass
				dMassAdd (&m,&m2);
			}

			// move all encapsulated objects so that the center of mass is (0,0,0)
			for (k=0; k<2; k++) {
				dGeomSetPosition (g2[k],
					dpos[k][0]-m.c[0],
					dpos[k][1]-m.c[1],
					dpos[k][2]-m.c[2]);
			}
			dMassTranslate (&m,-m.c[0],-m.c[1],-m.c[2]);
		}

		for (k=0; k < GPB; k++)
		{
			if (obj[i].geom[k]) dGeomSetBody (obj[i].geom[k],obj[i].body);
		}

		dBodySetMass (obj[i].body,&m);
	}


	//
	// Control Commands
	//

	if (cmd == ' ') {
		selected++;
		if (selected >= num) selected = 0;
		if (selected < 0) selected = 0;
	}
	else if (cmd == 'd' && selected >= 0 && selected < num) {
		dBodyDisable (obj[selected].body);
	}
	else if (cmd == 'e' && selected >= 0 && selected < num) {
		dBodyEnable (obj[selected].body);
	}
	else if (cmd == 'a') {
		show_aabb ^= 1;
	}
	else if (cmd == 't') {
		show_contacts ^= 1;
	}
	else if (cmd == 'r') {
		random_pos ^= 1;
	}
	else if (cmd == '1') {
		write_world = 1;
	}
}


// draw a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
  int i;

  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    dsDrawBox (pos,R,sides);
  }
  else if (type == dSphereClass) {
    dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
  }
  else if (type == dCapsuleClass) {
    dReal radius,length;
    dGeomCapsuleGetParams (g,&radius,&length);
    dsDrawCapsule (pos,R,length,radius);
  }
  //<---- Convex Object
  else if (type == dConvexClass)
    {
      //dVector3 sides={0.50,0.50,0.50};
      dsDrawConvex(pos,R,planes,
		   planecount,
		   points,
		   pointcount,
		   polygons);
    }
  //----> Convex Object
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
  else if (type == dGeomTransformClass) {
    dGeomID g2 = dGeomTransformGetGeom (g);
    const dReal *pos2 = dGeomGetPosition (g2);
    const dReal *R2 = dGeomGetRotation (g2);
    dVector3 actual_pos;
    dMatrix3 actual_R;
    dMultiply0_331 (actual_pos,R,pos2);
    actual_pos[0] += pos[0];
    actual_pos[1] += pos[1];
    actual_pos[2] += pos[2];
    dMultiply0_333 (actual_R,R,R2);
    drawGeom (g2,actual_pos,actual_R,0);
  }

  if (show_aabb) {
    // draw the bounding box for this geom
    dReal aabb[6];
    dGeomGetAABB (g,aabb);
    dVector3 bbpos;
    for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }

}

// simulation loop

static void simLoop (int pause)
{
  int i,j;
  
  dsSetColor (0,0,2);
  
  dSpaceCollide (space,0,&nearCallback);
  
  //if (!pause) dWorldStep (world,0.05);
  if (!pause) dWorldQuickStep (world,0.05);


  if (write_world) {
    FILE *f = fopen ("state.dif","wt");
    if (f) {
      dWorldExportDIF (world,f,"X");
      fclose (f);
    }
    write_world = 0;
  }

  // remove all contact joints
  dJointGroupEmpty (contactgroup);



	const dReal* pReal = dGeomGetPosition( gheight );

	const dReal* RReal = dGeomGetRotation( gheight );

	//
	// Draw Heightfield
	//

	// Set ox and oz to zero for DHEIGHTFIELD_CORNER_ORIGIN mode.
	int ox = (int) ( -HFIELD_WIDTH/2 );
	int oz = (int) ( -HFIELD_DEPTH/2 );

//	for ( int tx = -1; tx < 2; ++tx )
//	for ( int tz = -1; tz < 2; ++tz )
	{
		dsSetColorAlpha (0.5,1,0.5,0.5);
		dsSetTexture( DS_WOOD );

		for ( int i = 0; i < HFIELD_WSTEP - 1; ++i )
		for ( int j = 0; j < HFIELD_DSTEP - 1; ++j )
		{
			dReal a[3], b[3], c[3], d[3];

			a[ 0 ] = ox + ( i ) * HFIELD_WSAMP;
			a[ 1 ] = heightfield_callback( NULL, i, j );
			a[ 2 ] = oz + ( j ) * HFIELD_DSAMP;

			b[ 0 ] = ox + ( i + 1 ) * HFIELD_WSAMP;
			b[ 1 ] = heightfield_callback( NULL, i + 1, j );
			b[ 2 ] = oz + ( j ) * HFIELD_DSAMP;

			c[ 0 ] = ox + ( i ) * HFIELD_WSAMP;
			c[ 1 ] = heightfield_callback( NULL, i, j + 1 );
			c[ 2 ] = oz + ( j + 1 ) * HFIELD_DSAMP;

			d[ 0 ] = ox + ( i + 1 ) * HFIELD_WSAMP;
			d[ 1 ] = heightfield_callback( NULL, i + 1, j + 1 );
			d[ 2 ] = oz + ( j + 1 ) * HFIELD_DSAMP;

			dsDrawTriangle( pReal, RReal, a, c, b, 1 );
			dsDrawTriangle( pReal, RReal, b, c, d, 1 );
		}
	}





	dsSetColor (1,1,0);
	dsSetTexture (DS_WOOD);
	for (i=0; i<num; i++)
	{
		for (j=0; j < GPB; j++)
		{
			if (i==selected)
			{
				dsSetColor (0,0.7,1);
			}
			else if (! dBodyIsEnabled (obj[i].body))
			{
				dsSetColor (1,0.8,0);
			}
			else 
			{
				dsSetColor (1,1,0);
			}

			if ( obj[i].geom[j] && dGeomGetClass(obj[i].geom[j]) == dTriMeshClass )
			{
				dTriIndex* Indices = (dTriIndex*)::Indices;

				// assume all trimeshes are drawn as bunnies
				const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
				const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);

				for (int ii = 0; ii < IndexCount / 3; ii++)
				{
					const dReal v[9] = { // explicit conversion from float to dReal
						Vertices[Indices[ii * 3 + 0] * 3 + 0],
							Vertices[Indices[ii * 3 + 0] * 3 + 1],
							Vertices[Indices[ii * 3 + 0] * 3 + 2],
							Vertices[Indices[ii * 3 + 1] * 3 + 0],
							Vertices[Indices[ii * 3 + 1] * 3 + 1],
							Vertices[Indices[ii * 3 + 1] * 3 + 2],
							Vertices[Indices[ii * 3 + 2] * 3 + 0],
							Vertices[Indices[ii * 3 + 2] * 3 + 1],
							Vertices[Indices[ii * 3 + 2] * 3 + 2]
					};
					dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
				}

				// tell the tri-tri collider the current transform of the trimesh --
				// this is fairly important for good results.

				// Fill in the (4x4) matrix.
				dReal* p_matrix = obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 );

				p_matrix[ 0 ] = Rot[ 0 ];	p_matrix[ 1 ] = Rot[ 1 ];	p_matrix[ 2 ] = Rot[ 2 ];	p_matrix[ 3 ] = 0;
				p_matrix[ 4 ] = Rot[ 4 ];	p_matrix[ 5 ] = Rot[ 5 ];	p_matrix[ 6 ] = Rot[ 6 ];	p_matrix[ 7 ] = 0;
				p_matrix[ 8 ] = Rot[ 8 ];	p_matrix[ 9 ] = Rot[ 9 ];	p_matrix[10 ] = Rot[10 ];	p_matrix[11 ] = 0;
				p_matrix[12 ] = Pos[ 0 ];	p_matrix[13 ] = Pos[ 1 ];	p_matrix[14 ] = Pos[ 2 ];	p_matrix[15 ] = 1;

				// Flip to other matrix.
				obj[i].last_matrix_index = !obj[i].last_matrix_index;

				// Apply the 'other' matrix which is the oldest.
				dGeomTriMeshSetLastTransform( obj[i].geom[j], 
					*(dMatrix4*)( obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 ) ) );
			}
			else
			{
				drawGeom (obj[i].geom[j],0,0,show_aabb);
			}
		}
	}

	if ( show_aabb )
	{
		// draw the bounding box for this geom
		dReal aabb[6];
		dGeomGetAABB (gheight,aabb);
		dVector3 bbpos;
		for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
		dVector3 bbsides;
		for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
		dMatrix3 RI;
		dRSetIdentity (RI);
		dsSetColorAlpha (1,0,0,0.5);
		dsDrawBox (bbpos,RI,bbsides);
	}
}


int main (int argc, char **argv)
{
        printf("ODE configuration: %s\n", dGetConfiguration());
        
	// Is trimesh support built into this ODE?
	g_allow_trimesh = dCheckConfiguration( "ODE_EXT_trimesh" );

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-0.05);
	dWorldSetCFM (world,1e-5);
	dWorldSetAutoDisableFlag (world,1);
	dWorldSetContactMaxCorrectingVel (world,0.1);
	dWorldSetContactSurfaceLayer (world,0.001);
	memset (obj,0,sizeof(obj));

#if 1

  dWorldSetAutoDisableAverageSamplesCount( world, 1 );

#endif

	// base plane to catch overspill
	dCreatePlane( space, 0, 0, 1, 0 );


	// our heightfield floor

	dHeightfieldDataID heightid = dGeomHeightfieldDataCreate();

	// Create an finite heightfield.
	dGeomHeightfieldDataBuildCallback( heightid, NULL, heightfield_callback,
		HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
		REAL( 1.0 ), REAL( 0.0 ), REAL( 0.0 ), 0 );

	// Give some very bounds which, while conservative,
	// makes AABB computation more accurate than +/-INF.
	dGeomHeightfieldDataSetBounds( heightid, REAL( -4.0 ), REAL( +6.0 ) );

	gheight = dCreateHeightfield( space, heightid, 1 );

	dVector3 pos;
	pos[ 0 ] = 0;
	pos[ 1 ] = 0;
	pos[ 2 ] = 0;

	// Rotate so Z is up, not Y (which is the default orientation)
	dMatrix3 R;
	dRSetIdentity( R );
	dRFromAxisAndAngle( R, 1, 0, 0, DEGTORAD * 90 );

	// Place it.
	dGeomSetRotation( gheight, R );
	dGeomSetPosition( gheight, pos[0], pos[1], pos[2] );

  dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
  dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

	// run simulation
	dsSimulationLoop (argc,argv,352,288,&fn);

  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);

	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

	// destroy heightfield data, because _we_ own it not ODE
	dGeomHeightfieldDataDestroy( heightid );

	dCloseODE();
	return 0;
}
