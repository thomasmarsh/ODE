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

#ifdef MSVC
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif

//****************************************************************************
// globals

static int seed=0;

extern "C" int dBoxBox (const dVector3 p1, const dMatrix3 R1,
			const dVector3 side1, const dVector3 p2,
			const dMatrix3 R2, const dVector3 side2,
			dVector3 normal, dReal *depth, int *code,
			int maxc, dContactGeom *contact, int skip);

extern "C" void lineClosestApproach (const dVector3 pa, const dVector3 ua,
				     const dVector3 pb, const dVector3 ub,
				     dReal *alpha, dReal *beta);

//****************************************************************************
// a really inefficient, but hopefully correct implementation of
// dBoxTouchesBox(), that does 144 edge-face tests.

// return 1 if edge v1 -> v2 hits the rectangle described by p1,p2,p3

static int edgeIntersectsRect (dVector3 v1, dVector3 v2,
			       dVector3 p1, dVector3 p2, dVector3 p3)
{
  int k;
  dVector3 u1,u2,n,tmp;
  for (k=0; k<3; k++) u1[k] = p3[k]-p1[k];
  for (k=0; k<3; k++) u2[k] = p2[k]-p1[k];
  dReal d1 = dSqrt(dDOT(u1,u1));
  dReal d2 = dSqrt(dDOT(u2,u2));
  dNormalize3 (u1);
  dNormalize3 (u2);
  if (dFabs(dDOT(u1,u2)) > 1e-8) dDebug (0,"bad");
  dCROSS (n,=,u1,u2);
  for (k=0; k<3; k++) tmp[k] = v2[k]-v1[k];
  dReal d = -dDOT(n,p1);
  if (dFabs(dDOT(n,p1)+d) > 1e-8) dDebug (0,"bad");
  if (dFabs(dDOT(n,p2)+d) > 1e-8) dDebug (0,"bad");
  if (dFabs(dDOT(n,p3)+d) > 1e-8) dDebug (0,"bad");
  dReal alpha = -(d+dDOT(n,v1))/dDOT(n,tmp);
  for (k=0; k<3; k++) tmp[k] = v1[k]+alpha*(v2[k]-v1[k]);
  if (dFabs(dDOT(n,tmp)+d) > 1e-8) dDebug (0,"bad");
  if (alpha < 0) return 0;
  if (alpha > 1) return 0;
  for (k=0; k<3; k++) tmp[k] -= p1[k];
  dReal a1 = dDOT(u1,tmp);
  dReal a2 = dDOT(u2,tmp);
  if (a1<0 || a2<0 || a1>d1 || a2>d2) return 0;
  return 1;
}


// return 1 if box 1 is completely inside box 2

static int box1inside2 (const dVector3 p1, const dMatrix3 R1,
			const dVector3 side1, const dVector3 p2,
			const dMatrix3 R2, const dVector3 side2)
{
  for (int i=-1; i<=1; i+=2) {
    for (int j=-1; j<=1; j+=2) {
      for (int k=-1; k<=1; k+=2) {
	dVector3 v,vv;
	v[0] = i*0.5*side1[0];
	v[1] = j*0.5*side1[1];
	v[2] = k*0.5*side1[2];
	dMULTIPLY0_331 (vv,R1,v);
	vv[0] += p1[0] - p2[0];
	vv[1] += p1[1] - p2[1];
	vv[2] += p1[2] - p2[2];
	for (int axis=0; axis < 3; axis++) {
	  dReal z = dDOT14(vv,R2+axis);
	  if (z < (-side2[axis]*0.5) || z > (side2[axis]*0.5)) return 0;
	}
      }
    }
  }
  return 1;
}


// test if any edge from box 1 hits a face from box 2

static int testBoxesTouch2 (const dVector3 p1, const dMatrix3 R1,
			    const dVector3 side1, const dVector3 p2,
			    const dMatrix3 R2, const dVector3 side2)
{
  int j,k,j1,j2;

  // for 6 faces from box 2
  for (int fd=0; fd<3; fd++) {		// direction for face

    for (int fo=0; fo<2; fo++) {	// offset of face
      // get four points on the face. first get 2 indexes that are not fd
      int k1=0,k2=0;
      if (fd==0) { k1 = 1; k2 = 2; }
      if (fd==1) { k1 = 0; k2 = 2; }
      if (fd==2) { k1 = 0; k2 = 1; }
      dVector3 fp[4],tmp;
      k=0;
      for (j1=-1; j1<=1; j1+=2) {
	for (j2=-1; j2<=1; j2+=2) {
	  fp[k][k1] = j1;
	  fp[k][k2] = j2;
	  fp[k][fd] = fo*2-1;
	  k++;
	}
      }
      for (j=0; j<4; j++) {
	for (k=0; k<3; k++) fp[j][k] *= 0.5*side2[k];
	dMULTIPLY0_331 (tmp,R2,fp[j]);
	for (k=0; k<3; k++) fp[j][k] = tmp[k] + p2[k];
      }

      // for 8 vertices
      dReal v1[3];
      for (v1[0]=-1; v1[0] <= 1; v1[0] += 2) {
	for (v1[1]=-1; v1[1] <= 1; v1[1] += 2) {
	  for (v1[2]=-1; v1[2] <= 1; v1[2] += 2) {
	    // for all possible +ve leading edges from those vertices
	    for (int ei=0; ei < 3; ei ++) {
	      if (v1[ei] < 0) {
		// get vertex1 -> vertex2 = an edge from box 1
		dVector3 vv1,vv2;
		for (k=0; k<3; k++) vv1[k] = v1[k] * 0.5*side1[k];
		for (k=0; k<3; k++) vv2[k] = (v1[k] + (k==ei)*2)*0.5*side1[k];
		dVector3 vertex1,vertex2;
		dMULTIPLY0_331 (vertex1,R1,vv1);
		dMULTIPLY0_331 (vertex2,R1,vv2);
		for (k=0; k<3; k++) vertex1[k] += p1[k];
		for (k=0; k<3; k++) vertex2[k] += p1[k];

		// see if vertex1 -> vertex2 interesects face
		if (edgeIntersectsRect (vertex1,vertex2,fp[0],fp[1],fp[2]))
		  return 1;
	      }
	    }
	  }
	}
      }
    }
  }

  if (box1inside2 (p1,R1,side1,p2,R2,side2)) return 1;
  if (box1inside2 (p2,R2,side2,p1,R1,side1)) return 1;

  return 0;
}

//****************************************************************************
// check to see if dBoxTouchesBox() is correct. draw any incorrect results

static void testBoxTouchesBox()
{
  int k,bt1,bt2;
  dVector3 p1,p2,side1,side2;
  dMatrix3 R1,R2;

  do {
    dRandSetSeed (seed);

    dMakeRandomVector (p1,3,0.5);
    dMakeRandomVector (p2,3,0.5);
    for (k=0; k<3; k++) side1[k] = dRandReal() + 0.01;
    for (k=0; k<3; k++) side2[k] = dRandReal() + 0.01;
    dRFromAxisAndAngle (R1,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    dRFromAxisAndAngle (R2,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    p1[2] += 1;
    p2[2] += 1;

    int t1 = testBoxesTouch2 (p1,R1,side1,p2,R2,side2);
    int t2 = testBoxesTouch2 (p2,R2,side2,p1,R1,side1);
    bt1 = t1 || t2;
    bt2 = dBoxTouchesBox (p1,R1,side1,p2,R2,side2);

    if (bt1 == bt2) {
      seed++;
      if ((seed % 1000)==0) printf ("%d comparisons - ok\n",seed);
    }
  }
  while (bt1 == bt2);

  printf ("seed=%d  ",seed);
  if (bt1 && bt2) printf ("agree - boxes touch\n");
  if (!bt1 && !bt2) printf ("agree - boxes don't touch\n");
  if (bt1 && !bt2) printf ("disagree - boxes touch but dBoxTouchesBox "
			   "says no\n");
  if (!bt1 && bt2) printf ("disagree - boxes don't touch but dBoxTouchesBox "
			   "says yes\n");

  dsSetColor (1,1,1);
  dsDrawBox (p2,R2,side2);
  dsSetColor (0,1,1);
  dsDrawBox (p1,R1,side1);
}

//****************************************************************************
// test box-box collision

static void testBoxToBoxCollision()
{
  int k,bt;
  dVector3 p1,p2,side1,side2,normal;
  dMatrix3 R1,R2;
  dReal depth;
  int code,ok;
  dContactGeom contact[48];

  do {
    dRandSetSeed (seed);

    dMakeRandomVector (p1,3,0.5);
    dMakeRandomVector (p2,3,0.5);
    for (k=0; k<3; k++) side1[k] = dRandReal() + 0.01;
    for (k=0; k<3; k++) side2[k] = dRandReal() + 0.01;
    dRFromAxisAndAngle (R1,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    dRFromAxisAndAngle (R2,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    p1[2] += 1;
    p2[2] += 1;

    code = 0;
    depth = 0;
    bt = dBoxBox (p1,R1,side1,p2,R2,side2,normal,&depth,&code,1,contact,
		  sizeof(dContactGeom));

    //@@@
    p2[0] += normal[0] * 0.99 * depth;
    p2[1] += normal[1] * 0.99 * depth;
    p2[2] += normal[2] * 0.99 * depth;
    bt = dBoxBox (p1,R1,side1,p2,R2,side2,normal,&depth,&code,1,contact,
		  sizeof(dContactGeom));

    ok = bt; // && code <= 6 && bt==2; // && normal[2] > 0;

    if (!ok) seed++;
  }
  while (!ok);

  printf ("seed=%d  code=%2d  depth=%.4f  ",seed,code,depth);
  printf ("  |n|=%f\n",dSqrt(dDOT(normal,normal)));

  /*
  p2[0] += normal[0] * 0.9999 * depth;
  p2[1] += normal[1] * 0.9999 * depth;
  p2[2] += normal[2] * 0.9999 * depth;
  if (!dBoxTouchesBox (p1,R1,side1,p2,R2,side2))
    printf ("bad normal or depth\n");
  p2[0] += normal[0] * 0.0002 * depth;
  p2[1] += normal[1] * 0.0002 * depth;
  p2[2] += normal[2] * 0.0002 * depth;
  if (dBoxTouchesBox (p1,R1,side1,p2,R2,side2))
    printf ("bad normal or depth\n");
  */

  dsSetColor (1,1,1);
  dsDrawBox (p2,R2,side2);
  dsSetColor (0,1,1);
  dsDrawBox (p1,R1,side1);

  dsSetColor (1,1,1);
  p1[0] = 0;
  p1[1] = 0;
  p1[2] = 0;
  dsDrawLine (p1,normal);

  dsSetColor (2,2,2);
  const dReal ss[3] = {0.02,0.02,0.02};
  dsDrawBox (contact[0].pos,R1,ss);
}

//****************************************************************************
// graphics

// start simulation - set viewpoint

static void start()
{
  static float xyz[3] = {1.0382f,-1.0811f,1.4700f};
  static float hpr[3] = {135.0000f,-19.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
}


// called when a key pressed

static void command (int cmd)
{
  if (cmd == ' ') {
    seed++;
  }
}


// simulation loop

static void simLoop (int pause)
{
  //testBoxTouchesBox();
  testBoxToBoxCollision();
}

//****************************************************************************

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

  dsSimulationLoop (argc,argv,352,288,&fn);
  return 0;
}
