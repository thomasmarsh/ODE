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

/*

standard ODE geometry primitives: public API and pairwise collision functions.

the rule is that only the low level primitive collision functions should set
dContactGeom::g1 and dContactGeom::g2.

*/

#include <ode/common.h>
#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"

//****************************************************************************
// the basic geometry objects

struct dxSphere : public dxGeom {
  dReal radius;		// sphere radius
  dxSphere (dSpaceID space, dReal _radius);
  void computeAABB();
};


struct dxBox : public dxGeom {
  dVector3 side;	// side lengths (x,y,z)
  dxBox (dSpaceID space, dReal lx, dReal ly, dReal lz);
  void computeAABB();
};


struct dxCCylinder : public dxGeom {
  dReal radius,lz;	// radius, length along z axis
  dxCCylinder (dSpaceID space, dReal _radius, dReal _length);
  void computeAABB();
};


struct dxPlane : public dxGeom {
  dReal p[4];
  dxPlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d);
  void computeAABB();
};

//****************************************************************************
// sphere public API

dxSphere::dxSphere (dSpaceID space, dReal _radius) : dxGeom (space,1)
{
  dAASSERT (_radius > 0);
  type = dSphereClass;
  radius = _radius;
}


void dxSphere::computeAABB()
{
  aabb[0] = pos[0] - radius;
  aabb[1] = pos[0] + radius;
  aabb[2] = pos[1] - radius;
  aabb[3] = pos[1] + radius;
  aabb[4] = pos[2] - radius;
  aabb[5] = pos[2] + radius;
}


dGeomID dCreateSphere (dSpaceID space, dReal radius)
{
  return new dxSphere (space,radius);
}


void dGeomSphereSetRadius (dGeomID g, dReal radius)
{
  dUASSERT (g && g->type == dSphereClass,"argument not a sphere");
  dAASSERT (radius > 0);
  dxSphere *s = (dxSphere*) g;
  s->radius = radius;
  dGeomMoved (g);
}


dReal dGeomSphereGetRadius (dGeomID g)
{
  dUASSERT (g && g->type == dSphereClass,"argument not a sphere");
  dxSphere *s = (dxSphere*) g;
  return s->radius;
}

//****************************************************************************
// box public API

dxBox::dxBox (dSpaceID space, dReal lx, dReal ly, dReal lz) : dxGeom (space,1)
{
  dAASSERT (lx > 0 && ly > 0 && lz > 0);
  type = dBoxClass;
  side[0] = lx;
  side[1] = ly;
  side[2] = lz;
}


void dxBox::computeAABB()
{
  dReal xrange = REAL(0.5) * (dFabs (R[0] * side[0]) +
    dFabs (R[1] * side[1]) + dFabs (R[2] * side[2]));
  dReal yrange = REAL(0.5) * (dFabs (R[4] * side[0]) +
    dFabs (R[5] * side[1]) + dFabs (R[6] * side[2]));
  dReal zrange = REAL(0.5) * (dFabs (R[8] * side[0]) +
    dFabs (R[9] * side[1]) + dFabs (R[10] * side[2]));
  aabb[0] = pos[0] - xrange;
  aabb[1] = pos[0] + xrange;
  aabb[2] = pos[1] - yrange;
  aabb[3] = pos[1] + yrange;
  aabb[4] = pos[2] - zrange;
  aabb[5] = pos[2] + zrange;
}


dGeomID dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
{
  return new dxBox (space,lx,ly,lz);
}


void dGeomBoxSetLengths (dGeomID g, dReal lx, dReal ly, dReal lz)
{
  dUASSERT (g && g->type == dBoxClass,"argument not a box");
  dAASSERT (lx > 0 && ly > 0 && lz > 0);
  dxBox *b = (dxBox*) g;
  b->side[0] = lx;
  b->side[1] = ly;
  b->side[2] = lz;
  dGeomMoved (g);
}


void dGeomBoxGetLengths (dGeomID g, dVector3 result)
{
  dUASSERT (g && g->type == dBoxClass,"argument not a box");
  dxBox *b = (dxBox*) g;
  result[0] = b->side[0];
  result[1] = b->side[1];
  result[2] = b->side[2];
}

//****************************************************************************
// capped cylinder public API

dxCCylinder::dxCCylinder (dSpaceID space, dReal _radius, dReal _length) :
  dxGeom (space,1)
{
  dAASSERT (_radius > 0 && _length > 0);
  type = dCCylinderClass;
  radius = _radius;
  lz = _length;
}


void dxCCylinder::computeAABB()
{
  dReal xrange = dFabs(R[2]  * lz) * REAL(0.5) + radius;
  dReal yrange = dFabs(R[6]  * lz) * REAL(0.5) + radius;
  dReal zrange = dFabs(R[10] * lz) * REAL(0.5) + radius;
  aabb[0] = pos[0] - xrange;
  aabb[1] = pos[0] + xrange;
  aabb[2] = pos[1] - yrange;
  aabb[3] = pos[1] + yrange;
  aabb[4] = pos[2] - zrange;
  aabb[5] = pos[2] + zrange;
}


dGeomID dCreateCCylinder (dSpaceID space, dReal radius, dReal length)
{
  return new dxCCylinder (space,radius,length);
}


void dGeomCCylinderSetParams (dGeomID g, dReal radius, dReal length)
{
  dUASSERT (g && g->type == dCCylinderClass,"argument not a ccylinder");
  dAASSERT (radius > 0 && length > 0);
  dxCCylinder *c = (dxCCylinder*) g;
  c->radius = radius;
  c->lz = length;
  dGeomMoved (g);
}


void dGeomCCylinderGetParams (dGeomID g, dReal *radius, dReal *length)
{
  dUASSERT (g && g->type == dCCylinderClass,"argument not a ccylinder");
  dxCCylinder *c = (dxCCylinder*) g;
  *radius = c->radius;
  *length = c->lz;
}

//****************************************************************************
// plane public API

static void make_sure_plane_normal_has_unit_length (dxPlane *g)
{
  dReal l = g->p[0]*g->p[0] + g->p[1]*g->p[1] + g->p[2]*g->p[2];
  if (l > 0) {
    l = dRecipSqrt(l);
    g->p[0] *= l;
    g->p[1] *= l;
    g->p[2] *= l;
    g->p[3] *= l;
  }
  else {
    g->p[0] = 1;
    g->p[1] = 0;
    g->p[2] = 0;
    g->p[3] = 0;
  }
}


dxPlane::dxPlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d) :
  dxGeom (space,0)
{
  type = dPlaneClass;
  p[0] = a;
  p[1] = b;
  p[2] = c;
  p[3] = d;
  make_sure_plane_normal_has_unit_length (this);
}


void dxPlane::computeAABB()
{
  // @@@ planes that have normal vectors aligned along an axis can use a
  // @@@ less comprehensive (half space) bounding box.
  aabb[0] = -dInfinity;
  aabb[1] = dInfinity;
  aabb[2] = -dInfinity;
  aabb[3] = dInfinity;
  aabb[4] = -dInfinity;
  aabb[5] = dInfinity;
}


dGeomID dCreatePlane (dSpaceID space,
		      dReal a, dReal b, dReal c, dReal d)
{
  return new dxPlane (space,a,b,c,d);
}


void dGeomPlaneSetParams (dGeomID g, dReal a, dReal b, dReal c, dReal d)
{
  dUASSERT (g && g->type == dPlaneClass,"argument not a plane");
  dxPlane *p = (dxPlane*) g;
  p->p[0] = a;
  p->p[1] = b;
  p->p[2] = c;
  p->p[3] = d;
  make_sure_plane_normal_has_unit_length (p);
  dGeomMoved (g);
}


void dGeomPlaneGetParams (dGeomID g, dVector4 result)
{
  dUASSERT (g && g->type == dPlaneClass,"argument not a plane");
  dxPlane *p = (dxPlane*) g;
  result[0] = p->p[0];
  result[1] = p->p[1];
  result[2] = p->p[2];
  result[3] = p->p[3];
}

//****************************************************************************
// geom group public API

enum {
  dGeomGroupClass = dSimpleSpaceClass
};


dGeomID dCreateGeomGroup (dSpaceID space)
{
  dSpaceID s = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (s,0);
  return s;
}


void dGeomGroupAdd (dxGeom *g, dxGeom *x)
{
  dUASSERT (g && g->type == dGeomGroupClass,"argument not a geomgroup");
  dSpaceAdd ((dxSpace*)g,x);
}


void dGeomGroupRemove (dxGeom *g, dxGeom *x)
{
  dUASSERT (g && g->type == dGeomGroupClass,"argument not a geomgroup");
  dSpaceRemove ((dxSpace*)g,x);
}


int dGeomGroupGetNumGeoms (dxGeom *g)
{
  dUASSERT (g && g->type == dGeomGroupClass,"argument not a geomgroup");
  return dSpaceGetNumGeoms ((dxSpace*)g);
}


dGeomID dGeomGroupGetGeom (dxGeom *g, int i)
{
  dUASSERT (g && g->type == dGeomGroupClass,"argument not a geomgroup");
  return dSpaceGetGeom ((dxSpace*)g,i);
}


int dGeomGroupQuery (dxGeom *g, dxGeom *x)
{
  dUASSERT (g && g->type == dGeomGroupClass,"argument not a geomgroup");
  return dSpaceQuery ((dxSpace*)g,x);
}

//****************************************************************************
// box-box collision utility
//
// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `code' returns a number indicating the type of contact that was detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.
//
// @@@ some stuff to optimize here, reuse code in contact point calculations.

int dBoxBox (const dVector3 p1, const dMatrix3 R1,
	     const dVector3 side1, const dVector3 p2,
	     const dMatrix3 R2, const dVector3 side2,
	     dVector3 normal, dReal *depth, int *code,
	     int maxc, dContactGeom *contact, int skip)
{
  dVector3 p,pp,normalC;
  const dReal *normalR = 0;
  dReal A1,A2,A3,B1,B2,B3,R11,R12,R13,R21,R22,R23,R31,R32,R33,
    Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,l;
  int i,invert_normal;

  // get vector from centers of box 1 to box 2, relative to box 1
  p[0] = p2[0] - p1[0];
  p[1] = p2[1] - p1[1];
  p[2] = p2[2] - p1[2];
  dMULTIPLY1_331 (pp,R1,p);		// get pp = p relative to body 1

  // get side lengths / 2
  A1 = side1[0]*REAL(0.5); A2 = side1[1]*REAL(0.5); A3 = side1[2]*REAL(0.5);
  B1 = side2[0]*REAL(0.5); B2 = side2[1]*REAL(0.5); B3 = side2[2]*REAL(0.5);

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  R11 = dDOT44(R1+0,R2+0); R12 = dDOT44(R1+0,R2+1); R13 = dDOT44(R1+0,R2+2);
  R21 = dDOT44(R1+1,R2+0); R22 = dDOT44(R1+1,R2+1); R23 = dDOT44(R1+1,R2+2);
  R31 = dDOT44(R1+2,R2+0); R32 = dDOT44(R1+2,R2+1); R33 = dDOT44(R1+2,R2+2);

  Q11 = dFabs(R11); Q12 = dFabs(R12); Q13 = dFabs(R13);
  Q21 = dFabs(R21); Q22 = dFabs(R22); Q23 = dFabs(R23);
  Q31 = dFabs(R31); Q32 = dFabs(R32); Q33 = dFabs(R33);

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

#define TEST(expr1,expr2,norm,cc) \
  s2 = dFabs(expr1) - (expr2); \
  if (s2 > 0) return 0; \
  if (s2 > s) { \
    s = s2; \
    normalR = norm; \
    invert_normal = ((expr1) < 0); \
    *code = (cc); \
  }

  s = -dInfinity;
  invert_normal = 0;
  *code = 0;

  // separating axis = u1,u2,u3
  TEST (pp[0],(A1 + B1*Q11 + B2*Q12 + B3*Q13),R1+0,1);
  TEST (pp[1],(A2 + B1*Q21 + B2*Q22 + B3*Q23),R1+1,2);
  TEST (pp[2],(A3 + B1*Q31 + B2*Q32 + B3*Q33),R1+2,3);

  // separating axis = v1,v2,v3
  TEST (dDOT41(R2+0,p),(A1*Q11 + A2*Q21 + A3*Q31 + B1),R2+0,4);
  TEST (dDOT41(R2+1,p),(A1*Q12 + A2*Q22 + A3*Q32 + B2),R2+1,5);
  TEST (dDOT41(R2+2,p),(A1*Q13 + A2*Q23 + A3*Q33 + B3),R2+2,6);

  // note: cross product axes need to be scaled when s is computed.
  // normal (n1,n2,n3) is relative to box 1.
#undef TEST
#define TEST(expr1,expr2,n1,n2,n3,cc) \
  s2 = dFabs(expr1) - (expr2); \
  if (s2 > 0) return 0; \
  l = dSqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
  if (l > 0) { \
    s2 /= l; \
    if (s2 > s) { \
      s = s2; \
      normalR = 0; \
      normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
      invert_normal = ((expr1) < 0); \
      *code = (cc); \
    } \
  }

  // separating axis = u1 x (v1,v2,v3)
  TEST(pp[2]*R21-pp[1]*R31,(A2*Q31+A3*Q21+B2*Q13+B3*Q12),0,-R31,R21,7);
  TEST(pp[2]*R22-pp[1]*R32,(A2*Q32+A3*Q22+B1*Q13+B3*Q11),0,-R32,R22,8);
  TEST(pp[2]*R23-pp[1]*R33,(A2*Q33+A3*Q23+B1*Q12+B2*Q11),0,-R33,R23,9);

  // separating axis = u2 x (v1,v2,v3)
  TEST(pp[0]*R31-pp[2]*R11,(A1*Q31+A3*Q11+B2*Q23+B3*Q22),R31,0,-R11,10);
  TEST(pp[0]*R32-pp[2]*R12,(A1*Q32+A3*Q12+B1*Q23+B3*Q21),R32,0,-R12,11);
  TEST(pp[0]*R33-pp[2]*R13,(A1*Q33+A3*Q13+B1*Q22+B2*Q21),R33,0,-R13,12);

  // separating axis = u3 x (v1,v2,v3)
  TEST(pp[1]*R11-pp[0]*R21,(A1*Q21+A2*Q11+B2*Q33+B3*Q32),-R21,R11,0,13);
  TEST(pp[1]*R12-pp[0]*R22,(A1*Q22+A2*Q12+B1*Q33+B3*Q31),-R22,R12,0,14);
  TEST(pp[1]*R13-pp[0]*R23,(A1*Q23+A2*Q13+B1*Q32+B2*Q31),-R23,R13,0,15);

#undef TEST

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.
  if (normalR) {
    normal[0] = normalR[0];
    normal[1] = normalR[4];
    normal[2] = normalR[8];
  }
  else {
    dMULTIPLY0_331 (normal,R1,normalC);
  }
  if (invert_normal) {
    normal[0] = -normal[0];
    normal[1] = -normal[1];
    normal[2] = -normal[2];
  }
  *depth = -s;

  // compute contact point(s)

  if (*code > 6) {
    // an edge from box 1 touches an edge from box 2.
    // find a point pa on the intersecting edge of box 1
    dVector3 pa;
    dReal sign;
    for (i=0; i<3; i++) pa[i] = p1[i];
    sign = (dDOT14(normal,R1+0) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) pa[i] += sign * A1 * R1[i*4];
    sign = (dDOT14(normal,R1+1) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) pa[i] += sign * A2 * R1[i*4+1];
    sign = (dDOT14(normal,R1+2) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) pa[i] += sign * A3 * R1[i*4+2];

    // find a point pb on the intersecting edge of box 2
    dVector3 pb;
    for (i=0; i<3; i++) pb[i] = p2[i];
    sign = (dDOT14(normal,R2+0) > 0) ? REAL(-1.0) : REAL(1.0);
    for (i=0; i<3; i++) pb[i] += sign * B1 * R2[i*4];
    sign = (dDOT14(normal,R2+1) > 0) ? REAL(-1.0) : REAL(1.0);
    for (i=0; i<3; i++) pb[i] += sign * B2 * R2[i*4+1];
    sign = (dDOT14(normal,R2+2) > 0) ? REAL(-1.0) : REAL(1.0);
    for (i=0; i<3; i++) pb[i] += sign * B3 * R2[i*4+2];

    dReal alpha,beta;
    dVector3 ua,ub;
    for (i=0; i<3; i++) ua[i] = R1[((*code)-7)/3 + i*4];
    for (i=0; i<3; i++) ub[i] = R2[((*code)-7)%3 + i*4];

    dLineClosestApproach (pa,ua,pb,ub,&alpha,&beta);
    for (i=0; i<3; i++) pa[i] += ua[i]*alpha;
    for (i=0; i<3; i++) pb[i] += ub[i]*beta;

    for (i=0; i<3; i++) contact[0].pos[i] = REAL(0.5)*(pa[i]+pb[i]);
    contact[0].depth = *depth;
    return 1;
  }

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face).

  // @@@ temporary: make deepest vertex on the "other" box the contact point.
  // @@@ this kind of works, but we need multiple contact points for stability,
  // @@@ especially for face-face contact.

  dVector3 vertex;
  if (*code <= 3) {
    // face from box 1 touches a vertex/edge/face from box 2.
    dReal sign;
    for (i=0; i<3; i++) vertex[i] = p2[i];
    sign = (dDOT14(normal,R2+0) > 0) ? REAL(-1.0) : REAL(1.0);
    for (i=0; i<3; i++) vertex[i] += sign * B1 * R2[i*4];
    sign = (dDOT14(normal,R2+1) > 0) ? REAL(-1.0) : REAL(1.0);
    for (i=0; i<3; i++) vertex[i] += sign * B2 * R2[i*4+1];
    sign = (dDOT14(normal,R2+2) > 0) ? REAL(-1.0) : REAL(1.0);
    for (i=0; i<3; i++) vertex[i] += sign * B3 * R2[i*4+2];
  }
  else {
    // face from box 2 touches a vertex/edge/face from box 1.
    dReal sign;
    for (i=0; i<3; i++) vertex[i] = p1[i];
    sign = (dDOT14(normal,R1+0) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) vertex[i] += sign * A1 * R1[i*4];
    sign = (dDOT14(normal,R1+1) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) vertex[i] += sign * A2 * R1[i*4+1];
    sign = (dDOT14(normal,R1+2) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) vertex[i] += sign * A3 * R1[i*4+2];
  }
  for (i=0; i<3; i++) contact[0].pos[i] = vertex[i];
  contact[0].depth = *depth;
  return 1;
}

//****************************************************************************
// pairwise collision functions for standard geom types

int dCollideSphereSphere (dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dSphereClass);
  dIASSERT (o2->type == dSphereClass);
  dxSphere *sphere1 = (dxSphere*) o1;
  dxSphere *sphere2 = (dxSphere*) o2;

  contact->g1 = o1;
  contact->g2 = o2;

  return dCollideSpheres (o1->pos,sphere1->radius,
			  o2->pos,sphere2->radius,contact);
}


int dCollideSphereBox (dxGeom *o1, dxGeom *o2, int flags,
		       dContactGeom *contact, int skip)
{
  // this is easy. get the sphere center `p' relative to the box, and then clip
  // that to the boundary of the box (call that point `q'). if q is on the
  // boundary of the box and |p-q| is <= sphere radius, they touch.
  // if q is inside the box, the sphere is inside the box, so set a contact
  // normal to push the sphere to the closest box edge.

  dVector3 l,t,p,q,r;
  dReal depth;
  int onborder = 0;

  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dSphereClass);
  dIASSERT (o2->type == dBoxClass);
  dxSphere *sphere = (dxSphere*) o1;
  dxBox *box = (dxBox*) o2;

  contact->g1 = o1;
  contact->g2 = o2;

  p[0] = o1->pos[0] - o2->pos[0];
  p[1] = o1->pos[1] - o2->pos[1];
  p[2] = o1->pos[2] - o2->pos[2];

  l[0] = box->side[0]*REAL(0.5);
  t[0] = dDOT14(p,o2->R);
  if (t[0] < -l[0]) { t[0] = -l[0]; onborder = 1; }
  if (t[0] >  l[0]) { t[0] =  l[0]; onborder = 1; }

  l[1] = box->side[1]*REAL(0.5);
  t[1] = dDOT14(p,o2->R+1);
  if (t[1] < -l[1]) { t[1] = -l[1]; onborder = 1; }
  if (t[1] >  l[1]) { t[1] =  l[1]; onborder = 1; }

  t[2] = dDOT14(p,o2->R+2);
  l[2] = box->side[2]*REAL(0.5);
  if (t[2] < -l[2]) { t[2] = -l[2]; onborder = 1; }
  if (t[2] >  l[2]) { t[2] =  l[2]; onborder = 1; }

  if (!onborder) {
    // sphere center inside box. find largest `t' value
    dReal max = dFabs(t[0]);
    int maxi = 0;
    for (int i=1; i<3; i++) {
      dReal tt = dFabs(t[i]);
      if (tt > max) {
	max = tt;
	maxi = i;
      }
    }
    // contact position = sphere center
    contact->pos[0] = o1->pos[0];
    contact->pos[1] = o1->pos[1];
    contact->pos[2] = o1->pos[2];
    // contact normal aligned with box edge along largest `t' value
    dVector3 tmp;
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[maxi] = (t[maxi] > 0) ? REAL(1.0) : REAL(-1.0);
    dMULTIPLY0_331 (contact->normal,o2->R,tmp);
    // contact depth = distance to wall along normal plus radius
    contact->depth = l[maxi] - max + sphere->radius;
    return 1;
  }

  t[3] = 0;			//@@@ hmmm
  dMULTIPLY0_331 (q,o2->R,t);
  r[0] = p[0] - q[0];
  r[1] = p[1] - q[1];
  r[2] = p[2] - q[2];
  depth = sphere->radius - dSqrt(dDOT(r,r));
  if (depth < 0) return 0;
  contact->pos[0] = q[0] + o2->pos[0];
  contact->pos[1] = q[1] + o2->pos[1];
  contact->pos[2] = q[2] + o2->pos[2];
  contact->normal[0] = r[0];
  contact->normal[1] = r[1];
  contact->normal[2] = r[2];
  dNormalize3 (contact->normal);
  contact->depth = depth;
  return 1;
}


int dCollideSpherePlane (dxGeom *o1, dxGeom *o2, int flags,
			 dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dSphereClass);
  dIASSERT (o2->type == dPlaneClass);
  dxSphere *sphere = (dxSphere*) o1;
  dxPlane *plane = (dxPlane*) o2;

  contact->g1 = o1;
  contact->g2 = o2;
  dReal k = dDOT (o1->pos,plane->p);
  dReal depth = plane->p[3] - k + sphere->radius;
  if (depth >= 0) {
    contact->normal[0] = plane->p[0];
    contact->normal[1] = plane->p[1];
    contact->normal[2] = plane->p[2];
    contact->pos[0] = o1->pos[0] - plane->p[0] * sphere->radius;
    contact->pos[1] = o1->pos[1] - plane->p[1] * sphere->radius;
    contact->pos[2] = o1->pos[2] - plane->p[2] * sphere->radius;
    contact->depth = depth;
    return 1;
  }
  else return 0;
}


int dCollideBoxBox (dxGeom *o1, dxGeom *o2, int flags,
		    dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dBoxClass);
  dIASSERT (o2->type == dBoxClass);
  dVector3 normal;
  dReal depth;
  int code;
  dxBox *b1 = (dxBox*) o1;
  dxBox *b2 = (dxBox*) o2;
  int num = dBoxBox (o1->pos,o1->R,b1->side, o2->pos,o2->R,b2->side,
		     normal,&depth,&code,flags & NUMC_MASK,contact,skip);
  for (int i=0; i<num; i++) {
    CONTACT(contact,i*skip)->normal[0] = -normal[0];
    CONTACT(contact,i*skip)->normal[1] = -normal[1];
    CONTACT(contact,i*skip)->normal[2] = -normal[2];
    CONTACT(contact,i*skip)->g1 = o1;
    CONTACT(contact,i*skip)->g2 = o2;
  }
  return num;
}


int dCollideBoxPlane (dxGeom *o1, dxGeom *o2,
		      int flags, dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dBoxClass);
  dIASSERT (o2->type == dPlaneClass);
  dxBox *box = (dxBox*) o1;
  dxPlane *plane = (dxPlane*) o2;

  contact->g1 = o1;
  contact->g2 = o2;
  int ret = 0;

  //@@@ problem: using 4-vector (plane->p) as 3-vector (normal).
  const dReal *R = o1->R;		// rotation of box
  const dReal *n = plane->p;		// normal vector

  // project sides lengths along normal vector, get absolute values
  dReal Q1 = dDOT14(n,R+0);
  dReal Q2 = dDOT14(n,R+1);
  dReal Q3 = dDOT14(n,R+2);
  dReal A1 = box->side[0] * Q1;
  dReal A2 = box->side[1] * Q2;
  dReal A3 = box->side[2] * Q3;
  dReal B1 = dFabs(A1);
  dReal B2 = dFabs(A2);
  dReal B3 = dFabs(A3);

  // early exit test
  dReal depth = plane->p[3] + REAL(0.5)*(B1+B2+B3) - dDOT(n,o1->pos);
  if (depth < 0) return 0;

  // find number of contacts requested
  int maxc = flags & NUMC_MASK;
  if (maxc < 1) maxc = 1;
  if (maxc > 3) maxc = 3;	// no more than 3 contacts per box allowed

  // find deepest point
  dVector3 p;
  p[0] = o1->pos[0];
  p[1] = o1->pos[1];
  p[2] = o1->pos[2];
#define FOO(i,op) \
  p[0] op REAL(0.5)*box->side[i] * R[0+i]; \
  p[1] op REAL(0.5)*box->side[i] * R[4+i]; \
  p[2] op REAL(0.5)*box->side[i] * R[8+i];
#define BAR(i,iinc) if (A ## iinc > 0) { FOO(i,-=) } else { FOO(i,+=) }
  BAR(0,1);
  BAR(1,2);
  BAR(2,3);
#undef FOO
#undef BAR

  // the deepest point is the first contact point
  contact->pos[0] = p[0];
  contact->pos[1] = p[1];
  contact->pos[2] = p[2];
  contact->normal[0] = n[0];
  contact->normal[1] = n[1];
  contact->normal[2] = n[2];
  contact->depth = depth;
  ret = 1;		// ret is number of contact points found so far
  if (maxc == 1) goto done;

  // get the second and third contact points by starting from `p' and going
  // along the two sides with the smallest projected length.

#define FOO(i,j,op) \
  CONTACT(contact,i*skip)->pos[0] = p[0] op box->side[j] * R[0+j]; \
  CONTACT(contact,i*skip)->pos[1] = p[1] op box->side[j] * R[4+j]; \
  CONTACT(contact,i*skip)->pos[2] = p[2] op box->side[j] * R[8+j];
#define BAR(ctact,side,sideinc) \
  depth -= B ## sideinc; \
  if (depth < 0) goto done; \
  if (A ## sideinc > 0) { FOO(ctact,side,+) } else { FOO(ctact,side,-) } \
  CONTACT(contact,ctact*skip)->depth = depth; \
  ret++;

  CONTACT(contact,skip)->normal[0] = n[0];
  CONTACT(contact,skip)->normal[1] = n[1];
  CONTACT(contact,skip)->normal[2] = n[2];
  if (maxc == 3) {
    CONTACT(contact,2*skip)->normal[0] = n[0];
    CONTACT(contact,2*skip)->normal[1] = n[1];
    CONTACT(contact,2*skip)->normal[2] = n[2];
  }

  if (B1 < B2) {
    if (B3 < B1) goto use_side_3; else {
      BAR(1,0,1);	// use side 1
      if (maxc == 2) goto done;
      if (B2 < B3) goto contact2_2; else goto contact2_3;
    }
  }
  else {
    if (B3 < B2) {
      use_side_3:	// use side 3
      BAR(1,2,3);
      if (maxc == 2) goto done;
      if (B1 < B2) goto contact2_1; else goto contact2_2;
    }
    else {
      BAR(1,1,2);	// use side 2
      if (maxc == 2) goto done;
      if (B1 < B3) goto contact2_1; else goto contact2_3;
    }
  }

  contact2_1: BAR(2,0,1); goto done;
  contact2_2: BAR(2,1,2); goto done;
  contact2_3: BAR(2,2,3); goto done;
#undef FOO
#undef BAR

 done:
  for (int i=0; i<ret; i++) {
    CONTACT(contact,i*skip)->g1 = o1;
    CONTACT(contact,i*skip)->g2 = o2;
  }
  return ret;
}


int dCollideCCylinderSphere (dxGeom *o1, dxGeom *o2, int flags,
			     dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dCCylinderClass);
  dIASSERT (o2->type == dSphereClass);
  dxCCylinder *ccyl = (dxCCylinder*) o1;
  dxSphere *sphere = (dxSphere*) o2;

  contact->g1 = o1;
  contact->g2 = o2;

  // find the point on the cylinder axis that is closest to the sphere
  dReal alpha = 
    o1->R[2]  * (o2->pos[0] - o1->pos[0]) +
    o1->R[6]  * (o2->pos[1] - o1->pos[1]) +
    o1->R[10] * (o2->pos[2] - o1->pos[2]);
  dReal lz2 = ccyl->lz * REAL(0.5);
  if (alpha > lz2) alpha = lz2;
  if (alpha < -lz2) alpha = -lz2;

  // collide the spheres
  dVector3 p;
  p[0] = o1->pos[0] + alpha * o1->R[2];
  p[1] = o1->pos[1] + alpha * o1->R[6];
  p[2] = o1->pos[2] + alpha * o1->R[10];
  return dCollideSpheres (p,ccyl->radius,o2->pos,sphere->radius,contact);
}


int dCollideCCylinderBox (dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dCCylinderClass);
  dIASSERT (o2->type == dBoxClass);
  dxCCylinder *cyl = (dxCCylinder*) o1;
  dxBox *box = (dxBox*) o2;

  contact->g1 = o1;
  contact->g2 = o2;

  // get p1,p2 = cylinder axis endpoints, get radius
  dVector3 p1,p2;
  dReal clen = cyl->lz * REAL(0.5);
  p1[0] = o1->pos[0] + clen * o1->R[2];
  p1[1] = o1->pos[1] + clen * o1->R[6];
  p1[2] = o1->pos[2] + clen * o1->R[10];
  p2[0] = o1->pos[0] - clen * o1->R[2];
  p2[1] = o1->pos[1] - clen * o1->R[6];
  p2[2] = o1->pos[2] - clen * o1->R[10];
  dReal radius = cyl->radius;

  // copy out box center, rotation matrix, and side array
  dReal *c = o2->pos;
  dReal *R = o2->R;
  const dReal *side = box->side;

  // get the closest point between the cylinder axis and the box
  dVector3 pl,pb;
  dClosestLineBoxPoints (p1,p2,c,R,side,pl,pb);

  // generate contact point
  return dCollideSpheres (pl,radius,pb,0,contact);
}


// this returns at most one contact point when the two cylinder's axes are not
// aligned, and at most two (for stability) when they are aligned.
// the algorithm minimizes the distance between two "sample spheres" that are
// positioned along the cylinder axes according to:
//    sphere1 = pos1 + alpha1 * axis1
//    sphere2 = pos2 + alpha2 * axis2
// alpha1 and alpha2 are limited to +/- half the length of the cylinders.
// the algorithm works by finding a solution that has both alphas free, or
// a solution that has one or both alphas fixed to the ends of the cylinder.

int dCollideCCylinderCCylinder (dxGeom *o1, dxGeom *o2,
				int flags, dContactGeom *contact, int skip)
{
  int i;
  const dReal tolerance = REAL(1e-5);

  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dCCylinderClass);
  dIASSERT (o2->type == dCCylinderClass);
  dxCCylinder *cyl1 = (dxCCylinder*) o1;
  dxCCylinder *cyl2 = (dxCCylinder*) o2;

  contact->g1 = o1;
  contact->g2 = o2;

  // copy out some variables, for convenience
  dReal lz1 = cyl1->lz * REAL(0.5);
  dReal lz2 = cyl2->lz * REAL(0.5);
  dReal *pos1 = o1->pos;
  dReal *pos2 = o2->pos;
  dReal axis1[3],axis2[3];
  axis1[0] = o1->R[2];
  axis1[1] = o1->R[6];
  axis1[2] = o1->R[10];
  axis2[0] = o2->R[2];
  axis2[1] = o2->R[6];
  axis2[2] = o2->R[10];

  dReal alpha1,alpha2,sphere1[3],sphere2[3];
  int fix1 = 0;		// 0 if alpha1 is free, +/-1 to fix at +/- lz1
  int fix2 = 0;		// 0 if alpha2 is free, +/-1 to fix at +/- lz2

  for (int count=0; count<9; count++) {
    // find a trial solution by fixing or not fixing the alphas
    if (fix1) {
      if (fix2) {
	// alpha1 and alpha2 are fixed, so the solution is easy
	if (fix1 > 0) alpha1 = lz1; else alpha1 = -lz1;
	if (fix2 > 0) alpha2 = lz2; else alpha2 = -lz2;
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
      }
      else {
	// fix alpha1 but let alpha2 be free
	if (fix1 > 0) alpha1 = lz1; else alpha1 = -lz1;
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	alpha2 = (axis2[0]*(sphere1[0]-pos2[0]) +
		  axis2[1]*(sphere1[1]-pos2[1]) +
		  axis2[2]*(sphere1[2]-pos2[2]));
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
      }
    }
    else {
      if (fix2) {
	// fix alpha2 but let alpha1 be free
	if (fix2 > 0) alpha2 = lz2; else alpha2 = -lz2;
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
	alpha1 = (axis1[0]*(sphere2[0]-pos1[0]) +
		  axis1[1]*(sphere2[1]-pos1[1]) +
		  axis1[2]*(sphere2[2]-pos1[2]));
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
      }
      else {
	// let alpha1 and alpha2 be free
	// compute determinant of d(d^2)\d(alpha) jacobian
	dReal a1a2 = dDOT (axis1,axis2);
	dReal det = REAL(1.0)-a1a2*a1a2;
	if (det < tolerance) {
	  // the cylinder axes (almost) parallel, so we will generate up to two
	  // contacts. the solution matrix is rank deficient so alpha1 and
	  // alpha2 are related by:
	  //       alpha2 =   alpha1 + (pos1-pos2)'*axis1   (if axis1==axis2)
	  //    or alpha2 = -(alpha1 + (pos1-pos2)'*axis1)  (if axis1==-axis2)
	  // first compute where the two cylinders overlap in alpha1 space:
	  if (a1a2 < 0) {
	    axis2[0] = -axis2[0];
	    axis2[1] = -axis2[1];
	    axis2[2] = -axis2[2];
	  }
	  dReal q[3];
	  for (i=0; i<3; i++) q[i] = pos1[i]-pos2[i];
	  dReal k = dDOT (axis1,q);
	  dReal a1lo = -lz1;
	  dReal a1hi = lz1;
	  dReal a2lo = -lz2 - k;
	  dReal a2hi = lz2 - k;
	  dReal lo = (a1lo > a2lo) ? a1lo : a2lo;
	  dReal hi = (a1hi < a2hi) ? a1hi : a2hi;
	  if (lo <= hi) {
	    int num_contacts = flags & NUMC_MASK;
	    if (num_contacts >= 2 && lo < hi) {
	      // generate up to two contacts. if one of those contacts is
	      // not made, fall back on the one-contact strategy.
	      for (i=0; i<3; i++) sphere1[i] = pos1[i] + lo*axis1[i];
	      for (i=0; i<3; i++) sphere2[i] = pos2[i] + (lo+k)*axis2[i];
	      int n1 = dCollideSpheres (sphere1,cyl1->radius,
					sphere2,cyl2->radius,contact);
	      if (n1) {
		for (i=0; i<3; i++) sphere1[i] = pos1[i] + hi*axis1[i];
		for (i=0; i<3; i++) sphere2[i] = pos2[i] + (hi+k)*axis2[i];
		dContactGeom *c2 = CONTACT(contact,skip);
		int n2 = dCollideSpheres (sphere1,cyl1->radius,
					  sphere2,cyl2->radius, c2);
		if (n2) {
		  c2->g1 = o1;
		  c2->g2 = o2;
		  return 2;
		}
	      }
	    }

	    // just one contact to generate, so put it in the middle of
	    // the range
	    alpha1 = (lo + hi) * REAL(0.5);
	    alpha2 = alpha1 + k;
	    for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	    for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
	    return dCollideSpheres (sphere1,cyl1->radius,
				    sphere2,cyl2->radius,contact);
	  }
	  else return 0;
	}
	det = REAL(1.0)/det;
	dReal delta[3];
	for (i=0; i<3; i++) delta[i] = pos1[i] - pos2[i];
	dReal q1 = dDOT (delta,axis1);
	dReal q2 = dDOT (delta,axis2);
	alpha1 = det*(a1a2*q2-q1);
	alpha2 = det*(q2-a1a2*q1);
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
      }
    }

    // if the alphas are outside their allowed ranges then fix them and
    // try again
    if (fix1==0) {
      if (alpha1 < -lz1) {
	fix1 = -1;
	continue;
      }
      if (alpha1 > lz1) {
	fix1 = 1;
	continue;
      }
    }
    if (fix2==0) {
      if (alpha2 < -lz2) {
	fix2 = -1;
	continue;
      }
      if (alpha2 > lz2) {
	fix2 = 1;
	continue;
      }
    }

    // unfix the alpha variables if the local distance gradient indicates
    // that we are not yet at the minimum
    dReal tmp[3];
    for (i=0; i<3; i++) tmp[i] = sphere1[i] - sphere2[i];
    if (fix1) {
      dReal gradient = dDOT (tmp,axis1);
      if ((fix1 > 0 && gradient > 0) || (fix1 < 0 && gradient < 0)) {
	fix1 = 0;
	continue;
      }
    }
    if (fix2) {
      dReal gradient = -dDOT (tmp,axis2);
      if ((fix2 > 0 && gradient > 0) || (fix2 < 0 && gradient < 0)) {
	fix2 = 0;
	continue;
      }
    }
    return dCollideSpheres (sphere1,cyl1->radius,sphere2,cyl2->radius,contact);
  }
  // if we go through the loop too much, then give up. we should NEVER get to
  // this point (i hope).
  dMessage (0,"dCollideCC(): too many iterations");
  return 0;
}


int dCollideCCylinderPlane (dxGeom *o1, dxGeom *o2, int flags,
			    dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dCCylinderClass);
  dIASSERT (o2->type == dPlaneClass);
  dxCCylinder *ccyl = (dxCCylinder*) o1;
  dxPlane *plane = (dxPlane*) o2;

  // collide the deepest capping sphere with the plane
  dReal sign = (dDOT14 (plane->p,o1->R+2) > 0) ? REAL(-1.0) : REAL(1.0);
  dVector3 p;
  p[0] = o1->pos[0] + o1->R[2]  * ccyl->lz * REAL(0.5) * sign;
  p[1] = o1->pos[1] + o1->R[6]  * ccyl->lz * REAL(0.5) * sign;
  p[2] = o1->pos[2] + o1->R[10] * ccyl->lz * REAL(0.5) * sign;

  dReal k = dDOT (p,plane->p);
  dReal depth = plane->p[3] - k + ccyl->radius;
  if (depth < 0) return 0;
  contact->normal[0] = plane->p[0];
  contact->normal[1] = plane->p[1];
  contact->normal[2] = plane->p[2];
  contact->pos[0] = p[0] - plane->p[0] * ccyl->radius;
  contact->pos[1] = p[1] - plane->p[1] * ccyl->radius;
  contact->pos[2] = p[2] - plane->p[2] * ccyl->radius;
  contact->depth = depth;

  int ncontacts = 1;
  if ((flags & NUMC_MASK) >= 2) {
    // collide the other capping sphere with the plane
    p[0] = o1->pos[0] - o1->R[2]  * ccyl->lz * REAL(0.5) * sign;
    p[1] = o1->pos[1] - o1->R[6]  * ccyl->lz * REAL(0.5) * sign;
    p[2] = o1->pos[2] - o1->R[10] * ccyl->lz * REAL(0.5) * sign;

    k = dDOT (p,plane->p);
    depth = plane->p[3] - k + ccyl->radius;
    if (depth >= 0) {
      dContactGeom *c2 = CONTACT(contact,skip);
      c2->normal[0] = plane->p[0];
      c2->normal[1] = plane->p[1];
      c2->normal[2] = plane->p[2];
      c2->pos[0] = p[0] - plane->p[0] * ccyl->radius;
      c2->pos[1] = p[1] - plane->p[1] * ccyl->radius;
      c2->pos[2] = p[2] - plane->p[2] * ccyl->radius;
      c2->depth = depth;
      ncontacts = 2;
    }
  }

  for (int i=0; i < ncontacts; i++) {
    CONTACT(contact,i*skip)->g1 = o1;
    CONTACT(contact,i*skip)->g2 = o2;
  }
  return ncontacts;
}
