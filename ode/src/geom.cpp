/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001 Russell L. Smith.            *
 *   Email: russ@q12.org   Web: www.q12.org                              *
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

/*

TODO

* for objects that are never meant to collide, dCollide() will always try
  to find the collider functions, which wastes a bit of time.

* get rid of global constructors here, they wont work in C linkage.

*/

#include "ode/geom.h"
#include "ode/rotation.h"
#include "ode/odemath.h"
#include "ode/memory.h"
#include "ode/misc.h"
#include "ode/objects.h"
#include "ode/matrix.h"
#include "objects.h"
#include "array.h"
#include "geom_internal.h"

//****************************************************************************
// collision utilities

// given a pointer `p' to a dContactGeom, return the dContactGeom at
// p + skip bytes.

#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))


// if the spheres (p1,r1) and (p2,r2) collide, set the contact `c' and
// return 1, else return 0.

static int dCollideSpheres (dVector3 p1, dReal r1,
			    dVector3 p2, dReal r2, dContactGeom *c)
{
  // printf ("d=%.2f  (%.2f %.2f %.2f) (%.2f %.2f %.2f) r1=%.2f r2=%.2f\n",
  //	  d,p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],r1,r2);

  dReal d = dDISTANCE (p1,p2);
  if (d > (r1 + r2)) return 0;
  if (d <= 0) {
    c->pos[0] = p1[0];
    c->pos[1] = p1[1];
    c->pos[2] = p1[2];
    c->normal[0] = 1;
    c->normal[1] = 0;
    c->normal[2] = 0;
    c->depth = r1 + r2;
  }
  else {
    dReal d1 = dRecip (d);
    c->normal[0] = (p1[0]-p2[0])*d1;
    c->normal[1] = (p1[1]-p2[1])*d1;
    c->normal[2] = (p1[2]-p2[2])*d1;
    dReal k = REAL(0.5) * (r2 - r1 - d);
    c->pos[0] = p1[0] + c->normal[0]*k;
    c->pos[1] = p1[1] + c->normal[1]*k;
    c->pos[2] = p1[2] + c->normal[2]*k;
    c->depth = r1 + r2 - d;
  }
  return 1;
}


// given two lines
//    qa = pa + alpha* ua
//    qb = pb + beta * ub
// where pa,pb are two points, ua,ub are two unit length vectors, and alpha,
// beta go from [-inf,inf], return alpha and beta such that qa and qb are
// as close as possible

static void lineClosestApproach (const dVector3 pa, const dVector3 ua,
				 const dVector3 pb, const dVector3 ub,
				 dReal *alpha, dReal *beta)
{
  dVector3 p;
  p[0] = pb[0] - pa[0];
  p[1] = pb[1] - pa[1];
  p[2] = pb[2] - pa[2];
  dReal uaub = dDOT(ua,ub);
  dReal q1 =  dDOT(ua,p);
  dReal q2 = -dDOT(ub,p);
  dReal d = 1-uaub*uaub;
  if (d <= 0) {
    // @@@ this needs to be made more robust
    *alpha = 0;
    *beta  = 0;
  }
  else {
    d = dRecip(d);
    *alpha = (q1 + uaub*q2)*d;
    *beta  = (uaub*q1 + q2)*d;
  }
}


// given a box (R,side), `R' is the rotation matrix for the box, and `side'
// is a vector of x/y/z side lengths, return the size of the interval of the
// box projected along the given axis. if the axis has unit length then the
// return value will be the actual diameter, otherwise the result will be
// scaled by the axis length.

static inline dReal boxDiameter (const dMatrix3 R, const dVector3 side,
				 const dVector3 axis)
{
  dVector3 q;
  dMULTIPLY1_331 (q,R,axis);	// transform axis to body-relative
  return dFabs(q[0])*side[0] + dFabs(q[1])*side[1] + dFabs(q[2])*side[2];
}


// given boxes (p1,R1,side1) and (p1,R1,side1), return 1 if they intersect
// or 0 if not.

int dBoxTouchesBox (const dVector3 p1, const dMatrix3 R1,
		    const dVector3 side1, const dVector3 p2,
		    const dMatrix3 R2, const dVector3 side2)
{
  // two boxes are disjoint if (and only if) there is a separating axis
  // perpendicular to a face from one box or perpendicular to an edge from
  // either box. the following tests are derived from:
  //    "OBB Tree: A Hierarchical Structure for Rapid Interference Detection",
  //    S.Gottschalk, M.C.Lin, D.Manocha., Proc of ACM Siggraph 1996.

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2.
  // Qij is abs(Rij)
  dVector3 p,pp;
  dReal A1,A2,A3,B1,B2,B3,R11,R12,R13,R21,R22,R23,R31,R32,R33,
    Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33;

  // get vector from centers of box 1 to box 2, relative to box 1
  p[0] = p2[0] - p1[0];
  p[1] = p2[1] - p1[1];
  p[2] = p2[2] - p1[2];
  dMULTIPLY1_331 (pp,R1,p);		// get pp = p relative to body 1

  // get side lengths / 2
  A1 = side1[0]*0.5; A2 = side1[1]*0.5; A3 = side1[2]*0.5;
  B1 = side2[0]*0.5; B2 = side2[1]*0.5; B3 = side2[2]*0.5;

  // for the following tests, excluding computation of Rij, in the worst case,
  // 15 compares, 60 adds, 81 multiplies, and 24 absolutes.
  // notation: R1=[u1 u2 u3], R2=[v1 v2 v3]

  // separating axis = u1,u2,u3
  R11 = dDOT44(R1+0,R2+0); R12 = dDOT44(R1+0,R2+1); R13 = dDOT44(R1+0,R2+2);
  Q11 = dFabs(R11); Q12 = dFabs(R12); Q13 = dFabs(R13);
  if (dFabs(pp[0]) > (A1 + B1*Q11 + B2*Q12 + B3*Q13)) return 0;
  R21 = dDOT44(R1+1,R2+0); R22 = dDOT44(R1+1,R2+1); R23 = dDOT44(R1+1,R2+2);
  Q21 = dFabs(R21); Q22 = dFabs(R22); Q23 = dFabs(R23);
  if (dFabs(pp[1]) > (A2 + B1*Q21 + B2*Q22 + B3*Q23)) return 0;
  R31 = dDOT44(R1+2,R2+0); R32 = dDOT44(R1+2,R2+1); R33 = dDOT44(R1+2,R2+2);
  Q31 = dFabs(R31); Q32 = dFabs(R32); Q33 = dFabs(R33);
  if (dFabs(pp[2]) > (A3 + B1*Q31 + B2*Q32 + B3*Q33)) return 0;

  // separating axis = v1,v2,v3
  if (dFabs(dDOT41(R2+0,p)) > (A1*Q11 + A2*Q21 + A3*Q31 + B1)) return 0;
  if (dFabs(dDOT41(R2+1,p)) > (A1*Q12 + A2*Q22 + A3*Q32 + B2)) return 0;
  if (dFabs(dDOT41(R2+2,p)) > (A1*Q13 + A2*Q23 + A3*Q33 + B3)) return 0;

  // separating axis = u1 x (v1,v2,v3)
  if (dFabs(pp[2]*R21-pp[1]*R31) > A2*Q31 + A3*Q21 + B2*Q13 + B3*Q12) return 0;
  if (dFabs(pp[2]*R22-pp[1]*R32) > A2*Q32 + A3*Q22 + B1*Q13 + B3*Q11) return 0;
  if (dFabs(pp[2]*R23-pp[1]*R33) > A2*Q33 + A3*Q23 + B1*Q12 + B2*Q11) return 0;

  // separating axis = u2 x (v1,v2,v3)
  if (dFabs(pp[0]*R31-pp[2]*R11) > A1*Q31 + A3*Q11 + B2*Q23 + B3*Q22) return 0;
  if (dFabs(pp[0]*R32-pp[2]*R12) > A1*Q32 + A3*Q12 + B1*Q23 + B3*Q21) return 0;
  if (dFabs(pp[0]*R33-pp[2]*R13) > A1*Q33 + A3*Q13 + B1*Q22 + B2*Q21) return 0;

  // separating axis = u3 x (v1,v2,v3)
  if (dFabs(pp[1]*R11-pp[0]*R21) > A1*Q21 + A2*Q11 + B2*Q33 + B3*Q32) return 0;
  if (dFabs(pp[1]*R12-pp[0]*R22) > A1*Q22 + A2*Q12 + B1*Q33 + B3*Q31) return 0;
  if (dFabs(pp[1]*R13-pp[0]*R23) > A1*Q23 + A2*Q13 + B1*Q32 + B2*Q31) return 0;

  return 1;
}


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

extern "C" int dBoxBox (const dVector3 p1, const dMatrix3 R1,
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
  A1 = side1[0]*0.5; A2 = side1[1]*0.5; A3 = side1[2]*0.5;
  B1 = side2[0]*0.5; B2 = side2[1]*0.5; B3 = side2[2]*0.5;

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
    sign = (dDOT14(normal,R1+0) > 0) ? 1 : -1;
    for (i=0; i<3; i++) pa[i] += sign * A1 * R1[i*4];
    sign = (dDOT14(normal,R1+1) > 0) ? 1 : -1;
    for (i=0; i<3; i++) pa[i] += sign * A2 * R1[i*4+1];
    sign = (dDOT14(normal,R1+2) > 0) ? 1 : -1;
    for (i=0; i<3; i++) pa[i] += sign * A3 * R1[i*4+2];

    // find a point pb on the intersecting edge of box 2
    dVector3 pb;
    for (i=0; i<3; i++) pb[i] = p2[i];
    sign = (dDOT14(normal,R2+0) > 0) ? -1 : 1;
    for (i=0; i<3; i++) pb[i] += sign * B1 * R2[i*4];
    sign = (dDOT14(normal,R2+1) > 0) ? -1 : 1;
    for (i=0; i<3; i++) pb[i] += sign * B2 * R2[i*4+1];
    sign = (dDOT14(normal,R2+2) > 0) ? -1 : 1;
    for (i=0; i<3; i++) pb[i] += sign * B3 * R2[i*4+2];

    dReal alpha,beta;
    dVector3 ua,ub;
    for (i=0; i<3; i++) ua[i] = R1[((*code)-7)/3 + i*4];
    for (i=0; i<3; i++) ub[i] = R2[((*code)-7)%3 + i*4];

    lineClosestApproach (pa,ua,pb,ub,&alpha,&beta);
    for (i=0; i<3; i++) pa[i] += ua[i]*alpha;
    for (i=0; i<3; i++) pb[i] += ub[i]*beta;

    for (i=0; i<3; i++) contact[0].pos[i] = 0.5*(pa[i]+pb[i]);
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
    sign = (dDOT14(normal,R2+0) > 0) ? -1 : 1;
    for (i=0; i<3; i++) vertex[i] += sign * B1 * R2[i*4];
    sign = (dDOT14(normal,R2+1) > 0) ? -1 : 1;
    for (i=0; i<3; i++) vertex[i] += sign * B2 * R2[i*4+1];
    sign = (dDOT14(normal,R2+2) > 0) ? -1 : 1;
    for (i=0; i<3; i++) vertex[i] += sign * B3 * R2[i*4+2];
  }
  else {
    // face from box 2 touches a vertex/edge/face from box 1.
    dReal sign;
    for (i=0; i<3; i++) vertex[i] = p1[i];
    sign = (dDOT14(normal,R1+0) > 0) ? 1 : -1;
    for (i=0; i<3; i++) vertex[i] += sign * A1 * R1[i*4];
    sign = (dDOT14(normal,R1+1) > 0) ? 1 : -1;
    for (i=0; i<3; i++) vertex[i] += sign * A2 * R1[i*4+1];
    sign = (dDOT14(normal,R1+2) > 0) ? 1 : -1;
    for (i=0; i<3; i++) vertex[i] += sign * A3 * R1[i*4+2];
  }
  for (i=0; i<3; i++) contact[0].pos[i] = vertex[i];
  contact[0].depth = *depth;
  return 1;
}

//****************************************************************************
// general support for geometry objects and classes

struct dColliderEntry {
  dColliderFn *fn;	// collider function
  int mode;		// 1 = reverse o1 and o2, 2 = no function available
};

static dArray<dxGeomClass*> *classes=0;

// function pointers and modes for n^2 class collider functions. this is an
// n*n matrix stored by row. the functions pointers are extracted from the
// class get-collider-function function.
static dArray<dColliderEntry> *colliders=0;


static inline void initCollisionArrays()
{
  if (classes==0) {
    // old way:
    //   classes = (dArray<dxGeomClass*> *) dAllocNoFree (sizeof(dArrayBase));
    //   classes->constructor();
    classes = new dArray<dxGeomClass*>;
    classes->setSize (1);
    dAllocDontReport (classes);
    dAllocDontReport (classes->data());
  }
  if (colliders==0) {
    // old way:
    //   colliders=(dArray<dColliderEntry> *)dAllocNoFree (sizeof(dArrayBase));
    //   colliders->constructor();
    colliders = new dArray<dColliderEntry>;
    colliders->setSize (1);
    dAllocDontReport (colliders);
    dAllocDontReport (colliders->data());
  }
}


int dCreateGeomClass (const dGeomClass *c)
{
  dUASSERT(c && c->bytes >= 0 && c->collider && c->aabb,"bad geom class");
  initCollisionArrays();

  int n = classes->size();
  dxGeomClass *gc = (dxGeomClass*) dAlloc (sizeof(dxGeomClass));
  dAllocDontReport (gc);
  gc->collider = c->collider;
  gc->aabb = c->aabb;
  gc->num = n;
  gc->size = SIZEOF_DXGEOM + c->bytes;
  classes->push (gc);

  // make room for n^2 class collider function pointers - these entries will
  // be filled as dCollide() is called.
  colliders->setSize ((n+1)*(n+1));
  memset (colliders->data(),0,(n+1)*(n+1)*sizeof(dColliderEntry));

  return n;
}


int dCollide (dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact,
	      int skip)
{
  int i,c1,c2,a1,a2,count,swap;
  dColliderFn *fn;
  dAASSERT(o1 && o2 && contact);
  dUASSERT(classes && colliders,"no registered geometry classes");
  dColliderEntry *colliders2 = colliders->data();
  c1 = o1->_class->num;
  c2 = o2->_class->num;
  a1 = c1 * classes->size() + c2;	// address 1 in collider array
  a2 = c2 * classes->size() + c1;	// address 2 in collider array
  swap = 0;		// set to 1 to swap normals before returning

  // return if there are no collider functions available
  if ((colliders2[a1].mode==2) || (colliders2[a2].mode==2)) return 0;

  if ((fn = colliders2[a1].fn)) {
    swap = colliders2[a1].mode;
    if (swap) count = (*fn) (o2,o1,flags,contact,skip);
    else count = (*fn) (o1,o2,flags,contact,skip);
  }
  else if ((fn = (*classes)[c1]->collider (c2))) {
    colliders2 [a2].fn = fn;
    colliders2 [a2].mode = 1;
    colliders2 [a1].fn = fn;	// do mode=0 assignment second so that
    colliders2 [a1].mode = 0;	// diagonal entries will have mode 0
    count = (*fn) (o1,o2,flags,contact,skip);
    swap = 0;
  }
  else if ((fn = (*classes)[c2]->collider (c1))) {
    colliders2 [a1].fn = fn;
    colliders2 [a1].mode = 1;
    colliders2 [a2].fn = fn;	// do mode=0 assignment second so that
    colliders2 [a2].mode = 0;	// diagonal entries will have mode 0
    count = (*fn) (o2,o1,flags,contact,skip);
    swap = 1;
  }
  else {
    colliders2[a1].mode = 2;
    colliders2[a2].mode = 2;
    return 0;
  }

  if (swap) {
    for (i=0; i<count; i++) {
      contact[i].normal[0] = -contact[i].normal[0];
      contact[i].normal[1] = -contact[i].normal[1];
      contact[i].normal[2] = -contact[i].normal[2];
    }
  }

  return count;
}


int dGeomGetClass (dxGeom *g)
{
  dAASSERT (g);
  return g->_class->num;
}


void dGeomSetData (dxGeom *g, void *data)
{
  dAASSERT (g);
  g->data = data;
}


void *dGeomGetData (dxGeom *g)
{
  dAASSERT (g);
  return g->data;
}


void dGeomSetBody (dxGeom *g, dBodyID b)
{
  dAASSERT (g);
  if (b) {
    if (!g->body) dFree (g->pos,sizeof(dxPosR));
    g->body = b;
    g->pos = b->pos;
    g->R = b->R;
  }
  else {
    if (g->body) {
      g->body = 0;
      dxPosR *pr = (dxPosR*) dAlloc (sizeof(dxPosR));
      g->pos = pr->pos;
      g->R = pr->R;
    }
  }
}


dBodyID dGeomGetBody (dxGeom *g)
{
  dAASSERT (g);
  return g->body;
}


void dGeomSetPosition (dxGeom *g, dReal x, dReal y, dReal z)
{
  dAASSERT (g);
  if (g->body) dBodySetPosition (g->body,x,y,z);
  else {
    g->pos[0] = x;
    g->pos[1] = y;
    g->pos[2] = z;
  }
}


void dGeomSetRotation (dxGeom *g, const dMatrix3 R)
{
  dAASSERT (g);
  if (g->body) dBodySetRotation (g->body,R);
  else memcpy (g->R,R,sizeof(dMatrix3));
}


const dReal * dGeomGetPosition (dxGeom *g)
{
  dAASSERT (g);
  return g->pos;
}


const dReal * dGeomGetRotation (dxGeom *g)
{
  dAASSERT (g);
  return g->R;
}


// for external use only. use the CLASSDATA macro inside ODE.

void * dGeomGetClassData (dxGeom *g)
{
  dAASSERT (g);
  return (void*) CLASSDATA(g);
}


dxGeom * dCreateGeom (int classnum)
{
  dUASSERT (classes && colliders && classnum >= 0 &&
	    classnum < classes->size(),"bad class number");
  int size = (*classes)[classnum]->size;
  dxGeom *geom = (dxGeom*) dAlloc (size);
  memset (geom,0,size);

  geom->_class = (*classes)[classnum];
  geom->data = 0;
  geom->body = 0;

  dxPosR *pr = (dxPosR*) dAlloc (sizeof(dxPosR));
  geom->pos = pr->pos;
  geom->R = pr->R;
  dSetZero (geom->pos,4);
  dRSetIdentity (geom->R);

  // the `space' area is initially zeroed
  memset (&geom->space,0,sizeof(geom->space));

  return geom;
}


void dGeomDestroy (dxGeom *g)
{
  dAASSERT (g);
  dSpaceRemove (g->spaceid,g);
  if (!g->body) dFree (g->pos,sizeof(dxPosR));
  dFree (g,g->_class->size);
}

//****************************************************************************
// data for the standard classes

struct dxSphere {
  dReal radius;		// sphere radius
};

struct dxBox {
  dVector3 side;	// side lengths (x,y,z)
};

struct dxCCylinder {	// capped cylinder
  dReal radius,lz;	// radius, length along z axis */
};

struct dxPlane {
  dReal p[4];
};

struct dxComposite {
  // stuff...
};

//****************************************************************************
// primitive collision functions
// same interface as dCollide().
// S=sphere, B=box, C=capped cylinder, P=plane

int dCollideSS (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->_class->num == dSphereClass);
  dIASSERT (o2->_class->num == dSphereClass);
  dxSphere *s1 = (dxSphere*) CLASSDATA(o1);
  dxSphere *s2 = (dxSphere*) CLASSDATA(o2);
  return dCollideSpheres (o1->pos,s1->radius,
			  o2->pos,s2->radius,contact);
}


int dCollideSB (const dxGeom *o1, const dxGeom *o2, int flags,
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
  dIASSERT (o1->_class->num == dSphereClass);
  dIASSERT (o2->_class->num == dBoxClass);
  dxSphere *sphere = (dxSphere*) CLASSDATA(o1);
  dxBox *box = (dxBox*) CLASSDATA(o2);

  p[0] = o1->pos[0] - o2->pos[0];
  p[1] = o1->pos[1] - o2->pos[1];
  p[2] = o1->pos[2] - o2->pos[2];

  l[0] = box->side[0]*0.5;
  t[0] = dDOT14(p,o2->R);
  if (t[0] < -l[0]) { t[0] = -l[0]; onborder = 1; }
  if (t[0] >  l[0]) { t[0] =  l[0]; onborder = 1; }

  l[1] = box->side[1]*0.5;
  t[1] = dDOT14(p,o2->R+1);
  if (t[1] < -l[1]) { t[1] = -l[1]; onborder = 1; }
  if (t[1] >  l[1]) { t[1] =  l[1]; onborder = 1; }

  t[2] = dDOT14(p,o2->R+2);
  l[2] = box->side[2]*0.5;
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
    tmp[maxi] = (t[maxi] > 0) ? 1 : -1;
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


int dCollideSC (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideSP (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->_class->num == dSphereClass);
  dIASSERT (o2->_class->num == dPlaneClass);
  dxSphere *sphere = (dxSphere*) CLASSDATA(o1);
  dxPlane *plane = (dxPlane*) CLASSDATA(o2);
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


int dCollideBB (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  dVector3 normal;
  dReal depth;
  int code;
  dxBox *b1 = (dxBox*) CLASSDATA(o1);
  dxBox *b2 = (dxBox*) CLASSDATA(o2);
  int num = dBoxBox (o1->pos,o1->R,b1->side, o2->pos,o2->R,b2->side,
		     normal,&depth,&code,flags & 0xff,contact,skip);
  for (int i=0; i<num; i++) {
    CONTACT(contact,i*skip)->normal[0] = -normal[0];
    CONTACT(contact,i*skip)->normal[1] = -normal[1];
    CONTACT(contact,i*skip)->normal[2] = -normal[2];
  }
  return num;
}


int dCollideBC (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideBP (const dxGeom *o1, const dxGeom *o2,
		int flags, dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->_class->num == dBoxClass);
  dIASSERT (o2->_class->num == dPlaneClass);
  dxBox *box = (dxBox*) CLASSDATA(o1);
  dxPlane *plane = (dxPlane*) CLASSDATA(o2);

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
  dReal depth = plane->p[3] + 0.5*(B1+B2+B3) - dDOT(n,o1->pos);
  if (depth < 0) return 0;

  // find number of contacts requested
  int maxc = flags & 0xff;
  if (maxc < 1) maxc = 1;
  if (maxc > 3) maxc = 3;	// no more than 3 contacts per box allowed

  // find deepest point
  dVector3 p;
  p[0] = o1->pos[0];
  p[1] = o1->pos[1];
  p[2] = o1->pos[2];
#define FOO(i,op) \
  p[0] op 0.5*box->side[i] * R[0+i]; \
  p[1] op 0.5*box->side[i] * R[4+i]; \
  p[2] op 0.5*box->side[i] * R[8+i];
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
  if (maxc == 1) return 1;

  // get the second and third contact points by starting from `p' and going
  // along the two sides with the smallest projected length.

#define FOO(i,j,op) \
  CONTACT(contact,i*skip)->pos[0] = p[0] op box->side[j] * R[0+j]; \
  CONTACT(contact,i*skip)->pos[1] = p[1] op box->side[j] * R[4+j]; \
  CONTACT(contact,i*skip)->pos[2] = p[2] op box->side[j] * R[8+j];
#define BAR(ctact,side,sideinc) \
  depth -= B ## sideinc; \
  if (depth < 0) return 1; \
  if (A ## sideinc > 0) { FOO(ctact,side,+) } else { FOO(ctact,side,-) } \
  CONTACT(contact,ctact*skip)->depth = depth;

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
      if (maxc == 2) return 2;
      if (B2 < B3) goto contact2_2; else goto contact2_3;
    }
  }
  else {
    if (B3 < B2) {
      use_side_3:	// use side 3
      BAR(1,2,3);
      if (maxc == 2) return 2;
      if (B1 < B2) goto contact2_1; else goto contact2_2;
    }
    else {
      BAR(1,1,2);	// use side 2
      if (maxc == 2) return 2;
      if (B1 < B3) goto contact2_1; else goto contact2_3;
    }
  }

  contact2_1: BAR(2,0,1); return 3;
  contact2_2: BAR(2,1,2); return 3;
  contact2_3: BAR(2,2,3); return 3;
#undef FOO
#undef BAR
}


int dCollideCC (const dxGeom *o1, const dxGeom *o2,
		int flags, dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideCP (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}

//****************************************************************************
// standard classes

int dSphereClass = -1;
int dBoxClass = -1;
int dCCylinderClass = -1;
int dPlaneClass = -1;
int dCompositeClass = -1;


static dColliderFn * dSphereColliderFn (int num)
{
  if (num == dSphereClass) return (dColliderFn *) &dCollideSS;
  if (num == dBoxClass) return (dColliderFn *) &dCollideSB;
  if (num == dPlaneClass) return (dColliderFn *) &dCollideSP;
  return 0;
}


static void dSphereAABB (dxGeom *geom, dReal aabb[6])
{
  dxSphere *s = (dxSphere*) CLASSDATA(geom);
  aabb[0] = geom->pos[0] - s->radius;
  aabb[1] = geom->pos[0] + s->radius;
  aabb[2] = geom->pos[1] - s->radius;
  aabb[3] = geom->pos[1] + s->radius;
  aabb[4] = geom->pos[2] - s->radius;
  aabb[5] = geom->pos[2] + s->radius;
}


static dColliderFn * dBoxColliderFn (int num)
{
  if (num == dBoxClass) return (dColliderFn *) &dCollideBB;
  if (num == dPlaneClass) return (dColliderFn *) &dCollideBP;
  return 0;
}


static void dBoxAABB (dxGeom *geom, dReal aabb[6])
{
  dxBox *b = (dxBox*) CLASSDATA(geom);
  dReal xrange = REAL(0.5) * (dFabs (geom->R[0] * b->side[0]) +
    dFabs (geom->R[1] * b->side[1]) + dFabs (geom->R[2] * b->side[2]));
  dReal yrange = REAL(0.5) * (dFabs (geom->R[4] * b->side[0]) +
    dFabs (geom->R[5] * b->side[1]) + dFabs (geom->R[6] * b->side[2]));
  dReal zrange = REAL(0.5) * (dFabs (geom->R[8] * b->side[0]) +
    dFabs (geom->R[9] * b->side[1]) + dFabs (geom->R[10] * b->side[2]));
  aabb[0] = geom->pos[0] - xrange;
  aabb[1] = geom->pos[0] + xrange;
  aabb[2] = geom->pos[1] - yrange;
  aabb[3] = geom->pos[1] + yrange;
  aabb[4] = geom->pos[2] - zrange;
  aabb[5] = geom->pos[2] + zrange;
}


dColliderFn * dPlaneColliderFn (int num)
{
  return 0;
}


static void dPlaneAABB (dxGeom *geom, dReal aabb[6])
{
  // @@@ planes that have normal vectors aligned along an axis can use a
  // @@@ less comprehensive bounding box.
  aabb[0] = -dInfinity;
  aabb[1] = dInfinity;
  aabb[2] = -dInfinity;
  aabb[3] = dInfinity;
  aabb[4] = -dInfinity;
  aabb[5] = dInfinity;
}


dxGeom *dCreateSphere (dSpaceID space, dReal radius)
{
  dAASSERT (space);
  if (dSphereClass == -1) {
    dGeomClass c;
    c.bytes = sizeof (dxSphere);
    c.collider = &dSphereColliderFn;
    c.aabb = &dSphereAABB;
    dSphereClass = dCreateGeomClass (&c);
  }

  dxGeom *g = dCreateGeom (dSphereClass);
  dSpaceAdd (space,g);
  dxSphere *s = (dxSphere*) CLASSDATA(g);
  s->radius = radius;
  return g;
}


dxGeom *dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
{
  dAASSERT (space && lx > 0 && ly > 0 && lz > 0);
  if (dBoxClass == -1) {
    dGeomClass c;
    c.bytes = sizeof (dxBox);
    c.collider = &dBoxColliderFn;
    c.aabb = &dBoxAABB;
    dBoxClass = dCreateGeomClass (&c);
  }

  dxGeom *g = dCreateGeom (dBoxClass);
  dSpaceAdd (space,g);
  dxBox *b = (dxBox*) CLASSDATA(g);
  b->side[0] = lx;
  b->side[1] = ly;
  b->side[2] = lz;
  return g;
}


dxGeom *dCreatePlane (dSpaceID space,
		      dReal a, dReal b, dReal c, dReal d)
{
  dAASSERT (space);
  if (dPlaneClass == -1) {
    dGeomClass c;
    c.bytes = sizeof (dxPlane);
    c.collider = &dPlaneColliderFn;
    c.aabb = &dPlaneAABB;
    dPlaneClass = dCreateGeomClass (&c);
  }

  dxGeom *g = dCreateGeom (dPlaneClass);
  dSpaceAdd (space,g);
  dxPlane *p = (dxPlane*) CLASSDATA(g);

  // make sure plane normal has unit length
  dReal l = a*a + b*b + c*c;
  if (l > 0) {
    l = dRecipSqrt(l);
    p->p[0] = a*l;
    p->p[1] = b*l;
    p->p[2] = c*l;
    p->p[3] = d*l;
  }
  else {
    p->p[0] = 1;
    p->p[1] = 0;
    p->p[2] = 0;
    p->p[3] = 0;
  }
  return g;
}


dReal dGeomSphereGetRadius (dGeomID g)
{
  dUASSERT (g && g->_class->num == dSphereClass,"argument not a sphere");
  dxSphere *s = (dxSphere*) CLASSDATA(g);
  return s->radius;
}


void  dGeomBoxGetLengths (dGeomID g, dVector3 result)
{
  dUASSERT (g && g->_class->num == dBoxClass,"argument not a box");
  dxBox *b = (dxBox*) CLASSDATA(g);
  result[0] = b->side[0];
  result[1] = b->side[1];
  result[2] = b->side[2];
}


void  dGeomPlaneGetParams (dGeomID g, dVector4 result)
{
  dUASSERT (g && g->_class->num == dPlaneClass,"argument not a plane");
  dxPlane *p = (dxPlane*) CLASSDATA(g);
  result[0] = p->p[0];
  result[1] = p->p[1];
  result[2] = p->p[2];
  result[3] = p->p[3];
}


void dGeomCCylinderGetParams (dGeomID g, dReal *a, dReal *b, int *dir)
{
  dDebug (0,"not implemented");
}

//****************************************************************************
// composite

/*
void dAddToComposite (dxGeom *composite, int i, dxGeom *obj)
{
  dDebug (0,"unimplemented");
}
*/
