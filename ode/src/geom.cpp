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

/*

TODO

* for objects that are never meant to collide, dCollide() will always try
  to find the collider functions, which wastes a bit of time.

*/

#include "ode/geom.h"
#include "ode/rotation.h"
#include "ode/odemath.h"
#include "ode/memory.h"
#include "ode/misc.h"
#include "array.h"

//****************************************************************************
// collision utilities

// given a pointer `p' to a dContactGeom, return the dContactGeom at
// p + skip bytes.

#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + skip))


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

extern "C" int dBoxTouchesBox (const dVector3 p1, const dMatrix3 R1,
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


extern "C" int dBoxBox (const dVector3 p1, const dMatrix3 R1,
			const dVector3 side1, const dVector3 p2,
			const dMatrix3 R2, const dVector3 side2,
			dVector3 normal, dReal *depth,
			int *code, int *coderu,
			dVector3 contact)
{
  //@@@ coderu/sru = runner up values

  dVector3 p,pp,normalC;
  const dReal *normalR = 0;
  dReal A1,A2,A3,B1,B2,B3,R11,R12,R13,R21,R22,R23,R31,R32,R33,
    Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,sru,l;
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
  // depth. note: normalR is set to point to a column of R1,R2 if that is the
  // smallest depth normal so far. otherwise normalR is 0 and normalC is set
  // to a vector relative to body 1. invert_normal is 1 if the sign of the
  // normal should be flipped.

#define TEST(expr1,expr2,norm,cc) \
  s2 = dFabs(expr1) - (expr2); \
  if (s2 > 0) return 0; \
  if (s2 > s) { \
    sru = s; \
    s = s2; \
    normalR = norm; \
    invert_normal = ((expr1) < 0); \
    *coderu = *code; \
    *code = (cc); \
  } \
  else if (s2 > sru) { \
    sru = s2; \
    *coderu = (cc); \
  }

  s = -dInfinity;
  sru = -dInfinity;
  invert_normal = 0;
  *code = 0;
  *coderu = 0;

  // separating axis = u1,u2,u3
  TEST (pp[0],(A1 + B1*Q11 + B2*Q12 + B3*Q13),R1+0,1);
  TEST (pp[1],(A2 + B1*Q21 + B2*Q22 + B3*Q23),R1+1,2);
  TEST (pp[2],(A3 + B1*Q31 + B2*Q32 + B3*Q33),R1+2,3);

  // separating axis = v1,v2,v3
  TEST (dDOT41(R2+0,p),(A1*Q11 + A2*Q21 + A3*Q31 + B1),R2+0,4);
  TEST (dDOT41(R2+1,p),(A1*Q12 + A2*Q22 + A3*Q32 + B2),R2+1,5);
  TEST (dDOT41(R2+2,p),(A1*Q13 + A2*Q23 + A3*Q33 + B3),R2+2,6);

  //@@@ note: cross product axes need to be scaled when s computed!!!
  // normal (n1,n2,n3) is relative to box 1!
#undef TEST
#define TEST(expr1,expr2,n1,n2,n3,cc) \
  s2 = dFabs(expr1) - (expr2); \
  if (s2 > 0) return 0; \
  l = dSqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
  if (l > 0) { \
    s2 /= l; \
    if (s2 > s) { \
      sru = s; \
      s = s2; \
      normalR = 0; \
      normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
      invert_normal = ((expr1) < 0); \
      *coderu = *code; \
      *code = (cc); \
    } \
    else if (s2 > sru) { \
      sru = s2; \
      *coderu = (cc); \
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

  for (i=0; i<3; i++) contact[i] = 0;

  // compute contact point(s)
  if (*code > 6) {
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

    for (i=0; i<3; i++) contact[i] = 0.5*(pa[i]+pb[i]);
    return 1;
  }

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). if all elements of Q are "very
  // nonzero", then the (u1,u2,u3) don't line up with any of the (v1,v2,v3).
  // thus there should only be one point of contact with the face.

  //if () {
  //  ...
  //  return 1;
  //}

  //@@@ testing
  const dReal tol = 0.9999;
  int foo = 0;
  if (*code <= 3) {
    for (i=0; i<3; i++) if (dFabs(dDOT14(normal,R2+i)) > tol) foo=1;
  }
  else {
    for (i=0; i<3; i++) if (dFabs(dDOT14(normal,R1+i)) > tol) foo=1;
  }
  if (foo) {
    //printf ("%6.3f %6.3f %6.3f\n",Q11,Q12,Q13);
    //printf ("%6.3f %6.3f %6.3f\n",Q21,Q22,Q23);
    //printf ("%6.3f %6.3f %6.3f\n",Q31,Q32,Q33);
    return 2;
  }

  // we now know that either an edge or a face is aligned with the face.
  // this can generate two or more contact points.


  return 1;
}

//****************************************************************************
// primitives collision functions

int dCollideSS (const dSphere *o1, const dSphere *o2, int flags,
		dContactGeom *contact, int skip)
{
  dASSERT (skip >= (int)sizeof(dContactGeom));
  return dCollideSpheres (o1->geom.pos,o1->radius,
			  o2->geom.pos,o2->radius,contact);
}


int dCollideSB (const dSphere *o1, const dBox *o2, int flags,
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

  p[0] = o1->geom.pos[0] - o2->geom.pos[0];
  p[1] = o1->geom.pos[1] - o2->geom.pos[1];
  p[2] = o1->geom.pos[2] - o2->geom.pos[2];

  l[0] = o2->side[0]*0.5;
  t[0] = dDOT14(p,o2->geom.R);
  if (t[0] < -l[0]) { t[0] = -l[0]; onborder = 1; }
  if (t[0] >  l[0]) { t[0] =  l[0]; onborder = 1; }

  l[1] = o2->side[1]*0.5;
  t[1] = dDOT14(p,o2->geom.R+1);
  if (t[1] < -l[1]) { t[1] = -l[1]; onborder = 1; }
  if (t[1] >  l[1]) { t[1] =  l[1]; onborder = 1; }

  t[2] = dDOT14(p,o2->geom.R+2);
  l[2] = o2->side[2]*0.5;
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
    contact->pos[0] = o1->geom.pos[0];
    contact->pos[1] = o1->geom.pos[1];
    contact->pos[2] = o1->geom.pos[2];
    // contact normal aligned with box edge along largest `t' value
    dVector3 tmp;
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[maxi] = (t[maxi] > 0) ? 1 : -1;
    dMULTIPLY0_331 (contact->normal,o2->geom.R,tmp);
    // contact depth = distance to wall along normal plus radius
    contact->depth = l[maxi] - max + o1->radius;
    return 1;
  }

  t[3] = 0;			//@@@ hmmm
  dMULTIPLY0_331 (q,o2->geom.R,t);
  r[0] = p[0] - q[0];
  r[1] = p[1] - q[1];
  r[2] = p[2] - q[2];
  depth = o1->radius - dSqrt(dDOT(r,r));
  if (depth < 0) return 0;
  contact->pos[0] = q[0] + o2->geom.pos[0];
  contact->pos[1] = q[1] + o2->geom.pos[1];
  contact->pos[2] = q[2] + o2->geom.pos[2];
  contact->normal[0] = r[0];
  contact->normal[1] = r[1];
  contact->normal[2] = r[2];
  dNormalize3 (contact->normal);
  contact->depth = depth;
  return 1;
}


int dCollideSC (const dSphere *o1, const dCCylinder *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideSP (const dSphere *o1, const dPlane *o2, int flags,
		dContactGeom *contact, int skip)
{
  dASSERT (skip >= (int)sizeof(dContactGeom));
  dReal k = dDOT (o1->geom.pos,o2->p);
  dReal depth = o2->p[3] - k + o1->radius;
  if (depth >= 0) {
    contact->normal[0] = o2->p[0];
    contact->normal[1] = o2->p[1];
    contact->normal[2] = o2->p[2];
    contact->pos[0] = o1->geom.pos[0] - o2->p[0] * o1->radius;
    contact->pos[1] = o1->geom.pos[1] - o2->p[1] * o1->radius;
    contact->pos[2] = o1->geom.pos[2] - o2->p[2] * o1->radius;
    contact->depth = depth;
    return 1;
  }
  else return 0;
}


int dCollideBB (const dBox *o1, const dBox *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideBC (const dBox *o1, const dCCylinder *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideBP (const dBox *o1, const dPlane *o2,
		int flags, dContactGeom *contact, int skip)
{
  dASSERT (skip >= (int)sizeof(dContactGeom));

  //@@@ problem: using 4-vector (o2->p) as 3-vector (normal).
  const dReal *R = o1->geom.R;		// rotation of box
  const dReal *n = o2->p;		// normal vector

  // project sides lengths along normal vector, get absolute values
  dReal A1 = o1->side[0] * dDOT14(n,R+0);
  dReal A2 = o1->side[1] * dDOT14(n,R+1);
  dReal A3 = o1->side[2] * dDOT14(n,R+2);
  dReal B1 = dFabs(A1);
  dReal B2 = dFabs(A2);
  dReal B3 = dFabs(A3);

  // early exit test
  dReal depth = o2->p[3] + 0.5*(B1+B2+B3) - dDOT(n,o1->geom.pos);
  if (depth < 0) return 0;

  // find number of contacts requested
  int maxc = flags & 0xffff;
  if (maxc < 1) maxc = 1;
  if (maxc > 3) maxc = 3;	// no more than 3 contacts per box allowed

  // find deepest point
  dVector3 p;
  p[0] = o1->geom.pos[0];
  p[1] = o1->geom.pos[1];
  p[2] = o1->geom.pos[2];
#define FOO(i,op) \
  p[0] op 0.5*o1->side[i] * R[0+i]; \
  p[1] op 0.5*o1->side[i] * R[4+i]; \
  p[2] op 0.5*o1->side[i] * R[8+i];
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
  CONTACT(contact,i*skip)->pos[0] = p[0] op o1->side[j] * R[0+j]; \
  CONTACT(contact,i*skip)->pos[1] = p[1] op o1->side[j] * R[4+j]; \
  CONTACT(contact,i*skip)->pos[2] = p[2] op o1->side[j] * R[8+j];
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


int dCollideCC (const dCCylinder *o1, const dCCylinder *o2,
		int flags, dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideCP (const dCCylinder *o1, const dPlane *o2, int flags,
		dContactGeom *contact, int skip)
{
  dDebug (0,"unimplemented");
  return 0;
}

//****************************************************************************
// composite

void dAddToComposite (dGeomID composite, int i, dGeomID obj)
{
  dDebug (0,"unimplemented");
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


static void dSphereAABB (dGeomID geom, dReal aabb[6])
{
  dSphere *s = (dSphere*) geom;
  aabb[0] = s->geom.pos[0] - s->radius;
  aabb[1] = s->geom.pos[0] + s->radius;
  aabb[2] = s->geom.pos[1] - s->radius;
  aabb[3] = s->geom.pos[1] + s->radius;
  aabb[4] = s->geom.pos[2] - s->radius;
  aabb[5] = s->geom.pos[2] + s->radius;
}


static dColliderFn * dBoxColliderFn (int num)
{
  if (num == dPlaneClass) return (dColliderFn *) &dCollideBP;
  return 0;
}


static void dBoxAABB (dGeomID geom, dReal aabb[6])
{
  dDebug (0,"unimplemented");
}


dColliderFn * dPlaneColliderFn (int num)
{
  return 0;
}


static void dPlaneAABB (dGeomID geom, dReal aabb[6])
{
  dDebug (0,"PlaneAABB() not implemented, should not have to be");
}


dxGeom *dCreateSphere (dSpaceID space, dReal radius)
{
  if (dSphereClass == -1) {
    dGeomClass c;
    c.num = 0;
    c.size = sizeof (dSphere);
    c.collider = &dSphereColliderFn;
    c.aabb = &dSphereAABB;
    dSphereClass = dCreateGeomClass (&c);
  }

  dSphere *s = (dSphere*) dAlloc (sizeof(dSphere));
  dInitGeom (&s->geom,space,dSphereClass);
  s->radius = radius;
  return &s->geom;
}


dxGeom *dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
{
  if (dBoxClass == -1) {
    dGeomClass c;
    c.num = 0;
    c.size = sizeof (dBox);
    c.collider = &dBoxColliderFn;
    c.aabb = &dBoxAABB;
    dBoxClass = dCreateGeomClass (&c);
  }

  dBox *b = (dBox*) dAlloc (sizeof(dBox));
  dInitGeom (&b->geom,space,dBoxClass);
  b->side[0] = lx;
  b->side[1] = ly;
  b->side[2] = lz;
  return &b->geom;
}


dxGeom *dCreatePlane (dSpaceID space,
		      dReal a, dReal b, dReal c, dReal d)
{
  if (dPlaneClass == -1) {
    dGeomClass c;
    c.num = 0;
    c.size = sizeof (dPlane);
    c.collider = &dPlaneColliderFn;
    c.aabb = &dPlaneAABB;
    dPlaneClass = dCreateGeomClass (&c);
  }

  dPlane *p = (dPlane*) dAlloc (sizeof(dPlane));
  dInitGeom (&p->geom,space,dPlaneClass);
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
  return &p->geom;
}

//****************************************************************************
// geometry class stuff

// @@@ NOTE! global constructors here, which is bad!!!

struct dColliderEntry {
  dColliderFn *fn;	// collider function
  int mode;		// 1 = reverse o1 and o2, 2 = no function available
};

// pointers to the class structures
static dArray<dGeomClass*> classes;	// <--- update for pointers

// function pointers and modes for n^2 class collider functions. this is an
// n*n matrix stored by row. the functions pointers are extracted from the
// class get-collider-function function.
static dArray<dColliderEntry> colliders;


int dCreateGeomClass (const dGeomClass *c)
{
  if (c->size < (int)sizeof(dxGeom))
    dDebug (d_ERR_BAD_ARGS,"dGeomClass size too small");
  if (!c->collider || !c->aabb)
    dDebug (d_ERR_BAD_ARGS,"dGeomClass has zero function pointers");

  int n = classes.size();
  dGeomClass *gc = (dGeomClass*) dAllocNoFree (sizeof(dGeomClass));

  memcpy (gc,c,sizeof(dGeomClass));
  gc->num = n;			// set class number
  classes.push (gc);

  // make room for n^2 class collider function pointers - these entries will
  // be filled as dCollide() is called.
  colliders.setSize ((n+1)*(n+1));
  memset (&colliders[0],0,(n+1)*(n+1)*sizeof(dColliderEntry));

  return n;
}


void dInitGeom (dGeomID geom, dSpaceID space, int classnum)
{
  if (classnum < 0 || classnum >= classes.size())
    dDebug (d_ERR_BAD_ARGS,"dInitGeom(), bad class number");
  memset (geom,0,classes[classnum]->size);
  geom->_class = classes[classnum];
  geom->data = 0;
  geom->body = 0;
  geom->pos = 0;
  geom->R = 0;
  dSpaceAdd (space,geom);
}


void dDestroyGeom (dSpaceID space, dGeomID geom)
{
  dSpaceRemove (space,geom);
  dFree (geom,geom->_class->size);
}


int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact,
	      int skip)
{
  int i,c1,c2,a1,a2,count,swap;
  dColliderFn *fn;
  c1 = o1->_class->num;
  c2 = o2->_class->num;
  a1 = c1 * classes.size() + c2;	// address 1 in collider array
  a2 = c2 * classes.size() + c1;	// address 2 in collider array
  swap = 0;		// set to 1 to swap normals before returning

  // return if there are no collider functions available
  if ((colliders[a1].mode==2) || (colliders[a2].mode==2)) return 0;

  if ((fn = colliders[a1].fn)) {
    swap = colliders[a1].mode;
    if (swap) count = (*fn) (o2,o1,flags,contact,skip);
    else count = (*fn) (o1,o2,flags,contact,skip);
  }
  else if ((fn = classes[c1]->collider (c2))) {
    colliders [a2].fn = fn;
    colliders [a2].mode = 1;
    colliders [a1].fn = fn;	// do mode=0 assignment second so that
    colliders [a1].mode = 0;	// diagonal entries will have mode 0
    count = (*fn) (o1,o2,flags,contact,skip);
    swap = 0;
  }
  else if ((fn = classes[c2]->collider (c1))) {
    colliders [a1].fn = fn;
    colliders [a1].mode = 1;
    colliders [a2].fn = fn;	// do mode=0 assignment second so that
    colliders [a2].mode = 0;	// diagonal entries will have mode 0
    count = (*fn) (o2,o1,flags,contact,skip);
    swap = 1;
  }
  else {
    colliders[a1].mode = 2;
    colliders[a2].mode = 2;
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
