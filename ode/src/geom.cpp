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
#include "ode/odemath.h"
#include "ode/memory.h"
#include "array.h"

//****************************************************************************
// collision utilities

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


// given a box (p,R,side), where `p' is the center of the box, `R' is the
// rotation matrix for the box, and `side' is a vector of x/y/z side lengths,
// return the size of the interval of the box projected along the
// (unit length?) axis a.

static inline dReal boxDiameter (const dMatrix3 R, const dVector3 side,
				 const dVector3 axis)
{
  dVector3 q;
  dMULTIPLY1_331 (q,R,axis);	// transform axis to body-relative
  return dFabs(q[0])*side[0] + dFabs(q[1])*side[1] + dFabs(q[2])*side[2];
}


// given boxes (p1,R1,side1) and (p1,R1,side1), return 1 if they intersect
// or 0 if not.
// @@@ should be worst case ~200 ops

extern "C" int dBoxesTouch (const dVector3 _p1, const dMatrix3 R1,
			    const dVector3 side1, const dVector3 _p2,
			    const dMatrix3 R2, const dVector3 side2)
{
  // two boxes are disjoint if (and only if) there is a separating axis
  // perpendicular to a face from one box or perpendicular to an edge from
  // either box.

  int i,j,k;
  dVector3 p2,axis,u1,u2;
  dReal dist,d1,d2;

  // shift positions p1 and p2 so p1 is zero.
  p2[0] = _p2[0] - _p2[0];
  p2[1] = _p2[1] - _p2[1];
  p2[2] = _p2[2] - _p2[2];

  // test separating axes perpendicular to 3 faces of box 1
  for (i=0; i<3; i++) {
    for (k=0; k<3; k++) axis[k] = R1[4*k+i];
    dist = dDOT (axis,p2);
    d2 = boxDiameter (R2,side2,axis);
    if (2*dist > side1[i]+d2) return 0;
  }

  // test separating axes perpendicular to 3 faces of box 2
  for (i=0; i<3; i++) {
    for (k=0; k<3; k++) axis[k] = R2[4*k+i];
    dist = dDOT (axis,p2);
    d1 = boxDiameter (R1,side1,axis);
    if (2*dist > d1+side2[i]) return 0;
  }

  // test separating axes perpendicular to one edge from each box (9 cases)
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++) {
      for (k=0; k<3; k++) u1[k] = R1[4*k+i];
      for (k=0; k<3; k++) u2[k] = R2[4*k+j];
      dCROSS (axis,=,u1,u2);
      dNormalize3 (axis);	// axis is not necessarily unit length
      dist = dDOT (axis,p2);
      d1 = boxDiameter (R1,side1,axis);
      d2 = boxDiameter (R2,side2,axis);
      if (2*dist > d1+d2) return 0;
    }
  }

  return 1;
}

//****************************************************************************
// primitives collision functions

int dCollideSS (const dSphere *o1, const dSphere *o2, int flags,
		dContactGeom *contact)
{
  return dCollideSpheres (o1->geom.pos,o1->radius,
			   o2->geom.pos,o2->radius,contact);
}


int dCollideSB (const dSphere *o1, const dBox *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideSC (const dSphere *o1, const dCCylinder *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideSL (const dSphere *o1, const dSlab *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideSP (const dSphere *o1, const dPlane *o2, int flags,
		dContactGeom *contact)
{
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
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideBC (const dBox *o1, const dCCylinder *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideBL (const dBox *o1, const dSlab *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideBP (const dBox *o1, const dPlane *o2,
		int flags, dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideCC (const dCCylinder *o1, const dCCylinder *o2,
		int flags, dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideCL (const dCCylinder *o1, const dSlab *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideCP (const dCCylinder *o1, const dPlane *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideLL (const dSlab *o1, const dSlab *o2, int flags,
		dContactGeom *contact)
{
  dDebug (0,"unimplemented");
  return 0;
}


int dCollideLP (const dSlab *o1, const dPlane *o2, int flags,
		dContactGeom *contact)
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
int dSlabClass = -1;
int dPlaneClass = -1;
int dCompositeClass = -1;


dColliderFn * dSphereColliderFn (int num)
{
  if (num == dSphereClass) return (dColliderFn *) &dCollideSS;
  if (num == dPlaneClass) return (dColliderFn *) &dCollideSP;
  return 0;
}


void dSphereAABB (dGeomID geom, dReal aabb[6])
{
  dSphere *s = (dSphere*) geom;
  aabb[0] = s->geom.pos[0] - s->radius;
  aabb[1] = s->geom.pos[0] + s->radius;
  aabb[2] = s->geom.pos[1] - s->radius;
  aabb[3] = s->geom.pos[1] + s->radius;
  aabb[4] = s->geom.pos[2] - s->radius;
  aabb[5] = s->geom.pos[2] + s->radius;
}


dColliderFn * dPlaneColliderFn (int num)
{
  return 0;
}


void dPlaneAABB (dGeomID geom, dReal aabb[6])
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

// NOTE! global constructors here, which is bad!!!

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


int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact)
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
    if (swap) count = (*fn) (o2,o1,flags,contact);
    else count = (*fn) (o1,o2,flags,contact);
  }
  else if ((fn = classes[c1]->collider (c2))) {
    colliders [a2].fn = fn;
    colliders [a2].mode = 1;
    colliders [a1].fn = fn;	// do mode=0 assignment second so that
    colliders [a1].mode = 0;	// diagonal entries will have mode 0
    count = (*fn) (o1,o2,flags,contact);
    swap = 0;
  }
  else if ((fn = classes[c2]->collider (c1))) {
    colliders [a1].fn = fn;
    colliders [a1].mode = 1;
    colliders [a2].fn = fn;	// do mode=0 assignment second so that
    colliders [a2].mode = 0;	// diagonal entries will have mode 0
    count = (*fn) (o2,o1,flags,contact);
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
