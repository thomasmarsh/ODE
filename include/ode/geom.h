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

geometry objects for collision detection.

"REQUIRED" is used to mark the functions that are referenced by the dynamics.

*/

#ifndef _ODE_GEOM_H_
#define _ODE_GEOM_H_

#include "ode/common.h"
#include "ode/space.h"
#include "ode/contact.h"

#ifdef __cplusplus
extern "C" {
#endif

/* *********************************************************************** */
/* typedefs and structures */

/* this function returns an axis-aligned bounding box for an object of a
 * known class (minx,maxx,miny,maxy,minz,maxz).
 */
typedef void dGetAABBFn (dGeomID, dReal aabb[6]);

/* this function handles collision between specific object types. when this
 * is called, o1 and o2 have known types. the `flags' and `contact' arguments
 * are those passed to dCollide(). this function must fill in the fields of
 * the contact struct.
 */
typedef int dColliderFn (dGeomID o1, dGeomID o2,
			 int flags, dContactGeom *contact);

/* one of these functions is provided by each geometry class. it takes a class
 * number. it should return the collider function that can handle colliding
 * this class with class `num'. it should return 0 if that capability does not
 * exist in this class. this function is only ever called once for each num,
 * thereafter the cached pointers are used. the returned collider function can
 * assume that its first dGeomID argument is a member of this class.
 * note that if classes A and B are to collide, only *one* needs to provide a
 * function to collide with the other.
 */
typedef dColliderFn * dGetColliderFnFn (int num);


/* geometry class information. why don't we just use C++? well, this is a
 * bit more flexible, e.g. the class number is assigned at runtime, the class
 * can have int and string parameters as well as functions.
 */
struct dGeomClass {
  int num;			/* class number (globally unique) */
  int size;			/* size of geometry object for this class */
  dGetColliderFnFn *collider;	/* get collider functions */
  dGetAABBFn *aabb;		/* get axis aligned bounding box */
};


/* common data for all geometry objects. specific geometry objects subclass
 * this, adding their own data at the end. some geometry objects will want
 * to add buffers for their own pos and R at the end too.
 * body is 0 for a non dynamics object (e.g. the static environment).
 *
 * this structure is made public because you may want to add your own
 * collision objects, in which case you need to subclass this struct. we could
 * have hidden it and had the geometry-specific data separate, but that's
 * kind of inefficient.
 *
 * we could provide accessor method for this data, but since we're making it
 * public, you can just access it yourself.
 */
struct dxGeom {
  dGeomClass *_class;	/* class of this object */
  void *data;		/* user data pointer */
  dBodyID body;		/* dynamics body associated with this object */
  dReal *pos;		/* pointer to object's position */
  dReal *R;		/* pointer to object's rotation matrix */
  dGeomSpaceData space;	/* reserved for use by space this object is in */
};

/* ************************************************************************ */
/* collision objects */

/* register new geometry classes. the new class number is returned (0,1,2,...).
 * the class number should be assigned to a global variable with the name
 * dXxxClass (e.g. dSphereClass). the class number in the class structure
 * should be set to 0 on entry.
 */
int dCreateGeomClass (dGeomClass *);

/* initialize a newly created collision object and put it in a space.
 * this is a utility function called by geometry object creation functions.
 */
void dInitGeom (dGeomID, dSpaceID, int classnum);

/* destroy a collision object, removing it from the space first. */

void dDestroyGeom (dSpaceID, dGeomID);

/* REQUIRED.
 * given two COs that potentially touch (o1 and o2), generate contact
 * information for them. this just calls the correct geometry-specific
 * function for o1 and o2. return the number of contact points if they touch
 * (in which case the `contact' array is updated) or 0 if they don't
 * (in which case the `contact' array is not changed).
 * `flags' specifies what information should be computed for the collision:
 *    bits 16..1 : maximum number of contact points to generate
 *                 (size of contact array). if this is 0 it is taken to be 1.
 */
int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact);

/* *********************************************************************** */
/* standard object types. the point of reference of all these objects
 * corresponds to their center of mass - this makes them easy to connect to
 * dynamics objects. if other points of reference are required, composite
 * objects should be used to encapsulate these primitives.
 */

struct dSphere {
  dxGeom geom;
  dReal radius;		/* sphere radius */
};

struct dBox {
  dxGeom geom;
  dReal lx,ly,lz;	/* side lengths */
};

struct dCCylinder {	/* capped cylinder */
  dxGeom geom;
  dReal radius,lz;	/* radius, length along z axis */
};

struct dSlab {
  dxGeom geom;
  dReal radius,lx,ly;	/* radius, side length along x and y axes */
};

struct dPlane {
  dxGeom geom;
  dReal p[4];	/* plane equation params: p[0]*x+p[1]*y+p[2]*z = p[3] */
};		/* (p[0],p[1],p[2]) = normal vector, must have length=1 */

struct dComposite {
  dxGeom geom;
  /* ??? */
};



/* the pos and R fields of the plane are ignored, i.e. the plane is always
 * part of the static environment.
 */

dGeomID dCreateSphere (dSpaceID space, dReal radius);
dGeomID dCreatePlane (dSpaceID space,
		      dReal a, dReal b, dReal c, dReal d);



/* specific collision functions. save interface as dCollide().
 * S=sphere, B=box, C=capped cylinder, L=slab, P=plane
 */

int dCollideSS (dSphere *o1, dSphere *o2, int flags, dContactGeom *contact);
int dCollideSB (dSphere *o1, dBox *o2, int flags, dContactGeom *contact);
int dCollideSC (dSphere *o1, dCCylinder *o2, int flags, dContactGeom *contact);
int dCollideSL (dSphere *o1, dSlab *o2, int flags, dContactGeom *contact);
int dCollideSP (dSphere *o1, dPlane *o2, int flags, dContactGeom *contact);

int dCollideBB (dBox *o1, dBox *o2, int flags, dContactGeom *contact);
int dCollideBC (dBox *o1, dCCylinder *o2, int flags, dContactGeom *contact);
int dCollideBL (dBox *o1, dSlab *o2, int flags, dContactGeom *contact);
int dCollideBP (dBox *o1, dPlane *o2, int flags, dContactGeom *contact);

int dCollideCC (dCCylinder *o1, dCCylinder *o2, int flags,
		dContactGeom *contact);
int dCollideCL (dCCylinder *o1, dSlab *o2, int flags, dContactGeom *contact);
int dCollideCP (dCCylinder *o1, dPlane *o2, int flags, dContactGeom *contact);

int dCollideLL (dSlab *o1, dSlab *o2, int flags, dContactGeom *contact);
int dCollideLP (dSlab *o1, dPlane *o2, int flags, dContactGeom *contact);


void dAddToComposite (dGeomID composite, int i, dGeomID obj);

extern int dSphereClass;
extern int dBoxClass;
extern int dCCylinderClass;
extern int dSlabClass;
extern int dPlaneClass;
extern int dCompositeClass;


#ifdef __cplusplus
}
#endif

#endif
