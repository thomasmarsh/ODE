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

core collision functions and data structures, plus part of the public API
for geometry objects

*/

#include <ode/common.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/objects.h>
#include "collision_kernel.h"
#include "collision_util.h"
#include "collision_std.h"
#include "collision_transform.h"

//****************************************************************************
// helper functions for dCollide()ing a space with another geom

// this struct records the parameters passed to dCollideSpaceGeom()

struct SpaceGeomColliderData {
  int flags;			// space left in contacts array
  dContactGeom *contact;
  int skip;
};


static void space_geom_collider (void *data, dxGeom *o1, dxGeom *o2)
{
  SpaceGeomColliderData *d = (SpaceGeomColliderData*) data;
  if (d->flags & NUMC_MASK) {
    int n = dCollide (o1,o2,d->flags,d->contact,d->skip);
    d->contact = CONTACT (d->contact,d->skip*n);
    d->flags -= n;
  }
}


static int dCollideSpaceGeom (dxGeom *o1, dxGeom *o2, int flags,
			      dContactGeom *contact, int skip)
{
  SpaceGeomColliderData data;
  data.flags = flags;
  data.contact = contact;
  data.skip = skip;
  dSpaceCollide2 (o1,o2,&data,&space_geom_collider);
  return (flags & NUMC_MASK) - (data.flags & NUMC_MASK);
}

//****************************************************************************
// dispatcher for the N^2 collider functions

// function pointers and modes for n^2 class collider functions

struct dColliderEntry {
  dColliderFn *fn;	// collider function, 0 = no function available
  int reverse;		// 1 = reverse o1 and o2
};
static dColliderEntry colliders[dGeomNumClasses][dGeomNumClasses];
static int colliders_initialized = 0;


// setCollider() will refuse to write over a collider entry once it has
// been written.

static void setCollider (int i, int j, dColliderFn *fn)
{
  if (colliders[j][i].fn == 0) {
    colliders[j][i].fn = fn;
    colliders[j][i].reverse = 1;
  }
  if (colliders[i][j].fn == 0) {
    colliders[i][j].fn = fn;
    colliders[i][j].reverse = 0;
  }
}


static void setAllColliders (int i, dColliderFn *fn)
{
  for (int j=0; j<dGeomNumClasses; j++) setCollider (i,j,fn);
}


static void initColliders()
{
  int i,j;

  if (colliders_initialized) return;
  colliders_initialized = 1;

  memset (colliders,0,sizeof(colliders));

  // setup space colliders
  for (i=dFirstSpaceClass; i <= dLastSpaceClass; i++) {
    for (j=0; j < dGeomNumClasses; j++) {
      setCollider (i,j,&dCollideSpaceGeom);
    }
  }

  setCollider (dSphereClass,dSphereClass,&dCollideSphereSphere);
  setCollider (dSphereClass,dBoxClass,&dCollideSphereBox);
  setCollider (dSphereClass,dPlaneClass,&dCollideSpherePlane);
  setCollider (dBoxClass,dBoxClass,&dCollideBoxBox);
  setCollider (dBoxClass,dPlaneClass,&dCollideBoxPlane);
  setCollider (dCCylinderClass,dSphereClass,&dCollideCCylinderSphere);
  setCollider (dCCylinderClass,dBoxClass,&dCollideCCylinderBox);
  setCollider (dCCylinderClass,dCCylinderClass,&dCollideCCylinderCCylinder);
  setCollider (dCCylinderClass,dPlaneClass,&dCollideCCylinderPlane);
  setAllColliders (dGeomTransformClass,&dCollideTransform);

  /*
    @@@ put this in when the ray class is added
    setCollider (dRayClass,dSphereClass,&dCollideRaySphere);
    setCollider (dRayClass,dBoxClass,&dCollideRayBox);
    setCollider (dRayClass,dCCylinderClass,&dCollideRayCCylinder);
    setCollider (dRayClass,dPlaneClass,&dCollideRayPlane);
  */
}


int dCollide (dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact,
	      int skip)
{
  dAASSERT(o1 && o2 && contact);
  dUASSERT(colliders_initialized,"colliders array not initialized");
  dUASSERT(o1->type >= 0 && o1->type < dGeomNumClasses,"bad o1 class number");
  dUASSERT(o2->type >= 0 && o2->type < dGeomNumClasses,"bad o2 class number");

  // no contacts if both geoms are the same
  if (o1 == o2) return 0;

  // no contacts if both geoms on the same body, and the body is not 0
  if (o1->body == o2->body && o1->body) return 0;

  dColliderEntry *ce = &colliders[o1->type][o2->type];
  int count = 0;
  if (ce->fn) {
    if (ce->reverse) {
      count = (*ce->fn) (o2,o1,flags,contact,skip);
      for (int i=0; i<count; i++) {
	dContactGeom *c = CONTACT(contact,skip*i);
	c->normal[0] = -c->normal[0];
	c->normal[1] = -c->normal[1];
	c->normal[2] = -c->normal[2];
	dxGeom *tmp = c->g1;
	c->g1 = c->g2;
	c->g2 = tmp;
      }
    }
    else {
      count = (*ce->fn) (o1,o2,flags,contact,skip);
    }
  }
  return count;
}

//****************************************************************************
// dxGeom

dxGeom::dxGeom (dSpaceID _space, int is_placeable)
{
  initColliders();

  // setup body vars. invalid type of -1 must be changed by the constructor.
  type = -1;
  flags = GEOM_DIRTY | GEOM_AABB_BAD;
  if (is_placeable) flags |= GEOM_PLACEABLE;
  data = 0;
  body = 0;
  body_next = 0;
  if (is_placeable) {
    dxPosR *pr = (dxPosR*) dAlloc (sizeof(dxPosR));
    pos = pr->pos;
    R = pr->R;
    dSetZero (pos,4);
    dRSetIdentity (R);
  }
  else {
    pos = 0;
    R = 0;
  }

  // setup space vars
  next = 0;
  tome = 0;
  parent_space = 0;
  dSetZero (aabb,6);
  category_bits = ~0;
  collide_bits = ~0;

  // put this geom in a space if required
  if (_space) dSpaceAdd (_space,this);
}


dxGeom::~dxGeom()
{
  if (parent_space) dSpaceRemove (parent_space,this);
  if ((flags & GEOM_PLACEABLE) && !body) dFree (pos,sizeof(dxPosR));
  bodyRemove();
}


int dxGeom::AABBTest (dxGeom *o, dReal aabb[6])
{
  return 1;
}


void dxGeom::bodyRemove()
{
  if (body) {
    // delete this geom from body list
    dxGeom **last = &body->geom, *g = body->geom;
    while (g) {
      if (g == this) {
	*last = g->body_next;
	break;
      }
      last = &g->body_next;
      g = g->body_next;
    }
    body = 0;
    body_next = 0;
  }
}

//****************************************************************************
// misc

dxGeom *dGeomGetBodyNext (dxGeom *geom)
{
  return geom->body_next;
}

//****************************************************************************
// public API for geometry objects

#define CHECK_NOT_LOCKED(space) \
  dUASSERT (!(space && space->lock_count), \
	    "invalid operation for geom in locked space");


void dGeomDestroy (dxGeom *g)
{
  dAASSERT (g);
  delete g;
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


void dGeomSetBody (dxGeom *g, dxBody *b)
{
  dAASSERT (g);
  dUASSERT (g->flags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);

  if (b) {
    if (!g->body) dFree (g->pos,sizeof(dxPosR));
    g->pos = b->pos;
    g->R = b->R;
    dGeomMoved (g);
    if (g->body != b) {
      g->bodyRemove();
      g->bodyAdd (b);
    }
  }
  else {
    if (g->body) {
      dxPosR *pr = (dxPosR*) dAlloc (sizeof(dxPosR));
      g->pos = pr->pos;
      g->R = pr->R;
      memcpy (g->pos,g->body->pos,sizeof(g->pos));
      memcpy (g->R,g->body->R,sizeof(g->R));
      g->bodyRemove();
    }
    // dGeomMoved() should not be called if the body is being set to 0, as the
    // new position of the geom is set to the old position of the body, so the
    // effective position of the geom remains unchanged.
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
  dUASSERT (g->flags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);
  if (g->body) {
    // this will call dGeomMoved (g), so we don't have to
    dBodySetPosition (g->body,x,y,z);
  }
  else {
    g->pos[0] = x;
    g->pos[1] = y;
    g->pos[2] = z;
    dGeomMoved (g);
  }
}


void dGeomSetRotation (dxGeom *g, const dMatrix3 R)
{
  dAASSERT (g && R);
  dUASSERT (g->flags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);
  if (g->body) {
    // this will call dGeomMoved (g), so we don't have to
    dBodySetRotation (g->body,R);
  }
  else {
    memcpy (g->R,R,sizeof(dMatrix3));
    dGeomMoved (g);
  }
}


const dReal * dGeomGetPosition (dxGeom *g)
{
  dAASSERT (g);
  dUASSERT (g->flags & GEOM_PLACEABLE,"geom must be placeable");
  return g->pos;
}


const dReal * dGeomGetRotation (dxGeom *g)
{
  dAASSERT (g);
  dUASSERT (g->flags & GEOM_PLACEABLE,"geom must be placeable");
  return g->R;
}


void dGeomGetAABB (dxGeom *g, dReal aabb[6])
{
  dAASSERT (g);
  dAASSERT (aabb);
  g->recomputeAABB();
  memcpy (aabb,g->aabb,6 * sizeof(dReal));
}


int dGeomIsSpace (dxGeom *g)
{
  dAASSERT (g);
  return IS_SPACE(g);
}


int dGeomGetClass (dxGeom *g)
{
  dAASSERT (g);
  return g->type;
}


void dGeomSetCategoryBits (dxGeom *g, unsigned long bits)
{
  dAASSERT (g);
  CHECK_NOT_LOCKED (g->parent_space);
  g->category_bits = bits;
}


void dGeomSetCollideBits (dxGeom *g, unsigned long bits)
{
  dAASSERT (g);
  CHECK_NOT_LOCKED (g->parent_space);
  g->collide_bits = bits;
}


unsigned long dGeomGetCategoryBits (dxGeom *g)
{
  dAASSERT (g);
  return g->category_bits;
}


unsigned long dGeomGetCollideBits (dxGeom *g)
{
  dAASSERT (g);
  return g->collide_bits;
}

//****************************************************************************
// C interface that lets the user make new classes. this interface is a lot
// more cumbersome than C++ subclassing, which is what is used internally
// in ODE. this API is mainly to support legacy code.

static int num_user_classes = 0;
static dGeomClass user_classes [dMaxUserClasses];


struct dxUserGeom : public dxGeom {
  void *user_data;

  dxUserGeom (int class_num);
  ~dxUserGeom();
  void computeAABB();
  int AABBTest (dxGeom *o, dReal aabb[6]);
};


dxUserGeom::dxUserGeom (int class_num) : dxGeom (0,1)
{
  type = class_num;
  int size = user_classes[type-dFirstUserClass].bytes;
  user_data = dAlloc (size);
  memset (user_data,0,size);
}


dxUserGeom::~dxUserGeom()
{
  dGeomClass *c = &user_classes[type-dFirstUserClass];
  if (c->dtor) c->dtor (this);
  dFree (user_data,c->bytes);
}


void dxUserGeom::computeAABB()
{
  user_classes[type-dFirstUserClass].aabb (this,aabb);
}


int dxUserGeom::AABBTest (dxGeom *o, dReal aabb[6])
{
  dGeomClass *c = &user_classes[type-dFirstUserClass];
  if (c->aabb_test) return c->aabb_test (this,o,aabb);
  else return 1;
}


int dCreateGeomClass (const dGeomClass *c)
{
  dUASSERT(c && c->bytes >= 0 && c->collider && c->aabb,"bad geom class");

  if (num_user_classes >= dMaxUserClasses) {
    dDebug (0,"too many user classes, you must increase the limit and "
	      "recompile ODE");
  }
  user_classes[num_user_classes] = *c;

  // populate the colliders array
  initColliders();
  for (int i=0; i<dGeomNumClasses; i++) {
    setCollider (dFirstUserClass + num_user_classes,i,c->collider (i));
  }

  num_user_classes++;
  return num_user_classes-1 + dFirstUserClass;
}


void * dGeomGetClassData (dxGeom *g)
{
  dUASSERT (g && g->type >= dFirstUserClass &&
	    g->type <= dLastUserClass,"not a custom class");
  dxUserGeom *user = (dxUserGeom*) g;
  return user->user_data;
}


dGeomID dCreateGeom (int classnum)
{
  dUASSERT (classnum >= dFirstUserClass &&
	    classnum <= dLastUserClass,"not a custom class");
  return new dxUserGeom (classnum);
}

//****************************************************************************
// here is where we deallocate any memory that has been globally
// allocated, or free other global resources.

void dCloseODE()
{
  colliders_initialized = 0;
  num_user_classes = 0;
}
