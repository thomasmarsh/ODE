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

// this source file is mostly concerned with the data structures, not the
// numerics.

#include <stdio.h>
#include <malloc.h>		// for alloca under windows
#include "objects.h"
#include "ode/ode.h"
#include "joint.h"
#include "ode/odemath.h"
#include "ode/matrix.h"
#include "step.h"
#include "ode/memory.h"
#include "ode/error.h"

//****************************************************************************
// utility

static inline void initObject (dObject *obj, dxWorld *w)
{
  obj->world = w;
  obj->next = 0;
  obj->tome = 0;
  obj->userdata = 0;
  obj->tag = 0;
}


// add an object `obj' to the list who's head pointer is pointed to by `first'.

static inline void addObjectToList (dObject *obj, dObject **first)
{
  obj->next = *first;
  obj->tome = first;
  if (*first) (*first)->tome = &obj->next;
  (*first) = obj;
}


// remove the object from the linked list

static inline void removeObjectFromList (dObject *obj)
{
  if (obj->next) obj->next->tome = obj->tome;
  *(obj->tome) = obj->next;
  // safeguard
  obj->next = 0;
  obj->tome = 0;
}


// remove the joint from neighbour lists of all connected bodies

static void removeJointReferencesFromAttachedBodies (dxJoint *j)
{
  for (int i=0; i<2; i++) {
    dxBody *body = j->node[i].body;
    if (body) {
      dxJointNode *n = body->firstjoint;
      dxJointNode *last = 0;
      while (n) {
	if (n->joint == j) {
	  if (last) last->next = n->next;
	  else body->firstjoint = n->next;
	  break;
	}
	last = n;
	n = n->next;
      }
    }
  }
  j->node[0].body = 0;
  j->node[0].next = 0;
  j->node[1].body = 0;
  j->node[1].next = 0;
}

//****************************************************************************
// island processing

// this groups all joints and bodies in a world into islands. all objects
// in an island are reachable by going through connected bodies and joints.
// each island can be simulated separately.

static void processIslands (dxWorld *world, dReal stepsize)
{
  dxBody *b,*bb,**body;
  dxJoint *j,**joint;

  // nothing to do if no bodies
  if (world->nb <= 0) return;

  // make arrays for body and joint lists (for a single island) to go into
  body = (dxBody**) alloca (world->nb * sizeof(dxBody*));
  joint = (dxJoint**) alloca (world->nj * sizeof(dxJoint*));
  int bcount = 0;	// number of bodies in `body'
  int jcount = 0;	// number of joints in `joint'

  // set all body/joint tags to 0
  for (b=world->firstbody; b; b=(dxBody*)b->next) b->tag = 0;
  for (j=world->firstjoint; j; j=(dxJoint*)j->next) j->tag = 0;

  // allocate a stack of unvisited bodies in the island. the maximum size of
  // the stack can be the lesser of the number of bodies or joints, because
  // new bodies are only ever added to the stack by going through untagged
  // joints. all the bodies in the stack must be tagged!
  int stackalloc = (world->nj < world->nb) ? world->nj : world->nb;
  dxBody **stack = (dxBody**) alloca (stackalloc * sizeof(dxBody*));

  for (bb=world->firstbody; bb; bb=(dxBody*)bb->next) {
    // get bb = the next untagged body, and tag it
    if (bb->tag) continue;
    bb->tag = 1;

    // tag all bodies and joints starting from bb.
    int stacksize = 0;
    b = bb;
    body[0] = bb;
    bcount = 1;
    jcount = 0;
    goto quickstart;
    while (stacksize > 0) {
      b = stack[--stacksize];	// pop body off stack
      body[bcount++] = b;	// put body on body list
      quickstart:

      // traverse and tag all body's joints, add untagged connected bodies
      // to stack
      for (dxJointNode *n=b->firstjoint; n; n=n->next) {
	if (!n->joint->tag) {
	  n->joint->tag = 1;
	  joint[jcount++] = n->joint;
	  if (n->body && !n->body->tag) {
	    n->body->tag = 1;
	    stack[stacksize++] = n->body;
	  }
	}
      }
      dASSERT(stacksize <= world->nb);
      dASSERT(stacksize <= world->nj);
    }

    // now do something with body and joint lists
    dInternalStepIsland (world,body,bcount,joint,jcount,stepsize);

    // what we've just done may have altered the body/joint tag values.
    // we must make sure that these tags are nonzero.
    int i;
    for (i=0; i<bcount; i++) body[i]->tag = 1;
    for (i=0; i<jcount; i++) joint[i]->tag = 1;
  }

  // if debugging, check that all objects (except for unconnected joints)
  // were tagged
# ifndef dNODEBUG
  for (b=world->firstbody; b; b=(dxBody*)b->next)
    if (!b->tag) dDebug (0,"body not tagged");
  for (j=world->firstjoint; j; j=(dxJoint*)j->next) {
    if (j->node[0].body || j->node[1].body) {
      if (!j->tag) dDebug (0,"attached joint not tagged");
    }
    else {
      if (j->tag) dDebug (0,"unattached joint tagged");
    }
  }
# endif
}

//****************************************************************************
// debugging

// see if an object list loops on itself (if so, it's bad).

static int listHasLoops (dObject *first)
{
  if (first==0 || first->next==0) return 0;
  dObject *a=first,*b=first->next;
  int skip=0;
  while (b) {
    if (a==b) return 1;
    b = b->next;
    if (skip) a = a->next;
    skip ^= 1;
  }
  return 0;
}


// check the validity of the world data structures

static void checkWorld (dxWorld *w)
{
  dxBody *b;
  dxJoint *j;

  // check there are no loops
  if (listHasLoops (w->firstbody)) dDebug (0,"body list has loops");
  if (listHasLoops (w->firstjoint)) dDebug (0,"joint list has loops");

  // check lists are well formed (check `tome' pointers)
  for (b=w->firstbody; b; b=(dxBody*)b->next) {
    if (b->next && b->next->tome != &b->next)
      dDebug (0,"bad tome pointer in body list");
  }
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) {
    if (j->next && j->next->tome != &j->next)
      dDebug (0,"bad tome pointer in joint list");
  }

  // check counts
  int n = 0;
  for (b=w->firstbody; b; b=(dxBody*)b->next) n++;
  if (w->nb != n) dDebug (0,"body count incorrect");
  n = 0;
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) n++;
  if (w->nj != n) dDebug (0,"joint count incorrect");

  // set all tag values to a known value
  static int count = 0;
  count++;
  for (b=w->firstbody; b; b=(dxBody*)b->next) b->tag = count;
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) j->tag = count;

  // check all body/joint world pointers are ok
  for (b=w->firstbody; b; b=(dxBody*)b->next) if (b->world != w)
    dDebug (0,"bad world pointer in body list");
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) if (j->world != w)
    dDebug (0,"bad world pointer in joint list");

  /*
  // check for half-connected joints - actually now these are valid
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) {
    if (j->node[0].body || j->node[1].body) {
      if (!(j->node[0].body && j->node[1].body))
	dDebug (0,"half connected joint found");
    }
  }
  */

  // check that every joint node appears in the joint lists of both bodies it
  // attaches
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) {
    for (int i=0; i<2; i++) {
      if (j->node[i].body) {
	int ok = 0;
	for (dxJointNode *n=j->node[i].body->firstjoint; n; n=n->next) {
	  if (n->joint == j) ok = 1;
	}
	if (ok==0) dDebug (0,"joint not in joint list of attached body");
      }
    }
  }

  // check all body joint lists (correct body ptrs)
  for (b=w->firstbody; b; b=(dxBody*)b->next) {
    for (dxJointNode *n=b->firstjoint; n; n=n->next) {
      if (&n->joint->node[0] == n) {
	if (n->joint->node[1].body != b)
	  dDebug (0,"bad body pointer in joint node of body list (1)");
      }
      else {
	if (n->joint->node[0].body != b)
	  dDebug (0,"bad body pointer in joint node of body list (2)");
      }
      if (n->joint->tag != count) dDebug (0,"bad joint node pointer in body");
    }
  }

  // check all body pointers in joints, check they are distinct
  for (j=w->firstjoint; j; j=(dxJoint*)j->next) {
    if (j->node[0].body && (j->node[0].body == j->node[1].body))
      dDebug (0,"non-distinct body pointers in joint");
    if ((j->node[0].body && j->node[0].body->tag != count) ||
	(j->node[1].body && j->node[1].body->tag != count))
      dDebug (0,"bad body pointer in joint");
  }
}

//****************************************************************************
// body

dxBody *dBodyCreate (dxWorld *w)
{
  dCHECKPTR (w);
  dxBody *b = new dxBody;
  initObject (b,w);
  b->firstjoint = 0;
  dMassSetParameters (&b->mass,1,0,0,0,1,1,1,0,0,0);
  dSetZero (b->invI,4*3);
  b->invI[0] = 1;
  b->invI[5] = 1;
  b->invI[10] = 1;
  b->invMass = 1;
  dSetZero (b->pos,4);
  dSetZero (b->q,4);
  b->q[0] = 1;
  dRSetIdentity (b->R);
  dSetZero (b->lvel,4);
  dSetZero (b->avel,4);
  dSetZero (b->facc,4);
  dSetZero (b->tacc,4);
  addObjectToList (b,(dObject **) &w->firstbody);
  w->nb++;
  return b;
}


void dBodyDestroy (dxBody *b)
{
  dCHECKPTR (b);

  // detach all neighbouring joints, then delete this body.
  dxJointNode *n = b->firstjoint;
  while (n) {
    // sneaky trick to speed up removal of joint references (black magic)
    n->joint->node[(n == n->joint->node)].body = 0;

    dxJointNode *next = n->next;
    n->next = 0;
    removeJointReferencesFromAttachedBodies (n->joint);
    n = next;
  }
  removeObjectFromList (b);
  b->world->nb--;
  delete b;
}


void  dBodySetData (dBodyID b, void *data)
{
  dCHECKPTR(b);
  b->userdata = data;
}


void *dBodyGetData (dBodyID b)
{
  dCHECKPTR(b);
  return b->userdata;
}


void dBodySetPosition (dBodyID b, dReal x, dReal y, dReal z)
{
  dCHECKPTR(b);
  b->pos[0] = x;
  b->pos[1] = y;
  b->pos[2] = z;
}


void dBodySetRotation (dBodyID b, const dMatrix3 R)
{
  dCHECKPTR(b);
  dCHECKPTR(R);
  dQuaternion q;
  dRtoQ (R,q);
  dNormalize4 (q);
  b->q[0] = q[0];
  b->q[1] = q[1];
  b->q[2] = q[2];
  b->q[3] = q[3];
  dQtoR (b->q,b->R);
}


void dBodySetQuaternion (dBodyID b, const dQuaternion q)
{
  dCHECKPTR(b);
  dCHECKPTR(q);
  b->q[0] = q[0];
  b->q[1] = q[1];
  b->q[2] = q[2];
  b->q[3] = q[3];
  dNormalize4 (b->q);
  dQtoR (b->q,b->R);
}


void dBodySetLinearVel  (dBodyID b, dReal x, dReal y, dReal z)
{
  dCHECKPTR(b);
  b->lvel[0] = x;
  b->lvel[1] = y;
  b->lvel[2] = z;
}


void dBodySetAngularVel (dBodyID b, dReal x, dReal y, dReal z)
{
  dCHECKPTR(b);
  b->avel[0] = x;
  b->avel[1] = y;
  b->avel[2] = z;
}


const dReal * dBodyGetPosition (dBodyID b)
{
  dCHECKPTR(b);
  return b->pos;
}


const dReal * dBodyGetRotation (dBodyID b)
{
  dCHECKPTR(b);
  return b->R;
}


const dReal * dBodyGetQuaternion (dBodyID b)
{
  dCHECKPTR(b);
  return b->q;
}


const dReal * dBodyGetLinearVel (dBodyID b)
{
  dCHECKPTR(b);
  return b->lvel;
}


const dReal * dBodyGetAngularVel (dBodyID b)
{
  dCHECKPTR(b);
  return b->avel;
}


void dBodySetMass (dBodyID b, const dMass *mass)
{
  dCHECKPTR(b);
  dCHECKPTR(mass);
  memcpy (&b->mass,mass,sizeof(dMass));
  if (dInvertPDMatrix (b->mass.I,b->invI,3)==0)
    dError (d_ERR_NON_PD,"inertia must be positive definite");
  b->invMass = dRecip(b->mass.mass);
}


void dBodyGetMass (dBodyID b, dMass *mass)
{
  dCHECKPTR(b);
  dCHECKPTR(mass);
  memcpy (mass,&b->mass,sizeof(dMass));
}


void dBodyAddForce (dBodyID b, dReal fx, dReal fy, dReal fz)
{
  dCHECKPTR(b);
  b->facc[0] += fx;
  b->facc[1] += fy;
  b->facc[2] += fz;
}


void dBodyAddTorque (dBodyID b, dReal fx, dReal fy, dReal fz)
{
  dCHECKPTR(b);
  b->tacc[0] += fx;
  b->tacc[1] += fy;
  b->tacc[2] += fz;
}


void dBodyAddRelForce (dBodyID b, dReal fx, dReal fy, dReal fz)
{
  dCHECKPTR(b);
  dVector3 t1,t2;
  t1[0] = fx;
  t1[1] = fy;
  t1[2] = fz;
  t1[3] = 0;
  dMULTIPLY1_331 (t2,b->R,t1);
  b->facc[0] += t2[0];
  b->facc[1] += t2[1];
  b->facc[2] += t2[2];
}


void dBodyAddRelTorque (dBodyID b, dReal fx, dReal fy, dReal fz)
{
  dCHECKPTR(b);
  dVector3 t1,t2;
  t1[0] = fx;
  t1[1] = fy;
  t1[2] = fz;
  t1[3] = 0;
  dMULTIPLY1_331 (t2,b->R,t1);
  b->tacc[0] += t2[0];
  b->tacc[1] += t2[1];
  b->tacc[2] += t2[2];
}

//****************************************************************************
// joints

void dJointInit (dxWorld *w, dxJoint *j)
{
  dCHECKPTR (w);
  dCHECKPTR (j);
  initObject (j,w);
  j->vtable = 0;
  j->flags = 0;
  j->node[0].joint = j;
  j->node[0].body = 0;
  j->node[0].next = 0;
  j->node[1].joint = j;
  j->node[1].body = 0;
  j->node[1].next = 0;
  addObjectToList (j,(dObject **) &w->firstjoint);
  w->nj++;
}


static dxJoint *createJoint (dWorldID w, dJointGroupID group,
			     dxJoint::Vtable *vtable)
{
  dCHECKPTR (w);
  dxJoint *j;
  if (group) {
    j = (dxJoint*) group->stack.alloc (vtable->size);
    group->num++;
  }
  else j = (dxJoint*) dAlloc (vtable->size);
  dJointInit (w,j);
  j->vtable = vtable;
  if (group) j->flags |= dJOINT_INGROUP;
  vtable->init (j);
  return j;
}


dxJoint * dJointCreateBall (dWorldID w, dJointGroupID group)
{
  return createJoint (w,group,&dball_vtable);
}


dxJoint * dJointCreateHinge (dWorldID w, dJointGroupID group)
{
  return createJoint (w,group,&dhinge_vtable);
}


dxJoint * dJointCreateSlider (dWorldID w, dJointGroupID group)
{
  return createJoint (w,group,&dslider_vtable);
}


dxJoint * dJointCreateContact (dWorldID w, dJointGroupID group,
			       const dContact *c)
{
  dxJointContact *j = (dxJointContact *)
    createJoint (w,group,&dcontact_vtable);
  j->contact = *c;
  return j;
}


void dJointDestroy (dxJoint *j)
{
  dCHECKPTR (j);
  if (j->flags & dJOINT_INGROUP) return;
  removeJointReferencesFromAttachedBodies (j);
  removeObjectFromList (j);
  j->world->nj--;
  dFree (j,j->vtable->size);
}


dJointGroupID dJointGroupCreate (int max_size)
{
  dASSERT (max_size > 0);
  dxJointGroup *group = (dxJointGroup*) dAlloc (sizeof(dxJointGroup));
  group->stack.init (max_size);
  group->stack.pushFrame();
  group->num = 0;
  group->firstjoint = (dxJoint*) group->stack.nextAlloc();
  return group;
}


void dJointGroupDestroy (dJointGroupID group)
{
  dJointGroupEmpty (group);
  group->stack.destroy();
  dFree (group,sizeof(dxJointGroup));
}


void dJointGroupEmpty (dJointGroupID group)
{
  // the joints in this group are detached starting from the most recently
  // added (at the top of the stack). this helps ensure that the various
  // linked lists are not traversed too much, as the joints will hopefully
  // be at the start of those lists.

  int i;
  dxJoint **jlist = (dxJoint**) alloca (group->num * sizeof(dxJoint*));
  dxJoint *j = group->firstjoint;
  for (i=0; i < group->num; i++) {
    jlist[i] = j;
    j = (dxJoint*) ( ((char*)j) + j->vtable->size );
  }
  for (i=group->num-1; i >= 0; i--) {
    removeJointReferencesFromAttachedBodies (jlist[i]);
    removeObjectFromList (jlist[i]);
    jlist[i]->world->nj--;
  }
  group->num = 0;
  group->stack.popFrame();
  group->stack.pushFrame();
}


void dJointAttach (dxJoint *joint, dxBody *body1, dxBody *body2)
{
  // check arguments
  dCHECKPTR (joint);
  if (body1 == 0 && body2 == 0)
    dError (d_ERR_BAD_ARGS,"can't have body1==0 and body2==0");
  if (body1 == body2)
    dError (d_ERR_BAD_ARGS,"can't have body1==body2");
  dxWorld *world = joint->world;
  if ((body1 && body1->world != world) ||
      (body2 && body2->world != world))
    dError (d_ERR_SAME_WORLD,"joint and bodies must be in same world");

  // remove any existing body attachments
  if (joint->node[0].body || joint->node[1].body) {
    removeJointReferencesFromAttachedBodies (joint);
  }

  // if a body is zero, make sure that it is body2, so 0 --> node[1].body
  if (body1==0) {
    body1 = body2;
    body2 = 0;
    joint->flags &= (~dJOINT_REVERSE);
  }
  else {
    joint->flags |= dJOINT_REVERSE;
  }

  // attach to new bodies
  joint->node[0].body = body1;
  joint->node[1].body = body2;
  joint->node[1].next = body1->firstjoint;
  body1->firstjoint = &joint->node[1];
  if (body2) {
    joint->node[0].next = body2->firstjoint;
    body2->firstjoint = &joint->node[0];
  }
  else {
    joint->node[0].next = 0;
  }
}


void dJointSetAnchor (dJointID j, dReal x, dReal y, dReal z)
{
  dCHECKPTR(j);
  if (j->vtable->setAnchor) j->vtable->setAnchor (j,x,y,z);
  else dDebug (d_ERR_BAD_ARGS,"can not set anchor on this joint");
}


void dJointSetAxis (dJointID j, dReal x, dReal y, dReal z)
{
  dCHECKPTR(j);
  if (j->vtable->setAxis) j->vtable->setAxis (j,x,y,z);
  else dDebug (d_ERR_BAD_ARGS,"can not set axis on this joint");
}


void dJointGetAnchor (dJointID j, dVector3 result)
{
  dCHECKPTR(j);
  dCHECKPTR(result);
  if (j->vtable->getAnchor) j->vtable->getAnchor (j,result);
  else dDebug (d_ERR_BAD_ARGS,"can not get anchor on this joint");
}


void dJointGetAxis (dJointID j, dVector3 result)
{
  dCHECKPTR(j);
  dCHECKPTR(result);
  if (j->vtable->getAxis) j->vtable->getAxis (j,result);
  else dDebug (d_ERR_BAD_ARGS,"can not get axis on this joint");
}


int dAreConnected (dBodyID b1, dBodyID b2)
{
  dCHECKPTR(b1);
  dCHECKPTR(b2);
  // look through b1's neighbour list for b2
  for (dxJointNode *n=b1->firstjoint; n; n=n->next) {
    if (n->body == b2) return 1;
  }
  return 0;
}

//****************************************************************************
// world

dxWorld * dWorldCreate()
{
  dxWorld *w = new dxWorld;
  w->firstbody = 0;
  w->firstjoint = 0;
  w->nb = 0;
  w->nj = 0;
  dSetZero (w->gravity,4);
  return w;
}


void dWorldDestroy (dxWorld *w)
{
  // delete all bodies and joints
  dCHECKPTR (w);
  dxBody *nextb, *b = w->firstbody;
  while (b) {
    nextb = (dxBody*) b->next;
    delete b;
    b = nextb;
  }
  dxJoint *nextj, *j = w->firstjoint;
  while (j) {
    nextj = (dxJoint*)j->next;
    if (j->flags & dJOINT_INGROUP) {
      dMessage (0,"warning: destroying world containing grouped joints");
    }
    else {
      dFree (j,j->vtable->size);
    }
    j = nextj;
  }
  delete w;
}


void dWorldSetGravity (dWorldID w, dReal x, dReal y, dReal z)
{
  dCHECKPTR(w);
  w->gravity[0] = x;
  w->gravity[1] = y;
  w->gravity[2] = z;
}


void dWorldGetGravity (dWorldID w, dVector3 g)
{
  dCHECKPTR(w);
  g[0] = w->gravity[0];
  g[1] = w->gravity[1];
  g[2] = w->gravity[2];
}


void dWorldStep (dWorldID w, dReal stepsize)
{
  dCHECKPTR(w);
  processIslands (w,stepsize);
}

//****************************************************************************
// testing

#define NUM 100

#define DO(x)


extern "C" void testDynamicsStuff()
{
  int i;
  DO(printf ("testDynamicsStuff()\n"));

  dBodyID body [NUM];
  int nb = 0;
  dJointID joint [NUM];
  int nj = 0;

  for (i=0; i<NUM; i++) body[i] = 0;
  for (i=0; i<NUM; i++) joint[i] = 0;

  DO(printf ("creating world\n"));
  dWorldID w = dWorldCreate();
  checkWorld (w);

  for (;;) {
    if (nb < NUM && dRandReal() > 0.5) {
      DO(printf ("creating body\n"));
      body[nb] = dBodyCreate (w);
      DO(printf ("\t--> %p\n",body[nb]));
      nb++;
      checkWorld (w);
      DO(printf ("%d BODIES, %d JOINTS\n",nb,nj));
    }
    if (nj < NUM && nb > 2 && dRandReal() > 0.5) {
      dBodyID b1 = body [dRand() % nb];
      dBodyID b2 = body [dRand() % nb];
      if (b1 != b2) {
	DO(printf ("creating joint, attaching to %p,%p\n",b1,b2));
	joint[nj] = dJointCreateBall (w,0);
	DO(printf ("\t-->%p\n",joint[nj]));
	checkWorld (w);
	dJointAttach (joint[nj],b1,b2);
	nj++;
	checkWorld (w);
	DO(printf ("%d BODIES, %d JOINTS\n",nb,nj));
      }
    }
    if (nj > 0 && nb > 2 && dRandReal() > 0.5) {
      dBodyID b1 = body [dRand() % nb];
      dBodyID b2 = body [dRand() % nb];
      if (b1 != b2) {
	int k = dRand() % nj;
	DO(printf ("reattaching joint %p\n",joint[k]));
	dJointAttach (joint[k],b1,b2);
	checkWorld (w);
	DO(printf ("%d BODIES, %d JOINTS\n",nb,nj));
      }
    }
    if (nb > 0 && dRandReal() > 0.5) {
      int k = dRand() % nb;
      DO(printf ("destroying body %p\n",body[k]));
      dBodyDestroy (body[k]);
      checkWorld (w);
      for (; k < (NUM-1); k++) body[k] = body[k+1];
      nb--;
      DO(printf ("%d BODIES, %d JOINTS\n",nb,nj));
    }
    if (nj > 0 && dRandReal() > 0.5) {
      int k = dRand() % nj;
      DO(printf ("destroying joint %p\n",joint[k]));
      dJointDestroy (joint[k]);
      checkWorld (w);
      for (; k < (NUM-1); k++) joint[k] = joint[k+1];
      nj--;
      DO(printf ("%d BODIES, %d JOINTS\n",nb,nj));
    }
  }

  /*
  printf ("creating world\n");
  dWorldID w = dWorldCreate();
  checkWorld (w);
  printf ("creating body\n");
  dBodyID b1 = dBodyCreate (w);
  checkWorld (w);
  printf ("creating body\n");
  dBodyID b2 = dBodyCreate (w);
  checkWorld (w);
  printf ("creating joint\n");
  dJointID j = dJointCreateBall (w);
  checkWorld (w);
  printf ("attaching joint\n");
  dJointAttach (j,b1,b2);
  checkWorld (w);
  printf ("destroying joint\n");
  dJointDestroy (j);
  checkWorld (w);
  printf ("destroying body\n");
  dBodyDestroy (b1);
  checkWorld (w);
  printf ("destroying body\n");
  dBodyDestroy (b2);
  checkWorld (w);
  printf ("destroying world\n");
  dWorldDestroy (w);
  */
}
