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

#ifndef _ODE_JOINT_H_
#define _ODE_JOINT_H_


#include "objects.h"
#include "ode/contact.h"
#include "stack.h"


// joint flags
enum {
  // if this flag is set, the joint was allocated in a joint group
  dJOINT_INGROUP = 1,

  // if this flag is set, the joint was attached with arguments (0,body).
  // our convention is to treat all attaches as (body,0), i.e. so node[0].body
  // is always nonzero, so this flag records the fact that the arguments were
  // swapped.
  dJOINT_REVERSE = 2
};


// there are two of these nodes in the joint, one for each connection to a
// body. these are node of a linked list kept by each body of it's connecting
// joints. but note that the body pointer in each node points to the body that
// makes use of the *other* node, not this node. this trick makes it a bit
// easier to traverse the body/joint graph.

struct dxJointNode {
  dxJoint *joint;		// pointer to enclosing dxJoint object
  dxBody *body;			// *other* body this joint is connected to
  dxJointNode *next;		// next node in body's list of connected joints
};


struct dxJoint : public dObject {
  // naming convention: the "first" body this is connected to is node[0].body,
  // and the "second" body is node[1].body. if this joint is only connected
  // to one body then the second body is 0.

  // info returned by getInfo1 function. the constraint dimension is m (<=6).
  // i.e. that is the total number of rows in the jacobian. `nub' is the
  // number of unbounded variables (which have lo,hi = -/+ infinity).
  // variables).

  struct Info1 {
    int m,nub;
  };

  // info returned by getInfo2 function

  struct Info2 {
    // integrator parameters: frames per second (1/stepsize), error reduction
    // parameter (0..1).
    dReal fps,erp;

    // for the first and second body, pointers to two (linear and angular)
    // n*3 jacobian sub matrices, stored by rows. these matrices will have
    // been initialized to 0 on entry. if the second body is zero then the
    // J2xx pointers may be 0.
    dReal *J1l,*J1a,*J2l,*J2a;

    // elements to jump from one row to the next in J's
    int rowskip;

    // right hand side of the equation J*a = c. this vector will be set to
    // zero on entry.
    dReal *c;

    // lo and hi limits for variables (set to -/+ infinity on entry).
    dReal *lo,*hi;
  };

  // virtual function table: size of the joint structure, function pointers.
  // we do it this way instead of using C++ virtual functions because
  // sometimes we need to allocate joints ourself within a memory pool.

  typedef void init_fn (dxJoint *joint);
  typedef void getInfo1_fn (dxJoint *joint, Info1 *info);
  typedef void getInfo2_fn (dxJoint *joint, Info2 *info);
  struct Vtable {
    int size;
    init_fn *init;
    getInfo1_fn *getInfo1;
    getInfo2_fn *getInfo2;
  };

  Vtable *vtable;		// virtual function table
  int flags;			// dJOINT_xxx flags
  dxJointNode node[2];		// connections to bodies. node[1].body can be 0
};


struct dxJointGroup {
  int num;		// number of joints on the stack
  dxJoint *firstjoint;	// address of first joint on the stack
  dStack stack;		// a stack of (possibly differently sized) dxJoint
};			// objects.


// common limit and motor information for a single joint axis of movement
struct dxJointLimitMotor {
  dReal vel,fmax;		// powered joint: velocity, max force
  dReal lostop,histop;		// joint limits, relative to initial position
  dReal fudge_factor;		// when powering away from joint limits
  // variables used between getInfo1() and getInfo2()
  int limit;			// 0=free, 1=at lo limit, 2=at hi limit
  dReal limit_err;		// if at limit, amount over limit

  void init();
  void set (int num, dReal value);
  dReal get (int num);
  int testRotationalLimit (dxBody *body1, dxBody *body2,
			   dVector3 axis1, dQuaternion qrel);
  void addRotationalLimot (dxJoint *joint, dxJoint::Info2 *info, int row,
			   dVector3 ax1);
  void addLinearLimot (dxJoint *joint, dxJoint::Info2 *info, int row,
		       dVector3 ax1);
};


// ball and socket

struct dxJointBall : public dxJoint {
  dVector3 anchor1;		// anchor w.r.t first body
  dVector3 anchor2;		// anchor w.r.t second body
};
extern struct dxJoint::Vtable __dball_vtable;


// hinge

struct dxJointHinge : public dxJoint {
  dVector3 anchor1;		// anchor w.r.t first body
  dVector3 anchor2;		// anchor w.r.t second body
  dVector3 axis1;		// axis w.r.t first body
  dVector3 axis2;		// axis w.r.t second body
  dQuaternion qrel;		// initial relative rotation body1 -> body2
  dxJointLimitMotor limot;	// limit and motor information
};
extern struct dxJoint::Vtable __dhinge_vtable;


// slider. if body2 is 0 then qrel is the absolute rotation of body1 and
// offset is the position of body1 center along axis1.

struct dxJointSlider : public dxJoint {
  dVector3 axis1;		// axis w.r.t first body
  dQuaternion qrel;		// initial relative rotation body1 -> body2
  dVector3 offset;		// point relative to body2 that should be
				// aligned with body1 center along axis1
  dxJointLimitMotor limot;	// limit and motor information
};
extern struct dxJoint::Vtable __dslider_vtable;


// contact

struct dxJointContact : public dxJoint {
  int the_m;			// number of rows computed by getInfo1
  dContact contact;
};
extern struct dxJoint::Vtable __dcontact_vtable;


// rotational motor

struct dxJointRMotor : public dxJoint {
  dVector3 axis1;		// axis w.r.t first body
  dVector3 axis2;		// axis w.r.t second body
  dQuaternion qrel;		// initial relative rotation body1 -> body2
  dReal vel,tmax;		// motor desired velocity, maximum torque
};
extern struct dxJoint::Vtable __drmotor_vtable;


// hinge 2

struct dxJointHinge2 : public dxJoint {
  dVector3 anchor1;		// anchor w.r.t first body
  dVector3 anchor2;		// anchor w.r.t second body
  dVector3 axis1;		// axis 1 w.r.t first body
  dVector3 axis2;		// axis 2 w.r.t second body
  dReal c0,s0;			// cos,sin of desired angle between axis 1,2
};
extern struct dxJoint::Vtable __dhinge2_vtable;



#endif
