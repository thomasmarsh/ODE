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

#ifndef _ODE_ODECPP_H_
#define _ODE_ODECPP_H_
#ifdef __cplusplus

#include "ode/error.h"

// C++ interface for everything


class dWorld {
  dWorldID _id;

  dWorld (dWorld &) { dDebug (0,"bad"); }
  void operator= (dWorld &) { dDebug (0,"bad"); }

public:
  dWorld()
    { _id = dWorldCreate(); }
  ~dWorld()
    { dWorldDestroy (_id); }
  dWorldID id()
    { return _id; }

  void setGravity (dReal x, dReal y, dReal z)
    { dWorldSetGravity (_id,x,y,z); }
  void getGravity (dVector3 g)
    { dWorldGetGravity (_id,g); }
  void step (dReal stepsize)
    { dWorldStep (_id,stepsize); }
};


class dBody {
  dBodyID _id;

  dBody (dBody &) { dDebug (0,"bad"); }
  void operator= (dBody &) { dDebug (0,"bad"); }

public:
  dBody ()
    { _id = 0; }
  dBody (dWorld &world)
    { _id = dBodyCreate (world.id()); }
  ~dBody()
    { dBodyDestroy (_id); }
  void create (dWorld &world)
    { if (_id) dBodyDestroy (_id); _id = dBodyCreate (world.id()); }
  dBodyID id()
    { return _id; }

  void setData (void *data)
    { dBodySetData (_id,data); }
  void *getData()
    { return dBodyGetData (_id); }

  void setPosition (dReal x, dReal y, dReal z)
    { dBodySetPosition (_id,x,y,z); }
  void setRotation (dMatrix3 R)
    { dBodySetRotation (_id,R); }
  void setQuaternion (dQuaternion q)
    { dBodySetQuaternion (_id,q); }
  void setLinearVel  (dReal x, dReal y, dReal z)
    { dBodySetLinearVel (_id,x,y,z); }
  void setAngularVel (dReal x, dReal y, dReal z)
    { dBodySetAngularVel (_id,x,y,z); }

  dReal * getPosition()
    { return dBodyGetPosition (_id); }
  dReal * getRotation()
    { return dBodyGetRotation (_id); }
  dReal * getQuaternion()
    { return dBodyGetQuaternion (_id); }
  dReal * getLinearVel()
    { return dBodyGetLinearVel (_id); }
  dReal * getAngularVel()
    { return dBodyGetAngularVel (_id); }

  void setMass (dMass *mass)
    { dBodySetMass (_id,mass); }
  void getMass (dMass *mass)
    { dBodyGetMass (_id,mass); }

  //@@@
  //  void setMass (dMassObject &mass)
  //    { dBodySetMass (_id,mass.massPtr()) }
  //  void getMass (dMassObject &mass)
  //    { dBodyGetMass (_id,mass.massPtr()); }

  void addForce (dReal fx, dReal fy, dReal fz)
    { dBodyAddForce (_id, fx, fy, fz); }
  void addTorque (dReal fx, dReal fy, dReal fz)
    { dBodyAddTorque (_id, fx, fy, fz); }
  void addRelForce (dReal fx, dReal fy, dReal fz)
    { dBodyAddRelForce (_id, fx, fy, fz); }
  void addRelTorque (dReal fx, dReal fy, dReal fz)
    { dBodyAddRelTorque (_id, fx, fy, fz); }
  void addForceAtPos (dReal fx, dReal fy, dReal fz,
		      dReal px, dReal py, dReal pz)
    { dBodyAddForceAtPos (_id, fx, fy, fz, px, py, pz); }
  void addRelForceAtPos (dReal fx, dReal fy, dReal fz,
			 dReal px, dReal py, dReal pz)
    { dBodyAddRelForceAtPos (_id, fx, fy, fz, px, py, pz); }
  void addRelForceAtRelPos (dReal fx, dReal fy, dReal fz,
			    dReal px, dReal py, dReal pz)
    { dBodyAddRelForceAtRelPos (_id, fx, fy, fz, px, py, pz); }

  void getPointPos (dReal px, dReal py, dReal pz, dVector3 result)
    { dBodyGetPointPos (_id, px, py, pz, result); }
  void getPointVel (dReal px, dReal py, dReal pz, dVector3 result)
    { dBodyGetPointVel (_id, px, py, pz, result); }
  void getPointRelVel (dReal px, dReal py, dReal pz, dVector3 result)
    { dBodyGetPointRelVel (_id, px, py, pz, result); }

  int isConnectedTo (dBody &b)
    { return dAreConnected (_id,b._id); }
};


class dJointGroup {
  dJointGroupID _id;

  dJointGroup (dJointGroup &) { dDebug (0,"bad"); }
  void operator= (dJointGroup &) { dDebug (0,"bad"); }

public:
  dJointGroup()
    { _id = 0; }
  dJointGroup (int max_size)
    { _id = dJointGroupCreate (max_size); }
  ~dJointGroup()
    { dJointGroupDestroy (_id); }
  void create (int max_size)
    { if (_id) dJointGroupDestroy (_id); _id = dJointGroupCreate (max_size); }
  dJointGroupID id()
    { return _id; }

  void empty()
    { dJointGroupEmpty (_id); }
};


class dJoint {
  dJointID _id;

  dJoint (dJoint &) { dDebug (0,"bad"); }
  void operator= (dJoint &) { dDebug (0,"bad"); }

public:
  dJoint()
    { _id = 0; }
  ~dJoint()
    { dJointDestroy (_id); }
  dJointID id()
    { return _id; }

  void createBall (dWorld &world, dJointGroup *group=0) {
    if (_id) dJointDestroy (_id);
    _id = dJointCreateBall (world.id(), group ? group->id() : 0);
  }
  void createHinge (dWorld &world, dJointGroup *group=0) {
    if (_id) dJointDestroy (_id);
    _id = dJointCreateHinge (world.id(), group ? group->id() : 0);
  }
  void createSlider (dWorld &world, dJointGroup *group=0) {
    if (_id) dJointDestroy (_id);
    _id = dJointCreateSlider (world.id(), group ? group->id() : 0);
  }
  void createContact (dWorld &world, dJointGroup *group, dContact *contact) {
    if (_id) dJointDestroy (_id);
    _id = dJointCreateContact (world.id(), group ? group->id() : 0, contact);
  }

  void attach (dBody &body1, dBody &body2)
    { dJointAttach (_id, body1.id(), body2.id()); }
  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetAnchor (_id, x, y, z); }
  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetAxis (_id, x, y, z); }
  void getAnchor (dVector3 result)
    { dJointGetAnchor (_id, result); }
  void getAxis (dVector3 result)
    { dJointGetAxis (_id, result); }
};



// @@@ objects for Geom and Space


#endif
#endif
