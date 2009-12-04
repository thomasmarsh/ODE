/*************************************************************************
 *									 *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.	 *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org 	 *
 *									 *
 * This library is free software; you can redistribute it and/or	 *
 * modify it under the terms of EITHER: 				 *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *	 Software Foundation; either version 2.1 of the License, or (at  *
 *	 your option) any later version. The text of the GNU Lesser	 *
 *	 General Public License is included with this library in the	 *
 *	 file LICENSE.TXT.						 *
 *   (2) The BSD-style license that is included with this library in	 *
 *	 the file LICENSE-BSD.TXT.					 *
 *									 *
 * This library is distributed in the hope that it will be useful,	 *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of	 *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files	 *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.			 *
 *									 *
 *************************************************************************/

/* C++ interface for non-collision stuff */


#ifndef _ODE_ODECPP_H_
#define _ODE_ODECPP_H_
#ifdef __cplusplus




//namespace ode {


class dWorldSimpleIDContainer {
protected:
	dWorldID _id;

	dWorldSimpleIDContainer(): _id(0) {}
	~dWorldSimpleIDContainer() { destroy(); }

	void destroy() { 
		if (_id) {
			dWorldDestroy(_id); 
			_id = 0;
		}
	}
};

class dWorldDynamicIDContainer: public dWorldSimpleIDContainer {
protected:
	virtual ~dWorldDynamicIDContainer() {}
};

template <class dWorldTemplateBase>
class dWorldTemplate: public dWorldTemplateBase {
  // intentionally undefined, don't use these
  dWorldTemplate (const dWorldTemplate<dWorldTemplateBase> &);
  void operator= (const dWorldTemplate<dWorldTemplateBase> &);

public:
  dWorldTemplate()
    { _id = dWorldCreate(); }

  dWorldID id() const
    { return _id; }
  operator dWorldID() const
    { return _id; }

  void setGravity (dReal x, dReal y, dReal z)
    { dWorldSetGravity (_id,x,y,z); }
  void setGravity (const dVector3 g)
    { setGravity (g[0], g[1], g[2]); }
  void getGravity (dVector3 g) const
    { dWorldGetGravity (_id,g); }

  void setERP (dReal erp)
    { dWorldSetERP(_id, erp); }
  dReal getERP() const
    { return dWorldGetERP(_id); }

  void setCFM (dReal cfm)
    { dWorldSetCFM(_id, cfm); }
  dReal getCFM() const
    { return dWorldGetCFM(_id); }

  void step (dReal stepsize)
    { dWorldStep (_id,stepsize); }

  void quickStep(dReal stepsize)
    { dWorldQuickStep (_id, stepsize); }
  void setQuickStepNumIterations(int num)
    { dWorldSetQuickStepNumIterations (_id, num); }
  int getQuickStepNumIterations() const
    { return dWorldGetQuickStepNumIterations (_id); }
  void setQuickStepW(dReal over_relaxation)
    { dWorldSetQuickStepW (_id, over_relaxation); }
  dReal getQuickStepW() const
    { return dWorldGetQuickStepW (_id); }

  void  setAutoDisableLinearThreshold (dReal threshold) 
    { dWorldSetAutoDisableLinearThreshold (_id,threshold); }
  dReal getAutoDisableLinearThreshold() const
    { return dWorldGetAutoDisableLinearThreshold (_id); }
  void setAutoDisableAngularThreshold (dReal threshold)
    { dWorldSetAutoDisableAngularThreshold (_id,threshold); }
  dReal getAutoDisableAngularThreshold() const
    { return dWorldGetAutoDisableAngularThreshold (_id); }
  void setAutoDisableSteps (int steps)
    { dWorldSetAutoDisableSteps (_id,steps); }
  int getAutoDisableSteps() const
    { return dWorldGetAutoDisableSteps (_id); }
  void setAutoDisableTime (dReal time)
    { dWorldSetAutoDisableTime (_id,time); }
  dReal getAutoDisableTime() const
    { return dWorldGetAutoDisableTime (_id); }
  void setAutoDisableFlag (int do_auto_disable)
    { dWorldSetAutoDisableFlag (_id,do_auto_disable); }
  int getAutoDisableFlag() const
    { return dWorldGetAutoDisableFlag (_id); }

  dReal getLinearDampingThreshold() const
    { return dWorldGetLinearDampingThreshold(_id); }
  void setLinearDampingThreshold(dReal threshold)
    { dWorldSetLinearDampingThreshold(_id, threshold); }
  dReal getAngularDampingThreshold() const
    { return dWorldGetAngularDampingThreshold(_id); }
  void setAngularDampingThreshold(dReal threshold)
    { dWorldSetAngularDampingThreshold(_id, threshold); }
  dReal getLinearDamping() const
    { return dWorldGetLinearDamping(_id); }
  void setLinearDamping(dReal scale)
    { dWorldSetLinearDamping(_id, scale); }
  dReal getAngularDamping() const
    { return dWorldGetAngularDamping(_id); }
  void setAngularDamping(dReal scale)
    { dWorldSetAngularDamping(_id, scale); }
  void setDamping(dReal linear_scale, dReal angular_scale)
    { dWorldSetDamping(_id, linear_scale, angular_scale); }

  dReal getMaxAngularSpeed() const
    { return dWorldGetMaxAngularSpeed(_id); }
  void setMaxAngularSpeed(dReal max_speed)
    { dWorldSetMaxAngularSpeed(_id, max_speed); }

  void setContactSurfaceLayer(dReal depth)
    { dWorldSetContactSurfaceLayer (_id, depth); }
  dReal getContactSurfaceLayer() const
    { return dWorldGetContactSurfaceLayer (_id); }

  void impulseToForce (dReal stepsize, dReal ix, dReal iy, dReal iz,
		       dVector3 force)
    { dWorldImpulseToForce (_id,stepsize,ix,iy,iz,force); }
};


class dBodySimpleIDContainer {
protected:
	dBodyID _id;

	dBodySimpleIDContainer(): _id(0) {}
	~dBodySimpleIDContainer() { destroy(); }

	void destroy() { 
		if (_id) {
			dBodyDestroy(_id); 
			_id = 0;
		}
	}
};

class dBodyDynamicIDContainer: public dBodySimpleIDContainer {
protected:
	virtual ~dBodyDynamicIDContainer() {}
};

template <class dBodyTemplateBase, class dWorldTemplateBase>
class dBodyTemplate: public dBodyTemplateBase {
  // intentionally undefined, don't use these
  dBodyTemplate (const dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase> &);
  void operator= (const dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase> &);

public:
  dBodyTemplate()
    { }
  dBodyTemplate (dWorldID world)
    { _id = dBodyCreate (world); }
  dBodyTemplate (dWorldTemplate<dWorldTemplateBase>& world)
    { _id = dBodyCreate (world.id()); }

  void create (dWorldID world) {
    destroy();
    _id = dBodyCreate (world);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world) {
    create(world.id());
  }

  dBodyID id() const
    { return _id; }
  operator dBodyID() const
    { return _id; }

  void setData (void *data)
    { dBodySetData (_id,data); }
  void *getData() const
    { return dBodyGetData (_id); }

  void setPosition (dReal x, dReal y, dReal z)
    { dBodySetPosition (_id,x,y,z); }
  void setPosition (const dVector3 p)
    { setPosition(p[0], p[1], p[2]); }

  void setRotation (const dMatrix3 R)
    { dBodySetRotation (_id,R); }
  void setQuaternion (const dQuaternion q)
    { dBodySetQuaternion (_id,q); }
  void setLinearVel (dReal x, dReal y, dReal z)
    { dBodySetLinearVel (_id,x,y,z); }
  void setLinearVel (const dVector3 v)
    { setLinearVel(v[0], v[1], v[2]); }
  void setAngularVel (dReal x, dReal y, dReal z)
    { dBodySetAngularVel (_id,x,y,z); }
  void setAngularVel (const dVector3 v)
    { setAngularVel (v[0], v[1], v[2]); }

  const dReal * getPosition() const
    { return dBodyGetPosition (_id); }
  const dReal * getRotation() const
    { return dBodyGetRotation (_id); }
  const dReal * getQuaternion() const
    { return dBodyGetQuaternion (_id); }
  const dReal * getLinearVel() const
    { return dBodyGetLinearVel (_id); }
  const dReal * getAngularVel() const
    { return dBodyGetAngularVel (_id); }

  void setMass (const dMass *mass)
    { dBodySetMass (_id,mass); }
  void setMass (const dMass &mass)
    { setMass (&mass); }
  dMass getMass () const
    { dMass mass; dBodyGetMass (_id,&mass); return mass; }

  void addForce (dReal fx, dReal fy, dReal fz)
    { dBodyAddForce (_id, fx, fy, fz); }
  void addForce (const dVector3 f)
    { addForce (f[0], f[1], f[2]); }
  void addTorque (dReal fx, dReal fy, dReal fz)
    { dBodyAddTorque (_id, fx, fy, fz); }
  void addTorque (const dVector3 t)
    { addTorque(t[0], t[1], t[2]); }

  void addRelForce (dReal fx, dReal fy, dReal fz)
    { dBodyAddRelForce (_id, fx, fy, fz); }
  void addRelForce (const dVector3 f)
    { addRelForce (f[0], f[1], f[2]); }
  void addRelTorque (dReal fx, dReal fy, dReal fz)
    { dBodyAddRelTorque (_id, fx, fy, fz); }
  void addRelTorque (const dVector3 t)
    { addRelTorque (t[0], t[1], t[2]); }

  void addForceAtPos (dReal fx, dReal fy, dReal fz,
		      dReal px, dReal py, dReal pz)
    { dBodyAddForceAtPos (_id, fx, fy, fz, px, py, pz); }
  void addForceAtPos (const dVector3 f, const dVector3 p)
    { addForceAtPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  void addForceAtRelPos (dReal fx, dReal fy, dReal fz,
                         dReal px, dReal py, dReal pz)
    { dBodyAddForceAtRelPos (_id, fx, fy, fz, px, py, pz); }
  void addForceAtRelPos (const dVector3 f, const dVector3 p)
    { addForceAtRelPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  void addRelForceAtPos (dReal fx, dReal fy, dReal fz,
			 dReal px, dReal py, dReal pz)
    { dBodyAddRelForceAtPos (_id, fx, fy, fz, px, py, pz); }
  void addRelForceAtPos (const dVector3 f, const dVector3 p)
    { addRelForceAtPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  void addRelForceAtRelPos (dReal fx, dReal fy, dReal fz,
			    dReal px, dReal py, dReal pz)
    { dBodyAddRelForceAtRelPos (_id, fx, fy, fz, px, py, pz); }
  void addRelForceAtRelPos (const dVector3 f, const dVector3 p)
    { addRelForceAtRelPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  const dReal * getForce() const
    { return dBodyGetForce(_id); }
  const dReal * getTorque() const
    { return dBodyGetTorque(_id); }
  void setForce (dReal x, dReal y, dReal z)
    { dBodySetForce (_id,x,y,z); }
  void setForce (const dVector3 f)
    { setForce (f[0], f[1], f[2]); }
  void setTorque (dReal x, dReal y, dReal z)
    { dBodySetTorque (_id,x,y,z); }
  void setTorque (const dVector3 t)
  { setTorque (t[0], t[1], t[2]); }

  void setDynamic()
    { dBodySetDynamic (_id); }
  void setKinematic()
    { dBodySetKinematic (_id); }
  bool isKinematic() const
    { return dBodyIsKinematic (_id) != 0; }

  void enable()
    { dBodyEnable (_id); }
  void disable()
    { dBodyDisable (_id); }
  bool isEnabled() const
    { return dBodyIsEnabled (_id) != 0; }

  void getRelPointPos (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetRelPointPos (_id, px, py, pz, result); }
  void getRelPointPos (const dVector3 p, dVector3 result) const
    { getRelPointPos (p[0], p[1], p[2], result); }

  void getRelPointVel (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetRelPointVel (_id, px, py, pz, result); }
  void getRelPointVel (const dVector3 p, dVector3 result) const
    { getRelPointVel (p[0], p[1], p[2], result); }

  void getPointVel (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetPointVel (_id, px, py, pz, result); }
  void getPointVel (const dVector3 p, dVector3 result) const
    { getPointVel (p[0], p[1], p[2], result); }

  void getPosRelPoint (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetPosRelPoint (_id, px, py, pz, result); }
  void getPosRelPoint (const dVector3 p, dVector3 result) const
    { getPosRelPoint (p[0], p[1], p[2], result); }

  void vectorToWorld (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyVectorToWorld (_id, px, py, pz, result); }
  void vectorToWorld (const dVector3 p, dVector3 result) const
    { vectorToWorld (p[0], p[1], p[2], result); }

  void vectorFromWorld (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyVectorFromWorld (_id,px,py,pz,result); }
  void vectorFromWorld (const dVector3 p, dVector3 result) const
    { vectorFromWorld (p[0], p[1], p[2], result); }

  void setFiniteRotationMode (bool mode)
    { dBodySetFiniteRotationMode (_id, mode); }

  void setFiniteRotationAxis (dReal x, dReal y, dReal z)
    { dBodySetFiniteRotationAxis (_id, x, y, z); }
  void setFiniteRotationAxis (const dVector3 a)
    { setFiniteRotationAxis (a[0], a[1], a[2]); }

  bool getFiniteRotationMode() const
    { return dBodyGetFiniteRotationMode (_id) != 0; }
  void getFiniteRotationAxis (dVector3 result) const
    { dBodyGetFiniteRotationAxis (_id, result); }

  int getNumJoints() const
    { return dBodyGetNumJoints (_id); }
  dJointID getJoint (int index) const
    { return dBodyGetJoint (_id, index); }

  void setGravityMode (bool mode)
    { dBodySetGravityMode (_id,mode); }
  bool getGravityMode() const
    { return dBodyGetGravityMode (_id) != 0; }

  bool isConnectedTo (dBodyID body) const
    { return dAreConnected (_id, body) != 0; }

  void  setAutoDisableLinearThreshold (dReal threshold) 
    { dBodySetAutoDisableLinearThreshold (_id,threshold); }
  dReal getAutoDisableLinearThreshold() const
    { return dBodyGetAutoDisableLinearThreshold (_id); }
  void setAutoDisableAngularThreshold (dReal threshold)
    { dBodySetAutoDisableAngularThreshold (_id,threshold); }
  dReal getAutoDisableAngularThreshold() const
    { return dBodyGetAutoDisableAngularThreshold (_id); }
  void setAutoDisableSteps (int steps)
    { dBodySetAutoDisableSteps (_id,steps); }
  int getAutoDisableSteps() const
    { return dBodyGetAutoDisableSteps (_id); }
  void setAutoDisableTime (dReal time)
    { dBodySetAutoDisableTime (_id,time); }
  dReal getAutoDisableTime() const
    { return dBodyGetAutoDisableTime (_id); }
  void setAutoDisableFlag (bool do_auto_disable)
    { dBodySetAutoDisableFlag (_id,do_auto_disable); }
  bool getAutoDisableFlag() const
    { return dBodyGetAutoDisableFlag (_id) != 0; }

  dReal getLinearDamping() const
    { return dBodyGetLinearDamping(_id); }
  void setLinearDamping(dReal scale)
    { dBodySetLinearDamping(_id, scale); }
  dReal getAngularDamping() const
    { return dBodyGetAngularDamping(_id); }
  void setAngularDamping(dReal scale)
    { dBodySetAngularDamping(_id, scale); }
  void setDamping(dReal linear_scale, dReal angular_scale)
    { dBodySetDamping(_id, linear_scale, angular_scale); }
  dReal getLinearDampingThreshold() const
    { return dBodyGetLinearDampingThreshold(_id); }
  void setLinearDampingThreshold(dReal threshold) const
    { dBodySetLinearDampingThreshold(_id, threshold); }
  dReal getAngularDampingThreshold() const
    { return dBodyGetAngularDampingThreshold(_id); }
  void setAngularDampingThreshold(dReal threshold)
    { dBodySetAngularDampingThreshold(_id, threshold); }
  void setDampingDefaults()
    { dBodySetDampingDefaults(_id); }

  dReal getMaxAngularSpeed() const
    { return dBodyGetMaxAngularSpeed(_id); }
  void setMaxAngularSpeed(dReal max_speed)
    { dBodySetMaxAngularSpeed(_id, max_speed); }

  bool getGyroscopicMode() const
    { return dBodyGetGyroscopicMode(_id) != 0; }
  void setGyroscopicMode(bool mode)
    { dBodySetGyroscopicMode(_id, mode); }

};


class dJointGroupSimpleIDContainer {
protected:
	dJointGroupID _id;

	dJointGroupSimpleIDContainer(): _id(0) {}
	~dJointGroupSimpleIDContainer() { destroy(); }

	void destroy() { 
		if (_id) {
			dJointGroupDestroy(_id); 
			_id = 0;
		}
	}
};

class dJointGroupDynamicIDContainer: public dJointGroupSimpleIDContainer {
protected:
	virtual ~dJointGroupDynamicIDContainer() {}
};

template <class dJointGroupTemplateBase>
class dJointGroupTemplate: public dJointGroupTemplateBase {
  // intentionally undefined, don't use these
  dJointGroupTemplate (const dJointGroupTemplate<dJointGroupTemplateBase> &);
  void operator= (const dJointGroupTemplate<dJointGroupTemplateBase> &);

public:
  dJointGroupTemplate ()
    { _id = dJointGroupCreate (0); }
  
  void create () {
    destroy();
    _id = dJointGroupCreate (0);
  }

  dJointGroupID id() const
    { return _id; }
  operator dJointGroupID() const
    { return _id; }

  void empty()
    { dJointGroupEmpty (_id); }
  void clear()
    { empty(); }
};


class dJointSimpleIDContainer {
protected:
	dJointID _id;

	dJointSimpleIDContainer(): _id(0) {}
	~dJointSimpleIDContainer() { destroy(); }

	void destroy() { 
		if (_id) {
			dJointDestroy (_id); 
			_id = 0;
		}
	}
};

class dJointDynamicIDContainer: public dJointSimpleIDContainer {
protected:
	virtual ~dJointDynamicIDContainer() {}
};

template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dJointTemplate: public dJointTemplateBase {
private:
  // intentionally undefined, don't use these
  dJointTemplate (const dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ;
  void operator= (const dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

protected:
  dJointTemplate() // don't let user construct pure dJointTemplate objects
    { }

public:
  dJointID id() const
    { return _id; }
  operator dJointID() const
    { return _id; }

  int getNumBodies() const
    { return dJointGetNumBodies(_id); }

  void attach (dBodyID body1, dBodyID body2)
    { dJointAttach (_id, body1, body2); }
  void attach (dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase>& body1, dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase>& body2)
    { attach(body1.id(), body2.id()); }

  void enable()
    { dJointEnable (_id); }
  void disable()
    { dJointDisable (_id); }
  bool isEnabled() const
    { return dJointIsEnabled (_id) != 0; }

  void setData (void *data)
    { dJointSetData (_id, data); }
  void *getData() const
    { return dJointGetData (_id); }

  dJointType getType() const
    { return dJointGetType (_id); }

  dBodyID getBody (int index) const
    { return dJointGetBody (_id, index); }

  void setFeedback(dJointFeedback *fb)
    { dJointSetFeedback(_id, fb); }
  dJointFeedback *getFeedback() const
    { return dJointGetFeedback(_id); }

  // If not implemented it will do nothing as describe in the doc
  virtual void setParam (int, dReal) {};
  virtual dReal getParam (int) const { return 0; }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dBallJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
private:
  // intentionally undefined, don't use these
  dBallJointTemplate (const dBallJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator= (const dBallJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dBallJointTemplate() { }
  dBallJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateBall (world, group); }
  dBallJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateBall (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateBall (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetBallAnchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void getAnchor (dVector3 result) const
    { dJointGetBallAnchor (_id, result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetBallAnchor2 (_id, result); }
  virtual void setParam (int parameter, dReal value)
    { dJointSetBallParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetBallParam (_id, parameter); }
  // TODO: expose params through methods
} ;


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dHingeJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dHingeJointTemplate (const dHingeJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dHingeJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dHingeJointTemplate() { }
  dHingeJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateHinge (world, group); }
  dHingeJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateHinge (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateHinge (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }
  
  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetHingeAnchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void getAnchor (dVector3 result) const
    { dJointGetHingeAnchor (_id, result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetHingeAnchor2 (_id, result); }

  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetHingeAxis (_id, x, y, z); }
  void setAxis (const dVector3 a)
    { setAxis(a[0], a[1], a[2]); }
  void getAxis (dVector3 result) const
    { dJointGetHingeAxis (_id, result); }

  dReal getAngle() const
    { return dJointGetHingeAngle (_id); }
  dReal getAngleRate() const
    { return dJointGetHingeAngleRate (_id); }

  virtual void setParam (int parameter, dReal value)
    { dJointSetHingeParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetHingeParam (_id, parameter); }
  // TODO: expose params through methods

  void addTorque (dReal torque)
	{ dJointAddHingeTorque(_id, torque); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dSliderJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dSliderJointTemplate (const dSliderJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dSliderJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dSliderJointTemplate() { }
  dSliderJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateSlider (world, group); }
  dSliderJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateSlider (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateSlider (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetSliderAxis (_id, x, y, z); }
  void setAxis (const dVector3 a)
    { setAxis (a[0], a[1], a[2]); }
  void getAxis (dVector3 result) const
    { dJointGetSliderAxis (_id, result); }

  dReal getPosition() const
    { return dJointGetSliderPosition (_id); }
  dReal getPositionRate() const
    { return dJointGetSliderPositionRate (_id); }

  virtual void setParam (int parameter, dReal value)
    { dJointSetSliderParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetSliderParam (_id, parameter); }
  // TODO: expose params through methods

  void addForce (dReal force)
	{ dJointAddSliderForce(_id, force); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dUniversalJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dUniversalJointTemplate (const dUniversalJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dUniversalJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dUniversalJointTemplate() { }
  dUniversalJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateUniversal (world, group); }
  dUniversalJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateUniversal (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateUniversal (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetUniversalAnchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor(a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetUniversalAxis1 (_id, x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1 (a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
    { dJointSetUniversalAxis2 (_id, x, y, z); }
  void setAxis2 (const dVector3 a)
    { setAxis2 (a[0], a[1], a[2]); }

  void getAnchor (dVector3 result) const
    { dJointGetUniversalAnchor (_id, result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetUniversalAnchor2 (_id, result); }
  void getAxis1 (dVector3 result) const
    { dJointGetUniversalAxis1 (_id, result); }
  void getAxis2 (dVector3 result) const
    { dJointGetUniversalAxis2 (_id, result); }

  virtual void setParam (int parameter, dReal value)
    { dJointSetUniversalParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetUniversalParam (_id, parameter); }
  // TODO: expose params through methods
  
  void getAngles(dReal *angle1, dReal *angle2) const
    { dJointGetUniversalAngles (_id, angle1, angle2); }

  dReal getAngle1() const
    { return dJointGetUniversalAngle1 (_id); }
  dReal getAngle1Rate() const
    { return dJointGetUniversalAngle1Rate (_id); }
  dReal getAngle2() const
    { return dJointGetUniversalAngle2 (_id); }
  dReal getAngle2Rate() const
    { return dJointGetUniversalAngle2Rate (_id); }

  void addTorques (dReal torque1, dReal torque2)
	{ dJointAddUniversalTorques(_id, torque1, torque2); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dHinge2JointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dHinge2JointTemplate (const dHinge2JointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dHinge2JointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dHinge2JointTemplate() { }
  dHinge2JointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateHinge2 (world, group); }
  dHinge2JointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateHinge2 (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateHinge2 (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetHinge2Anchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor(a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetHinge2Axis1 (_id, x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1 (a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
    { dJointSetHinge2Axis2 (_id, x, y, z); }
  void setAxis2 (const dVector3 a)
    { setAxis2 (a[0], a[1], a[2]); }
    
  void getAnchor (dVector3 result) const
    { dJointGetHinge2Anchor (_id, result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetHinge2Anchor2 (_id, result); }
  void getAxis1 (dVector3 result) const
    { dJointGetHinge2Axis1 (_id, result); }
  void getAxis2 (dVector3 result) const
    { dJointGetHinge2Axis2 (_id, result); }

  dReal getAngle1() const
    { return dJointGetHinge2Angle1 (_id); }
  dReal getAngle1Rate() const
    { return dJointGetHinge2Angle1Rate (_id); }
  dReal getAngle2Rate() const
    { return dJointGetHinge2Angle2Rate (_id); }

  virtual void setParam (int parameter, dReal value)
    { dJointSetHinge2Param (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetHinge2Param (_id, parameter); }
  // TODO: expose params through methods

  void addTorques(dReal torque1, dReal torque2)
	{ dJointAddHinge2Torques(_id, torque1, torque2); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dPRJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  dPRJointTemplate (const dPRJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dPRJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dPRJointTemplate() { }
  dPRJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreatePR (world, group); }
  dPRJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreatePR (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreatePR (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetPRAnchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetPRAxis1 (_id, x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1(a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
    { dJointSetPRAxis2 (_id, x, y, z); }
  void setAxis2 (const dVector3 a)
    { setAxis2(a[0], a[1], a[2]); }

  void getAnchor (dVector3 result) const
    { dJointGetPRAnchor (_id, result); }
  void getAxis1 (dVector3 result) const
    { dJointGetPRAxis1 (_id, result); }
  void getAxis2 (dVector3 result) const
    { dJointGetPRAxis2 (_id, result); }

  dReal getPosition() const
    { return dJointGetPRPosition (_id); }
  dReal getPositionRate() const
    { return dJointGetPRPositionRate (_id); }

  dReal getAngle() const
    { return dJointGetPRAngle (_id); }
  dReal getAngleRate() const
    { return dJointGetPRAngleRate (_id); }

  virtual void setParam (int parameter, dReal value)
    { dJointSetPRParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetPRParam (_id, parameter); }
};



template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dPUJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase>
{
  dPUJointTemplate (const dPUJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dPUJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dPUJointTemplate() { }
  dPUJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreatePU (world, group); }
  dPUJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreatePU (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0)
  {
    destroy();
    _id = dJointCreatePU (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
  { create(world.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetPUAnchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetPUAxis1 (_id, x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1(a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
  { dJointSetPUAxis2 (_id, x, y, z); }
  void setAxis3 (dReal x, dReal y, dReal z)
  { dJointSetPUAxis3 (_id, x, y, z); }
  void setAxis3 (const dVector3 a)
    { setAxis3(a[0], a[1], a[2]); }
  void setAxisP (dReal x, dReal y, dReal z)
  { dJointSetPUAxis3 (_id, x, y, z); }
  void setAxisP (const dVector3 a)
    { setAxisP(a[0], a[1], a[2]); }

  virtual void getAnchor (dVector3 result) const
    { dJointGetPUAnchor (_id, result); }
  void getAxis1 (dVector3 result) const
    { dJointGetPUAxis1 (_id, result); }
  void getAxis2 (dVector3 result) const
    { dJointGetPUAxis2 (_id, result); }
  void getAxis3 (dVector3 result) const
    { dJointGetPUAxis3 (_id, result); }
  void getAxisP (dVector3 result) const
    { dJointGetPUAxis3 (_id, result); }

  dReal getAngle1() const
    { return dJointGetPUAngle1 (_id); }
  dReal getAngle1Rate() const
    { return dJointGetPUAngle1Rate (_id); }
  dReal getAngle2() const
    { return dJointGetPUAngle2 (_id); }
  dReal getAngle2Rate() const
    { return dJointGetPUAngle2Rate (_id); }

  dReal getPosition() const
    { return dJointGetPUPosition (_id); }
  dReal getPositionRate() const
    { return dJointGetPUPositionRate (_id); }

  virtual void setParam (int parameter, dReal value)
  { dJointSetPUParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetPUParam (_id, parameter); }
  // TODO: expose params through methods
};





template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dPistonJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase>
{
  // intentionally undefined, don't use these
  dPistonJointTemplate (const dPistonJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dPistonJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dPistonJointTemplate() { }
  dPistonJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreatePiston (world, group); }
  dPistonJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreatePiston (world, group); }

  void create (dWorldID world, dJointGroupID group=0)
  {
    destroy();
    _id = dJointCreatePiston (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetPistonAnchor (_id, x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void getAnchor (dVector3 result) const
    { dJointGetPistonAnchor (_id, result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetPistonAnchor2 (_id, result); }

  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetPistonAxis (_id, x, y, z); }
  void setAxis (const dVector3 a)
    { setAxis(a[0], a[1], a[2]); }
  void getAxis (dVector3 result) const
    { dJointGetPistonAxis (_id, result); }

  dReal getPosition() const
    { return dJointGetPistonPosition (_id); }
  dReal getPositionRate() const
    { return dJointGetPistonPositionRate (_id); }

  virtual void setParam (int parameter, dReal value)
  { dJointSetPistonParam (_id, parameter, value); }
  virtual dReal getParam (int parameter) const
    { return dJointGetPistonParam (_id, parameter); }
  // TODO: expose params through methods

  void addForce (dReal force)
  { dJointAddPistonForce (_id, force); }
};



template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dFixedJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase>
{
  // intentionally undefined, don't use these
  dFixedJointTemplate (const dFixedJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dFixedJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dFixedJointTemplate() { }
  dFixedJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateFixed (world, group); }
  dFixedJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateFixed (world, group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateFixed (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void set()
    { dJointSetFixed (_id); }

  virtual void setParam (int parameter, dReal value)
    { dJointSetFixedParam (_id, parameter, value); }

  virtual dReal getParam (int parameter) const
    { return dJointGetFixedParam (_id, parameter); }
  // TODO: expose params through methods
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dContactJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dContactJointTemplate (const dContactJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dContactJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dContactJointTemplate() { }
  dContactJointTemplate (dWorldID world, dJointGroupID group, dContact *contact)
    { _id = dJointCreateContact (world, group, contact); }
  dContactJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group, dContact *contact)
    { _id = dJointCreateContact (world.id(), group, contact); }

  void create (dWorldID world, dJointGroupID group, dContact *contact) {
    destroy();
    _id = dJointCreateContact (world, group, contact);
  }
  
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group, dContact *contact)
    { create(world.id(), group, contact); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dNullJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dNullJointTemplate (const dNullJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dNullJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dNullJointTemplate() { }
  dNullJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateNull (world, group); }
  dNullJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateNull (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateNull (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dAMotorJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dAMotorJointTemplate (const dAMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dAMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dAMotorJointTemplate() { }
  dAMotorJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateAMotor (world, group); }
  dAMotorJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateAMotor (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateAMotor (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setMode (int mode)
    { dJointSetAMotorMode (_id, mode); }
  int getMode() const
    { return dJointGetAMotorMode (_id); }

  void setNumAxes (int num)
    { dJointSetAMotorNumAxes (_id, num); }
  int getNumAxes() const
    { return dJointGetAMotorNumAxes (_id); }

  void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
    { dJointSetAMotorAxis (_id, anum, rel, x, y, z); }
  void setAxis (int anum, int rel, const dVector3 a)
    { setAxis(anum, rel, a[0], a[1], a[2]); }
  void getAxis (int anum, dVector3 result) const
    { dJointGetAMotorAxis (_id, anum, result); }
  int getAxisRel (int anum) const
    { return dJointGetAMotorAxisRel (_id, anum); }

  void setAngle (int anum, dReal angle)
    { dJointSetAMotorAngle (_id, anum, angle); }
  dReal getAngle (int anum) const
    { return dJointGetAMotorAngle (_id, anum); }
  dReal getAngleRate (int anum)
    { return dJointGetAMotorAngleRate (_id,anum); }

  void setParam (int parameter, dReal value)
    { dJointSetAMotorParam (_id, parameter, value); }
  dReal getParam (int parameter) const
    { return dJointGetAMotorParam (_id, parameter); }
  // TODO: expose params through methods

  void addTorques(dReal torque1, dReal torque2, dReal torque3)
	{ dJointAddAMotorTorques(_id, torque1, torque2, torque3); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dLMotorJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
  // intentionally undefined, don't use these
  dLMotorJointTemplate (const dLMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);
  void operator = (const dLMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &);

public:
  dLMotorJointTemplate() { }
  dLMotorJointTemplate (dWorldID world, dJointGroupID group=0)
    { _id = dJointCreateLMotor (world, group); }
  dLMotorJointTemplate (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { _id = dJointCreateLMotor (world.id(), group); }

  void create (dWorldID world, dJointGroupID group=0) {
    destroy();
    _id = dJointCreateLMotor (world, group);
  }
  void create (dWorldTemplate<dWorldTemplateBase>& world, dJointGroupID group=0)
    { create(world.id(), group); }

  void setNumAxes (int num)
    { dJointSetLMotorNumAxes (_id, num); }
  int getNumAxes() const
    { return dJointGetLMotorNumAxes (_id); }

  void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
    { dJointSetLMotorAxis (_id, anum, rel, x, y, z); }
  void setAxis (int anum, int rel, const dVector3 a)
    { setAxis(anum, rel, a[0], a[1], a[2]); }
  void getAxis (int anum, dVector3 result) const
    { dJointGetLMotorAxis (_id, anum, result); }

  void setParam (int parameter, dReal value)
    { dJointSetLMotorParam (_id, parameter, value); }
  dReal getParam (int parameter) const
    { return dJointGetLMotorParam (_id, parameter); }
  // TODO: expose params through methods
};

//}

#if !defined(dODECPP_WORLD_TEMPLATE_BASE)

#if defined(dODECPP_BODY_TEMPLATE_BASE) || defined(dODECPP_JOINTGROUP_TEMPLATE_BASE) || defined(dODECPP_JOINT_TEMPLATE_BASE)
#error All the odecpp template bases must be defined or not defined together
#endif

#define dODECPP_WORLD_TEMPLATE_BASE dWorldDynamicIDContainer
#define dODECPP_BODY_TEMPLATE_BASE dBodyDynamicIDContainer
#define dODECPP_JOINTGROUP_TEMPLATE_BASE dJointGroupDynamicIDContainer
#define dODECPP_JOINT_TEMPLATE_BASE dJointDynamicIDContainer

#else // #if defined(dODECPP_WORLD_TEMPLATE_BASE)

#if !defined(dODECPP_BODY_TEMPLATE_BASE) || !defined(dODECPP_JOINTGROUP_TEMPLATE_BASE) || !defined(dODECPP_JOINT_TEMPLATE_BASE)
#error All the odecpp template bases must be defined or not defined together
#endif

#endif // #if defined(dODECPP_WORLD_TEMPLATE_BASE)


typedef dWorldTemplate<dODECPP_WORLD_TEMPLATE_BASE> dWorld;
typedef dBodyTemplate<dODECPP_BODY_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE> dBody;
typedef dJointGroupTemplate<dODECPP_JOINTGROUP_TEMPLATE_BASE> dJointGroup;
typedef dJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dJoint;
typedef dBallJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dBallJoint;
typedef dHingeJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dHingeJoint;
typedef dSliderJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dSliderJoint;
typedef dUniversalJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dUniversalJoint;
typedef dHinge2JointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dHinge2Joint;
typedef dPRJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dPRJoint;
typedef dPUJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dPUJoint;
typedef dPistonJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dPistonJoint;
typedef dFixedJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dFixedJoint;
typedef dContactJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dContactJoint;
typedef dNullJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dNullJoint;
typedef dAMotorJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dAMotorJoint;
typedef dLMotorJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dLMotorJoint;


#endif
#endif

// Local variables:
// mode:c++
// c-basic-offset:2
// End:
