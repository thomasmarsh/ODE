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

#ifndef _ODE_OBJECTS_H_
#define _ODE_OBJECTS_H_

#include <ode/common.h>
#include <ode/mass.h>
#include <ode/contact.h>

#ifdef __cplusplus
extern "C" {
#endif

/* world */

ODE_API dWorldID dWorldCreate(void);
ODE_API void dWorldDestroy (dWorldID);

ODE_API void dWorldSetGravity (dWorldID, dReal x, dReal y, dReal z);
ODE_API void dWorldGetGravity (dWorldID, dVector3 gravity);
ODE_API void dWorldSetERP (dWorldID, dReal erp);
ODE_API dReal dWorldGetERP (dWorldID);
ODE_API void dWorldSetCFM (dWorldID, dReal cfm);
ODE_API dReal dWorldGetCFM (dWorldID);
ODE_API void dWorldStep (dWorldID, dReal stepsize);
ODE_API void dWorldImpulseToForce (dWorldID, dReal stepsize,
			   dReal ix, dReal iy, dReal iz, dVector3 force);

/* World QuickStep functions */

ODE_API void dWorldQuickStep (dWorldID w, dReal stepsize);
ODE_API void dWorldSetQuickStepNumIterations (dWorldID, int num);
ODE_API int dWorldGetQuickStepNumIterations (dWorldID);
ODE_API void dWorldSetQuickStepW (dWorldID, dReal param);
ODE_API dReal dWorldGetQuickStepW (dWorldID);

/* World contact parameter functions */

ODE_API void dWorldSetContactMaxCorrectingVel (dWorldID, dReal vel);
ODE_API dReal dWorldGetContactMaxCorrectingVel (dWorldID);
ODE_API void dWorldSetContactSurfaceLayer (dWorldID, dReal depth);
ODE_API dReal dWorldGetContactSurfaceLayer (dWorldID);

/* StepFast1 functions */

ODE_API void dWorldStepFast1(dWorldID, dReal stepsize, int maxiterations);
ODE_API void dWorldSetAutoEnableDepthSF1(dWorldID, int autoEnableDepth);
ODE_API int dWorldGetAutoEnableDepthSF1(dWorldID);

/* Auto-disable functions */

ODE_API dReal dWorldGetAutoDisableLinearThreshold (dWorldID);
ODE_API void  dWorldSetAutoDisableLinearThreshold (dWorldID, dReal linear_threshold);
ODE_API dReal dWorldGetAutoDisableAngularThreshold (dWorldID);
ODE_API void  dWorldSetAutoDisableAngularThreshold (dWorldID, dReal angular_threshold);
ODE_API int   dWorldGetAutoDisableSteps (dWorldID);
ODE_API void  dWorldSetAutoDisableSteps (dWorldID, int steps);
ODE_API dReal dWorldGetAutoDisableTime (dWorldID);
ODE_API void  dWorldSetAutoDisableTime (dWorldID, dReal time);
ODE_API int   dWorldGetAutoDisableFlag (dWorldID);
ODE_API void  dWorldSetAutoDisableFlag (dWorldID, int do_auto_disable);

ODE_API dReal dBodyGetAutoDisableLinearThreshold (dBodyID);
ODE_API void  dBodySetAutoDisableLinearThreshold (dBodyID, dReal linear_threshold);
ODE_API dReal dBodyGetAutoDisableAngularThreshold (dBodyID);
ODE_API void  dBodySetAutoDisableAngularThreshold (dBodyID, dReal angular_threshold);
ODE_API int   dBodyGetAutoDisableSteps (dBodyID);
ODE_API void  dBodySetAutoDisableSteps (dBodyID, int steps);
ODE_API dReal dBodyGetAutoDisableTime (dBodyID);
ODE_API void  dBodySetAutoDisableTime (dBodyID, dReal time);
ODE_API int   dBodyGetAutoDisableFlag (dBodyID);
ODE_API void  dBodySetAutoDisableFlag (dBodyID, int do_auto_disable);
ODE_API void  dBodySetAutoDisableDefaults (dBodyID);

/* bodies */

ODE_API dBodyID dBodyCreate (dWorldID);
ODE_API void dBodyDestroy (dBodyID);

ODE_API void  dBodySetData (dBodyID, void *data);
ODE_API void *dBodyGetData (dBodyID);

ODE_API void dBodySetPosition   (dBodyID, dReal x, dReal y, dReal z);
ODE_API void dBodySetRotation   (dBodyID, const dMatrix3 R);
ODE_API void dBodySetQuaternion (dBodyID, const dQuaternion q);
ODE_API void dBodySetLinearVel  (dBodyID, dReal x, dReal y, dReal z);
ODE_API void dBodySetAngularVel (dBodyID, dReal x, dReal y, dReal z);
ODE_API const dReal * dBodyGetPosition   (dBodyID);
ODE_API const dReal * dBodyGetRotation   (dBodyID);	/* ptr to 4x3 rot matrix */
ODE_API const dReal * dBodyGetQuaternion (dBodyID);
ODE_API const dReal * dBodyGetLinearVel  (dBodyID);
ODE_API const dReal * dBodyGetAngularVel (dBodyID);

ODE_API void dBodySetMass (dBodyID, const dMass *mass);
ODE_API void dBodyGetMass (dBodyID, dMass *mass);

ODE_API void dBodyAddForce            (dBodyID, dReal fx, dReal fy, dReal fz);
ODE_API void dBodyAddTorque           (dBodyID, dReal fx, dReal fy, dReal fz);
ODE_API void dBodyAddRelForce         (dBodyID, dReal fx, dReal fy, dReal fz);
ODE_API void dBodyAddRelTorque        (dBodyID, dReal fx, dReal fy, dReal fz);
ODE_API void dBodyAddForceAtPos       (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);
ODE_API void dBodyAddForceAtRelPos    (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);
ODE_API void dBodyAddRelForceAtPos    (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);
ODE_API void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);

ODE_API const dReal * dBodyGetForce   (dBodyID);
ODE_API const dReal * dBodyGetTorque  (dBodyID);
ODE_API void dBodySetForce  (dBodyID b, dReal x, dReal y, dReal z);
ODE_API void dBodySetTorque (dBodyID b, dReal x, dReal y, dReal z);

ODE_API void dBodyGetRelPointPos    (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
ODE_API void dBodyGetRelPointVel    (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
ODE_API void dBodyGetPointVel       (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
ODE_API void dBodyGetPosRelPoint    (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
ODE_API void dBodyVectorToWorld     (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
ODE_API void dBodyVectorFromWorld   (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);

ODE_API void dBodySetFiniteRotationMode (dBodyID, int mode);
ODE_API void dBodySetFiniteRotationAxis (dBodyID, dReal x, dReal y, dReal z);

ODE_API int dBodyGetFiniteRotationMode (dBodyID);
ODE_API void dBodyGetFiniteRotationAxis (dBodyID, dVector3 result);

ODE_API int dBodyGetNumJoints (dBodyID b);
ODE_API dJointID dBodyGetJoint (dBodyID, int index);

ODE_API void dBodyEnable (dBodyID);
ODE_API void dBodyDisable (dBodyID);
ODE_API int dBodyIsEnabled (dBodyID);

ODE_API void dBodySetGravityMode (dBodyID b, int mode);
ODE_API int dBodyGetGravityMode (dBodyID b);


/* joints */

ODE_API dJointID dJointCreateBall (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateHinge (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateSlider (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateContact (dWorldID, dJointGroupID, const dContact *);
ODE_API dJointID dJointCreateHinge2 (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateUniversal (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateFixed (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateNull (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateAMotor (dWorldID, dJointGroupID);
ODE_API dJointID dJointCreateLMotor (dWorldID, dJointGroupID);

ODE_API void dJointDestroy (dJointID);

ODE_API dJointGroupID dJointGroupCreate (int max_size);
ODE_API void dJointGroupDestroy (dJointGroupID);
ODE_API void dJointGroupEmpty (dJointGroupID);

ODE_API void dJointAttach (dJointID, dBodyID body1, dBodyID body2);
ODE_API void dJointSetData (dJointID, void *data);
ODE_API void *dJointGetData (dJointID);
ODE_API int dJointGetType (dJointID);
ODE_API dBodyID dJointGetBody (dJointID, int index);

ODE_API void dJointSetFeedback (dJointID, dJointFeedback *);
ODE_API dJointFeedback *dJointGetFeedback (dJointID);

ODE_API void dJointSetBallAnchor (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetBallAnchor2 (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetHingeAnchorDelta (dJointID, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);
ODE_API void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetHingeParam (dJointID, int parameter, dReal value);
ODE_API void dJointAddHingeTorque(dJointID joint, dReal torque);
ODE_API void dJointSetSliderAxis (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetSliderAxisDelta (dJointID, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);
ODE_API void dJointSetSliderParam (dJointID, int parameter, dReal value);
ODE_API void dJointAddSliderForce(dJointID joint, dReal force);
ODE_API void dJointSetHinge2Anchor (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetHinge2Axis1 (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetHinge2Axis2 (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetHinge2Param (dJointID, int parameter, dReal value);
ODE_API void dJointAddHinge2Torques(dJointID joint, dReal torque1, dReal torque2);
ODE_API void dJointSetUniversalAnchor (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetUniversalAxis1 (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetUniversalAxis2 (dJointID, dReal x, dReal y, dReal z);
ODE_API void dJointSetUniversalParam (dJointID, int parameter, dReal value);
ODE_API void dJointAddUniversalTorques(dJointID joint, dReal torque1, dReal torque2);
ODE_API void dJointSetFixed (dJointID);
ODE_API void dJointSetAMotorNumAxes (dJointID, int num);
ODE_API void dJointSetAMotorAxis (dJointID, int anum, int rel,
			  dReal x, dReal y, dReal z);
ODE_API void dJointSetAMotorAngle (dJointID, int anum, dReal angle);
ODE_API void dJointSetAMotorParam (dJointID, int parameter, dReal value);
ODE_API void dJointSetAMotorMode (dJointID, int mode);
ODE_API void dJointAddAMotorTorques (dJointID, dReal torque1, dReal torque2, dReal torque3);
ODE_API void dJointSetLMotorNumAxes (dJointID, int num);
ODE_API void dJointSetLMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z);
ODE_API void dJointSetLMotorParam (dJointID, int parameter, dReal value);

ODE_API void dJointGetBallAnchor (dJointID, dVector3 result);
ODE_API void dJointGetBallAnchor2 (dJointID, dVector3 result);
ODE_API void dJointGetHingeAnchor (dJointID, dVector3 result);
ODE_API void dJointGetHingeAnchor2 (dJointID, dVector3 result);
ODE_API void dJointGetHingeAxis (dJointID, dVector3 result);
ODE_API dReal dJointGetHingeParam (dJointID, int parameter);
ODE_API dReal dJointGetHingeAngle (dJointID);
ODE_API dReal dJointGetHingeAngleRate (dJointID);
ODE_API dReal dJointGetSliderPosition (dJointID);
ODE_API dReal dJointGetSliderPositionRate (dJointID);
ODE_API void dJointGetSliderAxis (dJointID, dVector3 result);
ODE_API dReal dJointGetSliderParam (dJointID, int parameter);
ODE_API void dJointGetHinge2Anchor (dJointID, dVector3 result);
ODE_API void dJointGetHinge2Anchor2 (dJointID, dVector3 result);
ODE_API void dJointGetHinge2Axis1 (dJointID, dVector3 result);
ODE_API void dJointGetHinge2Axis2 (dJointID, dVector3 result);
ODE_API dReal dJointGetHinge2Param (dJointID, int parameter);
ODE_API dReal dJointGetHinge2Angle1 (dJointID);
ODE_API dReal dJointGetHinge2Angle1Rate (dJointID);
ODE_API dReal dJointGetHinge2Angle2Rate (dJointID);
ODE_API void dJointGetUniversalAnchor (dJointID, dVector3 result);
ODE_API void dJointGetUniversalAnchor2 (dJointID, dVector3 result);
ODE_API void dJointGetUniversalAxis1 (dJointID, dVector3 result);
ODE_API void dJointGetUniversalAxis2 (dJointID, dVector3 result);
ODE_API dReal dJointGetUniversalParam (dJointID, int parameter);
ODE_API dReal dJointGetUniversalAngle1 (dJointID);
ODE_API dReal dJointGetUniversalAngle2 (dJointID);
ODE_API dReal dJointGetUniversalAngle1Rate (dJointID);
ODE_API dReal dJointGetUniversalAngle2Rate (dJointID);
ODE_API int dJointGetAMotorNumAxes (dJointID);
ODE_API void dJointGetAMotorAxis (dJointID, int anum, dVector3 result);
ODE_API int dJointGetAMotorAxisRel (dJointID, int anum);
ODE_API dReal dJointGetAMotorAngle (dJointID, int anum);
ODE_API dReal dJointGetAMotorAngleRate (dJointID, int anum);
ODE_API dReal dJointGetAMotorParam (dJointID, int parameter);
ODE_API int dJointGetAMotorMode (dJointID);
ODE_API int dJointGetLMotorNumAxes (dJointID);
ODE_API void dJointGetLMotorAxis (dJointID, int anum, dVector3 result);
ODE_API dReal dJointGetLMotorParam (dJointID, int parameter);

ODE_API dJointID dConnectingJoint (dBodyID, dBodyID);
ODE_API int dConnectingJointList (dBodyID, dBodyID, dJointID*);
ODE_API int dAreConnected (dBodyID, dBodyID);
ODE_API int dAreConnectedExcluding (dBodyID, dBodyID, int joint_type);


#ifdef __cplusplus
}
#endif

#endif
