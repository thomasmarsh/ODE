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

#ifndef _ODE_ODE_H_
#define _ODE_ODE_H_

/* include *everything* here */

#include "ode/config.h"
#include "ode/contact.h"
#include "ode/error.h"
#include "ode/memory.h"
#include "ode/odemath.h"
#include "ode/matrix.h"
#include "ode/timer.h"
#include "ode/common.h"
#include "ode/rotation.h"
#include "ode/mass.h"
#include "ode/space.h"
#include "ode/geom.h"
#include "ode/misc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* world */

dWorldID dWorldCreate();
void dWorldDestroy (dWorldID);

void dWorldSetGravity (dWorldID, dReal x, dReal y, dReal z);
void dWorldGetGravity (dWorldID, dVector3 g);

void dWorldStep (dWorldID, dReal stepsize);

/* bodies */

dBodyID dBodyCreate (dWorldID);
void dBodyDestroy (dBodyID);

void  dBodySetData (dBodyID, void *data);
void *dBodyGetData (dBodyID);

void dBodySetPosition   (dBodyID, dReal x, dReal y, dReal z);
void dBodySetRotation   (dBodyID, dMatrix3 R);
void dBodySetQuaternion (dBodyID, dQuaternion q);
void dBodySetLinearVel  (dBodyID, dReal x, dReal y, dReal z);
void dBodySetAngularVel (dBodyID, dReal x, dReal y, dReal z);
dReal    * dBodyGetPosition   (dBodyID);
dReal    * dBodyGetRotation   (dBodyID);	/* ptr to 4x3 rot matrix */
dReal    * dBodyGetQuaternion (dBodyID);
dReal    * dBodyGetLinearVel  (dBodyID);
dReal    * dBodyGetAngularVel (dBodyID);

void dBodySetMass (dBodyID, dMass *mass);
void dBodyGetMass (dBodyID, dMass *mass);

void dBodyAddForce            (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddTorque           (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddRelForce         (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddRelTorque        (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddForceAtPos       (dBodyID, dReal fx, dReal fy, dReal fz,
					  dReal px, dReal py, dReal pz);
void dBodyAddRelForceAtPos    (dBodyID, dReal fx, dReal fy, dReal fz,
					  dReal px, dReal py, dReal pz);
void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz,
					  dReal px, dReal py, dReal pz);

void dBodyGetPointPos    (dBodyID, dReal px, dReal py, dReal pz,
			   dVector3 result);
void dBodyGetPointVel    (dBodyID, dReal px, dReal py, dReal pz,
			   dVector3 result);
void dBodyGetPointRelVel (dBodyID, dReal px, dReal py, dReal pz,
			   dVector3 result);

/* joints */

dJointID dJointCreateBall (dWorldID, dJointGroupID);
dJointID dJointCreateHinge (dWorldID, dJointGroupID);
dJointID dJointCreateSlider (dWorldID, dJointGroupID);
dJointID dJointCreateContact (dWorldID, dJointGroupID, dContact *);

void dJointDestroy (dJointID);

dJointGroupID dJointGroupCreate (int max_size);
void dJointGroupDestroy (dJointGroupID);
void dJointGroupEmpty (dJointGroupID);

void dJointAttach (dJointID, dBodyID body1, dBodyID body2);

void dJointSetAnchor (dJointID, dReal x, dReal y, dReal z);
void dJointSetAxis (dJointID, dReal x, dReal y, dReal z);

void dJointGetAnchor (dJointID, dVector3 result);
void dJointGetAxis (dJointID, dVector3 result);

int dAreConnected (dBodyID, dBodyID);


#ifdef __cplusplus
}
#endif

#endif
