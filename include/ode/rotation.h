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

#ifndef _ODE_ROTATION_H_
#define _ODE_ROTATION_H_

#ifdef __cplusplus
extern "C" {
#endif


void dRSetIdentity (dMatrix3 R);

void dRFromAxisAndAngle (dMatrix3 R, dReal ax, dReal ay, dReal az,
			 dReal angle);

void dRFromEulerAngles (dMatrix3 R, dReal phi, dReal theta, dReal psi);

void dRFrom2Axes (dMatrix3 R, dReal ax, dReal ay, dReal az,
		  dReal bx, dReal by, dReal bz);

void dQSetIdentity (dQuaternion q);

void dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az,
			  dReal angle);

void dQMultiply0 (dQuaternion qa, dQuaternion qb, dQuaternion qc);
void dQMultiply1 (dQuaternion qa, dQuaternion qb, dQuaternion qc);
void dQMultiply2 (dQuaternion qa, dQuaternion qb, dQuaternion qc);
void dQMultiply3 (dQuaternion qa, dQuaternion qb, dQuaternion qc);

void dQtoR (dQuaternion q, dMatrix3 R);

void dRtoQ (dMatrix3 R, dQuaternion q);

void dWtoDQ (dVector3 w, dQuaternion q, dVector4 dq);


#ifdef __cplusplus
}
#endif

#endif
