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

#ifndef _ODE_CONTACT_H_
#define _ODE_CONTACT_H_

#include "ode/common.h"

#ifdef __cplusplus
extern "C" {
#endif


/* contact info returned by collision functions */

struct dContact {
  dVector3 pos;		/* position of the contact in global coordinates */
  dVector3 normal;	/* normal vector, oriented `in' to object 1, length=1 */
  dReal depth;		/* penetration depth */
};


/*

NOT USED YET

// flags that say which surface parameters are used
enum {
  dContactTypeMask	= 0x000f, // 0=frictionless, 1=1D, 2=2D friction
  dContactUseDirection	= 0x0010, // 1=direction used, 0=direction computed
  dContactBounce	= 0x0020, // objects bounce off each other
  dContactSoft		= 0x0040, // objects can penetrate if forced. damped.
  dContactSlip1		= 0x0100, // 1st order slip, main direction
  dContactSlip2		= 0x0200, // 1st order slip, 2nd direction
  dContactMotion1	= 0x0400, // relative surface motion, main direction
  dContactMotion2	= 0x0800  // relative surface motion, 2nd direction
};

struct dSurfaceParameters {
  int mode;			// contact type and flags
  dReal mu;			// friction coefficient (0..)
  dReal bouncyness;		// restitution parameter (0..1)
  dReal bounceThreshold;	// minimum velocity for restitution (m/s)
  dReal softness;		// contact softness parameter (soft mode) (0..)
  dReal slip1,slip2;		// 1st order slip in pri/sec direction
  dReal motion1,motion2;	// surface velocity in pri/sec direction
};

*/


#ifdef __cplusplus
}
#endif

#endif
