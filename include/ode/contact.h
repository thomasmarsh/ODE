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


/* contact mode flags: these flags say which parameters in dSurfaceParameters
 * structure are used. the friction directions 1 and 2 are perpendicular to
 * the normal, i.e. tangential to the contact surface. fdir2 is computed from
 * the fdir1 and the normal.
 * any number of these flags can be ORed together.
 * NOT implemented yet: bounce, soft, slip.
 */

enum {
  dContactMu2		= 0x001, /* 0=use mu for both f-directions,
				    1=use mu for fdir1, mu2 for fdir2 */
  dContactFDir1		= 0x002, /* 1=use `fdir1', else compute it */
  dContactBounce	= 0x004, /* objects bounce off each other */
  dContactSoft		= 0x008, /* objects can penetrate if forced. damped */
  dContactMotion1	= 0x010, /* relative surface motion, f-direction 1 */
  dContactMotion2	= 0x020, /* relative surface motion, f-direction 2 */
  dContactSlip1		= 0x040, /* 1st order slip, f-direction 1 */
  dContactSlip2		= 0x080, /* 1st order slip, f-direction 2 */
};


/* surface parameters structure.
 * mu=0 corresponds to frictionless.
 * mu=dInfinity is particularly efficient (no LCP switching)
 */

struct dSurfaceParameters {
  /* the following numbers must always be defined */
  int mode;              /* contact flags */
  dReal mu;              /* Coulomb friction coefficient (0..dInfinity) */

  /* the following numbers are only defined if the corresponding flag is set */
  dReal mu2;             /* Coulomb friction coefficient (0..dInfinity) */
  dReal bounce;	         /* restitution parameter (0..1) */
  dReal bounceVel;       /* minimum velocity for restitution (m/s) */
  dReal soft;            /* contact softness parameter (soft mode) (0..) */
  dReal motion1,motion2; /* surface velocity in direction 1/2 (m/s) */
  dReal slip1,slip2;     /* 1st order slip in direction 1/2 */
};


/* contact info set by collision functions */

struct dContactGeom {
  dVector3 pos;		/* position of the contact in global coordinates */
  dVector3 normal;	/* normal vector, points `in' to object 1, length=1 */
  dReal depth;		/* penetration depth */
};


/* contact info used by contact joint. if fdir1 is given it must be unit
 * length and perpendicular to the normal.
 */

struct dContact {
  dSurfaceParameters surface;	/* set by user */
  dContactGeom geom;		/* set by collision functions */
  dVector3 fdir1;		/* first tangential friction direction */
};


#ifdef __cplusplus
}
#endif

#endif
