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

#include "ode/ode.h"
#include "objects.h"
#include "util.h"


void dInternalHandleAutoDisabling (dxWorld *world, dReal stepsize)
{
	dxBody *bb;
	for (bb=world->firstbody; bb; bb=(dxBody*)bb->next) {
		// nothing to do unless this body is currently enabled and has
		// the auto-disable flag set
		if ((bb->flags & (dxBodyAutoDisable|dxBodyDisabled)) != dxBodyAutoDisable) continue;
		
		// see if the body is idle
		int idle = 1;			// initial assumption
		dReal lspeed2 = dDOT(bb->lvel,bb->lvel);
		if (lspeed2 > bb->adis.linear_threshold) {
			idle = 0;		// moving fast - not idle
		}
		else {
			dReal aspeed = dDOT(bb->avel,bb->avel);
			if (aspeed > bb->adis.angular_threshold) {
				idle = 0;	// turning fast - not idle
			}
		}
	
		// if it's idle, accumulate steps and time
		// @@@ watch out for overflow of these counters
		if (idle) {
			bb->adis_stepsleft--;
			bb->adis_timeleft -= stepsize;
		}
		else {
			bb->adis_stepsleft = bb->adis.idle_steps;
			bb->adis_timeleft = bb->adis.idle_time;
		}

		// disable the body if it's idle for a long enough time
		if (bb->adis_stepsleft < 0 && bb->adis_timeleft < 0) {
			bb->flags |= dxBodyDisabled;
		}
	}
}
