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

#ifndef _ODE_SPACE_H_
#define _ODE_SPACE_H_

#include "ode/common.h"

#ifdef __cplusplus
extern "C" {
#endif

struct dContactGeom;


/* extra information the space needs in every geometry object */

typedef struct dGeomSpaceData {
  dGeomID next;
} dGeomSpaceData;


dSpaceID dSpaceCreate();
void dSpaceDestroy (dSpaceID);

void dSpaceAdd (dSpaceID, dGeomID);
void dSpaceRemove (dSpaceID, dGeomID);

typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2);
void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback);


/* @@@ NOT FLEXIBLE ENOUGH
 *
 * generate contacts for those objects in the space that touch each other.
 * an array of contacts is created on the alternative stack using
 * StackAlloc(), and a pointer to the array is returned. the size of the
 * array is returned by the function.
 */
/* int dSpaceCollide (dSpaceID space, dContactGeom **contact_array); */


/* HMMMMM... i dont think so.
 * tell the space that an object has moved, so its representation in the
 * space should be changed.
 */
/* void dSpaceObjectMoved (dSpaceID, dGeomID); */


#ifdef __cplusplus
}
#endif

#endif
