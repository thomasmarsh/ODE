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

/*

space - a container for geometry objects that does high level collision
culling.

various algorithms will be available:
  - simple n^2 object intersections
  - oct tree
  - coordinate sorting
  - hash table

*/


#ifndef _ODE_SPACE_H_
#define _ODE_SPACE_H_

#include "ode/common.h"

#ifdef __cplusplus
extern "C" {
#endif

struct dContact;


/* extra information the space needs in every geometry object */

struct dGeomSpaceData {
  dGeomID next;
};


/* create and destroy a space. when a space is destroyed, all the geometry
 * objects in that space are automatically destroyed as well.
 */

dSpaceID dSpaceCreate();
void dSpaceDestroy (dSpaceID);


/* add and remove geometry objects to/from the space. these functions are
 * normally only called by the geometry object creation/deletion functions.
 */

void dSpaceAdd (dSpaceID space, dGeomID);
void dSpaceRemove (dSpaceID, dGeomID);


/* call the callback function for all potentially intersection objects in the
 * space. the callback function will usually:
 *   * determine if the objects can intersect based on other properties
 *   * if so, collide them
 */
typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2);
void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback);


/* @@@ NOT FLEXIBLE ENOUGH
 *
 * generate contacts for those objects in the space that touch each other.
 * an array of contacts is created on the alternative stack using
 * StackAlloc(), and a pointer to the array is returned. the size of the
 * array is returned by the function.
 */
/* int dSpaceCollide (dSpaceID space, dContact **contact_array); */


/* tell the space that an object has moved, so its representation in the
 * space should be changed.
 */

void dSpaceObjectMoved (dSpaceID, dGeomID);


#ifdef __cplusplus
}
#endif

#endif
