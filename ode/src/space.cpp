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

// simple space - reports all n^2 object intersections

#include "ode/space.h"
#include "ode/geom.h"
#include "ode/error.h"
#include "ode/memory.h"


struct dxSpace {
  dGeomID first;
};


dSpaceID dSpaceCreate()
{
  dxSpace *w = (dxSpace *) dAlloc (sizeof(dxSpace));
  w->first = 0;
  return w;
}


void dSpaceDestroy (dSpaceID space)
{
  dGeomID g,n;
  g = space->first;
  while (g) {
    n = g->space.next;
    dFree (g,g->_class->size);
    g = n;
  }
  dFree (space,sizeof(dxSpace));
}


void dSpaceAdd (dSpaceID space, dGeomID obj)
{
  obj->space.next = space->first;
  space->first = obj;
}


void dSpaceRemove (dSpaceID space, dGeomID geom_to_remove)
{
  dGeomID last=0,g=space->first;
  while (g) {
    if (g==geom_to_remove) {
      if (last) last->space.next = g->space.next;
      else space->first = g->space.next;
      geom_to_remove->space.next = 0;
      return;
    }
    last = g;
    g = g->space.next;
  }
}


void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback)
{
  for (dxGeom *g1=space->first; g1; g1=g1->space.next) {
    for (dxGeom *g2=g1->space.next; g2; g2=g2->space.next) {
      callback (data,g1,g2);
    }
  }
}


// @@@ NOT FLEXIBLE ENOUGH
//
//int dSpaceCollide (dSpaceID space, dContactGeom **contact_array)
//{
//  int n = 0;
//  dContactGeom *base = (dContact*) dStackAlloc (sizeof(dContact));
//  dContactGeom *c = base;
//  for (dxGeom *g1=space->first; g1; g1=g1->space.next) {
//    for (dxGeom *g2=g1->space.next; g2; g2=g2->space.next) {
//      // generate at most 1 contact for this pair
//      c->o1 = g1;
//      c->o2 = g2;
//      if (dCollide (0,c)) {
//	c = (dContactGeom*) dStackAlloc (sizeof(dContactGeom));
//	n++;
//      }
//    }
//  }
//  *contact_array = base;
//  return n;
//}


void dSpaceObjectMoved (dSpaceID, dGeomID)
{
}
