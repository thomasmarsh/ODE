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

/* this comes from the `reuse' library. copy any changes back to the source */

#ifndef _ODE_MEMORY_H_
#define _ODE_MEMORY_H_

#ifdef __cplusplus
extern "C" {
#endif

/* function types to allocate and free memory */
typedef void * dAllocFunction (int size);
typedef void * dReallocFunction (void *ptr, int oldsize, int newsize);
typedef void dFreeFunction (void *ptr, int size);

/* set new memory management functions. if fn is 0, the default handlers are
 * used. */
void dSetAllocHandler (dAllocFunction *fn);
void dSetReallocHandler (dReallocFunction *fn);
void dSetFreeHandler (dFreeFunction *fn);

/* allocate and free memory. */
void * dAlloc (int size);
void * dRealloc (void *ptr, int oldsize, int newsize);
void dFree (void *ptr, int size);

#ifdef __cplusplus
}
#endif

#endif
