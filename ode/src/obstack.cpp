/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001 Russell L. Smith.            *
 *   Email: russ@q12.org   Web: www.q12.org                              *
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

#include <string.h>
#include <errno.h>
#include "obstack.h"
#include "ode/config.h"
#include "ode/common.h"
#include "ode/error.h"
#include "ode/memory.h"

//****************************************************************************
// macros and constants

#define ROUND_UP_OFFSET_TO_EFFICIENT_SIZE(arena,ofs) \
  ofs = (int) (dEFFICIENT_SIZE( ((intP)(arena)) + ofs ) - ((intP)(arena)) );

#define MAX_ALLOC_SIZE \
  ((int)(dOBSTACK_ARENA_SIZE - sizeof (Arena) - EFFICIENT_ALIGNMENT + 1))

//****************************************************************************
// dObStack

dObStack::dObStack()
{
  first = 0;
  last = 0;
  current_arena = 0;
  current_ofs = 0;
}


dObStack::~dObStack()
{
  // free all arenas
  Arena *a,*nexta;
  a = first;
  while (a) {
    nexta = a->next;
    dFree (a,dOBSTACK_ARENA_SIZE);
    a = nexta;
  }
}


void *dObStack::alloc (int num_bytes)
{
  if (num_bytes > MAX_ALLOC_SIZE) dDebug (0,"num_bytes too large");

  // allocate or move to a new arena if necessary
  if (!first) {
    // allocate the first arena if necessary
    first = last = (Arena *) dAlloc (dOBSTACK_ARENA_SIZE);
    first->next = 0;
    first->used = sizeof (Arena);
    ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (first,first->used);
  }
  else {
    // we already have one or more arenas, see if a new arena must be used
    if ((last->used + num_bytes) > dOBSTACK_ARENA_SIZE) {
      if (!last->next) {
	last->next = (Arena *) dAlloc (dOBSTACK_ARENA_SIZE);
	last->next->next = 0;
      }
      last = last->next;
      last->used = sizeof (Arena);
      ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (last,last->used);
    }
  }

  // allocate an area in the arena
  char *c = ((char*) last) + last->used;
  last->used += num_bytes;
  ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (last,last->used);
  return c;
}


void dObStack::freeAll()
{
  last = first;
  first->used = sizeof(Arena);
  ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (first,first->used);
}


void *dObStack::rewind()
{
  current_arena = first;
  current_ofs = sizeof (Arena);
  if (current_arena) {
    ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (current_arena,current_ofs)
    return ((char*) current_arena) + current_ofs;
  }
  else return 0;
}


void *dObStack::next (int num_bytes)
{
  // this functions exactly like alloc, except that no new storage is ever
  // allocated
  if (num_bytes > MAX_ALLOC_SIZE) dDebug (0,"num_bytes too large");
  if (!current_arena) return 0;
  if ((current_ofs + num_bytes) > dOBSTACK_ARENA_SIZE) {
    current_arena = current_arena->next;
    if (!current_arena) return 0;
    current_ofs = sizeof (Arena);
    ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (current_arena,current_ofs)
  }
  char *c = ((char*) current_arena) + current_ofs;
  current_arena += num_bytes;
  ROUND_UP_OFFSET_TO_EFFICIENT_SIZE (current_arena,current_ofs)
  return c;
}
