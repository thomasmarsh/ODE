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

/*

@@@ WORK IN PROGRESS @@@

TODO
  * ALIGN allocations to dEFFICIENT_SIZE

*/

#include <string.h>
#include <errno.h>
#include "obstack.h"
#include "ode/error.h"
#include "ode/config.h"
#include "ode/memory.h"

//****************************************************************************
// constants

// each Arena pointer points to a block of this many bytes
#define ARENA_SIZE 4096

//****************************************************************************
// dObStack

struct dObStack {
  struct Arena {
    Arena *next;
    int used;		// total number of bytes used in this arena
  };

  Arena *first,last;

  // used for iterator
  Arena *current_arena;
  int current_ofs;
};


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
    dFree (a,a->size);
    a = nexta;
  }
}


void *dObStack::alloc (int num_bytes)
{
  // @@@ assumes num_bytes is not too large.

  // see if a new arena must be used
  int new_arena = 0;
  if (last) {
    if ((last->used + num_bytes) > ARENA_SIZE) {
      new_arena = 1;
    }
  }
  else {
    new_arena = 1;
  }

  // allocate or move to a new arena if necessary
  if (new_arena) {
    if (last && last->next) {
      // the next arena has already been allocated
      last = last->next;
      last->size = sizeof (Arena);
    }
    else {
      // the next arena must be allocated
      Arena *a = (Arena *) dAlloc (ARENA_SIZE);
      a->next = 0;
      a->used = sizeof (Arena);
      if (last) {
	last->next = a;
	last = a;
      }
      else {
	first = a;
	last = a;
      }
    }
  }

  // allocate an area in the arena
  char *c = ((char*) last) + last->used;
  last->used += num_bytes;
  return c;
}


void dObStack::freeAll()
{
  last = first;
  first->used = sizeof(Arena);
}


void dObStack::rewind()
{
  current_arena = first;
  current_ofs = sizeof (Arena);
}


void *dObStack::next (int num_bytes)
{
  // this functions exactly like alloc, except that no new storage is ever
  // allocated

  // @@@ assumes num_bytes is not too large.

  if (!first) return 0;

  // jump to a new arena if necessary
  int new_arena = 0;
  if ((current_ofs + num_bytes) > ARENA_SIZE) {
    new_arena = 1;
    if (!last->next) return 0;
    current_arena = current_arena->next;
    current_ofs = sizeof (Arena);
  }

  // allocate an area in the arena
  char *c = ((char*) current_arena) + current_offset;
  current_offset += num_bytes;
  return c;
}
