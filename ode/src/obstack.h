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

#ifndef _ODE_OBSTACK_H_
#define _ODE_OBSTACK_H_

#include "objects.h" 

// each obstack Arena pointer points to a block of this many bytes
#define dOBSTACK_ARENA_SIZE 16384


struct dObStack : public dBase {
  struct Arena {
    Arena *next;	// next arena in linked list
    int used;		// total number of bytes used in this arena, counting
  };			//   this header

  Arena *first;		// head of the arena linked list. 0 if no arenas yet
  Arena *last;		// arena where blocks are currently being allocated

  // used for iterator
  Arena *current_arena;
  int current_ofs;

  dObStack();
  ~dObStack();

  void *alloc (int num_bytes);
  // allocate a block in the last arena, allocating a new arena if necessary.
  // it is a runtime error if num_bytes is larger than the arena size.

  void freeAll();
  // free all blocks in all arenas. this does not deallocate the arenas
  // themselves, so future alloc()s will reuse them.

  void *rewind();
  // rewind the obstack iterator, and return the address of the first
  // allocated block. return 0 if there are no allocated blocks.

  void *next (int num_bytes);
  // return the address of the next allocated block. 'num_bytes' is the size
  // of the previous block. this returns null if there are no more arenas.
  // the sequence of 'num_bytes' parameters passed to next() during a
  // traversal of the list must exactly match the parameters passed to alloc().
};


#endif
