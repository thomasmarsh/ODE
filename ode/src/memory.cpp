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

// if the dDEBUG_ALLOC macro is defined, the following tests are performed in
// the default allocator:
//   - size > 0 for alloc and realloc
//   - realloc and free operate on uncorrupted blocks
//   - realloc and free with the correct sizes
//   - on exit, report of block allocation statistics
//   - on exit, report of unfreed blocks and if they are corrupted
// the allocator will also call Debug() when it allocates a block with
// sequence number `seqstop' or pointer value `ptrstop'. these variables
// are global and can be set in the debugger.

#include <malloc.h>
#include "ode/memory.h"
#include "ode/error.h"

#ifdef dDEBUG_ALLOC

#include <string.h>

// when debugging, this is a header that it put at start of all blocks.
// it contains `padding', which are PADSIZE elements of known value just
// before the user memory starts. another PADSIZE padding elements are put
// at the end of the user memory. the idea is that if the user accidentally
// steps outside the allocated memory, it can hopefully be detected by
// examining the padding elements.

#define PADSIZE 10
struct blockHeader {
  long pad1;			// protective padding
  long seq;			// sequence number
  long size;			// data size
  blockHeader *next,*prev;	// next & previous blocks
  long pad[PADSIZE];		// protective padding
};

// compute the memory block size, including padding. the user memory block is
// rounded up to a multiple of 4 bytes, to get alignment for the padding at
// the end of the block.

#define BSIZE(size) (((((size)-1) | 3)+1) + sizeof(blockHeader) + \
  PADSIZE * sizeof(long))

static blockHeader dummyblock = {0,0,0,&dummyblock,&dummyblock,
				 {0,0,0,0,0,0,0,0,0,0}};
static blockHeader *firstblock = &dummyblock;
static long num_blocks_alloced = 0;
static long num_bytes_alloced = 0;
static long total_num_blocks_alloced = 0;
static long total_num_bytes_alloced = 0;
static long max_blocks_alloced = 0;
static long max_bytes_alloced = 0;

long seqstop = 0;
void *ptrstop = 0;

static int checkBlockOk (void *ptr, int fatal)
{
  blockHeader *b = ((blockHeader*) ptr) - 1;
  int i,ok = 1;
  if (b->pad1 != (long)0xdeadbeef || b->seq < 0 || b->size < 0) ok = 0;
  if (ok) {
    for (i=0; i<PADSIZE; i++) if (b->pad[i] != (long)0xdeadbeef) ok = 0;
  }
  if (ok) {
    long *endpad = (long*) (((char*)ptr) + (((b->size-1) | 3)+1));
    for (i=0; i<PADSIZE; i++) if (endpad[i] != (long)0xdeadbeef) ok = 0;
  }
  if (!ok && fatal)
    dDebug (0,"corrupted memory block found, ptr=%p, size=%d, "
	    "seq=%ld", ptr,b->size,b->seq);
  return ok;
}

#endif


static dAllocFunction *allocfn = 0;
static dReallocFunction *reallocfn = 0;
static dFreeFunction *freefn = 0;



void dSetAllocHandler (dAllocFunction *fn)
{
  allocfn = fn;
}


void dSetReallocHandler (dReallocFunction *fn)
{
  reallocfn = fn;
}


void dSetFreeHandler (dFreeFunction *fn)
{
  freefn = fn;
}


void * dAlloc (int size)
{
#ifdef dDEBUG_ALLOC
  if (size < 1) dDebug (0,"bad block size to Alloc()");

  num_blocks_alloced++;
  num_bytes_alloced += size;
  total_num_blocks_alloced++;
  total_num_bytes_alloced += size;
  if (num_blocks_alloced > max_blocks_alloced)
    max_blocks_alloced = num_blocks_alloced;
  if (num_bytes_alloced > max_bytes_alloced)
    max_bytes_alloced = num_bytes_alloced;

  if (total_num_blocks_alloced == seqstop)
    dDebug (0,"ALLOCATOR TRAP ON SEQUENCE NUMBER %d",seqstop);
  long size2 = BSIZE(size);
  blockHeader *b = (blockHeader*) malloc (size2);
  if (b+1 == ptrstop)
    dDebug (0,"ALLOCATOR TRAP ON BLOCK POINTER %p",ptrstop);
  for (unsigned int i=0; i < (size2/sizeof(long)); i++)
    ((long*)b)[i] = 0xdeadbeef;
  b->seq = total_num_blocks_alloced;
  b->size = size;
  b->next = firstblock->next;
  b->prev = firstblock;
  firstblock->next->prev = b;
  firstblock->next = b;
  return b + 1;
#else
  if (allocfn) return allocfn (size); else return malloc (size);
#endif
}


void * dRealloc (void *ptr, int oldsize, int newsize)
{
#ifdef dDEBUG_ALLOC
  if (ptr==0) dDebug (0,"Realloc() called with ptr==0");
  checkBlockOk (ptr,1);
  blockHeader *b = ((blockHeader*) ptr) - 1;
  if (b->size != oldsize)
    dDebug (0,"Realloc(%p,%d,%d) has bad old size, good size "
	    "is %ld, seq=%ld",ptr,oldsize,newsize,b->size,b->seq);
  void *p = dAlloc (newsize);
  if (oldsize>=1) memcpy (p,ptr,oldsize);
  dFree (ptr,oldsize);
  return p;
#else
  if (reallocfn) return reallocfn (ptr,oldsize,newsize);
  else return realloc (ptr,newsize);
#endif
}


void dFree (void *ptr, int size)
{
  if (!ptr) return;
#ifdef dDEBUG_ALLOC
  checkBlockOk (ptr,1);
  blockHeader *b = ((blockHeader*) ptr) - 1;
  if (b->size != size)
    dDebug (0,"Free(%p,%d) has bad size, good size "
	    "is %ld, seq=%ld",ptr,size,b->size,b->seq);
  num_blocks_alloced--;
  num_bytes_alloced -= b->size;
  if (num_blocks_alloced < 0 || num_bytes_alloced < 0)
    dDebug (0,"Free called too many times");

  b->next->prev = b->prev;
  b->prev->next = b->next;
  memset (b,0,BSIZE(b->size));

  free (b);
#else
  if (freefn) freefn (ptr,size); else free (ptr);
#endif
}


#ifdef dDEBUG_ALLOC

#include <stdio.h>

static void printReport()
{
  fprintf (stderr,"\nALLOCATOR REPORT\n");
  fprintf (stderr,"\tblocks still allocated:  %ld\n",num_blocks_alloced);
  fprintf (stderr,"\tbytes still allocated:   %ld\n",num_bytes_alloced);
  fprintf (stderr,"\ttotal blocks allocated:  %ld\n",total_num_blocks_alloced);
  fprintf (stderr,"\ttotal bytes allocated:   %ld\n",total_num_bytes_alloced);
  fprintf (stderr,"\tmax blocks allocated:    %ld\n",max_blocks_alloced);
  fprintf (stderr,"\tmax bytes allocated:     %ld\n",max_bytes_alloced);

  blockHeader *b = firstblock;
  do {
    if (b != &dummyblock) {
      int ok = checkBlockOk (b+1,0);
      fprintf (stderr,"\tUNFREED: %p, size=%ld, seq=%ld (%s)\n",b+1,
	       b->size,b->seq, ok ? "ok" : "CORUPTED");
    }
    b = b->prev;
  }
  while (b != firstblock);
  fprintf (stderr,"\n");
}


struct dMemoryReportPrinter {
  ~dMemoryReportPrinter() { printReport(); }
} _dReportPrinter;

#endif
