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

#include <string.h>
#include <errno.h>
#include "stack.h"
#include "ode/error.h"

//****************************************************************************

#ifndef WIN32

#include <unistd.h>
#include <sys/mman.h>


void dStack::init (int max_size)
{
  if (sizeof(long int) != sizeof(char*)) dDebug (0,"internal");
  if (max_size <= 0) dDebug (0,"Stack::init() given size <= 0");
  base = (char*) mmap (0,max_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANON,0,0);
  if (int(base) == -1) dError (0,"Stack::init(), mmap() failed, "
    "max_size=%d (%s)",max_size,strerror(errno));
  size = max_size;
  pointer = base;
  frame = 0;
}


void dStack::destroy()
{
  munmap (base,size);
  base = 0;
  size = 0;
  pointer = 0;
  frame = 0;
}

#endif

//****************************************************************************

#ifdef WIN32

#include "windows.h"


void dStack::init (int max_size)
{
  if (sizeof(LPVOID) != sizeof(char*)) dDebug (0,"internal");
  if (max_size <= 0) dDebug (0,"Stack::init() given size <= 0");
  base = (char*) VirtualAlloc (NULL,max_size,MEM_RESERVE,PAGE_READWRITE);
  if (base == 0) dError (0,"Stack::init(), VirtualAlloc() failed, "
    "max_size=%d",max_size);
  size = max_size;
  pointer = base;
  frame = 0;
  committed = 0;

  // get page size
  SYSTEM_INFO info;
  GetSystemInfo (&info);
  pagesize = info.dwPageSize;
}


void dStack::destroy()
{
  VirtualFree (base,0,MEM_RELEASE);
  base = 0;
  size = 0;
  pointer = 0;
  frame = 0;
}

#endif
