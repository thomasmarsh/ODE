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

/* per-machine configuration */

#ifndef _ODE_CONFIG_H_
#define _ODE_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


/* some types. assume `int' >= 32 bits */
typedef unsigned int    uint;
typedef int             int32;
typedef unsigned int    uint32;
typedef short           int16;
typedef unsigned short  uint16;
typedef char            int8;
typedef unsigned char   uint8;


/* if we're compiling on a pentium, we may need to know the clock rate so
 * that the timing function can report accurate times. i have not worked
 * out how to determine this automatically yet.
 */

#ifdef PENTIUM
#ifndef PENTIUM_HZ
#define PENTIUM_HZ (496.318983e6)
#endif
#endif



/* for unix, define this if your system supports anonymous memory maps
 * (linux does).
 */

#define MMAP_ANONYMOUS


#ifdef __cplusplus
}
#endif

#endif
