

/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_MATRIX_IMPL_H_
#define _ODE_MATRIX_IMPL_H_


#include "fastsolve_impl.h"
#include "fastltsolve_impl.h"


template<unsigned a_stride, unsigned d_stride>
void dxtVectorScale (dReal *a, const dReal *d, unsigned n)
{
    dAASSERT (a && d && n >= 0);
    const dReal *const d_end = d + (size_t)n * d_stride;
    for (; d != d_end; a += a_stride, d += d_stride) {
        *a *= *d;
    }
}


template<unsigned d_stride, unsigned b_stride>
void dxtSolveLDLT (const dReal *L, const dReal *d, dReal *b, unsigned n, unsigned nskip)
{
    dAASSERT (L && d && b && n > 0 && nskip >= n);
    dxtSolveL1<b_stride> (L, b, n, nskip);
    dxtVectorScale<b_stride, d_stride> (b, d, n);
    dxtSolveL1T<b_stride> (L, b, n, nskip);
}


#endif
