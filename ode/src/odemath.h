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

#ifndef _ODE__PRIVATE_ODEMATH_H_
#define _ODE__PRIVATE_ODEMATH_H_

#include <ode/odemath.h>
#include "error.h"


bool dxCouldBeNormalized3(const dVector3 a);
bool dxSafeNormalize3 (dVector3 a);
bool dxCouldBeNormalized4(const dVector4 a);
bool dxSafeNormalize4 (dVector4 a);

ODE_PURE_INLINE 
void dxNormalize3(dVector3 a)
{
    bool bSafeNormalize3Fault;
    if ((bSafeNormalize3Fault = !dxSafeNormalize3(a)))
    {
        dIVERIFY(!bSafeNormalize3Fault);

        a[0] = REAL(1.0); a[2] = a[1] = REAL(0.0);
    }
}

ODE_PURE_INLINE 
void dxNormalize4(dVector4 a)
{
    bool bSafeNormalize4Fault;
    if ((bSafeNormalize4Fault = !dxSafeNormalize4(a)))
    {
        dIVERIFY(!bSafeNormalize4Fault);

        a[0] = REAL(1.0); a[3] = a[2] = a[1] = REAL(0.0);
    }
}

void dxPlaneSpace (const dVector3 n, dVector3 p, dVector3 q);
bool dxOrthogonalizeR(dMatrix3 m);

// For internal use
#define dSafeNormalize3(a) dxSafeNormalize3(a)
#define dSafeNormalize4(a) dxSafeNormalize4(a)
#define dNormalize3(a) dxNormalize3(a)
#define dNormalize4(a) dxNormalize4(a)

#define dPlaneSpace(n, p, q) dxPlaneSpace(n, p, q)
#define dOrthogonalizeR(m) dxOrthogonalizeR(m)


//////////////////////////////////////////////////////////////////////////

namespace dxTruncToType
{

ODE_PURE_INLINE 
void _dMultiplyHelper0_331(volatile dReal *res, const dReal *a, const dReal *b)
{
    const dReal res_0 = dCalcVectorDot3(a, b);
    const dReal res_1 = dCalcVectorDot3(a + 4, b);
    const dReal res_2 = dCalcVectorDot3(a + 8, b);
    /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
    res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE 
void dMultiply0_331(volatile dReal *res, const dReal *a, const dReal *b)
{
    _dMultiplyHelper0_331(res, a, b);
}


}; // namespace dxTruncToType


#endif
