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

#include <ode/common.h>
#include "config.h"
#include "odemath.h"


#undef dSafeNormalize3
#undef dSafeNormalize4
#undef dNormalize3
#undef dNormalize4

#undef dPlaneSpace
#undef dOrthogonalizeR


int  dSafeNormalize3 (dVector3 a)
{
    return dxSafeNormalize3(a);
}

int dSafeNormalize4 (dVector4 a)
{
    return dxSafeNormalize4(a);
}

void dNormalize3(dVector3 a)
{
    dxNormalize3(a);
}

void dNormalize4(dVector4 a)
{
    dxNormalize4(a);
}


void dPlaneSpace(const dVector3 n, dVector3 p, dVector3 q)
{
    return dxPlaneSpace(n, p, q);
}

void dOrthogonalizeR(dMatrix3 m)
{
    dxOrthogonalizeR(m);
}


// this may be called for vectors `a' with extremely small magnitude, for
// example the result of a cross product on two nearly perpendicular vectors.
// we must be robust to these small vectors. to prevent numerical error,
// first find the component a[i] with the largest magnitude and then scale
// all the components by 1/a[i]. then we can compute the length of `a' and
// scale the components by 1/l. this has been verified to work with vectors
// containing the smallest representable numbers.

bool dxSafeNormalize3 (dVector3 a)
{
    dAASSERT (a);

    bool ret = false;

    do {
        dReal abs_a0 = dFabs(a[0]);
        dReal abs_a1 = dFabs(a[1]);
        dReal abs_a2 = dFabs(a[2]);

        int idx;

        if (abs_a1 > abs_a0) {
            if (abs_a2 > abs_a1) { // abs_a2 is the largest
                idx = 2;
            }
            else {              // abs_a1 is the largest
                idx = 1;
            }
        }
        else if (abs_a2 > abs_a0) {// abs_a2 is the largest
            idx = 2;
        }
        else {              // aa[0] might be the largest
            if (!(abs_a0 > REAL(0.0))) { 
                // if all a's are zero, this is where we'll end up.
                // return the vector unchanged.
                break;
            }

            // abs_a0 is the largest
            idx = 0;
        }

        if (idx == 0) {
            dReal aa0_recip = dRecip(abs_a0);
            dReal a1 = a[1] * aa0_recip;
            dReal a2 = a[2] * aa0_recip;
            dReal l = dRecipSqrt(REAL(1.0) + a1 * a1 + a2 * a2);
            a[1] = a1 * l;
            a[2] = a2 * l;
            a[0] = dCopySign(l, a[0]);
        }
        else if (idx == 1) {
            dReal aa1_recip = dRecip(abs_a1);
            dReal a0 = a[0] * aa1_recip;
            dReal a2 = a[2] * aa1_recip;
            dReal l = dRecipSqrt(REAL(1.0) + a0 * a0 + a2 * a2);
            a[0] = a0 * l;
            a[2] = a2 * l;
            a[1] = dCopySign(l, a[1]);
        }
        else {
            dReal aa2_recip = dRecip(abs_a2);
            dReal a0 = a[0] * aa2_recip;
            dReal a1 = a[1] * aa2_recip;
            dReal l = dRecipSqrt(REAL(1.0) + a0 * a0 + a1 * a1);
            a[0] = a0 * l;
            a[1] = a1 * l;
            a[2] = dCopySign(l, a[2]);
        }

        ret = true;
    }
    while (false);

    return ret;
}

/* OLD VERSION */
/*
void dNormalize3 (dVector3 a)
{
    dIASSERT (a);
    dReal l = dCalcVectorDot3(a,a);
    if (l > 0) {
        l = dRecipSqrt(l);
        a[0] *= l;
        a[1] *= l;
        a[2] *= l;
    }
    else {
        a[0] = 1;
        a[1] = 0;
        a[2] = 0;
    }
}
*/

bool dxSafeNormalize4 (dVector4 a)
{
    dAASSERT (a);

    bool ret = false;

    dReal l = dCalcVectorDot3(a,a)+a[3]*a[3];
    if (l > 0) {
        l = dRecipSqrt(l);
        a[0] *= l;
        a[1] *= l;
        a[2] *= l;
        a[3] *= l;
        
        ret = true;
    }

    return ret;
}


void dxPlaneSpace (const dVector3 n, dVector3 p, dVector3 q)
{
    dAASSERT (n && p && q);
    if (dFabs(n[2]) > M_SQRT1_2) {
        // choose p in y-z plane
        dReal a = n[1]*n[1] + n[2]*n[2];
        dReal k = dRecipSqrt (a);
        p[0] = 0;
        p[1] = -n[2]*k;
        p[2] = n[1]*k;
        // set q = n x p
        q[0] = a*k;
        q[1] = -n[0]*p[2];
        q[2] = n[0]*p[1];
    }
    else {
        // choose p in x-y plane
        dReal a = n[0]*n[0] + n[1]*n[1];
        dReal k = dRecipSqrt (a);
        p[0] = -n[1]*k;
        p[1] = n[0]*k;
        p[2] = 0;
        // set q = n x p
        q[0] = -n[2]*p[1];
        q[1] = n[2]*p[0];
        q[2] = a*k;
    }
}


/*
* This takes what is supposed to be a rotation matrix,
* and make sure it is correct.
* Note: this operates on rows, not columns, because for rotations
* both ways give equivalent results.
*/
void dxOrthogonalizeR(dMatrix3 m)
{
    dReal n0 = dCalcVectorLengthSquare3(m);
    if (n0 != REAL(1.0)) {
        bool row0_norm_fault = !dxSafeNormalize3(m);
        dICHECK(!row0_norm_fault);
    }

    // project row[0] on row[1], should be zero
    dReal proj = dCalcVectorDot3(m, m+4);
    if (proj != 0) {
        // Gram-Schmidt step on row[1]
        m[4] -= proj * m[0];
        m[5] -= proj * m[1];
        m[6] -= proj * m[2];
    }

    dReal n1 = dCalcVectorLengthSquare3(m+4);
    if (n1 != REAL(1.0)) {
        bool row1_norm_fault = !dxSafeNormalize3(m+4);
        dICHECK(!row1_norm_fault);
    }

    /* just overwrite row[2], this makes sure the matrix is not
    a reflection */
    dCalcVectorCross3(m+8, m, m+4);
    m[3] = m[4+3] = m[8+3] = 0;
}
