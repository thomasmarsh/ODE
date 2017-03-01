

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

#ifndef _ODE_FASTLDLT_IMPL_H_
#define _ODE_FASTLDLT_IMPL_H_


#include "error.h"


static void dxSolveL1_2 (const dReal *L, dReal *B, unsigned rowCount, unsigned lSkip);
static void dxSolveL1_1 (const dReal *L, dReal *B, unsigned rowCount, unsigned lSkip);


template<unsigned int d_stride>
void dxtFactorLDLT(dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip)
{
    if (rowCount < 1) return;

    unsigned blockStartRow;
    const unsigned lastRowIndex = rowCount - 1;
    for (blockStartRow = 0; blockStartRow < lastRowIndex; blockStartRow += 2) 
    {
        /* solve L*(D*l)=a, l is scaled elements in 2 x i block at A(i,0) */
        dxSolveL1_2 (A, A + (size_t)rowSkip * blockStartRow, blockStartRow, rowSkip);

        dReal *ptrAElement = A + (size_t)rowSkip * blockStartRow;
        dReal *ptrDElement = d;

        /* scale the elements in a 2 x i block at A(i,0), and also */
        /* compute Z = the outer product matrix that we'll need. */
        dReal Z11 = 0, Z21 = 0, Z22 = 0;

        unsigned columnCounter;
        for (columnCounter = blockStartRow; columnCounter >= 6; columnCounter -= 6) 
        {
            dReal p1, q1, p2, q2, dd;
            dReal m11, m21, m22;

            p1 = ptrAElement[0];
            p2 = ptrAElement[rowSkip];
            dd = ptrDElement[0 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[0] = q1;
            ptrAElement[rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            p1 = ptrAElement[1];
            p2 = ptrAElement[1 + rowSkip];
            dd = ptrDElement[1 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[1] = q1;
            ptrAElement[1 + rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            p1 = ptrAElement[2];
            p2 = ptrAElement[2 + rowSkip];
            dd = ptrDElement[2 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[2] = q1;
            ptrAElement[2 + rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            p1 = ptrAElement[3];
            p2 = ptrAElement[3 + rowSkip];
            dd = ptrDElement[3 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[3] = q1;
            ptrAElement[3 + rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            p1 = ptrAElement[4];
            p2 = ptrAElement[4 + rowSkip];
            dd = ptrDElement[4 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[4] = q1;
            ptrAElement[4 + rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            p1 = ptrAElement[5];
            p2 = ptrAElement[5 + rowSkip];
            dd = ptrDElement[5 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[5] = q1;
            ptrAElement[5 + rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            ptrAElement += 6;
            ptrDElement += 6 * d_stride;
        }

        /* compute left-over iterations */
        for (; columnCounter >= 2; columnCounter -= 2) 
        {
            dReal p1, q1, p2, q2, dd;
            dReal m11, m21, m22;

            p1 = ptrAElement[0];
            p2 = ptrAElement[rowSkip];
            dd = ptrDElement[0 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[0] = q1;
            ptrAElement[rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            p1 = ptrAElement[1];
            p2 = ptrAElement[1 + rowSkip];
            dd = ptrDElement[1 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[1] = q1;
            ptrAElement[1 + rowSkip] = q2;
            m11 = p1 * q1;
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z21 += m21;
            Z22 += m22;

            ptrAElement += 2;
            ptrDElement += 2 * d_stride;
        }

        dIASSERT(columnCounter == 0);

        /* solve for diagonal 2 x 2 block at A(i,i) */
        dReal Y11 = ptrAElement[0] - Z11;
        dReal Y21 = ptrAElement[rowSkip] - Z21;
        dReal Y22 = ptrAElement[1 + rowSkip] - Z22;

        /* factorize 2 x 2 block Y, ptrDElement */
        /* factorize row 1 */
        dReal dd = dRecip(Y11);

        ptrDElement[0 * d_stride] = dd;
        dIASSERT(ptrDElement == d + (size_t)blockStartRow * d_stride);

        /* factorize row 2 */
        dReal q2 = Y21 * dd;
        ptrAElement[rowSkip] = q2;
        
        dReal sum = Y21 * q2;
        ptrDElement[1 * d_stride] = dRecip(Y22 - sum);
        /* done factorizing 2 x 2 block */
    }

    /* compute the (less than 2) rows at the bottom */
    if (blockStartRow != rowCount)
    {
        dxSolveL1_1 (A, A + (size_t)rowSkip * blockStartRow, blockStartRow, rowSkip);

        dReal *ptrAElement = A + (size_t)rowSkip * blockStartRow;
        dReal *ptrDElement = d;

        /* scale the elements in a 1 x i block at A(i,0), and also */
        /* compute Z = the outer product matrix that we'll need. */
        dReal Z11 = 0, Z22 = 0;

        unsigned columnCounter;
        for (columnCounter = blockStartRow; columnCounter >= 6; columnCounter -= 6) 
        {
            dReal p1, p2, q1, q2, dd1, dd2;
            dReal m11, m22;

            p1 = ptrAElement[0];
            p2 = ptrAElement[1];
            dd1 = ptrDElement[0 * d_stride];
            dd2 = ptrDElement[1 * d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ptrAElement[0] = q1;
            ptrAElement[1] = q2;
            m11 = p1 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z22 += m22;

            p1 = ptrAElement[2];
            p2 = ptrAElement[3];
            dd1 = ptrDElement[2 * d_stride];
            dd2 = ptrDElement[3 * d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ptrAElement[2] = q1;
            ptrAElement[3] = q2;
            m11 = p1 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z22 += m22;

            p1 = ptrAElement[4];
            p2 = ptrAElement[5];
            dd1 = ptrDElement[4 * d_stride];
            dd2 = ptrDElement[5 * d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ptrAElement[4] = q1;
            ptrAElement[5] = q2;
            m11 = p1 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z22 += m22;

            ptrAElement += 6;
            ptrDElement += 6 * d_stride;
        }

        /* compute left-over iterations */
        for (; columnCounter >= 2; columnCounter -= 2) 
        {
            dReal p1, p2, q1, q2, dd1, dd2;
            dReal m11, m22;

            p1 = ptrAElement[0];
            p2 = ptrAElement[1];
            dd1 = ptrDElement[0 * d_stride];
            dd2 = ptrDElement[1 * d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ptrAElement[0] = q1;
            ptrAElement[1] = q2;
            m11 = p1 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z22 += m22;

            ptrAElement += 2;
            ptrDElement += 2 * d_stride;
        }

        dIASSERT(columnCounter == 0);

        /* solve for diagonal 1 x 1 block at A(i,i) */
        dReal Y11 = ptrAElement[0] - (Z11 + Z22);
        dIASSERT(ptrDElement == d + (size_t)blockStartRow * d_stride);
        /* factorize 1 x 1 block Y, ptrDElement */
        /* factorize row 1 */
        ptrDElement[0 * d_stride] = dRecip(Y11);
        /* done factorizing 1 x 1 block */
    }
}


/* solve L*X=B, with B containing 2 right hand sides.
 * L is an n*n lower triangular matrix with ones on the diagonal.
 * L is stored by rows and its leading dimension is lSkip.
 * B is an n*2 matrix that contains the right hand sides.
 * B is stored by columns and its leading dimension is also lSkip.
 * B is overwritten with X.
 * this processes blocks of 2*2.
 * if this is in the factorizer source file, n must be a multiple of 2.
 */
static 
void dxSolveL1_2(const dReal *L, dReal *B, unsigned rowCount, unsigned lSkip)
{
    /* compute all 2 x 2 blocks of X */
    unsigned blockStartRow;
    for (blockStartRow = 0; blockStartRow < rowCount; blockStartRow += 2) 
    {
        /* compute all 2 x 2 block of X, from rows i..i+2-1 */
        const dReal *ptrLElement = L + blockStartRow * lSkip;
        dReal *ptrBElement = B;

        /* declare variables - Z matrix and set it to 0 */
        dReal Z11 = 0, Z12 = 0, Z21 = 0, Z22 = 0;

        /* the inner loop that computes outer products and adds them to Z */
        unsigned columnCounter;
        for (columnCounter = blockStartRow; columnCounter >= 2; columnCounter -= 2) 
        {
            /* declare p and q vectors, etc */
            dReal p1, q1, p2, q2;
            dReal m11, m12, m21, m22;

            /* compute outer product and add it to the Z matrix */
            p1 = ptrLElement[0];
            q1 = ptrBElement[0];
            m11 = p1 * q1;
            q2 = ptrBElement[lSkip];
            m12 = p1 * q2;
            p2 = ptrLElement[lSkip];
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z12 += m12;
            Z21 += m21;
            Z22 += m22;

            /* compute outer product and add it to the Z matrix */
            p1 = ptrLElement[1];
            q1 = ptrBElement[1];
            m11 = p1 * q1;
            q2 = ptrBElement[1 + lSkip];
            m12 = p1 * q2;
            p2 = ptrLElement[1 + lSkip];
            m21 = p2 * q1;
            m22 = p2 * q2;

            /* advance pointers */
            ptrLElement += 2;
            ptrBElement += 2;
            Z11 += m11;
            Z12 += m12;
            Z21 += m21;
            Z22 += m22;
            /* end of inner loop */
        }
        
        // The iteration starts with even number and decreases it by 2. So, it must end in zero
        dIASSERT(columnCounter == 0);

        /* finish computing the X(i) block */
        
        dReal Y11 = ptrBElement[0] - Z11;
        dReal Y12 = ptrBElement[lSkip] - Z12;

        ptrBElement[0] = Y11;
        ptrBElement[lSkip] = Y12;

        dReal p2 = ptrLElement[lSkip];

        dReal Y21 = ptrBElement[1] - Z21 - p2 * Y11;
        dReal Y22 = ptrBElement[1 + lSkip] - Z22 - p2 * Y12;

        ptrBElement[1] = Y21;
        ptrBElement[1+lSkip] = Y22;
        /* end of outer loop */
    }
}


/* solve L*X=B, with B containing 1 right hand sides.
 * L is an n*n lower triangular matrix with ones on the diagonal.
 * L is stored by rows and its leading dimension is lskip.
 * B is an n*1 matrix that contains the right hand sides.
 * B is stored by columns and its leading dimension is also lskip.
 * B is overwritten with X.
 * this processes blocks of 2*2.
 * if this is in the factorizer source file, n must be a multiple of 2.
 */
static 
void dxSolveL1_1(const dReal *L, dReal *B, unsigned rowCount, unsigned lSkip)
{  
    /* compute all 2 x 1 blocks of X */
    unsigned blockStartRow;
    for (blockStartRow = 0; blockStartRow < rowCount; blockStartRow += 2) 
    {
        const dReal *ptrLElement = L + (size_t)blockStartRow * lSkip;
        dReal *ptrBElement = B;

        /* variables - Z matrix and set it to 0 */
        dReal Z11 = 0, Z21 = 0;

        /* compute all 2 x 1 block of X, from rows i..i+2-1 */
        /* set the Z matrix to 0 */
        
        /* the inner loop that computes outer products and adds them to Z */
        unsigned columnCounter;
        for (columnCounter = blockStartRow; columnCounter >= 2; columnCounter -= 2) 
        {
            /* declare p and q vectors, etc */
            dReal p1, q1, p2;
            dReal m11, m21;

            /* compute outer product and add it to the Z matrix */
            p1 = ptrLElement[0];
            q1 = ptrBElement[0];
            m11 = p1 * q1;
            p2 = ptrLElement[lSkip];
            m21 = p2 * q1;
            Z11 += m11;
            Z21 += m21;
            
            /* compute outer product and add it to the Z matrix */
            p1 = ptrLElement[1];
            q1 = ptrBElement[1];
            m11 = p1 * q1;
            p2 = ptrLElement[1 + lSkip];
            m21 = p2 * q1;

            /* advance pointers */
            ptrLElement += 2;
            ptrBElement += 2;
            Z11 += m11;
            Z21 += m21;
            /* end of inner loop */
        }
        
        // The iteration starts with even number and decreases it by 2. So, it must end in zero
        dIASSERT(columnCounter == 0);

        /* finish computing the X(i) block */
        dReal p2 = ptrLElement[lSkip];

        dReal Y11 = ptrBElement[0] - Z11;
        dReal Y21 = ptrBElement[1] - Z21 - p2 * Y11;

        ptrBElement[0] = Y11;
        ptrBElement[1] = Y21;
        /* end of outer loop */
    }
}


#endif // #ifndef _ODE_FASTLDLT_IMPL_H_
