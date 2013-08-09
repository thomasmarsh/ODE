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

#include <ode/odeconfig.h>
#include <ode/misc.h>
#include "config.h"
#include "matrix.h"
#include "error.h"

//****************************************************************************
// random numbers

static unsigned long seed = 0;

unsigned long dRand()
{
    seed = (1664525UL*seed + 1013904223UL) & 0xffffffff;
    return seed;
}


unsigned long  dRandGetSeed()
{
    return seed;
}


void dRandSetSeed (unsigned long s)
{
    seed = s;
}


int dTestRand()
{
    unsigned long oldseed = seed;
    int ret = 1;
    seed = 0;
    if (dRand() != 0x3c6ef35f || dRand() != 0x47502932 ||
        dRand() != 0xd1ccf6e9 || dRand() != 0xaaf95334 ||
        dRand() != 0x6252e503) ret = 0;
    seed = oldseed;
    return ret;
}


// adam's all-int straightforward(?) dRandInt (0..n-1)
int dRandInt (int n)
{
    // Since there is no memory barrier macro in ODE assign via volatile variable 
    // to prevent compiler reusing seed as value of `r'
    volatile unsigned long raw_r = dRand();
    duint32 r = (duint32)raw_r;
    
    duint32 un = n;
    dIASSERT(sizeof(n) == sizeof(un));

    return (int)(((duint64)r * un) >> 32);
}


dReal dRandReal()
{
    return ((dReal) dRand()) / ((dReal) 0xffffffff);
}

//****************************************************************************
// matrix utility stuff

void dPrintMatrix (const dReal *A, int n, int m, char *fmt, FILE *f)
{
    int skip = dPAD(m);
    const dReal *Arow = A;
    for (int i=0; i<n; Arow+=skip, ++i) {
        for (int j=0; j<m; ++j) fprintf (f,fmt,Arow[j]);
        fprintf (f,"\n");
    }
}


void dMakeRandomVector (dReal *A, int n, dReal range)
{
    int i;
    for (i=0; i<n; i++) A[i] = (dRandReal()*REAL(2.0)-REAL(1.0))*range;
}


void dMakeRandomMatrix (dReal *A, int n, int m, dReal range)
{
    int skip = dPAD(m);
    //  dSetZero (A,n*skip);
    dReal *Arow = A;
    for (int i=0; i<n; Arow+=skip, ++i) {
        for (int j=0; j<m; ++j) Arow[j] = (dRandReal()*REAL(2.0)-REAL(1.0))*range;
    }
}


void dClearUpperTriangle (dReal *A, int n)
{
    int skip = dPAD(n);
    dReal *Arow = A;
    for (int i=0; i<n; Arow+=skip, ++i) {
        for (int j=i+1; j<n; ++j) Arow[j] = 0;
    }
}


dReal dMaxDifference (const dReal *A, const dReal *B, int n, int m)
{
    int skip = dPAD(m);
    dReal max = REAL(0.0);
    const dReal *Arow = A, *Brow = B;
    for (int i=0; i<n; Arow+=skip, Brow +=skip, ++i) {
        for (int j=0; j<m; ++j) {
            dReal diff = dFabs(Arow[j] - Brow[j]);
            if (diff > max) max = diff;
        }
    }
    return max;
}


dReal dMaxDifferenceLowerTriangle (const dReal *A, const dReal *B, int n)
{
    int skip = dPAD(n);
    dReal max = REAL(0.0);
    const dReal *Arow = A, *Brow = B;
    for (int i=0; i<n; Arow+=skip, Brow+=skip, ++i) {
        for (int j=0; j<=i; ++j) {
            dReal diff = dFabs(Arow[j] - Brow[j]);
            if (diff > max) max = diff;
        }
    }
    return max;
}

