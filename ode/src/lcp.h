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

/*

given (A,b), solve the LCP problem: A*x=b+w for x and w, with x and w
satisfying various conditions. A is a matrix of dimension n*n, everything
else is a vector of size n*1.

the first `nub' variables are unbounded, i.e. they can take any value.
the remaining variables must satisfy conditions specified in the mode, k1,
and k2 arrays:
  - if mode[i]=LCP_LO, x[i] >= k1[i]
  - if mode[i]=LCP_HI, x[i] <= k2[i]
  - if mode[i]=LCP_LO_HI, k1[i] <= x[i] <= k2[i]

for 2D friction, use the mode pair (LCP_LO,LCP_F2) for indexes (i..i+1).
the solution will satisfy: |x[i+1]| <= mu*x[i], where mu = k1[i].

for 3D friction, use the mode triplet (LCP_LO,LCP_F3,LCP_F3) for indexes
(i..i+2). the solution will satisfy: sqrt(x[i+1]^2+x[i+2]^2) <= mu*x[i],
where mu = k1[i].


TODO
----

must return (L,d) and some other information, so we can re-solve for other
right hand sides later on, but using the same complimentarity solution so
there are no discontinuities.

*/


#ifndef _ODE_LCP_H_
#define _ODE_LCP_H_

enum {LCP_LO = 0, LCP_HI, LCP_LO_HI, LCP_F2, LCP_F3};


void dSolveLCP (int n, dReal *A, dReal *x, dReal *b, dReal *w,
		 int nub, int *mode, dReal *k1, dReal *k2);



// test the LCP solver

void dTestSolveLCP();


#endif
