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

NOTES
-----

this is an implementation of "lcp_dantzig2_ldlt.m". a naive implementation
of the algorithm requires either a lot of data motion or a lot of
permutation-array lookup, because we are constantly re-ordering rows and
columns. to avoid this and make a more optimized algorithm, a non-trivial
data structure is used to represent the matrix A.

during execution of this algorithm, some indexes in A are clamped, some are
non-clamped, and some are "don't care".
  - set `C' is the clamped indexes, which have w(i)=0, x(i) >= 0
  - set `N' is the non-clamped indexes, which have x(i)=0, w(i) >= 0
  - the other indexes are "dont care", which are similar to non-clamped.
A,x,b, and w are permuted such that the clamped indexes are first, the
unclamped indexes are next, and the don't-care indexes are last.
this permutation is recorded in the array `p'. initially p = 0..n-1, and
as the rows and columns of A,x,b,w are swapped, the corresponding elements of
p are swapped.

because the C and N elements are grouped together in the rows of A, we can do
lots of work with a fast dot product function. if A,x,etc were not permuted
and we only had a permutation array, then those dot products would be much
slower as we would have a permutation array lookup in the inner loop.

A is accessed through an array of row pointers, so that element (i,j) of the
permuted matrix is A[i][j]. this makes row swapping fast. for column swapping
we still have to actually move the data.

during execution of this algorithm we maintain an L*D*L' factorization of
the clamped submatrix of A (call it `AC') which is the top left nC*nC
submatrix of A. there are two ways we could arrange the rows/columns in AC.

(1) AC is always permuted such that L*D*L' = AC. this causes a problem
    when a row/column is removed from C, because then all the rows/columns of A
    between the deleted index and the end of C need to be rotated downward.
    this results in a lot of data motion and slows things down.
(2) L*D*L' is actually a factorization of a *permutation* of AC (which is
    itself a permutation of the underlying A). this is what we do - the
    permutation is recorded in the vector C. call this permutation A[C,C].
    when a row/column is removed from C, all we have to do is swap two
    rows/columns and manipulate C.


TODO
----

* make sure only lower triangle of A needs to be defined.
* caller must save A because of permutation.
* convert some multiple-dot-products into matrix multiplications and take
  advantage of the outer product trick.

*/

#include <stdio.h>
#include <string.h>
#include <malloc.h>		// for alloca under windows
#include <math.h>
#include "ode/common.h"
#include "lcp.h"
#include "ode/matrix.h"
#include "ode/misc.h"
#include "mat.h"		// for testing
#include "ode/timer.h"		// for testing

//***************************************************************************

// LCP debugging - this slows things down a lot
//#define DEBUG_LCP

// option 1 : matrix row pointers (less data copying)
#define ROWPTRS
#define ATYPE dReal **
#define AROW(i) (A[i])

// option 2 : no matrix row pointers (slightly faster inner loops)
//#define NOROWPTRS
//#define ATYPE dReal *
//#define AROW(i) (A+(i)*nskip)

// misc defines
#define ALLOCA dALLOCA16
//#define dDot myDot

#define NUB_OPTIMIZATIONS

//***************************************************************************

// an alternative inline dot product, for speed comparisons

static inline dReal myDot (dReal *a, dReal *b, int n)
{
  dReal sum=0;
  while (n > 0) {
    sum += (*a) * (*b);
    a++;
    b++;
    n--;
  }
  return sum;
}


// swap row/column i1 with i2 in the n*n matrix A. the leading dimension of
// A is nskip. this only references and swaps the lower triangle.

static void swapRowsAndCols (ATYPE A, int n, int i1, int i2, int nskip)
{
  int i;
  dASSERT (A && n > 0 && i1 >= 0 && i2 >= 0 && i1 < n && i2 < n &&
	    nskip >= n && i1 < i2);

# ifdef ROWPTRS
  for (i=i1+1; i<i2; i++) A[i1][i] = A[i][i1];
  for (i=i1+1; i<i2; i++) A[i][i1] = A[i2][i];
  A[i1][i2] = A[i1][i1];
  A[i1][i1] = A[i2][i1];
  A[i2][i1] = A[i2][i2];
  // swap rows, by swapping row pointers
  dReal *tmpp;
  tmpp = A[i1];
  A[i1] = A[i2];
  A[i2] = tmpp;
  for (i=i2+1; i<n; i++) {
    dReal tmp = A[i][i1];
    A[i][i1] = A[i][i2];
    A[i][i2] = tmp;
  }
# else
  dReal tmp,*tmprow = (dReal*) alloca (n * sizeof(dReal));
  if (i1 > 0) {
    memcpy (tmprow,A+i1*nskip,i1*sizeof(dReal));
    memcpy (A+i1*nskip,A+i2*nskip,i1*sizeof(dReal));
    memcpy (A+i2*nskip,tmprow,i1*sizeof(dReal));
  }
  for (i=i1+1; i<i2; i++) {
    tmp = A[i2*nskip+i];
    A[i2*nskip+i] = A[i*nskip+i1];
    A[i*nskip+i1] = tmp;
  }
  tmp = A[i1*nskip+i1];
  A[i1*nskip+i1] = A[i2*nskip+i2];
  A[i2*nskip+i2] = tmp;
  for (i=i2+1; i<n; i++) {
    tmp = A[i*nskip+i1];
    A[i*nskip+i1] = A[i*nskip+i2];
    A[i*nskip+i2] = tmp;
  }
# endif
}


// swap two indexes in the n*n LCP problem. i1 must be <= i2.

static void swapProblem (ATYPE A, dReal *x, dReal *b, dReal *w,
			 int *p, int n, int i1, int i2, int nskip)
{
  dReal tmp;
  int tmpi;
  dASSERT (n>0 && i1 >=0 && i2 >= 0 && i1 < n && i2 < n && nskip >= n &&
	    i1 <= i2);
  if (i1==i2) return;
  swapRowsAndCols (A,n,i1,i2,nskip);
  tmp = x[i1];
  x[i1] = x[i2];
  x[i2] = tmp;
  tmp = b[i1];
  b[i1] = b[i2];
  b[i2] = tmp;
  tmp = w[i1];
  w[i1] = w[i2];
  w[i2] = tmp;
  tmpi = p[i1];
  p[i1] = p[i2];
  p[i2] = tmpi;
}


// for debugging - check that L,d is the factorization of A[C,C].
// A[C,C] has size nC*nC and leading dimension nskip.
// L has size nC*nC and leading dimension nskip.
// d has size nC.

static void checkFactorization (ATYPE A, dReal *_L, dReal *_d,
				int nC, int *C, int nskip)
{
  int i,j;
  if (nC==0) return;

  // get A1=A, copy the lower triangle to the upper triangle, get A2=A[C,C]
  dMatrix A1 (nC,nC);
  for (i=0; i<nC; i++) {
    for (j=0; j<=i; j++) A1(i,j) = A1(j,i) = AROW(i)[j];
  }
  dMatrix A2 = A1.select (nC,C,nC,C);

  // printf ("A1=\n"); A1.print(); printf ("\n");
  // printf ("A2=\n"); A2.print(); printf ("\n");

  // compute A3 = L*D*L'
  dMatrix L (nC,nC,_L,nskip,1);
  dMatrix D (nC,nC);
  for (i=0; i<nC; i++) D(i,i) = 1/_d[i];
  dMatrix A3 = L * D * L.transpose();

  // printf ("L=\n"); L.print(); printf ("\n");
  // printf ("D=\n"); D.print(); printf ("\n");
  // printf ("A3=\n"); A2.print(); printf ("\n");

  // compare A2 and A3
  dReal diff = A2.maxDifference (A3);
  if (diff > 1e-8)
    dDebug (0,"L*D*L' check, maximum difference = %.6e\n",diff);
}


// for debugging

static void checkPermutations (int i, int n, int nC, int nN, int *p, int *C)
{
  int j,k;
  dASSERT (nC>=0 && nN>=0 && (nC+nN)==i && i < n);
  for (k=0; k<i; k++) dASSERT (p[k] >= 0 && p[k] < i);
  for (k=i; k<n; k++) dASSERT (p[k] == k);
  for (j=0; j<nC; j++) {
    int C_is_bad = 1;
    for (k=0; k<nC; k++) if (C[k]==j) C_is_bad = 0;
    dASSERT (C_is_bad==0);
  }
}


// solve the LCP problem:  A*x = b+w, x>=0, w>=0, x'*w=0
// using Dantzig's algorithm. given A and b, we solve for x and w.
// A and b are modified during this algorithm, for speed reasons.


void dSolveLCP (int n, dReal *_A, dReal *return_x, dReal *b,
		 dReal *return_w, int nub, int *mode, dReal *k1, dReal *k2)
{
  int i,j,k;
  dReal sum,*aptr,delta_wi;
  dASSERT (n>0 && _A && return_x && b && return_w && nub >= 0 && nub <= n);
  // @@@ dASSERT (mode && k1 && k2);
  int nskip = dPAD(n);

  // if all the variables are unbounded then we can just factor, solve,
  // and return
  if (nub >= n) {
    dFactorLDLT (_A,return_w,n,nskip);		// use return_w for d
    dSolveLDLT (_A,return_w,b,n,nskip);
    memcpy (return_x,b,n*sizeof(dReal));
    dSetZero (return_w,n);
    return;
  }

  dReal *x = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *w = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *ell = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *Dell = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *tmp = (dReal*) ALLOCA (n*sizeof(dReal));

  // the delta_x and delta_w variables from the matlab code - except we 
  // only store the C and N indexes (respectively).
  dReal *delta_xC = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *delta_wN = (dReal*) ALLOCA (n*sizeof(dReal));

  // L*D*L'=A[C,C], i.e. a permuted top left nC*nC submatrix of A.
  // the leading dimension of the matrix L is always `nskip'.
  dReal *L = (dReal*) ALLOCA (n*nskip*sizeof(dReal));
  dReal *d = (dReal*) ALLOCA (n*sizeof(dReal));

# ifdef ROWPTRS
  // make matrix row pointers
  dReal **A = (dReal**) ALLOCA (n*sizeof(dReal*));
  for (j=0; j<n; j++) A[j] = _A + j*nskip;
# else
  dReal *A = _A;
# endif

# ifdef DEBUG_LCP
  // this aids in debugging
  dSetZero (delta_xC,n);
  dSetZero (delta_wN,n);
  dSetZero (ell,n);
  dSetZero (Dell,n);
  dSetZero (tmp,n);
  dSetZero (w,n);
# endif

  // the permutation of A,x,b,w is recorded in the array `p'.

  int nC = 0;		// number of clamped indexes
  int nN = 0;		// number of non-clamped indexes
  int *p = (int*) ALLOCA (n * sizeof(int));	// permutation list
  for (k=0; k<n; k++) p[k]=k;			// initially unpermuted
  int *C = (int*) ALLOCA (n * sizeof(int));
  for (k=0; k<n; k++) C[k]=0;

  // we start with all indexes initially "don't care", so x=0.
  dSetZero (x,n);

# ifdef DEBUG_LCP
  // this shouldn't be necessary, but it helps for debugging
  dSetZero (L,n*nskip);
  dSetZero (d,n);
  for (k=0; k<n; k++) L[k*nskip+k] = 1;
# endif

  // if there are unbounded variables at the start, factorize A up to that
  // point and solve for x
  if (nub > 0) {
    for (i=0; i<nub; i++) memcpy (L+i*nskip,AROW(i),(i+1)*sizeof(dReal));
    dFactorLDLT (L,d,nub,nskip);
#   ifdef DEBUG_LCP
    for (k=0; k<nub; k++) L[k*nskip+k] = 1;
#   endif
    memcpy (x,b,nub*sizeof(dReal));
    dSolveLDLT (L,d,x,nub,nskip);
    dSetZero (w,nub);
    for (k=0; k<nub; k++) C[k] = k;
    nC = nub;
  }

  // loop over all indexes nub..n-1. for index i, if w[i] is negative then it
  // will be driven up to zero by increasing x[i] and the other clamped x
  // values such that w[C] stays at zero. but as we increase x[i] we maintain
  // the complementarity conditions on the other variables 0..i-1. we do this
  // by watching out for other clamped-x or unclamped-w values that go below
  // zero, and then switching them between the C and N sets. this switching is
  // done by permuting (A,b,x,w).

  for (i=nub; i<n; i++) {
#   ifdef DEBUG_LCP
    checkPermutations (i,n,nC,nN,p,C);
    checkFactorization (A,L,d,nC,C,nskip);
#   endif

    // each time through this loop the indexes 0..i-1 will have been permuted
    // such that C and N are grouped. the index i is the driving index and
    // indexes i+1..n-1 are "dont care", i.e. when we make changes to the
    // system those x's will be zero and we don't care what happens to those
    // w's. in other words, we only consider an (i+1)*(i+1) sub-problem of
    // A*x=b+w.

    // thus far we have not even been computing the w values for indexes
    // greater than i, so compute w[i] now. first get sum = A(i,C)*x(C)
    sum = dDot (AROW(i),x,nC);
    w[i] = sum - b[i];

    if (w[i] < 0) {
      // w[i] is negative, we want to push it up to zero by changing x.
      // delta_x will have x[i]=1, x[N]=0 and x[C] such that the clamped w's
      // remain at zero.

      for (;;) {
#       ifdef DEBUG_LCP
	checkPermutations (i,n,nC,nN,p,C);
	checkFactorization (A,L,d,nC,C,nskip);
#       endif

#       ifdef DEBUG_LCP
	// this is not really necessary but helps for debugging
	dSetZero (delta_xC,n);
	dSetZero (delta_wN,n);
	delta_wi = 0;
#       endif

	if (nC > 0) {
	  // compute: delta_x[C] = -A[C,C]\A[C,i]; :
	  //    Dell = L1solve (L,A(i,C));
	  //    ell = D \ Dell;
	  //    delta_x(p) = -L1Tsolve (L,ell);
	  // note: ell and Dell will be reused later if row/col i is added to
	  // the factorization.
//@@@printf ("factorizing row %d\n",i);
	  aptr = AROW(i);				// row i
#         ifdef NUB_OPTIMIZATIONS
	  // if nub>0, initial part of aptr[] is guaranteed unpermuted
	  for (j=0; j<nub; j++) Dell[j] = aptr[j];
          for (j=nub; j<nC; j++) Dell[j] = aptr[C[j]];
#         else
          for (j=0; j<nC; j++) Dell[j] = aptr[C[j]];
#         endif
	  dSolveL1 (L,Dell,nC,nskip);
	  for (j=0; j<nC; j++) ell[j] = Dell[j] * d[j];
	  for (j=0; j<nC; j++) tmp[j] = ell[j];
	  //@@@ question: do we need to solve for entire delta_x??? yes, but
	  // only if an x goes below 0 during the step.
	  dSolveL1T (L,tmp,nC,nskip);
	  for (j=0; j<nC; j++) delta_xC[C[j]] = -tmp[j];
	}

      skip_solving_for_delta_x:

	// compute: delta_w = A*delta_x ... note we only care about
	// delta_w[N] and delta_w[i], the rest is ignored
	for (j=0; j<nN; j++) {
	  sum = dDot (AROW(nC+j),delta_xC,nC);
	  delta_wN[j] = sum + AROW(i)[nC+j];
	}
	sum = dDot (AROW(i),delta_xC,nC);
	delta_wi = sum + AROW(i)[i];

	// find s = the maximum multiple of delta_x that we can add to x, such
	// that either we drive w[i] to zero or a clamped x goes below zero, or
	// an unclamped w goes below zero.

	int si = i;		// si = switch index
	int si_in_N = 0;	// set to 1 if si in N
	dReal s = -w[i]/delta_wi;
	if (s <= 0) dDebug (0,"LCP internal error, s <= 0");
	for (k=0; k<nN; k++) {
	  if (delta_wN[k] < 0) {
	    dReal s2 = -w[nC+k] / delta_wN[k];
	    if (s2 < s) {
	      s = s2;
	      si = nC+k;
	      si_in_N = 1;
	    }
	  }
	}
	for (k=nub; k<nC; k++) {
	  if (delta_xC[k] < 0) {
	    dReal s2 = -x[k] / delta_xC[k];
	    if (s2 < s) {
	      s = s2;
	      si = k;
	      si_in_N = 0;
	    }
	  }
	}

	// apply x = x + s * delta_x
	for (k=0; k<nC; k++) x[k] += s * delta_xC[k];
	x[i] += s;		// remember, delta_x[i] is 1
	for (k=0; k<nN; k++) w[nC+k] += s * delta_wN[k];
	w[i] += s * delta_wi;

	// if necessary, switch indexes between sets and update the
	// factorization
	if (si==i) {
	  // pushed index i as far as it needs to go. factorize row/column i of
	  // A into the bottom of L.

	  //printf ("matrix update (1), si=%d\n",si);
	  if (nC > 0) {
	    // ell,Dell were computed above --> ell = D \ L1solve (L,A(i,C))
	    for (k=0; k<nC; k++) L[nC*nskip+k] = ell[k];
	    d[nC] = dRecip (AROW(i)[i] - dDot(ell,Dell,nC));
	  }
	  else {
	    d[0] = dRecip (AROW(i)[i]);
	  }
	  swapProblem (A,x,b,w,p,n,nC,i,nskip);
	  C[nC] = nC;
	  nC++;
	  break;
	}
	else if (si_in_N) {
	  // an unclamped w has gone below zero, it needs to be clamped.
	  // factorize a new row into L.

	  //printf ("matrix update (2), si=%d\n",si);
	  if (nC > 0) {
//@@@printf ("factorizing row %d\n",si);
	    aptr = AROW(si);
#           ifdef NUB_OPTIMIZATIONS
	    // if nub>0, initial part of aptr unpermuted
            for (j=0; j<nub; j++) Dell[j] = aptr[j];
            for (j=nub; j<nC; j++) Dell[j] = aptr[C[j]];
#           else
            for (j=0; j<nC; j++) Dell[j] = aptr[C[j]];
#           endif
	    dSolveL1 (L,Dell,nC,nskip);
	    for (j=0; j<nC; j++) ell[j] = Dell[j] * d[j];
	    for (j=0; j<nC; j++) L[nC*nskip+j] = ell[j];
	    d[nC] = dRecip (AROW(si)[si] - dDot(ell,Dell,nC));
	  }
	  else {
	    d[0] = dRecip (AROW(si)[si]);
	  }
	  swapProblem (A,x,b,w,p,n,nC,si,nskip);
	  C[nC] = nC;
	  nN--;
	  nC++;

	  // @@@ TO DO LATER @@@
	  // if we just finish here then we'll go back and re-solve for
	  // delta_x. but actually we can be more efficient and incrementally
	  // update delta_x here. but if we do this, we wont have ell and Dell
	  // to use in updating the factorization later.
	  //
	  // ...
	  // goto skip_solving_for_delta_x;
	}
	else {
	  // a clamped x has gone below zero, it needs to be unclamped.
	  // remove a row/column from the factorization, and adjust the
	  // indexes (black magic!)

	  //printf ("matrix downdate, si=%d\n",si);
          for (j=0; j<nC; j++) if (C[j]==si) {
	    dLDLTRemove (A,C,L,d,n,nC,j,nskip);
	    for (k=0; k<nC; k++) if (C[k]==nC-1) {
	      C[k] = C[j];
	      if (j < (nC-1)) memmove (C+j,C+j+1,(nC-j-1)*sizeof(int));
	      break;
	    }
	    dASSERT (k < nC);
	    break;
	  }
          dASSERT (j < nC);

	  swapProblem (A,x,b,w,p,n,si,nC-1,nskip);
	  nC--;
	  nN++;
	}
      }
    }
    else {
      // w(i) is >= 0, so add it to the unclamped set - easy!
      nN++;
    }
  }

  // now we have to un-permute x and w
  for (j=0; j<n; j++) return_x[p[j]] = x[j];
  for (j=0; j<n; j++) return_w[p[j]] = w[j];
}

//***************************************************************************

#include <time.h>

unsigned long start_seed=0;


void dTestSolveLCP()
{
  int n = 100;
  int i,nskip = dPAD(n);
  const dReal tol = 1e-9;
  printf ("dTestSolveLCP()\n");

  dRandSetSeed (0);
  //dRandSetSeed (time(0));
  //dRandSetSeed (0xe3ec57a4);
  // FILE *fx = fopen ("matlab/test/xdata","rt");
  // FILE *fw = fopen ("matlab/test/wdata","rt");

  dReal *A = (dReal*) ALLOCA (n*nskip*sizeof(dReal));
  dReal *x = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *b = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *w = (dReal*) ALLOCA (n*sizeof(dReal));

  dReal *A2 = (dReal*) ALLOCA (n*nskip*sizeof(dReal));
  dReal *b2 = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *tmp1 = (dReal*) ALLOCA (n*sizeof(dReal));
  dReal *tmp2 = (dReal*) ALLOCA (n*sizeof(dReal));

  double total_time = 0;

  for (int count=0; count < 10000; count++) {
    //printf ("************************************************* %d\n",count);

    start_seed = dRandGetSeed();
    // dRandSetSeed (0xbfbe7c7c);	// nearly singular A
    // dRandSetSeed (0x9bfed314);	// nearly singular A
    // dRandSetSeed (0xcafafa82);	// nearly singular A
    // dRandSetSeed (0x48ae4578);	// even more nearly singular A

    // form (A,b) = a random positive definite LCP problem
    dMakeRandomMatrix (A2,n,n,1.0);
    dMultiply2 (A,A2,A2,n,n,n);
    dMakeRandomMatrix (x,n,1,1.0);
    dMultiply0 (b,A,x,n,n,1);
    for (i=0; i<n; i++) b[i] += (dRandReal()*0.2)-0.1;

    // choose `nub' in the range 0..n-1
    int nub = 80; //dRandInt (n);

    // dPrintMatrix (A,n,n,"%.15e ");
    // dPrintMatrix (b,n,1,"%.15e ");

    // solve the LCP. we must make copy of A and b (A2,b2) for SolveLCP() to
    // permute. also, we'll clear the upper triangle of A2 to ensure that
    // it doesn't get referenced (if it does, the answer will be wrong).
    memcpy (A2,A,n*nskip*sizeof(dReal));
    dClearUpperTriangle (A2,n);
    memcpy (b2,b,n*sizeof(dReal));
    dSetZero (x,n);
    dSetZero (w,n);
    dStopwatch sw;
    dStopwatchReset (&sw);
    dStopwatchStart (&sw);

    dSolveLCP (n,A2,x,b2,w,nub,0,0,0);

    dStopwatchStop (&sw);
    double time = dStopwatchTime(&sw);
    total_time += time;
    double average = total_time / double(count+1) * 1000.0;

    // count the percentage of x's
    int nz = 0;
    for (i=0; i<n; i++) if (dFabs(x[i]) > tol) nz++;
    double percentage = 100.0*double(nz)/double(n);

    // pacifier
    printf ("time = %10.3f ms  average = %10.4f  %.2f %% x nonzero\n",
	    time * 1000.0,average,percentage);

    // check the solution

    dMultiply0 (tmp1,A,x,n,n,1);
    for (i=0; i<n; i++) tmp2[i] = b[i] + w[i];
    dReal diff = dMaxDifference (tmp1,tmp2,n,1);
    //printf ("\tA*x = b+w, maximum difference = %.6e - %s (1)\n",diff,
    //	    diff > tol ? "FAILED" : "passed");
    if (diff > tol) dDebug (0,"A*x = b+w, maximum difference = %.6e",diff);
    int bad = 0;
    for (i=nub; i<n; i++) if (x[i] < -tol) bad = 1;
    //printf ("\tx >= 0, %s (2)\n",bad ? "FAILED" : "passed");
    if (bad) dDebug (0,"x >= 0");
    bad = 0;
    for (i=nub; i<n; i++) if (w[i] < -tol) bad = 1;
    //printf ("\tw >= 0, %s (3)\n",bad ? "FAILED" : "passed");
    if (bad) dDebug (0,"w >= 0");
    bad = 0;
    for (i=0; i<n; i++) if (dFabs(x[i]*w[i]) > tol) bad = 1;
    //printf ("\tx'*w != 0, %s (4)\n",bad ? "FAILED" : "passed");
    if (bad) dDebug (0,"x'*w != 0");
    bad = 0;
    for (i=0; i<nub; i++) if (dFabs(w[i]) > tol) bad = 1;
    if (bad) dDebug (0,"w[0..nub-1] != 0");

    // check the solution (2)
    /*
    if (count < 1000) {
      for (i=0; i<n; i++) {
	double xx=0,diff;
	fscanf (fx,"%lf",&xx);
	diff = dFabs(x[i] - xx);
	//printf ("x--> %f %f %e\n",x[i],xx,diff);
	if (diff > 1e-8) dDebug (0,"x and xdata differ!\n");
      }
      for (i=0; i<n; i++) {
	double ww=0,diff;
	fscanf (fw,"%lf",&ww);
	diff = dFabs(w[i] - ww);
	//printf ("w--> %f %f %e\n",w[i],ww,diff);
	if (diff > 1e-8) dDebug (0,"w and wdata differ!\n");
      }
    }
    */
  }
}
