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

// matrix class. this is mostly for convenience in the testing code, it is
// not optimized at all. correctness is much more importance here.

#ifndef _ODE_MAT_H_
#define _ODE_MAT_H_

#include <ode/common.h>


class dMatrix {
  int n,m;		// matrix dimension, n,m >= 0
  dReal *data;		// if nonzero, n*m elements allocated on the heap

public:
  // constructors, destructors
  dMatrix();				// make default 0x0 matrix
  dMatrix (int rows, int cols);		// construct zero matrix of given size
  dMatrix (const dMatrix &);		// construct copy of given matrix
  // create copy of given data - element (i,j) is data[i*rowskip+j*colskip]
  dMatrix (int rows, int cols, dReal *_data, int rowskip, int colskip);
  ~dMatrix();				// destructor

  // data movement
  dReal & operator () (int i, int j);	// reference an element
  void operator= (const dMatrix &);	// matrix = matrix
  void operator= (dReal);		// matrix = scalar
  dMatrix transpose();			// return transposed matrix
  // return a permuted submatrix of this matrix, made up of the rows in p
  // and the columns in q. p has np elements, q has nq elements.
  dMatrix select (int np, int *p, int nq, int *q);

  // operators
  dMatrix operator + (const dMatrix &);
  dMatrix operator - (const dMatrix &);
  dMatrix operator - ();
  dMatrix operator * (const dMatrix &);
  void operator += (const dMatrix &);
  void operator -= (const dMatrix &);

  // utility
  void clearUpperTriangle();
  void clearLowerTriangle();
  void makeRandom (dReal range);
  void print (char *fmt = "%10.4f ", FILE *f=stdout);
  dReal maxDifference (const dMatrix &);
};


#endif
