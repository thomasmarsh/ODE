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

/* stuff used for testing */

#ifndef _ODE_TESTING_H_
#define _ODE_TESTING_H_

#include "ode/common.h"
#include "array.h"


// compare a sequence of named matrices/vectors, i.e. to make sure that two
// different pieces of code are giving the same results.

class dMatrixComparison {
  struct dMatInfo;
  dArray<dMatInfo*> mat;
  int afterfirst,index;

public:
  dMatrixComparison();
  ~dMatrixComparison();

  dReal nextMatrix (dReal *A, int n, int m, int lower_tri, char *name, ...);
  // add a new n*m matrix A to the sequence. the name of the matrix is given
  // by the printf-style arguments (name,...). if this is the first sequence
  // then this object will simply record the matrices and return 0.
  // if this the second or subsequent sequence then this object will compare
  // the matrices with the first sequence, and report any differences.
  // the matrix error will be returned. if `lower_tri' is 1 then only the
  // lower triangle of the matrix (including the diagonal) will be compared
  // (the matrix must be square).

  void end();
  // end a sequence.

  void reset();
  // restarts the object, so the next sequence will be the first sequence.

  void dump();
  // print out info about all the matrices in the sequence
};


#endif
