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

#include <stdio.h>
#include <malloc.h>		// for alloca under windows
#include "objects.h"
#include "joint.h"
#include "ode/odemath.h"
#include "ode/rotation.h"
#include "ode/timer.h"
#include "ode/error.h"
#include "ode/matrix.h"
#include "lcp.h"

#define FAST_FACTOR
//#define TIMING

//****************************************************************************
// debugging - comparison of various vectors and matrices produced by the
// slow and fast versions of the stepper.

//#define COMPARE_METHODS

#ifdef COMPARE_METHODS

static int slow_m=0;
static dReal *slow_A=0, *slow_c=0, *slow_rhs=0, *slow_lambda=0, *slow_vnew=0;
static dReal *fast_A=0, *fast_c=0, *fast_rhs=0, *fast_lambda=0, *fast_vnew=0;


static dReal * copyMatrix (dReal *A, int n, int m)
{
  if (A && n > 0 && m > 0) {
    int skip = dPAD(m);
    dReal *A2 = (dReal*) dAlloc (n*skip*sizeof(dReal));
    memcpy (A2,A,n*skip*sizeof(dReal));
    return A2;
  }
  else return 0;
}


static void deleteMatrix (dReal * & A, int n, int m)
{
  if (A) {
    int skip = dPAD(m);
    dFree (A,n*skip*sizeof(dReal));
  }
  A = 0;
}


static void compareMatrix (char *name, dReal *A, dReal *B, int n, int m)
{
  if (A) {
    dReal maxdiff=0;
    int skip = dPAD(m);
    for (int i=0; i < n*skip; i++) {
      dReal diff = dFabs (A[i] - B[i]);
      if (diff > maxdiff) maxdiff = diff;
    }
    printf ("comparing %s : max difference = %.4e\n",name,maxdiff);
  }
}


// compare the lower triangle of square matrices only

static void compareMatrixLT (char *name, dReal *A, dReal *B, int n)
{
  if (A) {
    dReal maxdiff=0;
    int skip = dPAD(n);
    for (int i=0; i<n; i++) {
      for (int j=0; j <= i; j++) {
	dReal diff = dFabs (A[i*skip+j] - B[i*skip+j]);
	if (diff > maxdiff) maxdiff = diff;
      }
    }
    printf ("comparing %s : max difference = %.4e\n",name,maxdiff);
    if (maxdiff > 1e-8) {
      printf ("A1=\n");
      dPrintMatrix (A,n,n);
      printf ("\nA2=\n");
      dPrintMatrix (B,n,n);
      exit(1);
    }
  }
}

#endif

//****************************************************************************
// special matrix multipliers

// this assumes the 4th and 8th rows of B and C are zero.

static void Multiply2_p8r (dReal *A, dReal *B, dReal *C,
			   int p, int r, int Askip)
{
  int i,j;
  dReal sum,*bb,*cc;
  dASSERT (p>0 && r>0 && A && B && C);
  bb = B;
  for (i=p; i; i--) {
    cc = C;
    for (j=r; j; j--) {
      sum = bb[0]*cc[0];
      sum += bb[1]*cc[1];
      sum += bb[2]*cc[2];
      sum += bb[4]*cc[4];
      sum += bb[5]*cc[5];
      sum += bb[6]*cc[6];
      *(A++) = sum; 
      cc += 8;
    }
    A += Askip - r;
    bb += 8;
  }
}


// this assumes the 4th and 8th rows of B and C are zero.

static void MultiplyAdd2_p8r (dReal *A, dReal *B, dReal *C,
			      int p, int r, int Askip)
{
  int i,j;
  dReal sum,*bb,*cc;
  dASSERT (p>0 && r>0 && A && B && C);
  bb = B;
  for (i=p; i; i--) {
    cc = C;
    for (j=r; j; j--) {
      sum = bb[0]*cc[0];
      sum += bb[1]*cc[1];
      sum += bb[2]*cc[2];
      sum += bb[4]*cc[4];
      sum += bb[5]*cc[5];
      sum += bb[6]*cc[6];
      *(A++) += sum; 
      cc += 8;
    }
    A += Askip - r;
    bb += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void Multiply0_p81 (dReal *A, dReal *B, dReal *C, int p)
{
  int i;
  dASSERT (p>0 && A && B && C);
  dReal sum;
  for (i=p; i; i--) {
    sum =  B[0]*C[0];
    sum += B[1]*C[1];
    sum += B[2]*C[2];
    sum += B[4]*C[4];
    sum += B[5]*C[5];
    sum += B[6]*C[6];
    *(A++) = sum;
    B += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplyAdd0_p81 (dReal *A, dReal *B, dReal *C, int p)
{
  int i;
  dASSERT (p>0 && A && B && C);
  dReal sum;
  for (i=p; i; i--) {
    sum =  B[0]*C[0];
    sum += B[1]*C[1];
    sum += B[2]*C[2];
    sum += B[4]*C[4];
    sum += B[5]*C[5];
    sum += B[6]*C[6];
    *(A++) += sum;
    B += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplyAdd1_8q1 (dReal *A, dReal *B, dReal *C, int q)
{
  int k;
  dReal sum;
  dASSERT (q>0 && A && B && C);
  sum = 0;
  for (k=0; k<q; k++) sum += B[k*8] * C[k];
  A[0] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[1+k*8] * C[k];
  A[1] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[2+k*8] * C[k];
  A[2] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[4+k*8] * C[k];
  A[4] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[5+k*8] * C[k];
  A[5] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[6+k*8] * C[k];
  A[6] += sum;
}

//****************************************************************************

static const dReal erp = 0.2;		// error reduction parameter
static const dReal epsilon = 0.0001;


// given lists of bodies and joints that form an island, perform a first
// order timestep.
//
// `body' is the body array, `nb' is the size of the array.
// `joint' is the body array, `nj' is the size of the array.

void dInternalStepIsland_x1 (dxWorld *world, dxBody **body, int nb,
			     dxJoint **joint, int nj, dReal stepsize)
{
  int i,j,k;
  int n6 = 6*nb;

# ifdef TIMING
  dTimerStart("preprocessing");
# endif

  // number all bodies in the body list - set their tag values
  for (i=0; i<nb; i++) body[i]->tag = i;

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator.
  // @@@ check computation of rotational force.
  dReal *I = (dReal*) alloca (3*nb*4 * sizeof(dReal));
  dReal *invI = (dReal*) alloca (3*nb*4 * sizeof(dReal));
  dSetZero (I,3*nb*4);
  dSetZero (invI,3*nb*4);
  for (i=0; i<nb; i++) {
    dReal tmp[12];
    // compute inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->mass.I,body[i]->R);
    dMULTIPLY0_333 (I+i*12,body[i]->R,tmp);
    // compute inverse inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->invI,body[i]->R);
    dMULTIPLY0_333 (invI+i*12,body[i]->R,tmp);
    // compute rotational force
    dMULTIPLY0_331 (tmp,I+i*12,body[i]->avel);
    dCROSS (body[i]->tacc,-=,body[i]->avel,tmp);
  }

  // add the gravity force to all bodies
  for (i=0; i<nb; i++) {
    body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
    body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
    body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
  }

  // get m = total constraint dimension, nub = number of unbounded variables,
  // nlcp = number of LCP variables. create constraint offset arrays for
  // unbounded and LCP variables. note that the unbounded variables go at
  // the start.
  // @@@ TEMPORARY - the LCP-only constraints go at the end, to match the
  // constraint ordering of the fast solver.
  int nub = 0;
  int nlcp = 0;
  int *ofs1 = (int*) alloca (nj*sizeof(int));		// unbounded variables
  int *ofs2 = (int*) alloca (nj*sizeof(int));		// lcp variables
  for (i=0; i<nj; i++) {
    ofs1[i] = nub;
    ofs2[i] = nlcp;
    dxJoint::Info1 info;
    joint[i]->vtable->getInfo1 (joint[i],&info);
    dASSERT (info.nub >= 0 && info.nlcp >= 0 && (info.nub + info.nlcp) <= 6);
    nub += info.nub;
    nlcp += info.nlcp;
  }
  for (i=0; i<nj; i++) ofs2[i] += nub;
  int m = nub + nlcp;

  // create (6*nb,6*nb) inverse mass matrix `invM', and fill it with mass
  // parameters
# ifdef TIMING
  dTimerNow ("create mass matrix");
# endif
  int nskip = dPAD (n6);
  dReal *invM = (dReal*) alloca (n6*nskip*sizeof(dReal));
  dSetZero (invM,n6*nskip);
  for (i=0; i<nb; i++) {
    dReal *MM = invM+(i*6)*nskip+(i*6);
    MM[0] = body[i]->invMass;
    MM[nskip+1] = body[i]->invMass;
    MM[2*nskip+2] = body[i]->invMass;
    MM += 3*nskip+3;
    for (j=0; j<3; j++) for (k=0; k<3; k++) {
      MM[j*nskip+k] = invI[i*12+j*4+k];
    }
  }

  // assemble some body vectors: fe = external forces, v = velocities
  dReal *fe = (dReal*) alloca (n6 * sizeof(dReal));
  dReal *v = (dReal*) alloca (n6 * sizeof(dReal));
  dSetZero (fe,n6);
  dSetZero (v,n6);
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) fe[i*6+j] = body[i]->facc[j];
    for (j=0; j<3; j++) fe[i*6+3+j] = body[i]->tacc[j];
    for (j=0; j<3; j++) v[i*6+j] = body[i]->lvel[j];
    for (j=0; j<3; j++) v[i*6+3+j] = body[i]->avel[j];
  }

  // this will be set to the velocity update
  dReal *vnew = (dReal*) alloca (n6 * sizeof(dReal));
  dSetZero (vnew,n6);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c'
    dReal *c = (dReal*) alloca (m*sizeof(dReal));
    dSetZero (c,m);

    // create (m,6*nb) jacobian mass matrix `J', and fill it with constraint
    // data. also fill the c vector.
#   ifdef TIMING
    dTimerNow ("create J");
#   endif
    dReal *J = (dReal*) alloca (m*nskip*sizeof(dReal));
    dSetZero (J,m*nskip);
    dxJoint::Info2 Jinfo;
    Jinfo.rowskip = nskip;
    Jinfo.fps = dRecip(stepsize);
    Jinfo.erp = erp;
    for (i=0; i<nj; i++) {
      Jinfo.J1lu = J + nskip*ofs1[i] + 6*joint[i]->node[0].body->tag;
      Jinfo.J1ll = J + nskip*ofs2[i] + 6*joint[i]->node[0].body->tag;
      Jinfo.J1au = Jinfo.J1lu + 3;
      Jinfo.J1al = Jinfo.J1ll + 3;
      if (joint[i]->node[1].body) {
	Jinfo.J2lu = J + nskip*ofs1[i] + 6*joint[i]->node[1].body->tag;
	Jinfo.J2ll = J + nskip*ofs2[i] + 6*joint[i]->node[1].body->tag;
	Jinfo.J2au = Jinfo.J2lu + 3;
	Jinfo.J2al = Jinfo.J2lu + 3;
      }
      else {
	Jinfo.J2lu = 0;
	Jinfo.J2au = 0;
	Jinfo.J2ll = 0;
	Jinfo.J2al = 0;
      }
      Jinfo.cu = c + ofs1[i];
      Jinfo.cl = c + ofs2[i];
      joint[i]->vtable->getInfo2 (joint[i],&Jinfo);
    }

    // compute A = J*invM*J'
#   ifdef TIMING
    dTimerNow ("compute A");
#   endif
    dReal *JinvM = (dReal*) alloca (m*nskip*sizeof(dReal));
    dSetZero (JinvM,m*nskip);
    dMultiply0 (JinvM,J,invM,m,n6,n6);
    int mskip = dPAD(m);
    dReal *A = (dReal*) alloca (m*mskip*sizeof(dReal));
    dSetZero (A,m*mskip);
    dMultiply2 (A,JinvM,J,m,n6,m);

    // add epsilon to the diagonal of A
    for (i=0; i<m; i++) A[i*mskip+i] += epsilon;

#   ifdef COMPARE_METHODS
    slow_m = m;
    slow_A = copyMatrix (A,m,m);
#   endif

    // compute `rhs', the right hand side of the equation J*a=c
#   ifdef TIMING
    dTimerNow ("compute rhs");
#   endif
    dReal *tmp1 = (dReal*) alloca (n6 * sizeof(dReal));
    dSetZero (tmp1,n6);
    dMultiply0 (tmp1,invM,fe,n6,n6,1);
    for (i=0; i<n6; i++) tmp1[i] += v[i]/stepsize;
    dReal *rhs = (dReal*) alloca (m * sizeof(dReal));
    dSetZero (rhs,m);
    dMultiply0 (rhs,J,tmp1,m,n6,1);
    for (i=0; i<m; i++) rhs[i] = c[i]/stepsize - rhs[i];

#   ifdef COMPARE_METHODS
    slow_c = copyMatrix (c,m,1);
    slow_rhs = copyMatrix (rhs,m,1);
#   endif

    // solve the LCP problem and get lambda.
    // this will destroy A but that's okay
#   ifdef TIMING
    dTimerNow ("solving LCP problem");
#   endif
    dReal *lambda = (dReal*) alloca (m * sizeof(dReal));
    dReal *residual = (dReal*) alloca (m * sizeof(dReal));
    dSolveLCP (m,A,lambda,rhs,residual,nub,0,0,0);

//  OLD WAY - direct factor and solve
//
//    // factorize A (L*L'=A)
//#   ifdef TIMING
//    dTimerNow ("factorize A");
//#   endif
//    dReal *L = (dReal*) alloca (m*mskip*sizeof(dReal));
//    memcpy (L,A,m*mskip*sizeof(dReal));
//    if (dFactorCholesky (L,m)==0) dDebug (0,"A is not positive definite");
//
//    // compute lambda
//#   ifdef TIMING
//    dTimerNow ("compute lambda");
//#   endif
//    dReal *lambda = (dReal*) alloca (m * sizeof(dReal));
//    memcpy (lambda,rhs,m * sizeof(dReal));
//    dSolveCholesky (L,lambda,m);

#   ifdef COMPARE_METHODS
    slow_lambda = copyMatrix (lambda,m,1);
#   endif

    // compute the velocity update `vnew'
#   ifdef TIMING
    dTimerNow ("compute velocity update");
#   endif
    dMultiply1 (tmp1,J,lambda,n6,m,1);
    for (i=0; i<n6; i++) tmp1[i] += fe[i];
    dMultiply0 (vnew,invM,tmp1,n6,n6,1);
    for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];

    // see if the constraint has worked: compute J*vnew and make sure it equals
    // `c' (to within a certain tolerance).
#   ifdef TIMING
    dTimerNow ("verify constraint equation");
#   endif
    dMultiply0 (tmp1,J,vnew,m,n6,1);
    dReal err = 0;
    for (i=0; i<m; i++) err += dFabs(tmp1[i]-c[i]);
    printf ("%.6e\n",err);
  }
  else {
    // no constraints
    dMultiply0 (vnew,invM,fe,n6,n6,1);
    for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];
  }

# ifdef COMPARE_METHODS
  slow_vnew = copyMatrix (vnew,n6,1);
# endif

  // apply the velocity update to the bodies
# ifdef TIMING
  dTimerNow ("update velocity");
# endif
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) body[i]->lvel[j] = vnew[i*6+j];
    for (j=0; j<3; j++) body[i]->avel[j] = vnew[i*6+3+j];
  }

  // update the position and orientation from the new linear/angular velocity
  // (over the given timestep)
# ifdef TIMING
  dTimerNow ("update position");
# endif
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) body[i]->pos[j] += stepsize * body[i]->lvel[j];
    dReal dq[4];
    dWtoDQ (body[i]->avel,body[i]->q,dq);
    for (j=0; j<4; j++) body[i]->q[j] += stepsize * dq[j];
  }

# ifdef TIMING
  dTimerNow ("tidy up");
# endif

  // normalize all quaternions and convert them to rotation matrices
  for (i=0; i<nb; i++) {
    dNormalize4 (body[i]->q);
    dQtoR (body[i]->q,body[i]->R);
  }

  // zero all force accumulators
  for (i=0; i<nb; i++) {
    body[i]->facc[0] = 0;
    body[i]->facc[1] = 0;
    body[i]->facc[2] = 0;
    body[i]->facc[3] = 0;
    body[i]->tacc[0] = 0;
    body[i]->tacc[1] = 0;
    body[i]->tacc[2] = 0;
    body[i]->tacc[3] = 0;
  }

# ifdef TIMING
  dTimerEnd();
  if (m > 0) dTimerReport (stdout,1);
# endif
}

//****************************************************************************
// an optimized version of dInternalStepIsland1()

void dInternalStepIsland_x2 (dxWorld *world, dxBody **body, int nb,
			     dxJoint **joint, int nj, dReal stepsize)
{
  int i,j,k;
# ifdef TIMING
  dTimerStart("preprocessing");
# endif

  dReal stepsize1 = dRecip(stepsize);

  // number all bodies and joints in the body list - set their tag values
  for (i=0; i<nb; i++) body[i]->tag = i;
  for (i=0; i<nj; i++) joint[i]->tag = i;

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator. I and invI are vertically stacked 3x4 matrices, one per body.
  // @@@ check computation of rotational force.
  dReal *I = (dReal*) alloca (3*nb*4 * sizeof(dReal));
  dReal *invI = (dReal*) alloca (3*nb*4 * sizeof(dReal));
  dSetZero (I,3*nb*4);
  dSetZero (invI,3*nb*4);
  for (i=0; i<nb; i++) {
    dReal tmp[12];
    // compute inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->mass.I,body[i]->R);
    dMULTIPLY0_333 (I+i*12,body[i]->R,tmp);
    // compute inverse inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->invI,body[i]->R);
    dMULTIPLY0_333 (invI+i*12,body[i]->R,tmp);
    // compute rotational force
    dMULTIPLY0_331 (tmp,I+i*12,body[i]->avel);
    dCROSS (body[i]->tacc,-=,body[i]->avel,tmp);
  }

  // add the gravity force to all bodies
  for (i=0; i<nb; i++) {
    body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
    body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
    body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
  }

  // get m = total constraint dimension, nub = number of unbounded variables
  // at the start. create constraint offset array and number-of-rows array
  // for all joints.
  // @@@ TEMPORARY - the constraints are re-ordered according to if they are
  // unbounded or LCP. mixed unbounded-LCP constraints are not supported here
  // yet.
  int m = 0;
  int nub = 0;
  dxJoint::Info1 *info = (dxJoint::Info1*) alloca (nj*sizeof(dxJoint::Info1));
  int *ofs = (int*) alloca (nj*sizeof(int));
  int *nrows = (int*) alloca (nj*sizeof(int));
  for (i=0; i<nj; i++) {
    joint[i]->vtable->getInfo1 (joint[i],info+i);
    dASSERT (info[i].nub >= 0 && info[i].nlcp >= 0 &&
	      (info[i].nub + info[i].nlcp) <= 6);
    dASSERT (info[i].nub == 0 || info[i].nlcp == 0);	// temporary
  }
  for (i=0; i<nj; i++) {
    if (info[i].nub > 0) {
      ofs[i] = m;
      nrows[i] = info[i].nub;
      m += info[i].nub;
    }
  }
  for (i=0; i<nj; i++) {
    if (info[i].nlcp > 0) {
      ofs[i] = m;
      nrows[i] = info[i].nlcp;
      m += info[i].nlcp;
    }
  }
  for (i=0; i<nj; i++) nub += info[i].nub;

  // this will be set to the force due to the constraints
  dReal *cforce = (dReal*) alloca (nb*8 * sizeof(dReal));
  dSetZero (cforce,nb*8);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c'.
    dReal *c = (dReal*) alloca (m*sizeof(dReal));
    dSetZero (c,m);

    // get jacobian data from constraints. a (2*m)x8 matrix will be created
    // to store the two jacobian blocks from each constraint. it has this
    // format:
    //
    //   l l l 0 a a a 0  \ 
    //   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
    //   l l l 0 a a a 0  /
    //   l l l 0 a a a 0  \ 
    //   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
    //   l l l 0 a a a 0  /
    //   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
    //   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
    //   etc...
    //
    //   (lll) = linear jacobian data
    //   (aaa) = angular jacobian data
    //
#   ifdef TIMING
    dTimerNow ("create J");
#   endif
    dReal *J = (dReal*) alloca (2*m*8*sizeof(dReal));
    dSetZero (J,2*m*8);
    dxJoint::Info2 Jinfo;
    Jinfo.rowskip = 8;
    Jinfo.fps = stepsize1;
    Jinfo.erp = erp;
    for (i=0; i<nj; i++) {
      Jinfo.J1lu = J + 2*8*ofs[i];
      Jinfo.J1au = Jinfo.J1lu + 4;
      Jinfo.J2lu = Jinfo.J1lu + 8*nrows[i];
      Jinfo.J2au = Jinfo.J2lu + 4;
      Jinfo.J1ll = Jinfo.J1lu + 8*info[i].nub;
      Jinfo.J1al = Jinfo.J1au + 8*info[i].nub;
      Jinfo.J2ll = Jinfo.J2lu + 8*info[i].nub;
      Jinfo.J2al = Jinfo.J2au + 8*info[i].nub;
      Jinfo.cu = c + ofs[i];
      Jinfo.cl = c + ofs[i] + info[i].nub;
      joint[i]->vtable->getInfo2 (joint[i],&Jinfo);
    }

    // compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
    // format as J so we just go through the constraints in J multiplying by
    // the appropriate scalars and matrices.
#   ifdef TIMING
    dTimerNow ("compute A");
#   endif
    dReal *JinvM = (dReal*) alloca (2*m*8*sizeof(dReal));
    dSetZero (JinvM,2*m*8);
    for (i=0; i<nj; i++) {
      int b = joint[i]->node[0].body->tag;
      dReal body_invMass = body[b]->invMass;
      dReal *body_invI = invI + b*12;
      dReal *Jsrc = J + 2*8*ofs[i];
      dReal *Jdst = JinvM + 2*8*ofs[i];
      for (j=nrows[i]-1; j>=0; j--) {
	for (k=0; k<3; k++) Jdst[k] = Jsrc[k] * body_invMass;
	dMULTIPLY0_133 (Jdst+4,Jsrc+4,body_invI);
	Jsrc += 8;
	Jdst += 8;
      }
      if (joint[i]->node[1].body) {
	b = joint[i]->node[1].body->tag;
	body_invMass = body[b]->invMass;
	body_invI = invI + b*12;
	for (j=nrows[i]-1; j>=0; j--) {
	  for (k=0; k<3; k++) Jdst[k] = Jsrc[k] * body_invMass;
	  dMULTIPLY0_133 (Jdst+4,Jsrc+4,body_invI);
	  Jsrc += 8;
	  Jdst += 8;
	}
      }
    }

    // now compute A = JinvM * J'. A's rows and columns are grouped by joint,
    // i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
    // if joints i and j have at least one body in common. this fact suggests
    // the algorithm used to fill A:
    //
    //    for b = all bodies
    //      n = number of joints attached to body b
    //      for i = 1..n
    //        for j = i+1..n
    //          ii = actual joint number for i
    //          jj = actual joint number for j
    //          // (ii,jj) will be set to all pairs of joints around body b
    //          compute blockwise: A(ii,jj) += JinvM(ii) * J(jj)'
    //
    // this algorithm catches all pairs of joints that have at least one body
    // in common. it does not compute the diagonal blocks of A however -
    // another similar algorithm does that.

    int mskip = dPAD(m);
    dReal *A = (dReal*) alloca (m*mskip*sizeof(dReal));
    dSetZero (A,m*mskip);
    for (i=0; i<nb; i++) {
      for (dxJointNode *n1=body[i]->firstjoint; n1; n1=n1->next) {
	for (dxJointNode *n2=n1->next; n2; n2=n2->next) {
	  // get joint numbers and ensure ofs[j1] >= ofs[j2]
	  int j1 = n1->joint->tag;
	  int j2 = n2->joint->tag;
	  if (ofs[j1] < ofs[j2]) {
	    int tmp = j1;
	    j1 = j2;
	    j2 = tmp;
	  }

	  // determine if body i is the 1st or 2nd body of joints j1 and j2
	  int jb1 = (joint[j1]->node[1].body == body[i]);
	  int jb2 = (joint[j2]->node[1].body == body[i]);
	  // jb1/jb2 must be 0 for joints with only one body
	  dASSERT(joint[j1]->node[1].body || jb1==0);
	  dASSERT(joint[j2]->node[1].body || jb2==0);

	  // set block of A
	  Multiply2_p8r (A + ofs[j1]*mskip + ofs[j2],
			 JinvM + 2*8*ofs[j1] + jb1*8*nrows[j1],
			 J     + 2*8*ofs[j2] + jb2*8*nrows[j2],
			 nrows[j1],nrows[j2], mskip);
	}
      }
    }
    // compute diagonal blocks of A
    for (i=0; i<nj; i++) {
      Multiply2_p8r (A + ofs[i]*(mskip+1),
		     JinvM + 2*8*ofs[i],
		     J + 2*8*ofs[i],
		     nrows[i],nrows[i], mskip);
      if (joint[i]->node[1].body) {
	MultiplyAdd2_p8r (A + ofs[i]*(mskip+1),
			  JinvM + 2*8*ofs[i] + 8*nrows[i],
			  J + 2*8*ofs[i] + 8*nrows[i],
			  nrows[i],nrows[i], mskip);
      }
    }

    // add epsilon to the diagonal of A
    for (i=0; i<m; i++) A[i*mskip+i] += epsilon;

#   ifdef COMPARE_METHODS
    fast_A = copyMatrix (A,m,m);
#   endif

    // compute the right hand side `rhs'
#   ifdef TIMING
    dTimerNow ("compute rhs");
#   endif
    dReal *tmp1 = (dReal*) alloca (nb*8 * sizeof(dReal));
    dSetZero (tmp1,nb*8);
    // put v/h + invM*fe into tmp1
    for (i=0; i<nb; i++) {
      dReal body_invMass = body[i]->invMass;
      dReal *body_invI = invI + i*12;
      for (j=0; j<3; j++) tmp1[i*8+j] = body[i]->facc[j] * body_invMass +
			    body[i]->lvel[j] * stepsize1;
      dMULTIPLY0_331 (tmp1 + i*8 + 4,body_invI,body[i]->tacc);
      for (j=0; j<3; j++) tmp1[i*8+4+j] += body[i]->avel[j] * stepsize1;
    }
    // put J*tmp1 into rhs
    dReal *rhs = (dReal*) alloca (m * sizeof(dReal));
    dSetZero (rhs,m);
    for (i=0; i<nj; i++) {
      dReal *JJ = J + 2*8*ofs[i];
      Multiply0_p81 (rhs+ofs[i],JJ,
		     tmp1 + 8*joint[i]->node[0].body->tag, nrows[i]);
      if (joint[i]->node[1].body) {
	MultiplyAdd0_p81 (rhs+ofs[i],JJ + 8*nrows[i],
			  tmp1 + 8*joint[i]->node[1].body->tag, nrows[i]);
      }
    }
    // complete rhs
    for (i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];

#   ifdef COMPARE_METHODS
    fast_c = copyMatrix (c,m,1);
    fast_rhs = copyMatrix (rhs,m,1);
#   endif

    // solve the LCP problem and get lambda.
    // this will destroy A but that's okay
#   ifdef TIMING
    dTimerNow ("solving LCP problem");
#   endif
    dReal *lambda = (dReal*) alloca (m * sizeof(dReal));
    dReal *residual = (dReal*) alloca (m * sizeof(dReal));
    dSolveLCP (m,A,lambda,rhs,residual,nub,0,0,0);

//  OLD WAY - direct factor and solve
//
//    // factorize A (L*L'=A)
//#   ifdef TIMING
//    dTimerNow ("factorize A");
//#   endif
//    dReal *L = (dReal*) alloca (m*mskip*sizeof(dReal));
//    memcpy (L,A,m*mskip*sizeof(dReal));
//#   ifdef FAST_FACTOR
//    dFastFactorCholesky (L,m);  // does not report non positive definiteness
//#   else
//    if (dFactorCholesky (L,m)==0) dDebug (0,"A is not positive definite");
//#   endif
//
//    // compute lambda
//#   ifdef TIMING
//    dTimerNow ("compute lambda");
//#   endif
//    dReal *lambda = (dReal*) alloca (m * sizeof(dReal));
//    memcpy (lambda,rhs,m * sizeof(dReal));
//    dSolveCholesky (L,lambda,m);

#   ifdef COMPARE_METHODS
    fast_lambda = copyMatrix (lambda,m,1);
#   endif

    // compute the constraint force `cforce'
#   ifdef TIMING
    dTimerNow ("compute constraint force");
#   endif
    // compute cforce = J'*lambda
    for (i=0; i<nj; i++) {
      dReal *JJ = J + 2*8*ofs[i];
      MultiplyAdd1_8q1 (cforce + 8*joint[i]->node[0].body->tag,JJ,
			lambda+ofs[i], nrows[i]);
      if (joint[i]->node[1].body) {
	MultiplyAdd1_8q1 (cforce + 8*joint[i]->node[1].body->tag,
			  JJ + 8*nrows[i], lambda+ofs[i], nrows[i]);
      }
    }
  }

  // compute the velocity update
# ifdef TIMING
  dTimerNow ("compute velocity update");
# endif

  // add fe to cforce
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) cforce[i*8+j] += body[i]->facc[j];
    for (j=0; j<3; j++) cforce[i*8+4+j] += body[i]->tacc[j];
  }
  // multiply cforce by stepsize
  for (i=0; i < nb*8; i++) cforce[i] *= stepsize;
  // add invM * cforce to the body velocity
  for (i=0; i<nb; i++) {
    dReal body_invMass = body[i]->invMass;
    dReal *body_invI = invI + i*12;
    for (j=0; j<3; j++) body[i]->lvel[j] += body_invMass * cforce[i*8+j];
    dMULTIPLYADD0_331 (body[i]->avel,body_invI,cforce+i*8+4);
  }

  // update the position and orientation from the new linear/angular velocity
  // (over the given timestep)
# ifdef TIMING
  dTimerNow ("update position");
# endif
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) body[i]->pos[j] += stepsize * body[i]->lvel[j];
    dReal dq[4];
    dWtoDQ (body[i]->avel,body[i]->q,dq);
    for (j=0; j<4; j++) body[i]->q[j] += stepsize * dq[j];
  }

# ifdef COMPARE_METHODS
  dReal *tmp_vnew = (dReal*) alloca (nb*6*sizeof(dReal));
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) tmp_vnew[i*6+j] = body[i]->lvel[j];
    for (j=0; j<3; j++) tmp_vnew[i*6+3+j] = body[i]->avel[j];
  }
  fast_vnew = copyMatrix (tmp_vnew,nb*6,1);
# endif

# ifdef TIMING
  dTimerNow ("tidy up");
# endif

  // normalize all quaternions and convert them to rotation matrices
  for (i=0; i<nb; i++) {
    dNormalize4 (body[i]->q);
    dQtoR (body[i]->q,body[i]->R);
  }

  // zero all force accumulators
  for (i=0; i<nb; i++) {
    body[i]->facc[0] = 0;
    body[i]->facc[1] = 0;
    body[i]->facc[2] = 0;
    body[i]->facc[3] = 0;
    body[i]->tacc[0] = 0;
    body[i]->tacc[1] = 0;
    body[i]->tacc[2] = 0;
    body[i]->tacc[3] = 0;
  }

# ifdef TIMING
  dTimerEnd();
  if (m > 0) dTimerReport (stdout,1);
# endif
}

//****************************************************************************

void dInternalStepIsland (dxWorld *world, dxBody **body, int nb,
			  dxJoint **joint, int nj, dReal stepsize)
{
# ifndef COMPARE_METHODS
  dInternalStepIsland_x2 (world,body,nb,joint,nj,stepsize);
# endif

# ifdef COMPARE_METHODS
  int i;

  // save body state
  dxBody *state = (dxBody*) alloca (nb*sizeof(dxBody));
  for (i=0; i<nb; i++) memcpy (state+i,body[i],sizeof(dxBody));

  // take slow step
  dInternalStepIsland_x1 (world,body,nb,joint,nj,stepsize);

  // restore state
  for (i=0; i<nb; i++) memcpy (body[i],state+i,sizeof(dxBody));

  // take fast step
  dInternalStepIsland_x2 (world,body,nb,joint,nj,stepsize);

  // see if there's a difference
  int m = slow_m;
  compareMatrixLT ("A     ",slow_A,fast_A,m);
  compareMatrix ("c     ",slow_c,fast_c,m,1);
  compareMatrix ("error ",slow_error,fast_error,m,1);
  compareMatrix ("rhs   ",slow_rhs,fast_rhs,m,1);
  compareMatrix ("lambda",slow_lambda,fast_lambda,m,1);
  compareMatrix ("vnew  ",slow_vnew,fast_vnew,nb*6,1);

  // delete comparison matrices
  deleteMatrix (slow_A,m,m);
  deleteMatrix (slow_c,m,1);
  deleteMatrix (slow_error,m,1);
  deleteMatrix (slow_rhs,m,1);
  deleteMatrix (slow_lambda,m,1);
  deleteMatrix (slow_vnew,nb*6,1);
  deleteMatrix (fast_A,m,m);
  deleteMatrix (fast_c,m,1);
  deleteMatrix (fast_error,m,1);
  deleteMatrix (fast_rhs,m,1);
  deleteMatrix (fast_lambda,m,1);
  deleteMatrix (fast_vnew,nb*6,1);

  //_exit (1);
# endif
}
