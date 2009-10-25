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

#include "objects.h"
#include "joints/joint.h"
#include <ode/odeconfig.h>
#include "config.h"
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include "lcp.h"
#include "util.h"

//****************************************************************************
// misc defines

//#define TIMING


//****************************************************************************
// debugging - comparison of various vectors and matrices produced by the
// slow and fast versions of the stepper.

//#define COMPARE_METHODS

#ifdef COMPARE_METHODS
#include "testing.h"
static dMatrixComparison comparator;
#endif

// undef to use the fast decomposition
#define DIRECT_CHOLESKY
#undef REPORT_ERROR

#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif

//****************************************************************************
// special matrix multipliers

// this assumes the 4th and 8th rows of B and C are zero.

static void Multiply2_p8r (dReal *A, const dReal *B, const dReal *C,
                           int p, int r, int Askip)
{
  dIASSERT (p>0 && r>0 && A && B && C);
  const int Askip_munus_r = Askip - r;
  dReal *aa = A;
  const dReal *bb = B;
  int i;
  for (i=p; i; --i) {
    const dReal *cc = C;
    int j;
    for (j=r; j; --j) {
      dReal sum;
      sum  = bb[0]*cc[0];
      sum += bb[1]*cc[1];
      sum += bb[2]*cc[2];
      sum += bb[4]*cc[4];
      sum += bb[5]*cc[5];
      sum += bb[6]*cc[6];
      *(aa++) = sum; 
      cc += 8;
    }
    bb += 8;
    aa += Askip_munus_r;
  }
}


// this assumes the 4th and 8th rows of B and C are zero.

static void MultiplyAdd2_p8r (dReal *A, const dReal *B, const dReal *C,
                              int p, int r, int Askip)
{
  dIASSERT (p>0 && r>0 && A && B && C);
  const int Askip_munus_r = Askip - r;
  dReal *aa = A;
  const dReal *bb = B;
  int i;
  for (i=p; i; --i) {
    const dReal *cc = C;
    int j;
    for (j=r; j; --j) {
      dReal sum;
      sum  = bb[0]*cc[0];
      sum += bb[1]*cc[1];
      sum += bb[2]*cc[2];
      sum += bb[4]*cc[4];
      sum += bb[5]*cc[5];
      sum += bb[6]*cc[6];
      *(aa++) += sum; 
      cc += 8;
    }
    bb += 8;
    aa += Askip_munus_r;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplySub0_p81 (dReal *A, const dReal *B, const dReal *C, int p)
{
  dIASSERT (p>0 && A && B && C);
  dReal *aa = A;
  const dReal *bb = B;
  int i;
  for (i=p; i; --i) {
    dReal sum;
    sum  = bb[0]*C[0];
    sum += bb[1]*C[1];
    sum += bb[2]*C[2];
    sum += bb[4]*C[4];
    sum += bb[5]*C[5];
    sum += bb[6]*C[6];
    *(aa++) -= sum;
    bb += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplyAdd1_8q1 (dReal *A, const dReal *B, const dReal *C, int q)
{
  dIASSERT (q>0 && A && B && C);
  const dReal *bb = B;
  dReal sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
  int k;
  for (k=0; k<q; ++k) {
    const dReal C_k = C[k];
    sum0 += bb[0] * C_k;
    sum1 += bb[1] * C_k;
    sum2 += bb[2] * C_k;
    sum4 += bb[4] * C_k;
    sum5 += bb[5] * C_k;
    sum6 += bb[6] * C_k;
    bb += 8;
  }
  A[0] += sum0;
  A[1] += sum1;
  A[2] += sum2;
  A[4] += sum4;
  A[5] += sum5;
  A[6] += sum6;
}


// this assumes the 4th and 8th rows of B are zero.

static void Multiply1_8q1 (dReal *A, const dReal *B, const dReal *C, int q)
{
  const dReal *bb = B;
  dReal sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
  int k;
  for (k=0; k<q; ++k) {
    const dReal C_k = C[k];
    sum0 += bb[0] * C_k;
    sum1 += bb[1] * C_k;
    sum2 += bb[2] * C_k;
    sum4 += bb[4] * C_k;
    sum5 += bb[5] * C_k;
    sum6 += bb[6] * C_k;
    bb += 8;
  }
  A[0] = sum0;
  A[1] = sum1;
  A[2] = sum2;
  A[4] = sum4;
  A[5] = sum5;
  A[6] = sum6;
}
/*
//****************************************************************************
// the slow, but sure way
// note that this does not do any joint feedback!

// given lists of bodies and joints that form an island, perform a first
// order timestep.
//
// `body' is the body array, `nb' is the size of the array.
// `_joint' is the body array, `nj' is the size of the array.

void dInternalStepIsland_x1 (dxWorld *world, dxBody * const *body, int nb,
                             dxJoint * const *_joint, int nj, dReal stepsize)
{
  int i,j,k;
  int n6 = 6*nb;

  IFTIMING(dTimerStart("preprocessing"));

  // number all bodies in the body list - set their tag values
  for (i=0; i<nb; i++) body[i]->tag = i;

  // make a local copy of the joint array, because we might want to modify it.
  // (the "dxJoint *const*" declaration says we're allowed to modify the joints
  // but not the joint array, because the caller might need it unchanged).
  ALLOCA(dxJoint*,joint,nj*sizeof(dxJoint*));
  memcpy (joint,_joint,nj * sizeof(dxJoint*));

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator.
  // @@@ check computation of rotational force.
  ALLOCA(dReal,I,3*nb*4*sizeof(dReal));
  ALLOCA(dReal,invI,3*nb*4*sizeof(dReal));

  //dSetZero (I,3*nb*4);
  //dSetZero (invI,3*nb*4);
  for (i=0; i<nb; i++) {
    dReal tmp[12];
    // compute inertia tensor in global frame
    dMultiply2_333 (tmp,body[i]->mass.I,body[i]->posr.R);
    dMultiply0_333 (I+i*12,body[i]->posr.R,tmp);
    // compute inverse inertia tensor in global frame
    dMultiply2_333 (tmp,body[i]->invI,body[i]->posr.R);
    dMultiply0_333 (invI+i*12,body[i]->posr.R,tmp);
    // compute rotational force
    dMultiply0_331 (tmp,I+i*12,body[i]->avel);
    dSubtractVectorCross3(body[i]->tacc,body[i]->avel,tmp);
  }

  // add the gravity force to all bodies
  for (i=0; i<nb; i++) {
    if ((body[i]->flags & dxBodyNoGravity)==0) {
      body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
      body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
      body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
    }
  }

  // get m = total constraint dimension, nub = number of unbounded variables.
  // create constraint offset array and number-of-rows array for all joints.
  // the constraints are re-ordered as follows: the purely unbounded
  // constraints, the mixed unbounded + LCP constraints, and last the purely
  // LCP constraints.
  //
  // joints with m=0 are inactive and are removed from the joints array
  // entirely, so that the code that follows does not consider them.
  int m = 0;
  ALLOCA(dxJoint::Info1,info,nj*sizeof(dxJoint::Info1));
  ALLOCA(int,ofs,nj*sizeof(int));

  for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
    joint[j]->getInfo1 (info+i);
    dIASSERT (info[i].m >= 0 && info[i].m <= 6 &&
      info[i].nub >= 0 && info[i].nub <= info[i].m);
    if (info[i].m > 0) {
      joint[i] = joint[j];
      i++;
    }
  }
  nj = i;

  // the purely unbounded constraints
  for (i=0; i<nj; i++) if (info[i].nub == info[i].m) {
    ofs[i] = m;
    m += info[i].m;
  }
  //int nub = m;
  // the mixed unbounded + LCP constraints
  for (i=0; i<nj; i++) if (info[i].nub > 0 && info[i].nub < info[i].m) {
    ofs[i] = m;
    m += info[i].m;
  }
  // the purely LCP constraints
  for (i=0; i<nj; i++) if (info[i].nub == 0) {
    ofs[i] = m;
    m += info[i].m;
  }

  // create (6*nb,6*nb) inverse mass matrix `invM', and fill it with mass
  // parameters
  IFTIMING(dTimerNow ("create mass matrix"));

  int nskip = dPAD (n6);
  ALLOCA(dReal, invM, n6*nskip*sizeof(dReal));

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
  ALLOCA(dReal,fe,n6*sizeof(dReal));
  ALLOCA(dReal,v,n6*sizeof(dReal));

  //dSetZero (fe,n6);
  //dSetZero (v,n6);
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) fe[i*6+j] = body[i]->facc[j];
    for (j=0; j<3; j++) fe[i*6+3+j] = body[i]->tacc[j];
    for (j=0; j<3; j++) v[i*6+j] = body[i]->lvel[j];
    for (j=0; j<3; j++) v[i*6+3+j] = body[i]->avel[j];
  }

  // this will be set to the velocity update
  ALLOCA(dReal,vnew,n6*sizeof(dReal));
  dSetZero (vnew,n6);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c', a constraint
    // force mixing vector `cfm', and LCP low and high bound vectors, and an
    // 'findex' vector.
    ALLOCA(dReal,c,m*sizeof(dReal));
    ALLOCA(dReal,cfm,m*sizeof(dReal));
    ALLOCA(dReal,lo,m*sizeof(dReal));
    ALLOCA(dReal,hi,m*sizeof(dReal));
    ALLOCA(int,findex,m*sizeof(int));
    dSetZero (c,m);
    dSetValue (cfm,m,world->global_cfm);
    dSetValue (lo,m,-dInfinity);
    dSetValue (hi,m, dInfinity);
    for (i=0; i<m; i++) findex[i] = -1;

    // create (m,6*nb) jacobian mass matrix `J', and fill it with constraint
    // data. also fill the c vector.
    IFTIMING(dTimerNow ("create J"));
    ALLOCA(dReal,J,m*nskip*sizeof(dReal));
    dSetZero (J,m*nskip);
    dxJoint::Info2 Jinfo;
    Jinfo.rowskip = nskip;
    Jinfo.fps = dRecip(stepsize);
    Jinfo.erp = world->global_erp;
    for (i=0; i<nj; i++) {
      Jinfo.J1l = J + nskip*ofs[i] + 6*joint[i]->node[0].body->tag;
      Jinfo.J1a = Jinfo.J1l + 3;
      if (joint[i]->node[1].body) {
        Jinfo.J2l = J + nskip*ofs[i] + 6*joint[i]->node[1].body->tag;
        Jinfo.J2a = Jinfo.J2l + 3;
      }
      else {
        Jinfo.J2l = 0;
        Jinfo.J2a = 0;
      }
      Jinfo.c = c + ofs[i];
      Jinfo.cfm = cfm + ofs[i];
      Jinfo.lo = lo + ofs[i];
      Jinfo.hi = hi + ofs[i];
      Jinfo.findex = findex + ofs[i];
      joint[i]->getInfo2 (&Jinfo);
      // adjust returned findex values for global index numbering
      for (j=0; j<info[i].m; j++) {
        if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
      }
    }

    // compute A = J*invM*J'
    IFTIMING(dTimerNow ("compute A"));
    ALLOCA(dReal,JinvM,m*nskip*sizeof(dReal));
    //dSetZero (JinvM,m*nskip);
    dMultiply0 (JinvM,J,invM,m,n6,n6);
    int mskip = dPAD(m);
    ALLOCA(dReal,A,m*mskip*sizeof(dReal));
    //dSetZero (A,m*mskip);
    dMultiply2 (A,JinvM,J,m,n6,m);

    // add cfm to the diagonal of A
    for (i=0; i<m; i++) A[i*mskip+i] += cfm[i] * Jinfo.fps;

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (A,m,m,1,"A");
#   endif

    // compute `rhs', the right hand side of the equation J*a=c
    IFTIMING(dTimerNow ("compute rhs"));
    ALLOCA(dReal,tmp1,n6*sizeof(dReal));
    //dSetZero (tmp1,n6);
    dMultiply0 (tmp1,invM,fe,n6,n6,1);
    for (i=0; i<n6; i++) tmp1[i] += v[i]/stepsize;
    ALLOCA(dReal,rhs,m*sizeof(dReal));
    //dSetZero (rhs,m);
    dMultiply0 (rhs,J,tmp1,m,n6,1);
    for (i=0; i<m; i++) rhs[i] = c[i]/stepsize - rhs[i];

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (c,m,1,0,"c");
    comparator.nextMatrix (rhs,m,1,0,"rhs");
#   endif





#ifndef DIRECT_CHOLESKY
    // solve the LCP problem and get lambda.
    // this will destroy A but that's okay
    IFTIMING(dTimerNow ("solving LCP problem"));
    ALLOCA(dReal,lambda,m*sizeof(dReal));
    dSolveLCP (m,A,lambda,rhs,NULL,nub,lo,hi,findex);

#ifdef dUSE_MALLOC_FOR_ALLOCA
    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY)
      return;
#endif


#else

    // OLD WAY - direct factor and solve

    // factorize A (L*L'=A)
    IFTIMING(dTimerNow ("factorize A"));
    ALLOCA(dReal,L,m*mskip*sizeof(dReal));
    memcpy (L,A,m*mskip*sizeof(dReal));
    if (dFactorCholesky (L,m)==0) dDebug (0,"A is not positive definite");

    // compute lambda
    IFTIMING(dTimerNow ("compute lambda"));
    ALLOCA(dReal,lambda,m*sizeof(dReal));
    memcpy (lambda,rhs,m * sizeof(dReal));
    dSolveCholesky (L,lambda,m);
#endif

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (lambda,m,1,0,"lambda");
#   endif

    // compute the velocity update `vnew'
    IFTIMING(dTimerNow ("compute velocity update"));
    dMultiply1 (tmp1,J,lambda,n6,m,1);
    for (i=0; i<n6; i++) tmp1[i] += fe[i];
    dMultiply0 (vnew,invM,tmp1,n6,n6,1);
    for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];

#ifdef REPORT_ERROR
    // see if the constraint has worked: compute J*vnew and make sure it equals
    // `c' (to within a certain tolerance).
    IFTIMING(dTimerNow ("verify constraint equation"));
    dMultiply0 (tmp1,J,vnew,m,n6,1);
    dReal err = 0;
    for (i=0; i<m; i++) {
      err += dFabs(tmp1[i]-c[i]);
    }
    printf ("total constraint error=%.6e\n",err);
#endif

  }
  else {
    // no constraints
    dMultiply0 (vnew,invM,fe,n6,n6,1);
    for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];
  }

#ifdef COMPARE_METHODS
  comparator.nextMatrix (vnew,n6,1,0,"vnew");
#endif

  // apply the velocity update to the bodies
  IFTIMING(dTimerNow ("update velocity"));
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) body[i]->lvel[j] = vnew[i*6+j];
    for (j=0; j<3; j++) body[i]->avel[j] = vnew[i*6+3+j];
  }

  // update the position and orientation from the new linear/angular velocity
  // (over the given timestep)
  IFTIMING(dTimerNow ("update position"));
  for (i=0; i<nb; i++) dxStepBody (body[i],stepsize);

  IFTIMING(dTimerNow ("tidy up"));

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

  IFTIMING(dTimerEnd());
  if (m > 0) IFTIMING(dTimerReport (stdout,1));

}
*/


//****************************************************************************
// an optimized version of dInternalStepIsland1()

struct dJointWithInfo1
{
  dxJoint *joint;
  dxJoint::Info1 info;
};

void dInternalStepIsland_x2 (dxWorldProcessContext *context, 
                             dxWorld *world, dxBody * const *body, int nb,
                             dxJoint * const *_joint, int _nj, dReal stepsize)
{
  IFTIMING(dTimerStart("preprocessing"));

  const dReal stepsizeRecip = dRecip(stepsize);

  {
    // number all bodies in the body list - set their tag values
    int i;
    for (i=0; i<nb; ++i) body[i]->tag = i;
  }

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator. invI are vertically stacked 3x4 matrices, one per body.
  // @@@ check computation of rotational force.

  dReal *invI = context->AllocateArray<dReal> (3*4*nb);

  { // Identical to QuickStep
    dReal *invIrow = invI;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow += 12, ++bodycurr) {
      dMatrix3 tmp;
      dxBody *b = *bodycurr;

      // compute inverse inertia tensor in global frame
      dMultiply2_333 (tmp,b->invI,b->posr.R);
      dMultiply0_333 (invIrow,b->posr.R,tmp);

      if (b->flags & dxBodyGyroscopic) {
        dMatrix3 I;
        // compute inertia tensor in global frame
        dMultiply2_333 (tmp,b->mass.I,b->posr.R);
        dMultiply0_333 (I,b->posr.R,tmp);
        // compute rotational force
        dMultiply0_331 (tmp,I,b->avel);
        dSubtractVectorCross3 (b->tacc,b->avel,tmp);
      }
    }
  }

  { // Identical to QuickStep
    // add the gravity force to all bodies
    // since gravity does normally have only one component it's more efficient
    // to run three loops for each individual component
    dxBody *const *const bodyend = body + nb;
    dReal gravity_x = world->gravity[0];
    if (gravity_x) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
        dxBody *b = *bodycurr;
        if ((b->flags & dxBodyNoGravity)==0) {
          b->facc[0] += b->mass.mass * gravity_x;
        }
      }
    }
    dReal gravity_y = world->gravity[1];
    if (gravity_y) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
        dxBody *b = *bodycurr;
        if ((b->flags & dxBodyNoGravity)==0) {
          b->facc[1] += b->mass.mass * gravity_y;
        }
      }
    }
    dReal gravity_z = world->gravity[2];
    if (gravity_z) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
        dxBody *b = *bodycurr;
        if ((b->flags & dxBodyNoGravity)==0) {
          b->facc[2] += b->mass.mass * gravity_z;
        }
      }
    }
  }

  // get m = total constraint dimension, nub = number of unbounded variables.
  // create constraint offset array and number-of-rows array for all joints.
  // the constraints are re-ordered as follows: the purely unbounded
  // constraints, the mixed unbounded + LCP constraints, and last the purely
  // LCP constraints. this assists the LCP solver to put all unbounded
  // variables at the start for a quick factorization.
  //
  // joints with m=0 are inactive and are removed from the joints array
  // entirely, so that the code that follows does not consider them.
  // also number all active joints in the joint list (set their tag values).
  // inactive joints receive a tag value of -1.

  // Reserve twice as much memory and start from the middle so that regardless of 
  // what direction the array grows to there would be sufficient room available.
  const size_t ji_reserve_count = 2 * _nj;
  dJointWithInfo1 *jointiinfos = context->AllocateArray<dJointWithInfo1> (ji_reserve_count);
  int nub, ji_start, ji_end;

  {
    int unb_start, mix_start, mix_end, lcp_end;
    unb_start = mix_start = mix_end = lcp_end = _nj;

    dJointWithInfo1 *jicurr = jointiinfos + lcp_end;
    dxJoint *const *const _jend = _joint + _nj;
    dxJoint *const *_jcurr = _joint;
    while (true) {
      // -------------------------------------------------------------------------
      // Switch to growing array forward
      {
        bool fwd_end_reached = false;
        dJointWithInfo1 *jimixend = jointiinfos + mix_end;
        while (true) {	// jicurr=dest, _jcurr=src
          if (_jcurr == _jend) {
            lcp_end = jicurr - jointiinfos;
            fwd_end_reached = true;
            break;
          }
          dxJoint *j = *_jcurr++;
          j->getInfo1 (&jicurr->info);
          dIASSERT (jicurr->info.m >= 0 && jicurr->info.m <= 6 && jicurr->info.nub >= 0 && jicurr->info.nub <= jicurr->info.m);
          if (jicurr->info.m > 0) {
            if (jicurr->info.nub == 0) { // A lcp info - a correct guess!!!
              jicurr->joint = j;
              ++jicurr;
            } else if (jicurr->info.nub < jicurr->info.m) { // A mixed case
              if (unb_start == mix_start) { // no unbounded infos yet - just move to opposite side of mixed-s
                unb_start = mix_start = mix_start - 1;
                dJointWithInfo1 *jimixstart = jointiinfos + mix_start;
                jimixstart->info = jicurr->info;
                jimixstart->joint = j;
              } else if (jimixend != jicurr) { // have to swap to the tail of mixed-s
                dxJoint::Info1 tmp_info = jicurr->info;
                *jicurr = *jimixend;
                jimixend->info = tmp_info;
                jimixend->joint = j;
                ++jimixend; ++jicurr;
              } else { // no need to swap as there are no LCP info-s yet
                jicurr->joint = j;
                jimixend = jicurr = jicurr + 1;
              }
            } else { // A purely unbounded case -- break out and proceed growing in opposite direction
              unb_start = unb_start - 1;
              dJointWithInfo1 *jiunbstart = jointiinfos + unb_start;
              jiunbstart->info = jicurr->info;
              jiunbstart->joint = j;
              lcp_end = jicurr - jointiinfos;
              mix_end = jimixend - jointiinfos;
              jicurr = jiunbstart - 1;
              break;
            }
          } else {
            j->tag = -1;
          }
        }
        if (fwd_end_reached) {
          break;
        }
      }
      // -------------------------------------------------------------------------
      // Switch to growing array backward
      {
        bool bkw_end_reached = false;
        dJointWithInfo1 *jimixstart = jointiinfos + mix_start - 1;
        while (true) {	// jicurr=dest, _jcurr=src
          if (_jcurr == _jend) {
            unb_start = (jicurr + 1) - jointiinfos;
            mix_start = (jimixstart + 1) - jointiinfos;
            bkw_end_reached = true;
            break;
          }
          dxJoint *j = *_jcurr++;
          j->getInfo1 (&jicurr->info);
          dIASSERT (jicurr->info.m >= 0 && jicurr->info.m <= 6 && jicurr->info.nub >= 0 && jicurr->info.nub <= jicurr->info.m);
          if (jicurr->info.m > 0) {
            if (jicurr->info.nub == jicurr->info.m) { // An unbounded info - a correct guess!!!
              jicurr->joint = j;
              --jicurr;
            } else if (jicurr->info.nub > 0) { // A mixed case
              if (mix_end = lcp_end) { // no lcp infos yet - just move to opposite side of mixed-s
                dJointWithInfo1 *jimixend = jointiinfos + mix_end;
                lcp_end = mix_end = mix_end + 1;
                jimixend->info = jicurr->info;
                jimixend->joint = j;
              } else if (jimixstart != jicurr) { // have to swap to the head of mixed-s
                dxJoint::Info1 tmp_info = jicurr->info;
                *jicurr = *jimixstart;
                jimixstart->info = tmp_info;
                jimixstart->joint = j;
                --jimixstart; --jicurr;
              } else { // no need to swap as there are no unbounded info-s yet
                jicurr->joint = j;
                jimixstart = jicurr = jicurr - 1;
              }
            } else { // A purely lcp case -- break out and proceed growing in opposite direction
              dJointWithInfo1 *jilcpend = jointiinfos + lcp_end;
              lcp_end = lcp_end + 1;
              jilcpend->info = jicurr->info;
              jilcpend->joint = j;
              unb_start = (jicurr + 1) - jointiinfos;
              mix_start = (jimixstart + 1) - jointiinfos;
              jicurr = jilcpend + 1;
              break;
            }
          } else {
            j->tag = -1;
          }
        }
        if (bkw_end_reached) {
          break;
        }
      }
    }
    
    nub = mix_start - unb_start;
    ji_start = unb_start;
    ji_end = lcp_end;
  }

  context->ShrinkArray<dJointWithInfo1>(jointiinfos, ji_reserve_count, ji_end);
  jointiinfos += ji_start;
  int nj = ji_end - ji_start;

  int m = 0;

  {
    int mcurr = 0;
    const dJointWithInfo1 *jicurr = jointiinfos;
    const dJointWithInfo1 *const jiend = jicurr + nj;
    for (int i = 0; jicurr != jiend; i++, ++jicurr) {
      jicurr->joint->tag = i;
      int jm = jicurr->info.m;
      mcurr += jm;
    }

    m = mcurr;
  }

  // this will be set to the force due to the constraints
  dReal *cforce = context->AllocateArray<dReal> (nb*8);
  dSetZero (cforce,nb*8);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c', a constraint
    // force mixing vector `cfm', and LCP low and high bound vectors, and an
    // 'findex' vector.
    dReal *lo, *hi, *J, *A, *rhs;
    int *findex;

    {
      int mlocal = m;

      lo = context->AllocateArray<dReal> (mlocal);
      dSetValue (lo,mlocal,-dInfinity);

      hi = context->AllocateArray<dReal> (mlocal);
      dSetValue (hi,mlocal, dInfinity);

      J = context->AllocateArray<dReal> (2*8*mlocal);
      dSetZero (J,2*8*mlocal);

      findex = context->AllocateArray<int> (mlocal);
      for (int i=0; i<mlocal; ++i) findex[i] = -1;

      int mskip = dPAD(mlocal);
      A = context->AllocateArray<dReal> (mlocal*mskip);
      dSetZero (A,mlocal*mskip);

      rhs = context->AllocateArray<dReal> (mlocal);
      dSetZero (rhs,mlocal);
    }

    // Put 'c' in the same memory as 'rhs' as they transit into each other
    dReal *c = rhs; rhs = NULL; // erase rhs pointer for now as it is not to be used yet

    BEGIN_STATE_SAVE(context, cfmstate) {
      dReal *cfm = context->AllocateArray<dReal> (m);
      dSetValue (cfm,m,world->global_cfm);

      dReal *JinvM = context->AllocateArray<dReal> (2*8*m);
      dSetZero (JinvM,2*8*m);

      {
        IFTIMING(dTimerNow ("create J"));
        // get jacobian data from constraints. a (2*m)x8 matrix will be created
        // to store the two jacobian blocks from each constraint. it has this
        // format:
        //
        //   l l l 0 a a a 0  \    .
        //   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
        //   l l l 0 a a a 0  /
        //   l l l 0 a a a 0  \    .
        //   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
        //   l l l 0 a a a 0  /
        //   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
        //   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
        //   etc...
        //
        //   (lll) = linear jacobian data
        //   (aaa) = angular jacobian data
        //

        dxJoint::Info2 Jinfo;
        Jinfo.rowskip = 8;
        Jinfo.fps = stepsizeRecip;
        Jinfo.erp = world->global_erp;

        unsigned ofsi = 0;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; ++jicurr) {
          const int infom = jicurr->info.m;
          dReal *const J1row = J + 2*8*ofsi;
          Jinfo.J1l = J1row;
          Jinfo.J1a = J1row + 4;
          dReal *const J2row = J1row + 8*infom;
          Jinfo.J2l = J2row;
          Jinfo.J2a = J2row + 4;
          Jinfo.c = c + ofsi;
          Jinfo.cfm = cfm + ofsi;
          Jinfo.lo = lo + ofsi;
          Jinfo.hi = hi + ofsi;
          Jinfo.findex = findex + ofsi;
          
          dxJoint *joint = jicurr->joint;
          joint->getInfo2 (&Jinfo);
          
          // adjust returned findex values for global index numbering
          int *findex_ofsi = findex + ofsi;
          for (int j=0; j<infom; ++j) {
            int fival = findex_ofsi[j];
            if (fival >= 0) 
              findex_ofsi[j] = fival + ofsi;
          }

          ofsi += infom;
        }
      }

      {
        IFTIMING(dTimerNow ("compute A"));
        {
          // compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
          // format as J so we just go through the constraints in J multiplying by
          // the appropriate scalars and matrices.
          unsigned ofsi = 0;
          const dJointWithInfo1 *jicurr = jointiinfos;
          const dJointWithInfo1 *const jiend = jicurr + nj;
          for (; jicurr != jiend; ++jicurr) {
            const int infom = jicurr->info.m;
            dxJoint *joint = jicurr->joint;
            int b0 = joint->node[0].body->tag;
            dReal body_invMass0 = body[b0]->invMass;
            dReal *body_invI0 = invI + b0*12;
            dReal *Jsrc = J + 2*8*ofsi;
            dReal *Jdst = JinvM + 2*8*ofsi;
            for (int j=infom-1; j>=0; --j) {
              for (int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass0;
              dMultiply0_133 (Jdst+4,Jsrc+4,body_invI0);
              Jsrc += 8;
              Jdst += 8;
            }

            if (joint->node[1].body) {
              int b1 = joint->node[1].body->tag;
              dReal body_invMass1 = body[b1]->invMass;
              dReal *body_invI1 = invI + b1*12;
              for (int j=infom-1; j>=0; --j) {
                for (int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass1;
                dMultiply0_133 (Jdst+4,Jsrc+4,body_invI1);
                Jsrc += 8;
                Jdst += 8;
              }
            }

            ofsi += infom;
          }
        }

        {
          // now compute A = JinvM * J'. A's rows and columns are grouped by joint,
          // i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
          // if joints i and j have at least one body in common. 

          BEGIN_STATE_SAVE(context, ofsstate) {
            int *ofs = context->AllocateArray<int> (m);
            const int mskip = dPAD(m);

            unsigned ofsi = 0;
            const dJointWithInfo1 *jicurr = jointiinfos;
            const dJointWithInfo1 *const jiend = jicurr + nj;
            for (int i = 0; jicurr != jiend; i++, ++jicurr) {
              const int infom = jicurr->info.m;
              dxJoint *joint = jicurr->joint;

              dReal *Arow = A + mskip*ofsi;
              dReal *JinvMrow = JinvM + 2*8*ofsi;

              dxBody *jb0 = joint->node[0].body;
              for (dxJointNode *n0=jb0->firstjoint; n0; n0=n0->next) {
                // if joint was tagged as -1 then it is an inactive (m=0)
                // joint that should not be considered
                int j0 = n0->joint->tag;
                if (j0==-1 || j0 >= i) continue;

                const dJointWithInfo1 *jiother = jointiinfos + j0;
                int ofsother = (jiother->joint->node[1].body == jb0) ? 8*jiother->info.m : 0;
                // set block of A
                MultiplyAdd2_p8r (Arow + ofs[j0], JinvMrow, 
                  J + 2*8*ofs[j0] + ofsother, infom, jiother->info.m, mskip);
              }

              dxBody *jb1 = joint->node[1].body;
              dIASSERT(jb1 != jb0);
              if (jb1)
              {
                for (dxJointNode *n1=jb1->firstjoint; n1; n1=n1->next) {
                  // if joint was tagged as -1 then it is an inactive (m=0)
                  // joint that should not be considered
                  int j1 = n1->joint->tag;
                  if (j1==-1 || j1 >= i) continue;

                  const dJointWithInfo1 *jiother = jointiinfos + j1;
                  int ofsother = (jiother->joint->node[1].body == jb1) ? 8*jiother->info.m : 0;
                  // set block of A
                  MultiplyAdd2_p8r (Arow + ofs[j1], JinvMrow + 8*infom, 
                    J + 2*8*ofs[j1] + ofsother, infom, jiother->info.m, mskip);
                }
              }

              ofs[i] = ofsi;
              ofsi += infom;
            }

          } END_STATE_SAVE(context, ofsstate);
        }

        {
          // compute diagonal blocks of A
          const int mskip = dPAD(m);

          unsigned ofsi = 0;
          const dJointWithInfo1 *jicurr = jointiinfos;
          const dJointWithInfo1 *const jiend = jicurr + nj;
          for (; jicurr != jiend; ++jicurr) {
            const int infom = jicurr->info.m;
            dReal *Arow = A + (mskip+1)*ofsi;
            dReal *JinvMrow = JinvM + 2*8*ofsi;
            dReal *Jrow = J + 2*8*ofsi;
            Multiply2_p8r (Arow, JinvMrow, Jrow, infom, infom, mskip);
            if (jicurr->joint->node[1].body) {
              MultiplyAdd2_p8r (Arow, JinvMrow + 8*infom, Jrow + 8*infom, infom, infom, mskip);
            }

            ofsi += infom;
          }
        }

        {
          // add cfm to the diagonal of A
          const int mskip = dPAD(m);

          dReal *Arow = A;
          for (int i=0; i<m; Arow += mskip, ++i) {
            Arow[i] += cfm[i] * stepsizeRecip;
          }
        }
       }

    } END_STATE_SAVE(context, cfmstate);

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (A,m,m,1,"A");
#   endif

    BEGIN_STATE_SAVE(context, tmp1state) {
      // compute the right hand side `rhs'
      IFTIMING(dTimerNow ("compute rhs"));

      dReal *tmp1 = context->AllocateArray<dReal> (nb*8);
      //dSetZero (tmp1,nb*8);

      {
        // put v/h + invM*fe into tmp1
        dReal *tmp1curr = tmp1;
        const dReal *invIrow = invI;
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; tmp1curr+=8, invIrow+=12, ++bodycurr) {
          dxBody *b = *bodycurr;
          for (int j=0; j<3; ++j) tmp1curr[j] = b->facc[j]*b->invMass + b->lvel[j]*stepsizeRecip;
          dMultiply0_331 (tmp1curr+4, invIrow, b->tacc);
          for (int k=0; k<3; ++k) tmp1curr[4+k] += b->avel[k]*stepsizeRecip;
        }
      }

#     ifdef COMPARE_METHODS
      comparator.nextMatrix (c,m,1,0,"c");
#     endif

      {
        // init rhs -- this erases 'c' as they reside in the same memory!!!
        rhs = c;
        for (int i=0; i<m; ++i) rhs[i] = c[i]*stepsizeRecip;
        c = NULL; // set 'c' to NULL to prevent unexpected access
      }

      {
        // put J*tmp1 into rhs
        unsigned ofsi = 0;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; ++jicurr) {
          const int infom = jicurr->info.m;
          dxJoint *joint = jicurr->joint;

          dReal *rhscurr = rhs+ofsi;
          const dReal *Jrow = J + 2*8*ofsi;
          MultiplySub0_p81 (rhscurr, Jrow, tmp1 + 8*joint->node[0].body->tag, infom);
          if (joint->node[1].body) {
            MultiplySub0_p81 (rhscurr, Jrow + 8*infom, tmp1 + 8*joint->node[1].body->tag, infom);
          }

          ofsi += infom;
        }
      }

#     ifdef COMPARE_METHODS
      comparator.nextMatrix (rhs,m,1,0,"rhs");
#     endif
    
    } END_STATE_SAVE(context, tmp1state);

    dReal *lambda = context->AllocateArray<dReal> (m);

    BEGIN_STATE_SAVE(context, lcpstate) {
      IFTIMING(dTimerNow ("solving LCP problem"));

      // solve the LCP problem and get lambda.
      // this will destroy A but that's OK
      dSolveLCP (context, m, A, lambda, rhs, NULL, nub, lo, hi, findex);

    } END_STATE_SAVE(context, lcpstate);

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (lambda,m,1,0,"lambda");
#   endif

    {
      IFTIMING(dTimerNow ("compute constraint force"));

      // compute the constraint force `cforce'
      // compute cforce = J'*lambda
      unsigned ofsi = 0;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; ++jicurr) {
        const int infom = jicurr->info.m;
        dxJoint *joint = jicurr->joint;
        
        const dReal *JJ = J + 2*8*ofsi;
        const dReal *lambdarow = lambda + ofsi;

        dJointFeedback *fb = joint->feedback;

        if (fb) {
          // the user has requested feedback on the amount of force that this
          // joint is applying to the bodies. we use a slightly slower
          // computation that splits out the force components and puts them
          // in the feedback structure.
          dReal data[8];
          Multiply1_8q1 (data, JJ, lambdarow, infom);

          dxBody* b1 = joint->node[0].body;
          dReal *cf1 = cforce + 8*b1->tag;
          cf1[0] += (fb->f1[0] = data[0]);
          cf1[1] += (fb->f1[1] = data[1]);
          cf1[2] += (fb->f1[2] = data[2]);
          cf1[4] += (fb->t1[0] = data[4]);
          cf1[5] += (fb->t1[1] = data[5]);
          cf1[6] += (fb->t1[2] = data[6]);

          dxBody* b2 = joint->node[1].body;
          if (b2){
            Multiply1_8q1 (data, JJ + 8*infom, lambdarow, infom);

            dReal *cf2 = cforce + 8*b2->tag;
            cf2[0] += (fb->f2[0] = data[0]);
            cf2[1] += (fb->f2[1] = data[1]);
            cf2[2] += (fb->f2[2] = data[2]);
            cf2[4] += (fb->t2[0] = data[4]);
            cf2[5] += (fb->t2[1] = data[5]);
            cf2[6] += (fb->t2[2] = data[6]);
          }
        }
        else {
          // no feedback is required, let's compute cforce the faster way
          dxBody* b1 = joint->node[0].body;
          dReal *cf1 = cforce + 8*b1->tag;
          MultiplyAdd1_8q1 (cf1, JJ, lambdarow, infom);
          
          dxBody* b2 = joint->node[1].body;
          if (b2) {
            dReal *cf2 = cforce + 8*b2->tag;
            MultiplyAdd1_8q1 (cf2, JJ + 8*infom, lambdarow, infom);
          }
        }

        ofsi += infom;
      }
    }
  } // if (m > 0)

  {
    // compute the velocity update
    IFTIMING(dTimerNow ("compute velocity update"));

    // add fe to cforce and multiply cforce by stepsize
    dReal data[4];
    const dReal *invIrow = invI;
    dReal *cforcecurr = cforce;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow+=12, cforcecurr+=8, ++bodycurr) {
      dxBody *b = *bodycurr;

      dReal body_invMass_mul_stepsize = stepsize * b->invMass;
      for (int j=0; j<3; ++j) b->lvel[j] += (cforcecurr[j] + b->facc[j]) * body_invMass_mul_stepsize;

      for (int k=0; k<3; ++k) data[k] = (cforcecurr[4+k] + b->tacc[k]) * stepsize;
      dMultiplyAdd0_331 (b->avel, invIrow, data);
    }
  }

  {
    // update the position and orientation from the new linear/angular velocity
    // (over the given timestep)
    IFTIMING(dTimerNow ("update position"));
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
      dxBody *b = *bodycurr;
      dxStepBody (b,stepsize);
    }
  }

#ifdef COMPARE_METHODS
  ALLOCA(dReal,tmp_vnew, nb*6*sizeof(dReal));
  for (i=0; i<nb; ++i) {
    for (j=0; j<3; ++j) tmp_vnew[i*6+j] = body[i]->lvel[j];
    for (j=0; j<3; ++j) tmp_vnew[i*6+3+j] = body[i]->avel[j];
  }
  comparator.nextMatrix (tmp_vnew,nb*6,1,0,"vnew");
#endif

  {
    IFTIMING(dTimerNow ("tidy up"));

    // zero all force accumulators
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
      dxBody *b = *bodycurr;
      b->facc[0] = 0;
      b->facc[1] = 0;
      b->facc[2] = 0;
      b->facc[3] = 0;
      b->tacc[0] = 0;
      b->tacc[1] = 0;
      b->tacc[2] = 0;
      b->tacc[3] = 0;
    }
  }

  IFTIMING(dTimerEnd());
  if (m > 0) IFTIMING(dTimerReport (stdout,1));

}

//****************************************************************************

void dInternalStepIsland (dxWorldProcessContext *context, 
                          dxWorld *world, dxBody * const *body, int nb,
                          dxJoint * const *joint, int nj, dReal stepsize)
{
#ifndef COMPARE_METHODS
  dInternalStepIsland_x2 (context,world,body,nb,joint,nj,stepsize);

#endif

#ifdef COMPARE_METHODS
  int i;

  // save body state
  ALLOCA(dxBody,state,nb*sizeof(dxBody));

  for (i=0; i<nb; i++) memcpy (state+i,body[i],sizeof(dxBody));

  // take slow step
  comparator.reset();
  dInternalStepIsland_x1 (context,world,body,nb,joint,nj,stepsize);
  comparator.end();

  // restore state
  for (i=0; i<nb; i++) memcpy (body[i],state+i,sizeof(dxBody));

  // take fast step
  dInternalStepIsland_x2 (context,world,body,nb,joint,nj,stepsize);
  comparator.end();

  //comparator.dump();
  //_exit (1);
#endif
}

size_t dxEstimateStepMemoryRequirements (dxBody * const *body, int nb, dxJoint * const *_joint, int _nj)
{
  int nj, m;

  {
    int njcurr = 0, mcurr = 0;
    dxJoint::SureMaxInfo info;
    dxJoint *const *const _jend = _joint + _nj;
    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; ++_jcurr) {	
      dxJoint *j = *_jcurr;
      j->getSureMaxInfo (&info);

      int jm = info.max_m;
      if (jm > 0) {
        njcurr++;

        mcurr += jm;
      }
    }
    nj = njcurr; m = mcurr;
  }

  size_t res = 0;

  res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb); // for invI

  {
    size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * 2 * _nj); // for initial jointiinfos

    // The array can't grow right more than by nj
    size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * (_nj + nj)); // for shrunk jointiinfos
    sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 8 * nb); // for cforce
    if (m > 0) {
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * m); // for J
      int mskip = dPAD(m);
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * mskip * m); // for A
      sub1_res2 += 3 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for lo, hi, rhs
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
      {
        size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for cfm
        sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * m); // for JinvM
        {
          size_t sub3_res1 = dEFFICIENT_SIZE(sizeof(int) * m); // for ofs

          size_t sub3_res2 = 0;

          sub2_res1 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        size_t sub2_res2 = 0;
        {
          size_t sub3_res1 = 0;
          {
            size_t sub4_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 8 * nb); // for tmp1

            size_t sub4_res2 = 0;

            sub3_res1 += (sub4_res1 >= sub4_res2) ? sub4_res1 : sub4_res2;
          }

          size_t sub3_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
          {
            size_t sub4_res1 = dEstimateSolveLCPMemoryReq(m, false);

            size_t sub4_res2 = 0;

            sub3_res2 += (sub4_res1 >= sub4_res2) ? sub4_res1 : sub4_res2;
          }

          sub2_res2 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        sub1_res2 += (sub2_res1 >= sub2_res2) ? sub2_res1 : sub2_res2;
      }
    }

    res += (sub1_res1 >= sub1_res2) ? sub1_res1 : sub1_res2;
  }

  return res;
}


