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
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include "config.h"
#include "odemath.h"
#include "matrix.h"
#include "objects.h"
#include "joints/joint.h"
#include "lcp.h"
#include "util.h"
#include "threadingutils.h"

#include <new>


#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((B)>(A) ? (B) : (A))


//****************************************************************************
// misc defines

//#define TIMING


#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif


struct dJointWithInfo1
{
    dxJoint *joint;
    dxJoint::Info1 info;
};

struct dxStepperStage0Outputs
{
    size_t                          ji_start;
    size_t                          ji_end;
    unsigned int                    m;
    unsigned int                    nub;
};

struct dxStepperStage1CallContext
{
    dxStepperStage1CallContext(dxStepperProcessingCallContext *stepperCallContext, void *stageMemArenaState, dReal *invI, dJointWithInfo1 *jointinfos):
        m_stepperCallContext(stepperCallContext), m_stageMemArenaState(stageMemArenaState), 
        m_invI(invI), m_jointinfos(jointinfos)
    {
    }

    dxStepperProcessingCallContext  *m_stepperCallContext;
    void                            *m_stageMemArenaState;
    dReal                           *m_invI;
    dJointWithInfo1                 *m_jointinfos;
    dxStepperStage0Outputs          m_stage0Outputs;
};

struct dxStepperStage0BodiesCallContext
{
    dxStepperStage0BodiesCallContext(dxStepperProcessingCallContext *stepperCallContext, dReal *invI):
        m_stepperCallContext(stepperCallContext), m_invI(invI), 
        m_tagsTaken(0), m_gravityTaken(0), m_inertiaBodyIndex(0)
    {
    }

    dxStepperProcessingCallContext  *m_stepperCallContext;
    dReal                           *m_invI;
    atomicord32                     m_tagsTaken;
    atomicord32                     m_gravityTaken;
    unsigned int                    volatile m_inertiaBodyIndex;
};

struct dxStepperStage0JointsCallContext
{
    dxStepperStage0JointsCallContext(dxStepperProcessingCallContext *stepperCallContext, dJointWithInfo1 *jointinfos, dxStepperStage0Outputs *stage0Outputs):
        m_stepperCallContext(stepperCallContext), m_jointinfos(jointinfos), m_stage0Outputs(stage0Outputs)
    {
    }

    dxStepperProcessingCallContext  *m_stepperCallContext;
    dJointWithInfo1                 *m_jointinfos;
    dxStepperStage0Outputs          *m_stage0Outputs;
};

static int dxStepIsland_Stage0_Bodies_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage0_Joints_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage1_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

static void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext);
static void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext *callContext);
static void dxStepIsland_Stage1(dxStepperStage1CallContext *callContext);


//****************************************************************************
// special matrix multipliers

// this assumes the 4th and 8th rows of B and C are zero.

static void Multiply2_p8r (dReal *A, const dReal *B, const dReal *C,
                           unsigned int p, unsigned int r, int Askip)
{
    dIASSERT (p>0 && r>0 && A && B && C);
    const int Askip_munus_r = Askip - r;
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned int i = p; i != 0; --i) {
        const dReal *cc = C;
        for (unsigned int j = r; j != 0; --j) {
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
                              unsigned int p, unsigned int r, unsigned int Askip)
{
    dIASSERT (p>0 && r>0 && A && B && C);
    const unsigned int Askip_munus_r = Askip - r;
    dIASSERT(Askip >= r);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned int i = p; i != 0; --i) {
        const dReal *cc = C;
        for (unsigned int j = r; j != 0; --j) {
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

static void MultiplySub0_p81 (dReal *A, const dReal *B, const dReal *C, unsigned int p)
{
    dIASSERT (p>0 && A && B && C);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned int i = p; i != 0; --i) {
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

static void MultiplyAdd1_8q1 (dReal *A, const dReal *B, const dReal *C, unsigned int q)
{
    dIASSERT (q>0 && A && B && C);
    const dReal *bb = B;
    dReal sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
    for (unsigned int k = 0; k < q; ++k) {
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

static void Multiply1_8q1 (dReal *A, const dReal *B, const dReal *C, unsigned int q)
{
    const dReal *bb = B;
    dReal sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
    for (unsigned int k = 0; k < q; ++k) {
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

//****************************************************************************

/*extern */
void dxStepIsland(dxStepperProcessingCallContext *callContext)
{
    IFTIMING(dTimerStart("preprocessing"));

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    dxWorld *world = callContext->m_world;
    unsigned int nb = callContext->m_islandBodiesCount;
    unsigned int _nj = callContext->m_islandJointsCount;

    dReal *invI = memarena->AllocateArray<dReal> (3*4*(size_t)nb);
    // Reserve twice as much memory and start from the middle so that regardless of 
    // what direction the array grows to there would be sufficient room available.
    const size_t ji_reserve_count = 2 * (size_t)_nj;
    dJointWithInfo1 *const jointinfos = memarena->AllocateArray<dJointWithInfo1> (ji_reserve_count);

    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;
    dIASSERT(allowedThreads != 0);

    void *stagesMemArenaState = memarena->SaveState();

    dxStepperStage1CallContext *stage1CallContext = (dxStepperStage1CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage1CallContext));
    new(stage1CallContext) dxStepperStage1CallContext(callContext, stagesMemArenaState, invI, jointinfos);

    dxStepperStage0BodiesCallContext *stage0BodiesCallContext = (dxStepperStage0BodiesCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0BodiesCallContext));
    new(stage0BodiesCallContext) dxStepperStage0BodiesCallContext(callContext, invI);
    
    dxStepperStage0JointsCallContext *stage0JointsCallContext = (dxStepperStage0JointsCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0JointsCallContext));
    new(stage0JointsCallContext) dxStepperStage0JointsCallContext(callContext, jointinfos, &stage1CallContext->m_stage0Outputs);

    if (allowedThreads == 1)
    {
        dxStepIsland_Stage0_Bodies(stage0BodiesCallContext);
        dxStepIsland_Stage0_Joints(stage0JointsCallContext);
        dxStepIsland_Stage1(stage1CallContext);
    }
    else
    {
        unsigned bodyThreads = allowedThreads;
        unsigned jointThreads = 1;

        dCallReleaseeID stage1CallReleasee;
        world->PostThreadedCallForUnawareReleasee(NULL, &stage1CallReleasee, bodyThreads + jointThreads, callContext->m_finalReleasee, 
            NULL, &dxStepIsland_Stage1_Callback, stage1CallContext, 0, "StepIsland Stage1");

        world->PostThreadedCallsGroup(NULL, bodyThreads, stage1CallReleasee, &dxStepIsland_Stage0_Bodies_Callback, stage0BodiesCallContext, "StepIsland Stage0-Bodies");

        world->PostThreadedCall(NULL, NULL, 0, stage1CallReleasee, NULL, &dxStepIsland_Stage0_Joints_Callback, stage0JointsCallContext, 0, "StepIsland Stage0-Joints");
        dIASSERT(jointThreads == 1);
    }
}    

static 
int dxStepIsland_Stage0_Bodies_Callback(void *_callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    dxStepperStage0BodiesCallContext *callContext = (dxStepperStage0BodiesCallContext *)_callContext;
    dxStepIsland_Stage0_Bodies(callContext);
    return 1;
}

static 
void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext)
{
    dxBody * const *body = callContext->m_stepperCallContext->m_islandBodiesStart;
    unsigned int nb = callContext->m_stepperCallContext->m_islandBodiesCount;

    if (ThrsafeExchange(&callContext->m_tagsTaken, 1) == 0)
    {
        // number all bodies in the body list - set their tag values
        for (unsigned int i=0; i<nb; i++) body[i]->tag = i;
    }

    if (ThrsafeExchange(&callContext->m_gravityTaken, 1) == 0)
    {
        dxWorld *world = callContext->m_stepperCallContext->m_world;

        // add the gravity force to all bodies
        // since gravity does normally have only one component it's more efficient
        // to run three loops for each individual component
        dxBody *const *const bodyend = body + nb;
        dReal gravity_x = world->gravity[0];
        if (gravity_x) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[0] += b->mass.mass * gravity_x;
                }
            }
        }
        dReal gravity_y = world->gravity[1];
        if (gravity_y) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[1] += b->mass.mass * gravity_y;
                }
            }
        }
        dReal gravity_z = world->gravity[2];
        if (gravity_z) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[2] += b->mass.mass * gravity_z;
                }
            }
        }
    }

    // for all bodies, compute the inertia tensor and its inverse in the global
    // frame, and compute the rotational force and add it to the torque
    // accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
    {
        dReal *invIrow = callContext->m_invI;
        unsigned int bodyIndex = ThrsafeIncrementIntUpToLimit(&callContext->m_inertiaBodyIndex, nb);

        for (unsigned int i = 0; i != nb; invIrow += 12, ++i) {
            if (i == bodyIndex) {
                dMatrix3 tmp;
                dxBody *b = body[i];

                // compute inverse inertia tensor in global frame
                dMultiply2_333 (tmp,b->invI,b->posr.R);
                dMultiply0_333 (invIrow,b->posr.R,tmp);

                if ((b->flags & dxBodyGyroscopic) != 0) {
                    dMatrix3 I;
                    // compute inertia tensor in global frame
                    dMultiply2_333 (tmp,b->mass.I,b->posr.R);
                    dMultiply0_333 (I,b->posr.R,tmp);
                    // compute rotational force
                    dMultiply0_331 (tmp,I,b->avel);
                    dSubtractVectorCross3(b->tacc,b->avel,tmp);
                }

                bodyIndex = ThrsafeIncrementIntUpToLimit(&callContext->m_inertiaBodyIndex, nb);
            }
        }
    }
}

static 
int dxStepIsland_Stage0_Joints_Callback(void *_callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    dxStepperStage0JointsCallContext *callContext = (dxStepperStage0JointsCallContext *)_callContext;
    dxStepIsland_Stage0_Joints(callContext);
    return 1;
}

static 
void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext *callContext)
{
    dxJoint * const *_joint = callContext->m_stepperCallContext->m_islandJointsStart;
    dJointWithInfo1 *jointinfos = callContext->m_jointinfos;
    unsigned int _nj = callContext->m_stepperCallContext->m_islandJointsCount;

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

    size_t ji_start, ji_end;
    {
        unsigned int mcurr = 0;
        size_t unb_start, mix_start, mix_end, lcp_end;
        unb_start = mix_start = mix_end = lcp_end = _nj;

        dJointWithInfo1 *jicurr = jointinfos + lcp_end;
        dxJoint *const *const _jend = _joint + _nj;
        dxJoint *const *_jcurr = _joint;
        while (true) {
            // -------------------------------------------------------------------------
            // Switch to growing array forward
            {
                bool fwd_end_reached = false;
                dJointWithInfo1 *jimixend = jointinfos + mix_end;
                while (true) {	// jicurr=dest, _jcurr=src
                    if (_jcurr == _jend) {
                        lcp_end = jicurr - jointinfos;
                        fwd_end_reached = true;
                        break;
                    }
                    dxJoint *j = *_jcurr++;
                    j->getInfo1 (&jicurr->info);
                    dIASSERT (/*jicurr->info.m >= 0 && */jicurr->info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurr->info.nub <= jicurr->info.m);
                    if (jicurr->info.m != 0) {
                        mcurr += jicurr->info.m;
                        if (jicurr->info.nub == 0) { // A lcp info - a correct guess!!!
                            jicurr->joint = j;
                            ++jicurr;
                        } else if (jicurr->info.nub < jicurr->info.m) { // A mixed case
                            if (unb_start == mix_start) { // no unbounded infos yet - just move to opposite side of mixed-s
                                unb_start = mix_start = mix_start - 1;
                                dJointWithInfo1 *jimixstart = jointinfos + mix_start;
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
                            dJointWithInfo1 *jiunbstart = jointinfos + unb_start;
                            jiunbstart->info = jicurr->info;
                            jiunbstart->joint = j;
                            lcp_end = jicurr - jointinfos;
                            mix_end = jimixend - jointinfos;
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
                dJointWithInfo1 *jimixstart = jointinfos + mix_start - 1;
                while (true) {	// jicurr=dest, _jcurr=src
                    if (_jcurr == _jend) {
                        unb_start = (jicurr + 1) - jointinfos;
                        mix_start = (jimixstart + 1) - jointinfos;
                        bkw_end_reached = true;
                        break;
                    }
                    dxJoint *j = *_jcurr++;
                    j->getInfo1 (&jicurr->info);
                    dIASSERT (/*jicurr->info.m >= 0 && */jicurr->info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurr->info.nub <= jicurr->info.m);
                    if (jicurr->info.m != 0) {
                        mcurr += jicurr->info.m;
                        if (jicurr->info.nub == jicurr->info.m) { // An unbounded info - a correct guess!!!
                            jicurr->joint = j;
                            --jicurr;
                        } else if (jicurr->info.nub != 0) { // A mixed case
                            if (mix_end == lcp_end) { // no lcp infos yet - just move to opposite side of mixed-s
                                dJointWithInfo1 *jimixend = jointinfos + mix_end;
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
                            dJointWithInfo1 *jilcpend = jointinfos + lcp_end;
                            lcp_end = lcp_end + 1;
                            jilcpend->info = jicurr->info;
                            jilcpend->joint = j;
                            unb_start = (jicurr + 1) - jointinfos;
                            mix_start = (jimixstart + 1) - jointinfos;
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

        callContext->m_stage0Outputs->m = mcurr;
        callContext->m_stage0Outputs->nub = (unsigned)(mix_start - unb_start);
        dIASSERT((size_t)(mix_start - unb_start) <= (size_t)UINT_MAX);
        ji_start = unb_start;
        ji_end = lcp_end;
    }

    {
        const dJointWithInfo1 *jicurr = jointinfos + ji_start;
        const dJointWithInfo1 *const jiend = jointinfos + ji_end;
        for (unsigned int i = 0; jicurr != jiend; i++, ++jicurr) {
            jicurr->joint->tag = i;
        }
    }

    callContext->m_stage0Outputs->ji_start = ji_start;
    callContext->m_stage0Outputs->ji_end = ji_end;
}

static 
int dxStepIsland_Stage1_Callback(void *_stage1CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    dxStepperStage1CallContext *stage1CallContext = (dxStepperStage1CallContext *)_stage1CallContext;
    dxStepIsland_Stage1(stage1CallContext);
    return 1;
}

static 
void dxStepIsland_Stage1(dxStepperStage1CallContext *stage1CallContext)
{
    dxStepperProcessingCallContext *callContext;
    dJointWithInfo1 *_jointinfos;
    dReal *invI;
    size_t ji_start, ji_end;
    unsigned int m;
    unsigned int nub;
    {
        callContext = stage1CallContext->m_stepperCallContext;
        _jointinfos = stage1CallContext->m_jointinfos;
        invI = stage1CallContext->m_invI;
        ji_start = stage1CallContext->m_stage0Outputs.ji_start;
        ji_end = stage1CallContext->m_stage0Outputs.ji_end;
        m = stage1CallContext->m_stage0Outputs.m;
        nub = stage1CallContext->m_stage0Outputs.nub;
    }

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    {
        memarena->RestoreState(stage1CallContext->m_stageMemArenaState);
        stage1CallContext = NULL; // WARNING! _stage1CallContext is not valid after this point!
        dIVERIFY(stage1CallContext == NULL); // To suppress compiler warnings about unused variable assignment

        unsigned int _nj = callContext->m_islandJointsCount;
        const size_t ji_reserve_count = 2 * (size_t)_nj;
        memarena->ShrinkArray<dJointWithInfo1>(_jointinfos, ji_reserve_count, ji_end);
    }

    dxWorld *world = callContext->m_world;
    dxBody * const *body = callContext->m_islandBodiesStart;
    unsigned int nb = callContext->m_islandBodiesCount;
    dJointWithInfo1 *jointinfos = _jointinfos + ji_start;
    unsigned int nj = (unsigned int)(ji_end - ji_start);
    dIASSERT((size_t)(ji_end - ji_start) <= (size_t)UINT_MAX);

    // this will be set to the force due to the constraints
    dReal *cforce = memarena->AllocateArray<dReal> ((size_t)nb*8);
    dSetZero (cforce,(size_t)nb*8);

    // if there are constraints, compute cforce
    if (m > 0) {
        // create a constraint equation right hand side vector `c', a constraint
        // force mixing vector `cfm', and LCP low and high bound vectors, and an
        // 'findex' vector.
        dReal *lo, *hi, *J, *A, *rhs;
        int *findex;

        {
            unsigned int mlocal = m;

            lo = memarena->AllocateArray<dReal> (mlocal);
            dSetValue (lo,mlocal,-dInfinity);

            hi = memarena->AllocateArray<dReal> (mlocal);
            dSetValue (hi,mlocal, dInfinity);

            J = memarena->AllocateArray<dReal> (2*8*(size_t)mlocal);
            dSetZero (J,2*8*(size_t)mlocal);

            findex = memarena->AllocateArray<int> (mlocal);
            for (unsigned int i=0; i<mlocal; ++i) findex[i] = -1;

            unsigned int mskip = dPAD(mlocal);
            A = memarena->AllocateArray<dReal> (mlocal*(size_t)mskip);
            dSetZero (A,mlocal*(size_t)mskip);

            rhs = memarena->AllocateArray<dReal> (mlocal);
            dSetZero (rhs,mlocal);
        }

        // Put 'c' in the same memory as 'rhs' as they transit into each other
        dReal *c = rhs; rhs = NULL; // erase rhs pointer for now as it is not to be used yet

        BEGIN_STATE_SAVE(memarena, cfmstate) {
            dReal *cfm = memarena->AllocateArray<dReal> (m);
            dSetValue (cfm,m,world->global_cfm);

            const dReal stepsizeRecip = dRecip(callContext->m_stepSize);

            dReal *JinvM = memarena->AllocateArray<dReal> (2*8*(size_t)m);
            dSetZero (JinvM,2*8*(size_t)m);

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

                dxJoint::Info2Descr Jinfo;
                Jinfo.rowskip = 8;

                unsigned ofsi = 0;
                const dJointWithInfo1 *jicurr = jointinfos;
                const dJointWithInfo1 *const jiend = jicurr + nj;
                for (; jicurr != jiend; ++jicurr) {
                    const unsigned int infom = jicurr->info.m;
                    dReal *const J1row = J + 2*8*(size_t)ofsi;
                    Jinfo.J1l = J1row;
                    Jinfo.J1a = J1row + 4;
                    dReal *const J2row = J1row + 8*(size_t)infom;
                    Jinfo.J2l = J2row;
                    Jinfo.J2a = J2row + 4;
                    Jinfo.c = c + ofsi;
                    Jinfo.cfm = cfm + ofsi;
                    Jinfo.lo = lo + ofsi;
                    Jinfo.hi = hi + ofsi;
                    Jinfo.findex = findex + ofsi;

                    dxJoint *joint = jicurr->joint;
                    joint->getInfo2 (stepsizeRecip, world->global_erp, &Jinfo);

                    // adjust returned findex values for global index numbering
                    int *findex_ofsi = findex + ofsi;
                    for (unsigned int j=0; j<infom; ++j) {
                        int fival = findex_ofsi[j];
                        if (fival != -1) 
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
                    const dJointWithInfo1 *jicurr = jointinfos;
                    const dJointWithInfo1 *const jiend = jicurr + nj;
                    for (; jicurr != jiend; ++jicurr) {
                        const unsigned int infom = jicurr->info.m;
                        dxJoint *joint = jicurr->joint;
                        unsigned int b0 = joint->node[0].body->tag;
                        dReal body_invMass0 = body[b0]->invMass;
                        dReal *body_invI0 = invI + (size_t)b0*12;
                        dReal *Jsrc = J + 2*8*(size_t)ofsi;
                        dReal *Jdst = JinvM + 2*8*(size_t)ofsi;
                        for (unsigned int j=infom; j>0;) {
                            j -= 1;
                            for (unsigned int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass0;
                            dMultiply0_133 (Jdst+4,Jsrc+4,body_invI0);
                            Jsrc += 8;
                            Jdst += 8;
                        }

                        if (joint->node[1].body) {
                            unsigned int b1 = joint->node[1].body->tag;
                            dReal body_invMass1 = body[b1]->invMass;
                            dReal *body_invI1 = invI + (size_t)b1*12;
                            for (unsigned int j=infom; j>0; ) {
                                j -= 1;
                                for (unsigned int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass1;
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

                    BEGIN_STATE_SAVE(memarena, ofsstate) {
                        unsigned int *ofs = memarena->AllocateArray<unsigned int> (m);
                        const unsigned int mskip = dPAD(m);

                        unsigned ofsi = 0;
                        const dJointWithInfo1 *jicurr = jointinfos;
                        const dJointWithInfo1 *const jiend = jicurr + nj;
                        for (unsigned int i = 0; jicurr != jiend; i++, ++jicurr) {
                            const unsigned int infom = jicurr->info.m;
                            dxJoint *joint = jicurr->joint;

                            dReal *Arow = A + mskip*(size_t)ofsi;
                            dReal *JinvMrow = JinvM + 2*8*(size_t)ofsi;

                            dxBody *jb0 = joint->node[0].body;
                            for (dxJointNode *n0=jb0->firstjoint; n0; n0=n0->next) {
                                // if joint was tagged as -1 then it is an inactive (m=0 or disabled)
                                // joint that should not be considered
                                int j0 = n0->joint->tag;
                                if (j0 != -1 && (unsigned)j0 < i) {
                                    const dJointWithInfo1 *jiother = jointinfos + j0;
                                    size_t ofsother = (jiother->joint->node[1].body == jb0) ? 8*(size_t)jiother->info.m : 0;
                                    // set block of A
                                    MultiplyAdd2_p8r (Arow + ofs[j0], JinvMrow, 
                                        J + 2*8*(size_t)ofs[j0] + ofsother, infom, jiother->info.m, mskip);
                                }
                            }

                            dxBody *jb1 = joint->node[1].body;
                            dIASSERT(jb1 != jb0);
                            if (jb1)
                            {
                                for (dxJointNode *n1=jb1->firstjoint; n1; n1=n1->next) {
                                    // if joint was tagged as -1 then it is an inactive (m=0 or disabled)
                                    // joint that should not be considered
                                    int j1 = n1->joint->tag;
                                    if (j1 != -1 && (unsigned)j1 < i) {
                                        const dJointWithInfo1 *jiother = jointinfos + j1;
                                        size_t ofsother = (jiother->joint->node[1].body == jb1) ? 8*(size_t)jiother->info.m : 0;
                                        // set block of A
                                        MultiplyAdd2_p8r (Arow + ofs[j1], JinvMrow + 8*(size_t)infom, 
                                            J + 2*8*(size_t)ofs[j1] + ofsother, infom, jiother->info.m, mskip);
                                    }
                                }
                            }

                            ofs[i] = ofsi;
                            ofsi += infom;
                        }

                    } END_STATE_SAVE(memarena, ofsstate);
                }

                {
                    // compute diagonal blocks of A
                    const unsigned int mskip = dPAD(m);

                    unsigned ofsi = 0;
                    const dJointWithInfo1 *jicurr = jointinfos;
                    const dJointWithInfo1 *const jiend = jicurr + nj;
                    for (; jicurr != jiend; ++jicurr) {
                        const unsigned int infom = jicurr->info.m;
                        dReal *Arow = A + (mskip+1)*(size_t)ofsi;
                        dReal *JinvMrow = JinvM + 2*8*(size_t)ofsi;
                        dReal *Jrow = J + 2*8*(size_t)ofsi;
                        Multiply2_p8r (Arow, JinvMrow, Jrow, infom, infom, mskip);
                        if (jicurr->joint->node[1].body) {
                            MultiplyAdd2_p8r (Arow, JinvMrow + 8*(size_t)infom, Jrow + 8*(size_t)infom, infom, infom, mskip);
                        }

                        ofsi += infom;
                    }
                }

                {
                    // add cfm to the diagonal of A
                    const unsigned int mskip = dPAD(m);

                    dReal *Arow = A;
                    for (unsigned int i=0; i<m; Arow += mskip, ++i) {
                        Arow[i] += cfm[i] * stepsizeRecip;
                    }
                }
            }

        } END_STATE_SAVE(memarena, cfmstate);

        BEGIN_STATE_SAVE(memarena, tmp1state) {
            // compute the right hand side `rhs'
            IFTIMING(dTimerNow ("compute rhs"));

            const dReal stepsizeRecip = dRecip(callContext->m_stepSize);

            dReal *tmp1 = memarena->AllocateArray<dReal> ((size_t)nb*8);
            //dSetZero (tmp1,nb*8);

            {
                // put v/h + invM*fe into tmp1
                dReal *tmp1curr = tmp1;
                const dReal *invIrow = invI;
                dxBody *const *const bodyend = body + nb;
                for (dxBody *const *bodycurr = body; bodycurr != bodyend; tmp1curr+=8, invIrow+=12, ++bodycurr) {
                    dxBody *b = *bodycurr;
                    for (unsigned int j=0; j<3; ++j) tmp1curr[j] = b->facc[j]*b->invMass + b->lvel[j]*stepsizeRecip;
                    dMultiply0_331 (tmp1curr+4, invIrow, b->tacc);
                    for (unsigned int k=0; k<3; ++k) tmp1curr[4+k] += b->avel[k]*stepsizeRecip;
                }
            }

            {
                // init rhs -- this erases 'c' as they reside in the same memory!!!
                rhs = c;
                for (unsigned int i=0; i<m; ++i) rhs[i] = c[i]*stepsizeRecip;
                c = NULL; // set 'c' to NULL to prevent unexpected access
            }

            {
                // put J*tmp1 into rhs
                unsigned ofsi = 0;
                const dJointWithInfo1 *jicurr = jointinfos;
                const dJointWithInfo1 *const jiend = jicurr + nj;
                for (; jicurr != jiend; ++jicurr) {
                    const unsigned int infom = jicurr->info.m;
                    dxJoint *joint = jicurr->joint;

                    dReal *rhscurr = rhs+ofsi;
                    const dReal *Jrow = J + 2*8*(size_t)ofsi;
                    MultiplySub0_p81 (rhscurr, Jrow, tmp1 + 8*(size_t)(unsigned)joint->node[0].body->tag, infom);
                    if (joint->node[1].body) {
                        MultiplySub0_p81 (rhscurr, Jrow + 8*(size_t)infom, tmp1 + 8*(size_t)(unsigned)joint->node[1].body->tag, infom);
                    }

                    ofsi += infom;
                }
            }
        } END_STATE_SAVE(memarena, tmp1state);

        dReal *lambda = memarena->AllocateArray<dReal> (m);

        BEGIN_STATE_SAVE(memarena, lcpstate) {
            IFTIMING(dTimerNow ("solving LCP problem"));

            // solve the LCP problem and get lambda.
            // this will destroy A but that's OK
            dSolveLCP (memarena, m, A, lambda, rhs, NULL, nub, lo, hi, findex);

        } END_STATE_SAVE(memarena, lcpstate);

        {
            IFTIMING(dTimerNow ("compute constraint force"));

            // compute the constraint force `cforce'
            // compute cforce = J'*lambda
            unsigned ofsi = 0;
            const dJointWithInfo1 *jicurr = jointinfos;
            const dJointWithInfo1 *const jiend = jicurr + nj;
            for (; jicurr != jiend; ++jicurr) {
                const unsigned int infom = jicurr->info.m;
                dxJoint *joint = jicurr->joint;

                const dReal *JJ = J + 2*8*(size_t)ofsi;
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
                    dReal *cf1 = cforce + 8*(size_t)(unsigned)b1->tag;
                    cf1[0] += (fb->f1[0] = data[0]);
                    cf1[1] += (fb->f1[1] = data[1]);
                    cf1[2] += (fb->f1[2] = data[2]);
                    cf1[4] += (fb->t1[0] = data[4]);
                    cf1[5] += (fb->t1[1] = data[5]);
                    cf1[6] += (fb->t1[2] = data[6]);

                    dxBody* b2 = joint->node[1].body;
                    if (b2){
                        Multiply1_8q1 (data, JJ + 8*(size_t)infom, lambdarow, infom);

                        dReal *cf2 = cforce + 8*(size_t)(unsigned)b2->tag;
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
                    dReal *cf1 = cforce + 8*(size_t)(unsigned)b1->tag;
                    MultiplyAdd1_8q1 (cf1, JJ, lambdarow, infom);

                    dxBody* b2 = joint->node[1].body;
                    if (b2) {
                        dReal *cf2 = cforce + 8*(size_t)(unsigned)b2->tag;
                        MultiplyAdd1_8q1 (cf2, JJ + 8*(size_t)infom, lambdarow, infom);
                    }
                }

                ofsi += infom;
            }
        }
    } // if (m > 0)

    {
        // compute the velocity update
        IFTIMING(dTimerNow ("compute velocity update"));

        const dReal stepsize = callContext->m_stepSize;

        // add fe to cforce and multiply cforce by stepsize
        dReal data[4];
        const dReal *invIrow = invI;
        dReal *cforcecurr = cforce;
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow+=12, cforcecurr+=8, ++bodycurr) {
            dxBody *b = *bodycurr;

            dReal body_invMass_mul_stepsize = stepsize * b->invMass;
            for (unsigned int j=0; j<3; ++j) b->lvel[j] += (cforcecurr[j] + b->facc[j]) * body_invMass_mul_stepsize;

            for (unsigned int k=0; k<3; ++k) data[k] = (cforcecurr[4+k] + b->tacc[k]) * stepsize;
            dMultiplyAdd0_331 (b->avel, invIrow, data);
        }
    }

    {
        // update the position and orientation from the new linear/angular velocity
        // (over the given timestep)
        IFTIMING(dTimerNow ("update position"));

        const dReal stepsize = callContext->m_stepSize;

        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
            dxBody *b = *bodycurr;
            dxStepBody (b, stepsize);
        }
    }

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

/*extern */
size_t dxEstimateStepMemoryRequirements (dxBody * const *body, unsigned int nb, dxJoint * const *_joint, unsigned int _nj)
{
    unsigned int nj, m;

    {
        unsigned int njcurr = 0, mcurr = 0;
        dxJoint::SureMaxInfo info;
        dxJoint *const *const _jend = _joint + _nj;
        for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; ++_jcurr) {	
            dxJoint *j = *_jcurr;
            j->getSureMaxInfo (&info);

            unsigned int jm = info.max_m;
            if (jm > 0) {
                njcurr++;

                mcurr += jm;
            }
        }
        nj = njcurr; m = mcurr;
    }

    size_t res = 0;

    res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * (size_t)nb); // for invI

    {
        size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * 2 * (size_t)_nj); // for initial jointinfos

        // The array can't grow right more than by nj
        size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * ((size_t)_nj + (size_t)nj)); // for shrunk jointinfos
        sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 8 * (size_t)nb); // for cforce
        if (m > 0) {
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * (size_t)m); // for J
            unsigned int mskip = dPAD(m);
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * (size_t)mskip * (size_t)m); // for A
            sub1_res2 += 3 * dEFFICIENT_SIZE(sizeof(dReal) * (size_t)m); // for lo, hi, rhs
            sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * (size_t)m); // for findex
            {
                size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dReal) * (size_t)m); // for cfm
                sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * (size_t)m); // for JinvM
                {
                    size_t sub3_res1 = dEFFICIENT_SIZE(sizeof(int) * (size_t)m); // for ofs

                    size_t sub3_res2 = 0;

                    sub2_res1 += dMAX(sub3_res1, sub3_res2);
                }

                size_t sub2_res2 = 0;
                {
                    size_t sub3_res1 = 0;
                    {
                        size_t sub4_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 8 * (size_t)nb); // for tmp1

                        size_t sub4_res2 = 0;

                        sub3_res1 += dMAX(sub4_res1, sub4_res2);
                    }

                    size_t sub3_res2 = dEFFICIENT_SIZE(sizeof(dReal) * (size_t)m); // for lambda
                    {
                        size_t sub4_res1 = dEstimateSolveLCPMemoryReq(m, false);

                        size_t sub4_res2 = 0;

                        sub3_res2 += dMAX(sub4_res1, sub4_res2);
                    }

                    sub2_res2 += dMAX(sub3_res1, sub3_res2);
                }

                sub1_res2 += dMAX(sub2_res1, sub2_res2);
            }
        }

        size_t sub1_res12_max = dMAX(sub1_res1, sub1_res2);
        size_t stage01_contexts = dEFFICIENT_SIZE(sizeof(dxStepperStage0BodiesCallContext))
            + dEFFICIENT_SIZE(sizeof(dxStepperStage0JointsCallContext))
            + dEFFICIENT_SIZE(sizeof(dxStepperStage1CallContext));
        res += dMAX(sub1_res12_max, stage01_contexts);
    }

    return res;
}


/*extern */
unsigned dxEstimateStepMaxCallCount(
    unsigned activeThreadCount, unsigned allowedThreadCount)
{
    unsigned result = 1 // dxStepIsland itself
        + (allowedThreadCount + 1) // allowedThreadCount * dxStepIsland_Stage0_Bodies + dxStepIsland_Stage0_Joints
        + 1; // dxStepIsland_Stage1
    return result;
}
