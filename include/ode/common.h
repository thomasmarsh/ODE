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

#ifndef _ODE_COMMON_H_
#define _ODE_COMMON_H_

#include "ode/error.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* constants */

/* pi and 1/sqrt(2) are defined here if necessary because they don't get
 * defined in <math.h> on some platforms (like MS-Windows)
 */

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif
#ifndef M_SQRT1_2
#define M_SQRT1_2 REAL(0.7071067811865475244008443621048490)
#endif


/* debugging:
 *   IASSERT  is an internal assertion, i.e. a consistency check. if it fails
 *            we want to know where.
 *   UASSERT  is a user assertion, i.e. if it fails a nice error message
 *            should be printed for the user.
 *   AASSERT  is an arguments assertion, i.e. if it fails "bad argument(s)"
 *            is printed.
 *   DEBUGMSG just prints out a message
 */

#ifndef dNODEBUG
#ifdef __GNUC__
#define dIASSERT(a) if (!(a)) dDebug (d_ERR_IASSERT, \
  "assertion \"" #a "\" failed in %s() [%s]",__FUNCTION__,__FILE__);
#define dUASSERT(a,msg) if (!(a)) dDebug (d_ERR_UASSERT, \
  msg " in %s()", __FUNCTION__);
#define dDEBUGMSG(msg) dMessage (d_ERR_UASSERT, \
  msg " in %s()", __FUNCTION__);
#else
#define dIASSERT(a) if (!(a)) dDebug (d_ERR_IASSERT, \
  "assertion \"" #a "\" failed in %s:%d",__FILE__,__LINE__);
#define dUASSERT(a,msg) if (!(a)) dDebug (d_ERR_UASSERT, \
  msg " (%s:%d)", __FILE__,__LINE__);
#define dDEBUGMSG(msg) dMessage (d_ERR_UASSERT, \
  msg " (%s:%d)", __FILE__,__LINE__);
#endif
#else
#define dIASSERT(a) ;
#define dUASSERT(a,msg) ;
#define dDEBUGMSG(msg) ;
#endif
#define dAASSERT(a) dUASSERT(a,"Bad argument(s)")


/* floating point data type, vector, matrix and quaternion types */

#if defined(dSINGLE)
typedef float dReal;
#define dInfinity HUGE_VALF
#elif defined(dDOUBLE)
typedef double dReal;
#define dInfinity HUGE_VAL
#else
#error You must #define dSINGLE or dDOUBLE
#endif

/* round an integer up to a multiple of 4, except that 0 and 1 are unmodified
 * (used to compute matrix leading dimensions)
 */
#define dPAD(a) (((a) > 1) ? ((((a)-1)|3)+1) : (a))

/* these types are mainly just used in headers */
typedef dReal dVector3[4];
typedef dReal dVector4[4];
typedef dReal dMatrix3[4*3];
typedef dReal dMatrix4[4*4];
typedef dReal dMatrix6[8*6];
typedef dReal dQuaternion[4];


/* precision dependent scalar math functions */

#if defined(dSINGLE)

#define REAL(x) (x ## f)		/* form a constant */
#define dRecip(x) (1.0f/(x))		/* reciprocal */
#define dSqrt(x) sqrt(x)		/* square root */
#define dRecipSqrt(x) (1.0f/sqrt(x))	/* reciprocal square root */
#define dSin(x) sin(x)			/* sine */
#define dCos(x) cos(x)			/* cosine */
#define dFabs(x) fabs(x)		/* absolute value */
#define dAtan2(y,x) atan2((y),(x))	/* arc tangent with 2 args */

#elif defined(dDOUBLE)

#define REAL(x) (x)
#define dRecip(x) (1.0/(x))
#define dSqrt(x) sqrt(x)
#define dRecipSqrt(x) (1.0/sqrt(x))
#define dSin(x) sin(x)
#define dCos(x) cos(x)
#define dFabs(x) fabs(x)
#define dAtan2(y,x) atan2((y),(x))

#else
#error You must #define dSINGLE or dDOUBLE
#endif


/* utility */

/* alloca aligned to 16 bytes. on a pentium it is important to align doubles
 * to 8 byte boundaries, and the 4 floats in a SIMD register to 16 byte
 * boundaries. this alignment wont hurt for other architectures either.
 * note this can waste up to 15 bytes, depending on what alloca() returns.
 */

#define dALLOCA16(n) ((char*)(((((int)alloca((n)+15))-1)|15)+1))


/* internal object types (all prefixed with `dx') */

struct dxWorld;		/* dynamics world */
struct dxSpace;		/* collision space */
struct dxBody;		/* rigid body (dynamics object) */
struct dxGeom;		/* geometry (collision object) */
struct dxJoint;
struct dxJointNode;
struct dxJointGroup;

typedef struct dxWorld *dWorldID;
typedef struct dxSpace *dSpaceID;
typedef struct dxBody *dBodyID;
typedef struct dxGeom *dGeomID;
typedef struct dxJoint *dJointID;
typedef struct dxJointGroup *dJointGroupID;


/* error numbers */

#define d_ERR_UNKNOWN		0	/* unknown error */
#define d_ERR_IASSERT		1	/* internal assertion failed */
#define d_ERR_UASSERT		2	/* user assertion failed */


/* joint type numbers */

enum {
  dJointTypeNone = 0,		/* or "unknown" */
  dJointTypeBall,
  dJointTypeHinge,
  dJointTypeSlider,
  dJointTypeContact,
  dJointTypeHinge2,
  dJointTypeFixed,
};


/* an alternative way of setting joint parameters, using joint parameter
 * structures and member constants. we don't actually do this yet.
 */

/*
typedef struct dLimot {
  int mode;
  dReal lostop, histop;
  dReal vel, fmax;
  dReal fudge_factor;
  dReal bounce, soft;
  dReal suspension_erp, suspension_cfm;
} dLimot;

enum {
  dLimotLoStop		= 0x0001,
  dLimotHiStop		= 0x0002,
  dLimotVel		= 0x0004,
  dLimotFMax		= 0x0008,
  dLimotFudgeFactor	= 0x0010,
  dLimotBounce		= 0x0020,
  dLimotSoft		= 0x0040
};
*/


/* standard joint parameter names. why are these here? - because we don't want
 * to include all the joint function definitions in joint.cpp. hmmmm.
 */
enum {
  /* parameters for limits and motors */
  dParamLoStop,
  dParamHiStop,
  dParamVel,
  dParamFMax,
  dParamFudgeFactor,
  dParamBounce,
  dParamStopERP,
  dParamStopCFM,

  /* parameters for suspension */
  dParamSuspensionERP,
  dParamSuspensionCFM,

  /* a second set of all parameters, e.g. for the second joint limit, the
   * second suspension axis etc. this has bit 8 set.
   */
  dParamLoStop2=0x100,
  dParamHiStop2,
  dParamVel2,
  dParamFMax2,
  dParamFudgeFactor2,
  dParamBounce2,
  dParamStopERP2,
  dParamStopCFM2,
  dParamSuspensionERP2,
  dParamSuspensionCFM2
};


#ifdef __cplusplus
}
#endif

#endif
