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

#include "joint.h"
#include "ode/odemath.h"
#include "ode/rotation.h"
#include "ode/matrix.h"

//****************************************************************************
// utility

// set three "ball-and-socket" rows in the constraint equation, and the
// corresponding right hand side.

static inline void setBall (dxJoint *joint, dxJoint::Info2 *info,
			    dVector3 anchor1, dVector3 anchor2)
{
  // anchor points in global coordinates with respect to body PORs.
  dVector3 a1,a2;

  int s = info->rowskip;

  // set jacobian
  info->J1lu[0] = 1;
  info->J1lu[s+1] = 1;
  info->J1lu[2*s+2] = 1;
  dMULTIPLY0_331 (a1,joint->node[0].body->R,anchor1);
  dCROSSMAT (info->J1au,a1,s,-,+);
  if (joint->node[1].body) {
    info->J2lu[0] = -1;
    info->J2lu[s+1] = -1;
    info->J2lu[2*s+2] = -1;
    dMULTIPLY0_331 (a2,joint->node[1].body->R,anchor2);
    dCROSSMAT (info->J2au,a2,s,+,-);
  }

  // set right hand side
  dReal k = info->fps * info->erp;
  if (joint->node[1].body) {
    for (int j=0; j<3; j++) {
      info->cu[j] = k * (a2[j] + joint->node[1].body->pos[j] -
			 a1[j] - joint->node[0].body->pos[j]);
    }
  }
  else {
    for (int j=0; j<3; j++) {
      info->cu[j] = k * (anchor2[j] - a1[j] -
			 joint->node[0].body->pos[j]);
    }
  }
}


// compute anchor points relative to bodies

static void setAnchors (dxJoint *j, dReal x, dReal y, dReal z,
			dVector3 anchor1, dVector3 anchor2)
{
  if (j->node[0].body) {
    dReal q[4];
    q[0] = x - j->node[0].body->pos[0];
    q[1] = y - j->node[0].body->pos[1];
    q[2] = z - j->node[0].body->pos[2];
    q[3] = 0;
    dMULTIPLY1_331 (anchor1,j->node[0].body->R,q);
    if (j->node[1].body) {
      q[0] = x - j->node[1].body->pos[0];
      q[1] = y - j->node[1].body->pos[1];
      q[2] = z - j->node[1].body->pos[2];
      q[3] = 0;
      dMULTIPLY1_331 (anchor2,j->node[1].body->R,q);
    }
    else {
      anchor2[0] = x;
      anchor2[1] = y;
      anchor2[2] = z;
    }
  }
  anchor1[3] = 0;
  anchor2[3] = 0;
}


// compute axes relative to bodies. axis2 can be 0

static void setAxes (dxJoint *j, dReal x, dReal y, dReal z,
		     dVector3 axis1, dVector3 axis2)
{
  if (j->node[0].body) {
    dReal q[4];
    q[0] = x;
    q[1] = y;
    q[2] = z;
    q[3] = 0;
    dNormalize3 (q);
    dMULTIPLY1_331 (axis1,j->node[0].body->R,q);
    if (axis2) {
      if (j->node[1].body) {
	dMULTIPLY1_331 (axis2,j->node[1].body->R,q);
      }
      else {
	axis2[0] = x;
	axis2[1] = y;
	axis2[2] = z;
      }
      axis2[3] = 0;
    }
  }
  axis1[3] = 0;
}


static void getAnchor (dxJoint *j, dVector3 result, dVector3 anchor1)
{
  if (j->node[0].body) {
    dMULTIPLY0_331 (result,j->node[0].body->R,anchor1);
    result[0] += j->node[0].body->pos[0];
    result[1] += j->node[0].body->pos[1];
    result[2] += j->node[0].body->pos[2];
  }
}


static void getAxis (dxJoint *j, dVector3 result, dVector3 axis1)
{
  if (j->node[0].body) {
    dMULTIPLY0_331 (result,j->node[0].body->R,axis1);
  }
}

//****************************************************************************
// ball and socket

static void ballInit (dxJointBall *j)
{
  dSetZero (j->anchor1,4);
  dSetZero (j->anchor2,4);
}


static void ballGetInfo1 (dxJointBall *j, dxJoint::Info1 *info)
{
  info->nub = 3;
  info->nlcp = 0;
}


static void ballGetInfo2 (dxJointBall *joint, dxJoint::Info2 *info)
{
  setBall (joint,info,joint->anchor1,joint->anchor2);
}


static void ballSetAnchor (dxJointBall *joint, dReal x, dReal y, dReal z)
{
  setAnchors (joint,x,y,z,joint->anchor1,joint->anchor2);
}


static void ballGetAnchor (dxJointBall *joint, dVector3 result)
{
  getAnchor (joint,result,joint->anchor1);
}


dxJoint::Vtable dball_vtable = {
  sizeof(dxJointBall),
  (dxJoint::init_fn*) ballInit,
  (dxJoint::getInfo1_fn*) ballGetInfo1,
  (dxJoint::getInfo2_fn*) ballGetInfo2,
  (dxJoint::setAnchor_fn*) ballSetAnchor, 0,
  (dxJoint::getAnchor_fn*) ballGetAnchor, 0};

//****************************************************************************
// hinge

static void hingeInit (dxJointHinge *j)
{
  dSetZero (j->anchor1,4);
  dSetZero (j->anchor2,4);
  dSetZero (j->axis1,4);
  dSetZero (j->axis2,4);
}


static void hingeGetInfo1 (dxJointHinge *j, dxJoint::Info1 *info)
{
  info->nub = 5;
  info->nlcp = 0;
}


static void hingeGetInfo2 (dxJointHinge *joint, dxJoint::Info2 *info)
{
  // set the three ball-and-socket rows
  setBall (joint,info,joint->anchor1,joint->anchor2);

  // set the two hinge rows. the hinge axis should be the only unconstrained
  // rotational axis, the angular velocity of the two bodies perpendicular to
  // the hinge axis should be equal. thus the constraint equations are
  //    p*w1 - p*w2 = 0
  //    q*w1 - q*w2 = 0
  // where p and q are unit vectors normal to the hinge axis, and w1 and w2
  // are the angular velocity vectors of the two bodies.

  dVector3 ax1;  // length 1 joint axis in global coordinates, from 1st body
  dVector3 p,q;  // plane space vectors for ax1
  dMULTIPLY0_331 (ax1,joint->node[0].body->R,joint->axis1);
  dPlaneSpace (ax1,p,q);

  int s3=3*info->rowskip;
  int s4=4*info->rowskip;

  info->J1au[s3+0] = p[0];
  info->J1au[s3+1] = p[1];
  info->J1au[s3+2] = p[2];
  info->J1au[s4+0] = q[0];
  info->J1au[s4+1] = q[1];
  info->J1au[s4+2] = q[2];

  if (joint->node[1].body) {
    info->J2au[s3+0] = -p[0];
    info->J2au[s3+1] = -p[1];
    info->J2au[s3+2] = -p[2];
    info->J2au[s4+0] = -q[0];
    info->J2au[s4+1] = -q[1];
    info->J2au[s4+2] = -q[2];
  }

  // compute the right hand side of the constraint equation. set relative
  // body velocities along p and q to bring the hinge back into alignment.
  // if ax1,ax2 are the unit length hinge axes as computed from body1 and
  // body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
  // if `theta' is the angle between ax1 and ax2, we need an angular velocity
  // along u to cover angle erp*theta in one step :
  //   |angular_velocity| = angle/time = erp*theta / stepsize
  //                      = (erp*fps) * theta
  //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
  //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
  // ...as ax1 and ax2 are unit length. if theta is smallish,
  // theta ~= sin(theta), so
  //    angular_velocity  = (erp*fps) * (ax1 x ax2)
  // ax1 x ax2 is in the plane space of ax1, so we project the angular
  // velocity to p and q to find the right hand side.

  dVector3 ax2,b;
  if (joint->node[1].body) {
    dMULTIPLY0_331 (ax2,joint->node[1].body->R,joint->axis2);
  }
  else {
    ax2[0] = joint->axis2[0];
    ax2[1] = joint->axis2[1];
    ax2[2] = joint->axis2[2];
  }
  dCROSS (b,=,ax1,ax2);
  dReal k = info->fps * info->erp;
  info->cu[3] = k * dDOT(b,p);
  info->cu[4] = k * dDOT(b,q);
}


static void hingeSetAnchor (dxJointHinge *joint, dReal x, dReal y, dReal z)
{
  setAnchors (joint,x,y,z,joint->anchor1,joint->anchor2);
}


static void hingeSetAxis (dxJointHinge *joint, dReal x, dReal y, dReal z)
{
  setAxes (joint,x,y,z,joint->axis1,joint->axis2);
}


static void hingeGetAnchor (dxJointHinge *joint, dVector3 result)
{
  getAnchor (joint,result,joint->anchor1);
}


static void hingeGetAxis (dxJointHinge *joint, dVector3 result)
{
  getAxis (joint,result,joint->axis1);
}


dxJoint::Vtable dhinge_vtable = {
  sizeof(dxJointHinge),
  (dxJoint::init_fn*) hingeInit,
  (dxJoint::getInfo1_fn*) hingeGetInfo1,
  (dxJoint::getInfo2_fn*) hingeGetInfo2,
  (dxJoint::setAnchor_fn*) hingeSetAnchor,
  (dxJoint::setAxis_fn*) hingeSetAxis,
  (dxJoint::getAnchor_fn*) hingeGetAnchor,
  (dxJoint::getAxis_fn*) hingeGetAxis};

//****************************************************************************
// slider

static void sliderInit (dxJointSlider *j)
{
  dSetZero (j->axis1,4);
  dSetZero (j->qrel,4);
  dSetZero (j->offset,4);
}


static void sliderGetInfo1 (dxJointSlider *j, dxJoint::Info1 *info)
{
  info->nub = 5;
  info->nlcp = 0;
}


static void sliderGetInfo2 (dxJointSlider *joint, dxJoint::Info2 *info)
{
  int i,s = info->rowskip;
  int s2=2*s,s3=3*s,s4=4*s;

  // pull out pos and R for both bodies. also get the `connection'
  // vector pos2-pos1.

  dReal *pos1,*pos2,*R1,*R2;
  dVector3 c;
  pos1 = joint->node[0].body->pos;
  R1 = joint->node[0].body->R;
  if (joint->node[1].body) {
    pos2 = joint->node[1].body->pos;
    R2 = joint->node[1].body->R;
    for (i=0; i<3; i++) c[i] = pos2[i] - pos1[i];
  }
  else {
    pos2 = 0;
    R2 = 0;
  }

  // 3 rows to make body rotations equal
  info->J1au[0] = 1;
  info->J1au[s+1] = 1;
  info->J1au[s2+2] = 1;
  if (joint->node[1].body) {
    info->J2au[0] = -1;
    info->J2au[s+1] = -1;
    info->J2au[s2+2] = -1;
  }

  // remaining two rows. we want: vel2 = vel1 + w1 x c ... but this would
  // result in three equations, so we project along the planespace vectors
  // so that sliding along the slider axis is disregarded. for symmetry we
  // also substitute (w1+w2)/2 for w1, as w1 is supposed to equal w2.

  dVector3 ax1;	// joint axis in global coordinates (unit length)
  dVector3 p,q;	// plane space of ax1
  dMULTIPLY0_331 (ax1,R1,joint->axis1);
  dPlaneSpace (ax1,p,q);
  if (joint->node[1].body) {
    dVector3 tmp;
    dCROSS (tmp, = REAL(0.5) * ,c,p);
    for (i=0; i<3; i++) info->J2au[s3+i] = tmp[i];
    for (i=0; i<3; i++) info->J2au[s3+i] = tmp[i];
    dCROSS (tmp, = REAL(0.5) * ,c,q);
    for (i=0; i<3; i++) info->J2au[s4+i] = tmp[i];
    for (i=0; i<3; i++) info->J2au[s4+i] = tmp[i];
    for (i=0; i<3; i++) info->J2lu[s3+i] = -p[i];
    for (i=0; i<3; i++) info->J2lu[s4+i] = -q[i];
  }
  for (i=0; i<3; i++) info->J1lu[s3+i] = p[i];
  for (i=0; i<3; i++) info->J1lu[s4+i] = q[i];

  // compute the right hand side. the first three elements will result in
  // relative angular velocity of the two bodies - this is set to bring them
  // back into alignment. the correcting angular velocity is 
  //   |angular_velocity| = angle/time = erp*theta / stepsize
  //                      = (erp*fps) * theta
  //    angular_velocity  = |angular_velocity| * u
  //                      = (erp*fps) * theta * u
  // where rotation along unit length axis u by theta brings body 2's frame
  // to qrel with respect to body 1's frame. using a small angle approximation
  // for sin(), this gives
  //    angular_velocity  = (erp*fps) * 2 * v
  // where the quaternion of the relative rotation between the two bodies is
  //    q = [cos(theta/2) sin(theta/2)*u] = [s v]

  // get qerr = rotation error between two bodies
  dQuaternion qerr;
  if (joint->node[1].body) {
    dQuaternion qq;
    dQMultiply1 (qq,joint->node[0].body->q,joint->node[1].body->q);
    dQMultiply2 (qerr,qq,joint->qrel);
  }
  else {
    dQMultiply3 (qerr,joint->node[0].body->q,joint->qrel);
  }
  if (qerr[0] < 0) {
    qerr[1] = -qerr[1];		// adjust sign of qerr to make theta small
    qerr[2] = -qerr[2];
    qerr[3] = -qerr[3];
  }
  dVector3 e;
  dMULTIPLY0_331 (e,joint->node[0].body->R,qerr+1); // @@@ bad SIMD padding!
  dReal k = info->fps * info->erp;
  info->cu[0] = 2*k * e[0];
  info->cu[1] = 2*k * e[1];
  info->cu[2] = 2*k * e[2];

  // compute last two elements of right hand side. we want to align the offset
  // point (in body 2's frame) with the center of body 1.
  if (joint->node[1].body) {
    dVector3 ofs;		// offset point in global coordinates
    dMULTIPLY0_331 (ofs,R2,joint->offset);
    for (i=0; i<3; i++) c[i] += ofs[i];
    info->cu[3] = k * dDOT(p,c);
    info->cu[4] = k * dDOT(q,c);
  }
  else {
    dVector3 ofs;		// offset point in global coordinates
    for (i=0; i<3; i++) ofs[i] = joint->offset[i] - pos1[i];
    info->cu[3] = k * dDOT(p,ofs);
    info->cu[4] = k * dDOT(q,ofs);
  }
}


static void sliderSetAxis (dxJointSlider *joint, dReal x, dReal y, dReal z)
{
  int i;
  setAxes (joint,x,y,z,joint->axis1,0);

  if (joint->node[1].body) {
    // compute relative rotation body1 -> body2
    dQMultiply2 (joint->qrel,joint->node[1].body->q,joint->node[0].body->q);

    // compute center of body1 w.r.t body 2
    dVector3 c;
    for (i=0; i<3; i++)
      c[i] = joint->node[0].body->pos[i] - joint->node[1].body->pos[i];
    dMULTIPLY1_331 (joint->offset,joint->node[1].body->R,c);
  }
  else {
    for (i=0; i<4; i++) joint->qrel[i] = joint->node[0].body->q[i];
    for (i=0; i<3; i++) joint->offset[i] = joint->node[0].body->pos[i];
  }
}


static void sliderGetAxis (dxJointSlider *joint, dVector3 result)
{
  getAxis (joint,result,joint->axis1);
}


dxJoint::Vtable dslider_vtable = {
  sizeof(dxJointSlider),
  (dxJoint::init_fn*) sliderInit,
  (dxJoint::getInfo1_fn*) sliderGetInfo1,
  (dxJoint::getInfo2_fn*) sliderGetInfo2,
  0,
  (dxJoint::setAxis_fn*) sliderSetAxis,
  0,
  (dxJoint::getAxis_fn*) sliderGetAxis};

//****************************************************************************
// contact

static void contactInit (dxJointContact *j)
{
  dSetZero (j->contact.pos,4);
  dSetZero (j->contact.normal,4);
  j->contact.depth = 0;
}


static void contactGetInfo1 (dxJointContact *j, dxJoint::Info1 *info)
{
  info->nub = 0;
  info->nlcp = 1;
}


static void contactGetInfo2 (dxJointContact *j, dxJoint::Info2 *info)
{
  int i; // s = info->rowskip;

  // c1,c2 = contact points with respect to body PORs
  dVector3 c1,c2;
  for (i=0; i<3; i++) c1[i] = j->contact.pos[i] - j->node[0].body->pos[i];

  // set jacobian
  info->J1ll[0] = j->contact.normal[0];
  info->J1ll[1] = j->contact.normal[1];
  info->J1ll[2] = j->contact.normal[2];
  dCROSS (info->J1al,=  ,c1,j->contact.normal);
  if (j->node[1].body) {
    for (i=0; i<3; i++) c2[i] = j->contact.pos[i] - j->node[1].body->pos[i];
    info->J2ll[0] = -j->contact.normal[0];
    info->J2ll[1] = -j->contact.normal[1];
    info->J2ll[2] = -j->contact.normal[2];
    dCROSS (info->J2al,= -,c2,j->contact.normal);
  }

  // set right hand side
  dReal k = info->fps * info->erp;
  if (j->flags & dJOINT_REVERSE) {
    info->cl[0] = k*j->contact.depth;
  }
  else {
    info->cl[0] = -k*j->contact.depth;
  }

  // set LCP limits
  info->lo[0] = 0;
  info->hi[0] = dInfinity;
}


dxJoint::Vtable dcontact_vtable = {
  sizeof(dxJointContact),
  (dxJoint::init_fn*) contactInit,
  (dxJoint::getInfo1_fn*) contactGetInfo1,
  (dxJoint::getInfo2_fn*) contactGetInfo2,
  0,0,0,0};
