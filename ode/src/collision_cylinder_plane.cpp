/*************************************************************************
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

/*
 * Cylinder-Plane collider by Bram Stolk
 *
 * This testing basically comes down to testing the intersection
 * of the cylinder caps (discs) with the plane.
 * As a special case, we should consider co-planar discs and plane,
 * In which case we simply test for intersection between cyl axis
 * and plane.
 */

#define PRTVEC3(A) \
  fprintf(stderr, #A " %6.3f %6.3f %6.3f\n", A[0],A[1],A[2]);

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include <ode/objects.h>

#include "collision_util.h"	// for dCollideSpheres
#include "collision_kernel.h"	// for dxGeom



// Find the line of intersection between two planes.
// Returns false if planes are parallel
static bool isect_plane_plane
(
  const dVector3 &pos0,
  const dVector3 &nrm0,
  const dVector3 &pos1,
  const dVector3 &nrm1,
  dVector3 &org,
  dVector3 &dir
)
{
  dReal norms_dot = dDOT(nrm0,nrm1);
  if (dFabs(norms_dot) >= (1.0-dEpsilon))
    return false; // planes are parallel
  dReal inv_det = dReal(1.0) / (dReal(1.0) - norms_dot*norms_dot);

  dReal planeconstant0 = dDOT(nrm0, pos0);
  dReal planeconstant1 = dDOT(nrm1, pos1);
  dReal c0 = (planeconstant0 - norms_dot*planeconstant1) * inv_det;
  dReal c1 = (planeconstant1 - norms_dot*planeconstant0) * inv_det;

  org[0] =  c0 * nrm0[0] + c1 * nrm1[0];
  org[1] =  c0 * nrm0[1] + c1 * nrm1[1];
  org[2] =  c0 * nrm0[2] + c1 * nrm1[2];

  dCROSS(dir, =, nrm0, nrm1);
  dNormalize3(dir);
//  PRTVEC3(org);
//  PRTVEC3(dir);
  return true;
}


// Finds a point on the line of intersection between a plane and a disc.
// Direction of intersection (doi) points from disc center to this point.
// Intersection depth is the distance of the point to the edge of the disc.
// We could re-use this func when writing a cyl-cyl collider, probably.
static bool isect_disc_plane
(
  const dVector3 &discpos,
  const dVector3 &discnorm,
  const dVector3 &planepos,
  const dVector3 &planenorm,
  dReal rad,
  dVector3 &poi,		// point of intersection
  dVector3 &doi,		// direction of intersection
  dVector3 &deepest,		// point on disc with deepest penetration of plane
  dReal &intersectiondepth
)
{
  dVector3 org, dir;
  bool planes_do_intersect = isect_plane_plane(discpos,discnorm,planepos,planenorm,org,dir);
  if (!planes_do_intersect)
    return false;

  dIASSERT(dDOT(dir, planenorm)<dEpsilon);

  dVector3 tmp;

  // Determine the closest point on the intersection line to the position of disc
  dVector3Subtract(discpos, org, tmp);
  dReal t0 = dDOT(dir, tmp) / dDOT(dir,dir);

  poi[0] = org[0] + t0 * dir[0];
  poi[1] = org[1] + t0 * dir[1];
  poi[2] = org[2] + t0 * dir[2];

  dVector3Subtract(poi, discpos, doi);
  dReal dist = dVector3Length(doi);

  if (dist >= rad)
    return false;

  intersectiondepth = rad - dist;
  dNormalize3(doi);

  deepest[0] = discpos[0] + rad * doi[0];
  deepest[1] = discpos[1] + rad * doi[1];
  deepest[2] = discpos[2] + rad * doi[2];

//  fprintf(stderr,"disc intersects plane with depth %f\n", intersectiondepth);
  return true;
}
 

// Test code
static bool test_disc_isect()
{
  dVector3 discpos={1,0,3};
  dVector3 discnorm={0,1,0};
  dVector3 planepos={5,10,0};
  dVector3 planenorm={0,0,1};
  dVector3 deepest={0,0,0};
  dReal    radius=4.0;

  dVector3 poi, doi;
  dReal depth;

  return isect_disc_plane(discpos, discnorm, planepos, planenorm, radius, poi, doi, deepest, depth);
}
//static bool dummy=test_disc_isect();


// Finds out which global axis, x y or z, is the most perpendicular one to 
// a given vector.
static int minor_axis(const dVector3 &v, dVector3 &axis)
{
  dVector3 x={1,0,0};
  dVector3 y={0,1,0};
  dVector3 z={0,0,1};
  float dotx = (float)dFabs(dDOT(v,x));
  float doty = (float)dFabs(dDOT(v,y));
  float dotz = (float)dFabs(dDOT(v,z));
  if (dotx <= doty && dotx <= dotz) 
  {
    dVector3Copy(x, axis);
    return 0;
  }
  if (doty <= dotx && doty <= dotz)
  {
    dVector3Copy(y, axis);
    return 1;
  }
  dVector3Copy(z, axis);
  return 2;
}


int dCollideCylinderPlane(dxGeom *cylgeom, dxGeom *planegeom, int flags, dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT ((flags & 0xffff) >= 1);

  int nContacts=0;

  // Get the properties of the cylinder (len+radius)
  dReal radius, length;
  dGeomCylinderGetParams(cylgeom, &radius, &length);
  dVector3 &cylpos = cylgeom->final_posr->pos;
  // Get the properties of the plane
  dVector4 planevec;
  dGeomPlaneGetParams(planegeom, planevec);
  dVector3 planenorm = {planevec[0],planevec[1],planevec[2]};
  dVector3 planepos = {planevec[0],planevec[1],planevec[2]};
  dVector3Scale(planepos, planevec[3]);

  dVector3 axis = 
  {
    cylgeom->final_posr->R[2],
    cylgeom->final_posr->R[6],
    cylgeom->final_posr->R[10]
  };
  dReal axislen = dVector3Length(axis);
  dIASSERT(axislen>=0.9999);
  dIASSERT(axislen<=1.0001);

  dReal hl = length/dReal(2.0);
  dVector3 disc_top_pos = { cylpos[0]+axis[0]*hl, cylpos[1]+axis[1]*hl, cylpos[2]+axis[2]*hl };
  dVector3 disc_bot_pos = { cylpos[0]-axis[0]*hl, cylpos[1]-axis[1]*hl, cylpos[2]-axis[2]*hl };
  dVector3 disc_top_nrm = {  axis[0],  axis[1],  axis[2] };
  dVector3 disc_bot_nrm = { -axis[0], -axis[1], -axis[2] };

  // Discs and plane are parallel?
  // If so, the cyl may be resting on the plane, with one of its discs

  dReal norm_dot_top = dDOT(planenorm, disc_top_nrm);
  dReal norm_dot_bot = dDOT(planenorm, disc_bot_nrm);

  bool face_to_face_top = (norm_dot_top <= -0.999);
  bool face_to_face_bot = (norm_dot_bot <= -0.999);

  if (face_to_face_top || face_to_face_bot)
  {
    dReal dist_top = dPointPlaneDistance(disc_top_pos, planevec);
    if (face_to_face_top && dist_top <= 0)
    {
      dVector3 x;
      minor_axis(axis, x);
      dVector3 y;
      dVector3Cross(axis, x, y);
      dNormalize3(y);
      // y and x are unit, so will their cross product be.
      dVector3Cross(axis, y, x);

      dVector3 p;
      dVector3Scale(x, radius); dVector3Scale(y, radius);
      dContactGeom *Contact;

      // Contact 1
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_top_pos, p);
      dVector3Add(p, x, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_top;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;

      // Contact 2
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_top_pos, p);
      dVector3Add(p, y, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_top;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;

      // Contact 3
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_top_pos, p);
      dVector3Subtract(p, x, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_top;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;

      // Contact 4
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_top_pos, p);
      dVector3Subtract(p, y, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_top;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;
      return nContacts;
    }
    dReal dist_bot = dPointPlaneDistance(disc_bot_pos, planevec);
    if (face_to_face_bot && dist_bot <= 0)
    {
      dVector3 x;
      minor_axis(axis, x);
      dVector3 y;
      dVector3Cross(axis, x, y);
      dNormalize3(y);
      // y and x are unit, so will their cross product be.
      dVector3Cross(axis, y, x);

      dVector3 p;
      dVector3Scale(x, radius); dVector3Scale(y, radius);
      dContactGeom *Contact;

      // Contact 1
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_bot_pos, p);
      dVector3Add(p, x, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_bot;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;

      // Contact 2
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_bot_pos, p);
      dVector3Add(p, y, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_bot;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;

      // Contact 3
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_bot_pos, p);
      dVector3Subtract(p, x, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_bot;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;

      // Contact 4
      Contact = SAFECONTACT(flags, contact, nContacts, skip);
      dVector3Copy(disc_bot_pos, p);
      dVector3Subtract(p, y, Contact->pos);
      Contact->g1 = cylgeom;
      Contact->g2 = planegeom;
      Contact->depth = -dist_bot;
      dVector3Copy(planenorm, Contact->normal);
      nContacts++;
      return nContacts;
    }
    return 0;
  }

  // Test the discs!

  dVector3 poi_top, doi_top;
  dVector3 poi_bot, doi_bot;
  dVector3 deepest_top, deepest_bot;
  dReal depth_top, depth_bot;

  bool top_disc_does_intersect = isect_disc_plane(disc_top_pos, disc_top_nrm, planepos, planenorm, radius,  poi_top, doi_top, deepest_top, depth_top);
  bool bot_disc_does_intersect = isect_disc_plane(disc_bot_pos, disc_bot_nrm, planepos, planenorm, radius,  poi_bot, doi_bot, deepest_bot, depth_bot);

  if (top_disc_does_intersect)
  {
    dContactGeom *Contact = SAFECONTACT(flags, contact, nContacts, skip);
    // Note: ODE convention is: normal points *into* g1
    Contact->g1 = cylgeom;
    Contact->g2 = planegeom;
    dVector3Copy(planenorm, Contact->normal);

    if (dDOT(doi_top,planenorm)>0)
    {
      // Most of the disc is below plane
      deepest_top[0] = disc_top_pos[0] - radius * doi_top[0];
      deepest_top[1] = disc_top_pos[1] - radius * doi_top[1];
      deepest_top[2] = disc_top_pos[2] - radius * doi_top[2];
    }
    dVector3Copy(deepest_top, Contact->pos);
    Contact->depth = -dPointPlaneDistance(deepest_top, planevec);

    dIASSERT(Contact->depth >= 0.0);
    nContacts++;
  }

  if (bot_disc_does_intersect)
  {
    dContactGeom *Contact = SAFECONTACT(flags, contact, nContacts, skip);
    // Note: ODE convention is: normal points *into* g1
    Contact->g1 = cylgeom;
    Contact->g2 = planegeom;
    dVector3Copy(planenorm, Contact->normal);

    if (dDOT(doi_bot,planenorm)>0)
    {
      // Most of the disc is below plane
      deepest_bot[0] = disc_bot_pos[0] - radius * doi_bot[0];
      deepest_bot[1] = disc_bot_pos[1] - radius * doi_bot[1];
      deepest_bot[2] = disc_bot_pos[2] - radius * doi_bot[2];
    }
    dVector3Copy(deepest_bot, Contact->pos);
    Contact->depth = -dPointPlaneDistance(deepest_bot, planevec);

    dIASSERT(Contact->depth >= 0.0);
    nContacts++;
  }

  return nContacts;
}


