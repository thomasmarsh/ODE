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
	(c) 2004 Vadim Macagon
*/
 
#include <ode/odemath.h>
#include <ode/collision.h>
#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

/*
  For the time being at most one contact will be generated.
*/

int dCollideCCTL( dxGeom* g1, dxGeom* capsuleGeom, int flags, 
                  dContactGeom* contacts, int skip )
{
    int retval = 0;
  
    dxTriMesh* triMesh = (dxTriMesh*)g1;
    dContactGeom* contact = SAFECONTACT( flags, contacts, 0, skip );
    memset( contact, 0, sizeof(dContactGeom) );

    // Init
    const dVector3& meshPosition = *(const dVector3*)dGeomGetPosition( triMesh );
    const dMatrix3& meshRotation = *(const dMatrix3*)dGeomGetRotation( triMesh );

    LSSCollider& collider = dxTriMesh::_LSSCollider;
  
    const dVector3& capPosition = *(const dVector3*)dGeomGetPosition( capsuleGeom );
    const dMatrix3& capRotation = *(const dMatrix3*)dGeomGetRotation( capsuleGeom );
    
    dReal capRad, capLength;
    dGeomCCylinderGetParams( capsuleGeom, &capRad, &capLength );
    // obtain capsule in model space
    dVector3 tempCapOrigin = { REAL(0.0), REAL(0.0), capLength * REAL(0.5), REAL(0.0) };
    dVector3 tempCapEnd = { REAL(0.0), REAL(0.0), -capLength * REAL(0.5), REAL(0.0) };
    // transform capsule to world space
    dVector3 capOrigin;
    dVector3 capEnd;
    TransformVector3( tempCapOrigin, capRotation, capPosition, capOrigin );
    TransformVector3( tempCapEnd, capRotation, capPosition, capEnd );
    // construct opcode capsule
    Point p1 ( capOrigin[0], capOrigin[1], capOrigin[2] );
    Point p2 ( capEnd[0], capEnd[1], capEnd[2] );
    Segment capSeg(p1,p2);
    LSS opcodeCapsule( capSeg, capRad );
  
    Matrix4x4 meshTransform;
    MakeMatrix( meshPosition, meshRotation, meshTransform );
  
    // TC results
    if ( triMesh->doCCylinderTC )
    { 
        dxTriMesh::CCylinderTC* capsuleTC = 0;
  
        for ( int i = 0; i < triMesh->CCylinderTCCache.size(); i++ )
        {
            if ( triMesh->CCylinderTCCache[i].Geom == capsuleGeom )
            {
                capsuleTC = &(triMesh->CCylinderTCCache[i]);
                break;
            }
        }
        if ( !capsuleTC )
        {
            triMesh->CCylinderTCCache.push( dxTriMesh::CCylinderTC() );
            capsuleTC = &(triMesh->CCylinderTCCache[triMesh->CCylinderTCCache.size() - 1]);
            capsuleTC->Geom = capsuleGeom;
        }
    
        // Intersect
        collider.SetTemporalCoherence(true);
        collider.Collide( *capsuleTC, opcodeCapsule, triMesh->Data->BVTree,
                          0/*&capTransform*/, &meshTransform );
    }
    else
    {
        collider.SetTemporalCoherence(false);
        collider.Collide( dxTriMesh::defaultCCylinderCache, opcodeCapsule, 
                          triMesh->Data->BVTree, 
                          0/*&capTransform*/, &meshTransform );
    }
       
    int triCount = collider.GetNbTouchedPrimitives();
  
    if ( 0 != triCount )
    {
        const int* triangles = (const int*)collider.GetTouchedPrimitives();
    
        if ( triMesh->ArrayCallback )
            triMesh->ArrayCallback( triMesh, capsuleGeom, triangles, triCount );
    
        int outTriCount = 0;
        dVector3 triVertices[3];
    
        for ( int i = 0; i < triCount; i++ )
        {
            // get touched triangle in world space
            FetchTriangle( triMesh, triangles[i], meshPosition, meshRotation, 
                           triVertices );
    
            dVector3& v0 = triVertices[0];
            dVector3& v1 = triVertices[1];
            dVector3& v2 = triVertices[2];

            dVector3 e1;
            e1[0] = v1[0] - v0[0];
            e1[1] = v1[1] - v0[1];
            e1[2] = v1[2] - v0[2];
      
            dVector3 e2;
            e2[0] = v2[0] - v0[0];
            e2[1] = v2[1] - v0[1];
            e2[2] = v2[2] - v0[2];
                 
            dReal t;  // t value of a point along the capsule's axis
            dReal u, v; // berycentric coords
            if ( !IntersectCapsuleTri( capOrigin, capEnd, capRad, v0, e1, e2, 
                                       0, &t, &u, &v ) )
            {
                continue; // capsule doesn't overlap with triangle
            }
            dReal w = REAL(1.0) - u - v;
                 
            // plane normal will be reverse of the corresponding triangle's normal
            dVector4 plane;
            dCROSS( plane, =, e2, e1 ); // compute the reversed normal
            plane[3] = dDOT( plane, v0 ); // compute 'd'.
            // normalize the plane
            dReal area = dSqrt( dDOT( plane, plane ) );
            plane[0] /= area;
            plane[1] /= area;
            plane[2] /= area;
            plane[3] /= area;
      
            Point opPoint;
            opcodeCapsule.ComputePoint( opPoint, t );
            dVector3 p = { opPoint.x, opPoint.y, opPoint.z, 0 };
            dReal depth = dDOT( plane, p ) - plane[3] + capRad;
      
            if ( depth >= REAL(0.0) )
            {
                // store the contact
                dVector3 contactPos;
                contactPos[0] = (v0[0] * w) + (v1[0] * u) + (v2[0] * v);
                contactPos[1] = (v0[1] * w) + (v1[1] * u) + (v2[1] * v);
                contactPos[2] = (v0[2] * w) + (v1[2] * u) + (v2[2] * v);
                Vector3Add( contactPos, contact->pos, contact->pos );
                contact->normal[0] += plane[0] * depth;
                contact->normal[1] += plane[1] * depth;
                contact->normal[2] += plane[2] * depth;
                ++outTriCount;
            }
        } // loop: for each triangle
 
        if ( 0 != outTriCount )
        {
            // average out the contacts, normals and determine depths
            dReal rec = REAL(1.0) / dReal(outTriCount);
            Vector3Multiply( contact->pos, rec, contact->pos );
            contact->depth = dSqrt( dDOT( contact->normal, contact->normal ) ) * rec;
            dNormalize3( contact->normal );
            contact->g1 = triMesh;
            contact->g2 = capsuleGeom;
            retval = 1;
        } // if 0 != outTriCount
    } // if 0 != triCount
  
    return retval;
}
