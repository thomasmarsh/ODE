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

// TriMesh code by Erwin de Vries.

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"

#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

int dCollideRTL(dxGeom* g1, dxGeom* RayGeom, int Flags, dContactGeom* Contacts, int Stride){
	dxTriMesh* TriMesh = (dxTriMesh*)g1;

	const dVector3& TLPosition = *(const dVector3*)dGeomGetPosition(TriMesh);
	const dMatrix3& TLRotation = *(const dMatrix3*)dGeomGetRotation(TriMesh);

	RayCollider& Collider = TriMesh->_RayCollider;

	dReal Length = dGeomRayGetLength(RayGeom);

	int FirstContact, BackfaceCull;
	dGeomRayGetParams(RayGeom, &FirstContact, &BackfaceCull);

	Collider.SetClosestHit(FirstContact);
	Collider.SetCulling(BackfaceCull);
	Collider.SetMaxDist(Length);

	dVector3 Origin, Direction;
	dGeomRayGet(RayGeom, Origin, Direction);

	/* Make Ray */
	Ray WorldRay;
	WorldRay.mOrig.x = Origin[0];
	WorldRay.mOrig.y = Origin[1];
	WorldRay.mOrig.z = Origin[2];
	WorldRay.mDir.x = Direction[0];
	WorldRay.mDir.y = Direction[1];
	WorldRay.mDir.z = Direction[2];

	/* Intersect */
	Matrix4x4 amatrix;
	Collider.Collide(WorldRay, TriMesh->Data->BVTree, &MakeMatrix(TLPosition, TLRotation, amatrix));

	/* Retrieve data */
	int TriCount = TriMesh->Faces.GetNbFaces();
	
	if (TriCount != 0){
		const CollisionFace* Faces = TriMesh->Faces.GetFaces();

		int OutTriCount = 0;
		for (int i = 0; i < TriCount; i++){
			if (TriMesh->RayCallback == null || TriMesh->RayCallback(TriMesh, RayGeom, Faces[i].mFaceID, Faces[i].mU, Faces[i].mV)){
				const int& TriIndex = Faces[i].mFaceID;

				if (!Callback(TriMesh, RayGeom, TriIndex)) continue;

				dContactGeom* Contact = SAFECONTACT(Flags, Contacts, OutTriCount, Stride);

				dVector3 dv[3];
				FetchTriangle(TriMesh, TriIndex, TLPosition, TLRotation, dv);

				dVector3 Temp;
				GetPointFromBarycentric(dv, Faces[i].mU, Faces[i].mV, Temp);

				dMULTIPLY0_331(Contact->pos, TLRotation, Temp);
				Contact->pos[0] += TLPosition[0];
				Contact->pos[1] += TLPosition[1];
				Contact->pos[2] += TLPosition[2];
				Contact->pos[3] = REAL(0.0);

				dVector3 vu;
				vu[0] = dv[1][0] - dv[0][0];
				vu[1] = dv[1][1] - dv[0][1];
				vu[2] = dv[1][2] - dv[0][2];
				vu[3] = REAL(0.0);
				
				dVector3 vv;
				vv[0] = dv[2][0] - dv[0][0];
				vv[1] = dv[2][1] - dv[0][1];
				vv[2] = dv[2][2] - dv[0][2];
				vv[3] = REAL(0.0);

				dCROSS(Contact->normal, =, vv, vu);	// Reversed

				float T = -(dDOT(Contact->normal, Origin) - dDOT(Contact->normal, dv[0])) / dDOT(Contact->normal, Direction);

				dNormalize3(Contact->normal);

				Contact->depth = T;		// was Length - T, but that seemed wrong
				Contact->g1 = TriMesh;
				Contact->g2 = RayGeom;
				
				OutTriCount++;
			}
		}
		return OutTriCount;
	}
	else return 0;
}
