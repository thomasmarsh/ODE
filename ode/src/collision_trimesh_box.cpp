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
//
// This file contains some code from Magic Software, originally by Nolan
// Walker. That code is available under a Free Source License Agreement
// that can be found at http://www.magic-software.com/License/free.pdf

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"

#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

struct LineContactSet{
    dVector3 Points[8];

	int Count;
};

const dReal Epsilon = REAL(1e-08);

static void ClipConvexPolygonAgainstPlane(const dVector3 N, dReal C, LineContactSet& Contacts){
    // The input vertices are assumed to be in counterclockwise order.  The
    // ordering is an invariant of this function.
	
    // test on which side of line are the vertices
    int Positive = 0, Negative = 0, PIndex = -1;
    int Quantity = Contacts.Count;
	
    dReal Test[8];
    for (int i = 0; i < Contacts.Count; i++){
        // An epsilon is used here because it is possible for the dot product
        // and fC to be exactly equal to each other (in theory), but differ
        // slightly because of floating point problems.  Thus, add a little
        // to the test number to push actually equal numbers over the edge
        // towards the positive.
		
        // NOLAN: This should probably be somehow a relative tolerance, and
        // I don't think multiplying by the constant is somehow the best
        // way to do this.
        Test[i] = dDOT(N, Contacts.Points[i]) - C + dFabs(C) * Epsilon;
		
        if (Test[i] >= REAL(0.0)){
            Positive++;
            if (PIndex < 0){
                PIndex = i;
			}
        }
        else Negative++;
    }
	
	if (Positive > 0){
		if (Negative > 0){
			// plane transversely intersects polygon
			dVector3 CV[8];
			int CQuantity = 0, Cur, Prv;
			dReal T;
			
			if (PIndex > 0){
				// first clip vertex on line
				Cur = PIndex;
				Prv = Cur - 1;
				T = Test[Cur] / (Test[Cur] - Test[Prv]);
				CV[CQuantity][0] = Contacts.Points[Cur][0] + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
				CV[CQuantity][1] = Contacts.Points[Cur][1] + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
				CV[CQuantity][2] = Contacts.Points[Cur][2] + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
				CV[CQuantity][3] = Contacts.Points[Cur][3] + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
				CQuantity++;
				
				// vertices on positive side of line
				while (Cur < Quantity && Test[Cur] >= REAL(0.0)){
					CV[CQuantity][0] = Contacts.Points[Cur][0];
					CV[CQuantity][1] = Contacts.Points[Cur][1];
					CV[CQuantity][2] = Contacts.Points[Cur][2];
					CV[CQuantity][3] = Contacts.Points[Cur][3];
					CQuantity++;
					Cur++;
				}
				
				// last clip vertex on line
				if (Cur < Quantity){
					Prv = Cur - 1;
				}
				else{
					Cur = 0;
					Prv = Quantity - 1;
				}
				T = Test[Cur] / (Test[Cur] - Test[Prv]);
				CV[CQuantity][0] = Contacts.Points[Cur][0] + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
				CV[CQuantity][1] = Contacts.Points[Cur][1] + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
				CV[CQuantity][2] = Contacts.Points[Cur][2] + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
				CV[CQuantity][3] = Contacts.Points[Cur][3] + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
				CQuantity++;
			}
			else{  // iPIndex is 0
				// vertices on positive side of line
				Cur = 0;
				while (Cur < Quantity && Test[Cur] >= REAL(0.0)){
					CV[CQuantity][0] = Contacts.Points[Cur][0];
					CV[CQuantity][1] = Contacts.Points[Cur][1];
					CV[CQuantity][2] = Contacts.Points[Cur][2];
					CV[CQuantity][3] = Contacts.Points[Cur][3];
					CQuantity++;
					Cur++;
				}
				
				// last clip vertex on line
				Prv = Cur - 1;
				T = Test[Cur] / (Test[Cur] - Test[Prv]);
				CV[CQuantity][0] = Contacts.Points[Cur][0] + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
				CV[CQuantity][1] = Contacts.Points[Cur][1] + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
				CV[CQuantity][2] = Contacts.Points[Cur][2] + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
				CV[CQuantity][3] = Contacts.Points[Cur][3] + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
				CQuantity++;
				
				// skip vertices on negative side
				while (Cur < Quantity && Test[Cur] < REAL(0.0)){
					Cur++;
				}
				
				// first clip vertex on line
				if (Cur < Quantity){
					Prv = Cur - 1;
					T = Test[Cur] / (Test[Cur] - Test[Prv]);
					CV[CQuantity][0] = Contacts.Points[Cur][0] + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
					CV[CQuantity][1] = Contacts.Points[Cur][1] + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
					CV[CQuantity][2] = Contacts.Points[Cur][2] + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
					CV[CQuantity][3] = Contacts.Points[Cur][3] + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
					CQuantity++;
					
					// vertices on positive side of line
					while (Cur < Quantity && Test[Cur] >= REAL(0.0)){
						CV[CQuantity][0] = Contacts.Points[Cur][0];
						CV[CQuantity][1] = Contacts.Points[Cur][1];
						CV[CQuantity][2] = Contacts.Points[Cur][2];
						CV[CQuantity][3] = Contacts.Points[Cur][3];
						CQuantity++;
						Cur++;
					}
				}
				else{
					// iCur = 0
					Prv = Quantity - 1;
					T = Test[0] / (Test[0] - Test[Prv]);
					CV[CQuantity][0] = Contacts.Points[0][0] + T * (Contacts.Points[Prv][0] - Contacts.Points[0][0]);
					CV[CQuantity][1] = Contacts.Points[0][1] + T * (Contacts.Points[Prv][1] - Contacts.Points[0][1]);
					CV[CQuantity][2] = Contacts.Points[0][2] + T * (Contacts.Points[Prv][2] - Contacts.Points[0][2]);
					CV[CQuantity][3] = Contacts.Points[0][3] + T * (Contacts.Points[Prv][3] - Contacts.Points[0][3]);
					CQuantity++;
				}
			}
			Quantity = CQuantity;
			memcpy(Contacts.Points, CV, CQuantity * sizeof(dVector3));
        }
		// else polygon fully on positive side of plane, nothing to do
		
		Contacts.Count = Quantity;
	}
	else Contacts.Count = 0;	// This should not happen, but for safety
}

static bool FindTriBoxIntersection(const dVector3 Tri[3], const dVector4 Planes[6], LineContactSet& Contacts){	
    Contacts.Count = 3;
	memcpy(Contacts.Points, Tri, 3 * sizeof(dVector3));
	
	for (int i = 0; i < 6; i++){
		ClipConvexPolygonAgainstPlane(Planes[i], Planes[i][3], Contacts);
	}
	return Contacts.Count > 0;
}
	
static void ComputeVertices(const dVector3 Center, const dVector3 Extents, const dMatrix3 Matrix, dVector3 Vertices[8]){
	dVector3 Axis[3];
	Decompose(Matrix, Axis);

	dVector3 TransExtents[3];
	for (int i = 0; i < 3; i++){
		TransExtents[i][0] = Axis[i][0] * Extents[i];
		TransExtents[i][1] = Axis[i][1] * Extents[i];
		TransExtents[i][2] = Axis[i][2] * Extents[i];
		TransExtents[i][3] = Axis[i][3] * Extents[i];
	}

#define COMPUTEVERTEX(a, op1, b, op2, c, op3, d, op4, e)	\
	a[0] op1 b[0] op2 c[0] op3 d[0] op4 e[0];				\
	a[1] op1 b[1] op2 c[1] op3 d[1] op4 e[1];				\
	a[2] op1 b[2] op2 c[2] op3 d[2] op4 e[2];				\
	a[3] op1 REAL(0.0);

	COMPUTEVERTEX(Vertices[0], =, Center, -, TransExtents[0], +, TransExtents[1], +, TransExtents[2]);
	COMPUTEVERTEX(Vertices[1], =, Center, +, TransExtents[0], +, TransExtents[1], +, TransExtents[2]);
	COMPUTEVERTEX(Vertices[2], =, Center, +, TransExtents[0], -, TransExtents[1], +, TransExtents[2]);
	COMPUTEVERTEX(Vertices[3], =, Center, -, TransExtents[0], -, TransExtents[1], +, TransExtents[2]);
	COMPUTEVERTEX(Vertices[4], =, Center, -, TransExtents[0], +, TransExtents[1], -, TransExtents[2]);
	COMPUTEVERTEX(Vertices[5], =, Center, +, TransExtents[0], +, TransExtents[1], -, TransExtents[2]);
	COMPUTEVERTEX(Vertices[6], =, Center, +, TransExtents[0], -, TransExtents[1], -, TransExtents[2]);
	COMPUTEVERTEX(Vertices[7], =, Center, -, TransExtents[0], -, TransExtents[1], -, TransExtents[2]);
#undef COMPUTEVERTEX
}

static dReal PointLineDist(const dVector3 Point, const dVector3 Origin, const dVector3 Direction, dReal& T){
	dVector3 Diff;
	Diff[0] = Point[0] - Origin[0];
	Diff[1] = Point[1] - Origin[1];
	Diff[2] = Point[2] - Origin[2];
	Diff[3] = Point[3] - Origin[3];

	T = dDOT(Diff, Direction);

	if (T <= REAL(0.0)){
		T = REAL(0.0);
	}
	else{
        dReal MagSq = dDOT(Direction, Direction);
        if (T >= MagSq){
            T = REAL(1.0);
            Diff[0] -= Direction[0];
			Diff[1] -= Direction[1];
			Diff[2] -= Direction[2];
			Diff[3] -= Direction[3];
        }
        else{
            T /= MagSq;

			Diff[0] -= T * Direction[0];
			Diff[1] -= T * Direction[1];
			Diff[2] -= T * Direction[2];
			Diff[3] -= T * Direction[3];
        }
    }
    return dSqrt(dDOT(Diff, Diff));
}

extern int dcBoxPrimCount;
extern int dcPrimPrimCount;
extern int dcTCSucceed;
extern int dcTCFail;

int dCollideBTL(dxGeom* g1, dxGeom* BoxGeom, int Flags, dContactGeom* Contacts, int Stride){
	dxTriMesh* TriMesh = (dxTriMesh*)g1;

	const dVector3& TLPosition = *(const dVector3*)dGeomGetPosition(TriMesh);
	const dMatrix3& TLRotation = *(const dMatrix3*)dGeomGetRotation(TriMesh);

	OBBCollider& Collider = TriMesh->_OBBCollider;

	// Get box
	const dVector3& BoxCenter = *(const dVector3*)dGeomGetPosition(BoxGeom);

	dVector3 BoxExtents;
	dGeomBoxGetLengths(BoxGeom, BoxExtents);

	BoxExtents[0] /= 2;
	BoxExtents[1] /= 2;
	BoxExtents[2] /= 2;
	BoxExtents[3] /= 2;

	const dMatrix3& BoxRotation = *(const dMatrix3*)dGeomGetRotation(BoxGeom);

	// Make OBB
 	OBB Box;
	Box.mCenter.x = BoxCenter[0];
	Box.mCenter.y = BoxCenter[1];
	Box.mCenter.z = BoxCenter[2];

	Box.mExtents.x = BoxExtents[0];
	Box.mExtents.y = BoxExtents[1];
	Box.mExtents.z = BoxExtents[2];

	Box.mRot.m[0][0] = BoxRotation[0];
	Box.mRot.m[1][0] = BoxRotation[1];
	Box.mRot.m[2][0] = BoxRotation[2];

	Box.mRot.m[0][1] = BoxRotation[4];
	Box.mRot.m[1][1] = BoxRotation[5];
	Box.mRot.m[2][1] = BoxRotation[6];

	Box.mRot.m[0][2] = BoxRotation[8];
	Box.mRot.m[1][2] = BoxRotation[9];
	Box.mRot.m[2][2] = BoxRotation[10];

	Matrix4x4 amatrix;
	Matrix4x4 BoxMatrix = MakeMatrix(BoxCenter, BoxRotation, amatrix);

	Matrix4x4 InvBoxMatrix;
	InvertPRMatrix(InvBoxMatrix, BoxMatrix);

	// TC results
	dxTriMesh::BoxTC* BoxTC = 0;
	for (int i = 0; i < TriMesh->BoxTCCache.size(); i++){
		if (TriMesh->BoxTCCache[i].Geom == BoxGeom){
			BoxTC = &TriMesh->BoxTCCache[i];
			break;
		}
	}
	if (!BoxTC){
		TriMesh->BoxTCCache.push(dxTriMesh::BoxTC());

		BoxTC = &TriMesh->BoxTCCache[TriMesh->BoxTCCache.size() - 1];
		BoxTC->Geom = BoxGeom;
		BoxTC->FatCoeff = 5.0f;
	}

	// Intersect
	Collider.Collide(*BoxTC, Box, TriMesh->Data->BVTree, null, &MakeMatrix(TLPosition, TLRotation, amatrix));
	
	// Retrieve data
	int TriCount = Collider.GetNbTouchedPrimitives();
	const int* Triangles = (const int*)Collider.GetTouchedPrimitives();

	if (TriCount != 0){
		if (TriMesh->ArrayCallback != null){
			TriMesh->ArrayCallback(TriMesh, BoxGeom, Triangles, TriCount);
		}

		// Decompose transformation
		dVector3 Axis[3];
		Decompose(BoxRotation, Axis);

		// Compute box planes
		dVector4 Planes[6];
		int PlaneCounter = 0;
		
		for (dReal Dir = REAL(-1.0); Dir <= REAL(1.0); Dir += REAL(2.0)){
			for (int Side = 0; Side < 3; Side++){
				Planes[PlaneCounter][0] = Dir * Axis[Side][0];
				Planes[PlaneCounter][1] = Dir * Axis[Side][1];
				Planes[PlaneCounter][2] = Dir * Axis[Side][2];
				
				dVector3 Temp;
				Temp[0] = BoxCenter[0] - Dir * BoxExtents[Side] * Axis[Side][0];
				Temp[1] = BoxCenter[1] - Dir * BoxExtents[Side] * Axis[Side][1];
				Temp[2] = BoxCenter[2] - Dir * BoxExtents[Side] * Axis[Side][2];
				Temp[3] = BoxCenter[3] - Dir * BoxExtents[Side] * Axis[Side][3];
				
				Planes[PlaneCounter][3] = Dir * dDOT(Axis[Side], Temp);
				
				PlaneCounter++;
			}
		}

		Point Trans;
		InvBoxMatrix.GetTrans(Trans);

		Matrix3x3 InvBoxMatrix3 = (Matrix3x3)InvBoxMatrix;

		int OutTriCount = 0;
		for (int i = 0; i < TriCount; i++){
			const int& TriIndex = Triangles[i];

			if (!Callback(TriMesh, BoxGeom, TriIndex)) continue;

			dVector3 dv[3];
			FetchTriangle(TriMesh, TriIndex, TLPosition, TLRotation, dv);

			// Apply some magic to find collision points
			LineContactSet InContacts;
			if (FindTriBoxIntersection(dv, Planes, InContacts)){
				// Compute triangle plane
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

				dVector4 Plane;
				dCROSS(Plane, =, vu, vv);
				Plane[3] = dDOT(Plane, dv[0]);

				// Normalize the plane
				dReal Area = dSqrt(dDOT(Plane, Plane));	// Use this later for epsilons
				Plane[0] /= Area;
				Plane[1] /= Area;
				Plane[2] /= Area;
				Plane[3] /= Area;

				// Compute box vertices
				dVector3 Vertices[8];
				ComputeVertices(BoxCenter, BoxExtents, BoxRotation, Vertices);

				// Find the smallest penetration depth of the box and a plane going through the triangle.
				// This needs to be optimized.
				dReal Depth = 0;
				for (int j = 0; j < 8; j++){
					dReal Behind = dDOT(Plane, Vertices[j]) - Plane[3];

					if (Behind < REAL(0.00001)){	// Should be relative. How?
						Depth = dcMAX(Depth, -Behind);
					}
				}
				Depth = dcMAX(Depth, REAL(0.0));	// For small inaccuracies

				// Shouldnt we remove some irrelevant points? We do not care for >3 contacts.
				for (int j = 0; j < InContacts.Count; j++){
					if (OutTriCount >= Flags) break;
					dContactGeom* Contact = SAFECONTACT(Flags, Contacts, OutTriCount, Stride);

					Contact->pos[0] = InContacts.Points[j][0];
					Contact->pos[1] = InContacts.Points[j][1];
					Contact->pos[2] = InContacts.Points[j][2];
					Contact->pos[3] = InContacts.Points[j][3];

					// Always use triangle's normal. For edge contacts this is not preferable.
					// How do we determine a correct edge-contact normal?
					Contact->normal[0] = -Plane[0];
					Contact->normal[1] = -Plane[1];
					Contact->normal[2] = -Plane[2];
					Contact->normal[3] = REAL(0.0);

					// Test all 3 triangle edges to see if we can find a smaller penetration.
					// This is good for sharp edges. Doesnt this interfere with sliding objects? Hopefully not!
					dReal MinDepth = Depth;
					for (int k = 0; k < 3; k++){
						const dVector3& Origin = dv[k];
						dVector3 Direction;
						Direction[0] = dv[(k + 1) % 3][0] - Origin[0];
						Direction[1] = dv[(k + 1) % 3][1] - Origin[1];
						Direction[2] = dv[(k + 1) % 3][2] - Origin[2];
						Direction[3] = dv[(k + 1) % 3][3] - Origin[3];
						
						dReal T;	// Maybe use this to adjust contact position? Sounds logical.
						dReal Dist = PointLineDist(Contact->pos, Origin, Direction, T);

						if (Dist <= MinDepth){
							MinDepth = Dist;

							/*if (MinDepth > REAL(0.0001)){	// Is this any help?
								Contact->normal[0] = Contact->pos[0] - (Origin[0] + Direction[0] * T);
								Contact->normal[1] = Contact->pos[1] - (Origin[1] + Direction[1] * T);
								Contact->normal[2] = Contact->pos[2] - (Origin[2] + Direction[2] * T);
								Contact->normal[3] = REAL(0.0);
								dNormalize3(Contact->normal);
							}*/
						}
					}
					
					Contact->depth = MinDepth;
					Contact->g1 = (dxGeom*)1;	// WARNING: Large hack. Using g1 as a counter.

					int Index;
					for (Index = 0; Index < OutTriCount; Index++){
						dContactGeom* TempContact = SAFECONTACT(Flags, Contacts, Index, Stride);
						
						dVector3 Diff;
						Diff[0] = TempContact->pos[0] - Contact->pos[0];
						Diff[1] = TempContact->pos[1] - Contact->pos[1];
						Diff[2] = TempContact->pos[2] - Contact->pos[2];
						Diff[3] = TempContact->pos[3] - Contact->pos[3];
						
						dReal DistSq = dDOT(Diff, Diff);
						
						if (DistSq < 0.001){//BoxRadius * REAL(0.1)){	// Tweak?
							break;
						}
					}
					if (Index != OutTriCount){
						dContactGeom* TempContact = SAFECONTACT(Flags, Contacts, Index, Stride);
						TempContact->normal[0] += Contact->normal[0];
						TempContact->normal[1] += Contact->normal[1];
						TempContact->normal[2] += Contact->normal[2];
						TempContact->normal[3] += Contact->normal[3];

						TempContact->depth += Contact->depth;
						Contact->g1 = (dxGeom*)(((char*)Contact->g1) + 1);
					}
					else OutTriCount++;
				}
			}
		}
		for (int i = 0; i < OutTriCount; i++){	// Now normalize normals
			dContactGeom* Contact = SAFECONTACT(Flags, Contacts, i, Stride);
			dNormalize3(Contact->normal);

			Contact->depth /= (int&)Contact->g1;	// Hacking again.

			Contact->g1 = TriMesh;
			Contact->g2 = BoxGeom;
		}
		return OutTriCount;
	}
	return 0;
}
