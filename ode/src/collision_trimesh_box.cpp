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

static void ComputeVertices(const dVector3 Center, const dVector3 Extents, const dVector3 Axis[3], dVector3 Vertices[8]){
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

static bool PointPlaneIntersect(const dVector3 Origin, const dVector4 Plane){
	dReal k = dDOT(Origin, Plane);
	dReal depth = Plane[3] - k;
	if (depth >= REAL(-0.00001)) {
		return true;
	}
	else return false;
}

static bool LinePlaneIntersect(const dVector3 Origin, const dVector3 Dir, dReal Length, const dVector4 Plane, dVector3 OutPos){
	dReal alpha = Plane[3] - dDOT(Plane, Origin);

	// note: if alpha > 0 the starting point is below the plane
	dReal k = dDOT(Plane, Dir);
	if (k == 0){
		return false;		// line parallel to plane
	}

	alpha /= k;
	if (alpha < 0 || alpha > Length){
		return false;
	}
	OutPos[0] = Origin[0] + alpha * Dir[0];
	OutPos[1] = Origin[1] + alpha * Dir[1];
	OutPos[2] = Origin[2] + alpha * Dir[2];
	return true;
}

// Horrible precision in single precision mode. Did i do something wrong?
// Not in use currently
static dReal PointTriDist(const dVector3 Point, const dVector3 Origin, const dVector3 Edge0, const dVector3 Edge1, dReal& S, dReal& T){
	dVector3 Diff;
	Diff[0] = Origin[0] - Point[0];
	Diff[1] = Origin[1] - Point[1];
	Diff[2] = Origin[2] - Point[2];
	Diff[3] = Origin[3] - Point[3];

	dReal A00 = dDOT(Edge0, Edge0);
    dReal A01 = dDOT(Edge0, Edge1);
    dReal A11 = dDOT(Edge1, Edge1);
    dReal B0 = dDOT(Diff, Edge0);
    dReal B1 = dDOT(Diff, Edge1);
    dReal C = dDOT(Diff, Diff);
    dReal Det = dFabs(A00 * A11 - A01 * A01);
    S = A01 * B1 - A11 * B0;
    T = A01 * B0 - A00 * B1;
    dReal DistSq;

    if (S + T <= Det){
        if (S < REAL(0.0)){
            if (T < REAL(0.0)){  // region 4
                if (B0 < REAL(0.0)){
                    T = REAL(0.0);
                    if (-B0 >= A00){
                        S = 1.0f;
                        DistSq = A00 + REAL(2.0) * B0 + C;
                    }
                    else{
                        S = -B0 / A00;
                        DistSq =  B0 * S + C;
                    }
                }
                else{
                    S = REAL(0.0);
                    if (B1 >= REAL(0.0)){
                        T = REAL(0.0);
                        DistSq = C;
                    }
                    else if (-B1 >= A11){
                        T = REAL(1.0);
                        DistSq = A11 + REAL(2.0) * B1 + C;
                    }
                    else{
                        T = -B1 / A11;
                        DistSq = B1 * T + C;
                    }
                }
            }
            else{  // region 3
                S = REAL(0.0);
                if (B1 >= REAL(0.0)){
                    T = REAL(0.0);
                    DistSq = C;
                }
                else if (-B1 >= A11){
                    T = REAL(1.0);
                    DistSq = A11 + REAL(2.0) * B1 + C;
                }
                else{
                    T = -B1 / A11;
                    DistSq = B1 * T + C;
                }
            }
        }
        else if (T < REAL(0.0)){  // region 5
            T = REAL(0.0);
            if (B0 >= REAL(0.0)){
                S = REAL(0.0);
                DistSq = C;
            }
            else if (-B0 >= A00){
                S = 1.0f;
                DistSq = A00 + REAL(2.0) * B0 + C;
            }
            else{
                S = -B0 / A00;
                DistSq = B0 * S + C;
            }
        }
        else{  // region 0
            // minimum at interior point
            dReal InvDet = REAL(1.0) / Det;
            S *= InvDet;
            T *= InvDet;
            DistSq = S * (A00 * S + A01 * T + REAL(2.0) * B0) + T * (A01 * S + A11 * T + REAL(2.0) * B1) + C;
        }
    }
    else{
        dReal Tmp0, Tmp1, Numer, Denom;

        if (S < REAL(0.0)){  // region 2
            Tmp0 = A01 + B0;
            Tmp1 = A11 + B1;
            if (Tmp1 > Tmp0){
                Numer = Tmp1 - Tmp0;
                Denom = A00 - REAL(2.0) * A01 + A11;
                if (Numer >= Denom){
                    S = REAL(1.0);
                    T = REAL(0.0);
                    DistSq = A00 + REAL(2.0) * B0 + C;
                }
                else{
                    S = Numer / Denom;
                    T = REAL(1.0) - S;
                    DistSq = S * (A00 * S + A01 * T + REAL(2.0) * B0) + T * (A01 * S + A11 * T + REAL(2.0) * B1) + C;
                }
            }
            else{
                S = REAL(0.0);
                if (Tmp1 <= REAL(0.0)){
                    T = REAL(1.0);
                    DistSq = A11 + REAL(2.0) * B1 + C;
                }
                else if (B1 >= REAL(0.0)){
                    T = REAL(0.0);
                    DistSq = C;
                }
                else{
                    T = -B1 / A11;
                    DistSq = B1 * T + C;
                }
            }
        }
        else if (T < REAL(0.0)){  // region 6
            Tmp0 = A01 + B1;
            Tmp1 = A00 + B0;
            if (Tmp1 > Tmp0){
                Numer = Tmp1 - Tmp0;
                Denom = A00 - REAL(2.0) * A01 + A11;
                if (Numer >= Denom){
                    T = REAL(1.0);
                    S = REAL(0.0);
                    DistSq = A11 + REAL(2.0) * B1 + C;
                }
                else{
                    T = Numer / Denom;
                    S = REAL(1.0) - T;
                    DistSq = S * (A00 * S + A01 * T + REAL(2.0) * B0) + T * (A01 * S + A11 * T + REAL(2.0) * B1) + C;
                }
            }
            else{
                T = REAL(0.0);
                if (Tmp1 <= REAL(0.0)){
                    S = REAL(1.0);
                    DistSq = A00 + REAL(2.0) * B0 + C;
                }
                else if (B0 >= REAL(0.0)){
                    S = REAL(0.0);
                    DistSq = C;
                }
                else{
                    S = -B0 / A00;
                    DistSq = B0 * S + C;
                }
            }
        }
        else{  // region 1
            Numer = A11 + B1 - A01 - B0;
            if (Numer <= REAL(0.0)){
                S = REAL(0.0);
                T = REAL(1.0);
                DistSq = A11 + REAL(2.0) * B1 + C;
            }
            else{
                Denom = A00 - REAL(2.0) * A01 + A11;
                if (Numer >= Denom){
                    S = REAL(1.0);
                    T = REAL(0.0);
                    DistSq = A00 + REAL(2.0) * B0 + C;
                }
                else{
                    S = Numer / Denom;
                    T = REAL(1.0) - S;
                    DistSq = S * (A00 * S + A01 * T + REAL(2.0) * B0) + T * (A01 * S + A11 * T + REAL(2.0) * B1) + C;
                }
            }
        }
    }
    return dSqrt(dFabs(DistSq));
}

void GenerateContact(int Flags, dContactGeom* Contacts, int Stride, const dVector3 ContactPos, const dVector4 Plane, int& OutTriCount, int BoxVertex){
	dReal Depth = -(dDOT(Plane, ContactPos) - Plane[3]);

	if (Depth > 0.0){
		int ContactIndex = OutTriCount;
		if (ContactIndex != 0){
			dVector3 Normal;
			Normal[0] = -Plane[0];
			Normal[1] = -Plane[1];
			Normal[2] = -Plane[2];

			for (int i = 0; i < OutTriCount; i++){
				dContactGeom* Contact = SAFECONTACT(Flags, Contacts, i, Stride);
 				if ((int&)Contact->g1 == BoxVertex){
					if (dDOT(Contact->normal, Normal) > REAL(0.9)){
						if (Depth > Contact->depth){
							ContactIndex = i;
							OutTriCount--;
							break;
						}
						else return;
					}
				}
			}
			OutTriCount++;
		}
		else OutTriCount++;

		dContactGeom* Contact = SAFECONTACT(Flags, Contacts, ContactIndex, Stride);

		Contact->pos[0] = ContactPos[0];
		Contact->pos[1] = ContactPos[1];
		Contact->pos[2] = ContactPos[2];
		Contact->pos[3] = REAL(0.0);
		
		Contact->normal[0] = -Plane[0];
		Contact->normal[1] = -Plane[1];
		Contact->normal[2] = -Plane[2];
		Contact->normal[3] = REAL(0.0);

		Contact->depth = Depth;

		Contact->g1 = (dxGeom*&)BoxVertex;
	}
}

//#include "..\..\Include\drawstuff\\drawstuff.h"

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
	if (TriMesh->doBoxTC) {
		dxTriMesh::BoxTC* boxTC = 0;
		for (int i = 0; i < TriMesh->BoxTCCache.size(); i++){
			if (TriMesh->BoxTCCache[i].Geom == BoxGeom){
				boxTC = &TriMesh->BoxTCCache[i];
				break;
			}
		}
		if (!boxTC){
			TriMesh->BoxTCCache.push(dxTriMesh::BoxTC());

			boxTC = &TriMesh->BoxTCCache[TriMesh->BoxTCCache.size() - 1];
			boxTC->Geom = BoxGeom;
			boxTC->FatCoeff = 1.0f;
		}
		
		// Intersect
		Collider.SetTemporalCoherence(true);
		Collider.Collide(*boxTC, Box, TriMesh->Data->BVTree, null, 
						 &MakeMatrix(TLPosition, TLRotation, amatrix));
	}
	else {
		Collider.SetTemporalCoherence(false);
		Collider.Collide(dxTriMesh::defaultBoxCache, Box, TriMesh->Data->BVTree, null, 
						 &MakeMatrix(TLPosition, TLRotation, amatrix));	
	}

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

		dVector3 BoxVertices[8];
		ComputeVertices(BoxCenter, BoxExtents, Axis, BoxVertices);

		//const Box3& MagicBox = (const Box3&)Box;

		int OutTriCount = 0;
		for (int i = 0; i < TriCount; i++){
			const int& TriIndex = Triangles[i];

			if (!Callback(TriMesh, BoxGeom, TriIndex)) continue;

			dVector3 dv[3];
			FetchTriangle(TriMesh, TriIndex, TLPosition, TLRotation, dv);

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
			dReal Area = dSqrt(dDOT(Plane, Plane));
			Plane[0] /= Area;
			Plane[1] /= Area;
			Plane[2] /= Area;
			Plane[3] /= Area;

			bool PointBehind[8];
			int BehindPointCount = 0;
			for (int j = 0; j < 8; j++){
				dReal Behind = dDOT(Plane, BoxVertices[j]) - Plane[3];
				
				if (Behind < REAL(0.0)){//00001)){
					PointBehind[j] = true;
					BehindPointCount++;
				}
				else PointBehind[j] = false;
			}

			if (BehindPointCount != 0){
				dVector4 EdgePlanes[3];
				for (int j = 0; j < 3; j++){
					dVector3 ev0;
					ev0[0] =  dv[j][0];
					ev0[1] =  dv[j][1];
					ev0[2] =  dv[j][2];
					ev0[3] =  dv[j][3];

					dVector3 ev1;
					ev1[0] =  dv[(j + 1) % 3][0];
					ev1[1] =  dv[(j + 1) % 3][1];
					ev1[2] =  dv[(j + 1) % 3][2];
					ev1[3] =  dv[(j + 1) % 3][3];

					dVector3 ev2;
					ev2[0] =  dv[j][0] + Plane[0];
					ev2[1] =  dv[j][1] + Plane[1];
					ev2[2] =  dv[j][2] + Plane[2];
					ev2[3] =  dv[j][3];

					/*dVector3 Pos;
					Pos[0] = 0;
					Pos[1] = 0;
					Pos[2] = 0;
					dMatrix3 Rotation;
					dRSetIdentity(Rotation);
					
					dsDrawTriangle(Pos, Rotation, ev0, ev1, ev2, 1);*/

					dVector3 u;
					u[0] = ev1[0] - ev0[0];
					u[1] = ev1[1] - ev0[1];
					u[2] = ev1[2] - ev0[2];
					u[3] = REAL(0.0);
					
					dVector3 v;
					v[0] = ev2[0] - ev0[0];
					v[1] = ev2[1] - ev0[1];
					v[2] = ev2[2] - ev0[2];
					v[3] = REAL(0.0);

					dCROSS(EdgePlanes[j], =, u, v);
					EdgePlanes[j][3] = dDOT(EdgePlanes[j], ev0);

					dReal Area = dSqrt(dDOT(EdgePlanes[j], EdgePlanes[j]));
					EdgePlanes[j][0] /= Area;
					EdgePlanes[j][1] /= Area;
					EdgePlanes[j][2] /= Area;
					EdgePlanes[j][3] /= Area;

					/*dVector3 Temp;
					Temp[0] = ev0[0] + EdgePlanes[j][0];
					Temp[1] = ev0[1] + EdgePlanes[j][1];
					Temp[2] = ev0[2] + EdgePlanes[j][2];
					Temp[3] = ev0[3] + EdgePlanes[j][3];

					dsDrawLine(ev0, Temp);

					Temp[0] = ev1[0] + EdgePlanes[j][0];
					Temp[1] = ev1[1] + EdgePlanes[j][1];
					Temp[2] = ev1[2] + EdgePlanes[j][2];
					Temp[3] = ev1[3] + EdgePlanes[j][3];

					dsDrawLine(ev1, Temp);*/
				}

				int BoxEdges[8][3];
				BoxEdges[0][0] = 1;
				BoxEdges[0][1] = 3;
				BoxEdges[0][2] = 4;
				
				BoxEdges[1][0] = 2;
				BoxEdges[1][1] = 5;
				BoxEdges[1][2] = 0;
				
				BoxEdges[2][0] = 3;
				BoxEdges[2][1] = 1;
				BoxEdges[2][2] = 6;
				
				BoxEdges[3][0] = 0;
				BoxEdges[3][1] = 7;
				BoxEdges[3][2] = 2;
				
				BoxEdges[4][0] = 7;
				BoxEdges[4][1] = 0;
				BoxEdges[4][2] = 5;
				
				BoxEdges[5][0] = 6;
				BoxEdges[5][1] = 4;
				BoxEdges[5][2] = 1;
				
				BoxEdges[6][0] = 2;
				BoxEdges[6][1] = 5;
				BoxEdges[6][2] = 7;
				
				BoxEdges[7][0] = 6;
				BoxEdges[7][1] = 4;
				BoxEdges[7][2] = 3;

				for (int j = 0; j < 8; j++){
					if (!PointBehind[j]){
						continue;
					}

					bool NeedClipping = false;
					for (int k = 0; k < 3; k++){
						if (!PointPlaneIntersect(BoxVertices[j], EdgePlanes[k])){
							dVector3 ContactPos;
							ContactPos[0] = BoxVertices[j][0];
							ContactPos[1] = BoxVertices[j][1];
							ContactPos[2] = BoxVertices[j][2];
							ContactPos[3] = REAL(0.0);

							/*dMatrix3 Rotation;
							dRSetIdentity(Rotation);
							const dReal ss[3] = {0.02,0.02,0.02};
							dsDrawBox(ContactPos, Rotation, ss);*/

							for (int l = 0; l < 3; l++){
								dVector3 Dir;
								Dir[0] = BoxVertices[BoxEdges[j][l]][0] - ContactPos[0];
								Dir[1] = BoxVertices[BoxEdges[j][l]][1] - ContactPos[1];
								Dir[2] = BoxVertices[BoxEdges[j][l]][2] - ContactPos[2];
								
								dReal Length = dSqrt(dDOT(Dir, Dir));
								dNormalize3(Dir);

								for (int m = 0; m < 3; m++){
									dVector3 OutPos;
									if (LinePlaneIntersect(ContactPos, Dir, Length, EdgePlanes[m], OutPos)){
										//dsDrawLine(ContactPos, BoxVertices[BoxEdges[j][l]]);
										int Index;
										for (Index = 0; Index < 3; Index++){
											if (!PointPlaneIntersect(OutPos, EdgePlanes[Index])){
												break;
											}
										}
										if (Index == 3){
											if (PointPlaneIntersect(OutPos, Plane)){
												/*dMatrix3 Rotation;
												dRSetIdentity(Rotation);
												dsDrawSphere(OutPos, Rotation, REAL(0.01));*/

											        GenerateContact(Flags, Contacts, Stride, OutPos, Plane, OutTriCount, j);
												if(OutTriCount == (Flags & 0xffff)) {
												  goto _fullContacts;
												}
											}
										}
									}
								}
							}
							NeedClipping = true;
						}
					}

					if (!NeedClipping){
						GenerateContact(Flags, Contacts, Stride, BoxVertices[j], Plane, OutTriCount, j);
						if(OutTriCount == (Flags & 0xffff)) {
						  goto _fullContacts;
						}
					}
				}
			}
		}

	_fullContacts:

		for (int i = 0; i < OutTriCount; i++){
			dContactGeom* Contact = SAFECONTACT(Flags, Contacts, i, Stride);
			//Contact->depth = dDOT(Contact->normal, Contact->normal);
			//dNormalize3(Contact->normal);

			Contact->g1 = TriMesh;
			Contact->g2 = BoxGeom;
		}
		return OutTriCount;
	}
	return 0;
}
