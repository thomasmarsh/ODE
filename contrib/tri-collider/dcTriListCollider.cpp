// Do NOT build this file seperately. It is included in dTriList.cpp automatically.
#include "array.h"
#include "dxTriList.h"
#include "dcTriListCollider.h"

#include <Opcode.h>
using namespace Opcode;
#include "dcAABBCollider.h"
#include "dcOBBCollider.h"

#define GENERATEBODIES

dcTriListCollider::dcTriListCollider(dxGeom* Geometry) : AABBCollider(Vertices, Indices), OBBCollider(Vertices, Indices){
	this->Geometry = Geometry;
	GeomData = (dxTriList*)dGeomGetClassData(Geometry);

	memset(BoxContacts, 0, sizeof(BoxContacts));
}

dcTriListCollider::~dcTriListCollider(){
	//
}

void dcTriListCollider::Build(int VertexCount, int IndexCount){
	OPCODECREATE TreeBuilder;
	TreeBuilder.NbTris = IndexCount / 3;
	TreeBuilder.NbVerts	= VertexCount;
	TreeBuilder.Tris = (unsigned int*)Indices;
	TreeBuilder.Verts = (Point*)Vertices;
	TreeBuilder.Rules = SPLIT_COMPLETE | SPLIT_SPLATTERPOINTS;
	TreeBuilder.NoLeaf = true;
	TreeBuilder.Quantized = false;
	TreeBuilder.KeepOriginal = false;
	BVTree.Build(TreeBuilder);
}

int dCollideBP (const dxGeom* o1, const dxGeom* o2, int flags, dContactGeom *contact, int skip);	// ODE internal function

#define CONTACT(Ptr, Stride) ((dContactGeom*) (((byte*)Ptr) + (Stride)))

int dcTriListCollider::CollideBox(dxGeom* Box, int Flags, dContactGeom* Contacts, int Stride){
	dcOBBTreeCollider& Collider = OBBCollider;

	/* Get box */
	const dcVector3& BoxCenter = *(dcVector3*)dGeomGetPosition(Box);

	dVector3 BoxSides;
	dGeomBoxGetLengths(Box, BoxSides);

	const dcVector3 BoxExtents(BoxSides[0] / 2.0f, BoxSides[1] / 2.0f, BoxSides[2] / 2.0f);

	const dReal* BoxRotation = dGeomGetRotation(Box);

	/* Make OBB */
	Matrix4x4 BoxMatrix;
	BoxMatrix.m[0][0] = BoxRotation[0];
	BoxMatrix.m[1][0] = BoxRotation[1];
	BoxMatrix.m[2][0] = BoxRotation[2];

	BoxMatrix.m[0][1] = BoxRotation[4];
	BoxMatrix.m[1][1] = BoxRotation[5];
	BoxMatrix.m[2][1] = BoxRotation[6];

	BoxMatrix.m[0][2] = BoxRotation[8];
	BoxMatrix.m[1][2] = BoxRotation[9];
	BoxMatrix.m[2][2] = BoxRotation[10];

	BoxMatrix.m[3][0] = BoxCenter[0];
	BoxMatrix.m[3][1] = BoxCenter[1];
	BoxMatrix.m[3][2] = BoxCenter[2];

	/* Intersect */
	Collider.Collide((const AABBNoLeafTree*)BVTree.GetTree(), (Point&)BoxExtents, (Matrix4x4&)BoxMatrix);

	/* Retrieve data */
	int TriangleIDCount = Collider.Contacts.size();

	if (TriangleIDCount != 0){
		dArray<int> TriangleIDs = Collider.Contacts;

		Callback(Box, TriangleIDs);
		
		/* Generate data */
		for (int i = 0; i < TriangleIDCount; i++){
			const int& TriIndex = TriangleIDs[i];

			if (!Valid(Box, TriIndex)) continue;

			/* Create data */
			CollisionTriangle ColTri;
			for (int j = 0; j < 3; j++){
				ColTri.Vertices[j] = &Vertices[Indices[TriIndex * 3 + j]];
			}
			ColTri.Plane = dcPlane(*ColTri.Vertices[0], *ColTri.Vertices[1], *ColTri.Vertices[2]);

			if (ColTri.Plane.Contains(BoxCenter)){
				for (int j = 0; j < 3; j++){
					const dcVector3& v0 = *ColTri.Vertices[j];
					const dcVector3& v1 = *ColTri.Vertices[(j + 1) % 3];
					ColTri.Edges[j] = dcPlane(v0, v1, v1 - ColTri.Plane.Normal);
				}
				float DistSq = ColTri.Plane.Normal.DotProduct(BoxCenter - *ColTri.Vertices[0]);
				ColTri.ClosestPoint = BoxCenter - (ColTri.Plane.Normal * ColTri.DistSq);
				ColTri.DistSq = fabsf(ColTri.DistSq);
				
				/* Insert data */
				int Index;
				for (Index = 0; Index < CollisionTriangles.size(); Index++){
					if (ColTri.DistSq < CollisionTriangles[Index].DistSq){
						CollisionTriangles.insert(Index, ColTri);
						break;
					}
				}
				if (Index == CollisionTriangles.size()){
					CollisionTriangles.push(ColTri);
				}
			}
		}

		if (CollisionTriangles.size() != 0){
			/* Generating contacts */
			int OutTriCount = 0;
			for (int i = 0; i < CollisionTriangles.size(); i++){
				dcPlane& TriPlane = *(dcPlane*)dGeomGetClassData(Geometry);
				TriPlane = CollisionTriangles[i].Plane;
				
				int ContactCount = dCollideBP(Box, Geometry, 3, BoxContacts, sizeof(dContactGeom));

				for (int j = 0; j < ContactCount; j++){
					dcVector3& Pos = (dcVector3&)BoxContacts[j].pos;
					dcVector3 Normal = ((dcVector3&)BoxContacts[j].normal) * -1;
					dReal& Depth = BoxContacts[j].depth;
					
					for (int k = 0; k < 3; k++){
						dcPlane& Edge = CollisionTriangles[i].Edges[k];
						const float PointDepth = Edge.Distance - Pos.DotProduct(Edge.Normal);
						if (PointDepth > 0.0f){
							Pos += Edge.Normal * PointDepth;
							Depth = 0.0f;
						}
					}
					int Index;
					for (Index = 0; Index < OutTriCount; Index++){
						dContactGeom* Ref = CONTACT(Contacts, Index * Stride);
						float DistSq = ((dcVector3&)Ref->pos - Pos).MagnitudeSq();
						if (DistSq < 0.1f){
							dcVector3& RefNormal = (dcVector3&)Ref->normal;
							RefNormal += Normal;
							RefNormal.Normalize();
							//break;/// PIERRE!
						}
					}
					if (Index == OutTriCount){
						dContactGeom& OutContact = *CONTACT(Contacts, OutTriCount * Stride);
						OutContact.pos[0] = Pos.x;
						OutContact.pos[1] = Pos.y;
						OutContact.pos[2] = Pos.z;
						OutContact.normal[0] = Normal.x;
						OutContact.normal[1] = Normal.y;
						OutContact.normal[2] = Normal.z;
						OutContact.depth = Depth;
						OutContact.g1 = BoxContacts[j].g2;
						OutContact.g2 = BoxContacts[j].g1;
#ifdef GENERATEBODIES
						OutContact.b1 = BoxContacts[j].b2;
						OutContact.b2 = BoxContacts[j].b1;
#endif	//GENERATEBODIES

						OutTriCount++;
					}
				}
			}
			CollisionTriangles.setSize(0);

			return OutTriCount;
		}
		else return 0;
	}
	else return 0;
}

int dcTriListCollider::CollideSphere(dxGeom* Sphere, int Flags, dContactGeom* Contacts, int Stride){
	dcAABBTreeCollider& Collider = AABBCollider;

	/* Get sphere */
	const dcVector3 SphereCenter(dGeomGetPosition(Sphere));
	const float SphereRadius = dGeomSphereGetRadius(Sphere);

	/* Make AABB */
	CollisionAABB Box;
	Box.mCenter = (Point&)SphereCenter;
	Box.mExtents.x = SphereRadius;
	Box.mExtents.y = SphereRadius;
	Box.mExtents.z = SphereRadius;

	/* Intersect */
	Collider.Collide((AABBNoLeafTree*)BVTree.GetTree(), Box);
	
	/* Retrieve data */
	int TriangleIDCount = Collider.Contacts.size();
	
	if (TriangleIDCount != 0){
		dArray<int>& TriangleIDs = Collider.Contacts;

		Callback(Sphere, TriangleIDs);

		/* Creating minimum contacts */
		int OutTriCount = 0;
		for (int i = 0; i < TriangleIDCount; i++){
			const int& TriIndex = TriangleIDs[i];

			if (!Valid(Sphere, TriIndex)) continue;

			const dcVector3& v0 = Vertices[Indices[TriIndex * 3 + 0]];
			const dcVector3& v1 = Vertices[Indices[TriIndex * 3 + 1]];
			const dcVector3& v2 = Vertices[Indices[TriIndex * 3 + 2]];

			dcPlane TriPlane(v0, v1, v2);

			if (!TriPlane.Contains(SphereCenter)){
				continue;
			}

			const dcVector3& ContactNormal = TriPlane.Normal;
			const dcVector3 ContactPos = SphereCenter - ContactNormal * SphereRadius;
			const float ContactDepth = TriPlane.Distance - SphereCenter.DotProduct(TriPlane.Normal) + SphereRadius;

			if (ContactDepth >= 0){
				int Index;
				for (Index = 0; Index < 3; Index++){
					const dcVector3& v0 = Vertices[Indices[TriIndex * 3 + Index]];
					const dcVector3& v1 = Vertices[Indices[TriIndex * 3 + (Index + 1) % 3]];
					dcPlane Plane(v0, v1, v1 - TriPlane.Normal);

					if (!Plane.Contains(ContactPos)){
						break;
					}
				}
				if (Index == 3){
					for (Index = 0; Index < OutTriCount; Index++){
						dContactGeom* RefContact = CONTACT(Contacts, Index * Stride);
						const dcVector3& RefNormal = (dcVector3&)RefContact->normal;

						if (TriPlane.Normal.DotProduct(RefNormal) > 0.9f){	// Coplanitary test
							RefContact->depth = dcMAX((float)RefContact->depth, ContactDepth);
							break;
						}
					}

					if (Index == OutTriCount){
						dContactGeom* Contact = CONTACT(Contacts, OutTriCount * Stride);
						((dcVector3&)Contact->normal) = TriPlane.Normal;
						Contact->depth = ContactDepth;
						OutTriCount++;
					}
				}
			}
		}

		if (OutTriCount != 0){
			dcVector3 OutNormal(0, 0, 0);
		
			/* Combining contacts */
			for (int i = 0; i < OutTriCount; i++){
				dContactGeom* Contact = CONTACT(Contacts, i * Stride);

				OutNormal += ((dcVector3&)Contact->normal) * Contact->depth;
			}

			const float DepthSq = OutNormal.MagnitudeSq();

			/* Generating final contact */
			if (DepthSq > 0.0f){
				const float Depth = sqrtf(DepthSq);

				OutNormal /= Depth;	// Normalizing

				dContactGeom& OutContact = *Contacts;
				((dcVector3&)OutContact.pos) = SphereCenter - OutNormal * SphereRadius;
				((dcVector3&)OutContact.normal) = OutNormal * -1;
				OutContact.depth = Depth;

				OutContact.g1 = Geometry;
				OutContact.g2 = Sphere;
#ifdef GENERATEBODIES
				OutContact.b1 = dGeomGetBody(Sphere);
				OutContact.b2 = dGeomGetBody(Geometry);
#endif	//GENERATEBODIES

				return 1;
			}
			else return 0;
		}
		else return 0;
	}
	else return 0;
}

bool dcTriListCollider::GenerateTC(dxGeom* Object, const CollisionAABB& Box){
	TCData.setSize(0);

	AABBCollider.TCData = &TCData;
	AABBCollider.GenerateTC((AABBNoLeafTree*)BVTree.GetTree(), Box);

	if (TCData.size() != 0){
		OBBCollider.TCData = &TCData;
		return true;
	}
	else {
		AABBCollider.TCData = null;
		return false;
	}
}

void dcTriListCollider::ClearTC(){
	AABBCollider.TCData = null;
	OBBCollider.TCData = null;
}

void dcTriListCollider::Callback(dxGeom* Object, dArray<int>& TriIndices){
	if (GeomData->ArrayCallback != null){
		GeomData->ArrayCallback(Geometry, Object, TriIndices.data(), TriIndices.size());
	}
}

bool dcTriListCollider::Valid(dxGeom* Object, int TriIndex){
	if (GeomData->Callback != null){
		return GeomData->Callback(Geometry, Object, TriIndex);
	}
	else return true;
}
