// Do NOT build this file seperately. It is included in dTriList.cpp automatically.

#include "dcTriListCollider.h"

#include <Opcode.h>
using namespace Opcode;
#include "dcAABBCollider.h"
#include "dcOBBCollider.h"

dcTriListCollider::dcTriListCollider(dxGeom* Geometry) : AABBCollider(Vertices, Indices), OBBCollider(Vertices, Indices){
	this->Geometry = Geometry;
	GeomData = (dxTriList*)dGeomGetClassData(Geometry);

	memset(BoxContacts, 0, sizeof(BoxContacts));
}

dcTriListCollider::~dcTriListCollider(){
	//
}

void dcTriListCollider::Build(){
	OPCODECREATE TreeBuilder;
	TreeBuilder.NbTris = Indices.GetLength() / 3;
	TreeBuilder.NbVerts	= Vertices.GetLength();
	TreeBuilder.Tris = (udword*)(dword*)Indices;
	TreeBuilder.Verts = (Point*)(Vertex*)Vertices;
	TreeBuilder.Rules = SPLIT_COMPLETE | SPLIT_SPLATTERPOINTS;
	TreeBuilder.NoLeaf = true;
	TreeBuilder.Quantized = false;
	BVTree.Build(TreeBuilder);
}

int dCollideBP (const dxGeom* o1, const dxGeom* o2, int flags, dContactGeom *contact, int skip);	// ODE internal function

#define CONTACT(Ptr, Stride) ((dContactGeom*) (((byte*)Ptr) + (Stride)))

dword dcTriListCollider::CollideBox(dxGeom* Box, dword Flags, dContactGeom* Contacts, dword Stride){
	dcOBBTreeCollider& Collider = OBBCollider;

	/* Get box */
	const Vertex& BoxCenter = *(Vertex*)dGeomGetPosition(Box);

	dVector3 BoxSides;
	dGeomBoxGetLengths(Box, BoxSides);

	const Vertex BoxExtents(BoxSides[0] / 2.0f, BoxSides[1] / 2.0f, BoxSides[2] / 2.0f);

	const dReal* BoxRotation = dGeomGetRotation(Box);

	/* Make OBB */
	Matrix44 BoxMatrix;
	BoxMatrix[0][0] = BoxRotation[0];
	BoxMatrix[1][0] = BoxRotation[1];
	BoxMatrix[2][0] = BoxRotation[2];

	BoxMatrix[0][1] = BoxRotation[4];
	BoxMatrix[1][1] = BoxRotation[5];
	BoxMatrix[2][1] = BoxRotation[6];

	BoxMatrix[0][2] = BoxRotation[8];
	BoxMatrix[1][2] = BoxRotation[9];
	BoxMatrix[2][2] = BoxRotation[10];

	BoxMatrix[3][0] = BoxCenter[0];
	BoxMatrix[3][1] = BoxCenter[1];
	BoxMatrix[3][2] = BoxCenter[2];

	/* Intersect */
	Collider.Collide((const AABBNoLeafTree*)BVTree.GetTree(), (Point&)BoxExtents, (Matrix4x4&)BoxMatrix);

	/* Retrieve data */
	dword TriangleIDCount = Collider.Contacts.GetLength();

	if (TriangleIDCount != 0){
		dcVector<dword> TriangleIDs = Collider.Contacts;

		Callback(Box, TriangleIDs);
		
		/* Generate data */
		for (dword i = 0; i < TriangleIDCount; i++){
			const dword& TriIndex = TriangleIDs[i];

			if (!Valid(Box, TriIndex)) continue;

			/* Create data */
			CollisionTriangle ColTri;
			for (dword j = 0; j < 3; j++){
				ColTri.Vertices[j] = &Vertices[Indices[TriIndex * 3 + j]];
			}
			ColTri.Plane = dcPlane(*ColTri.Vertices[0], *ColTri.Vertices[1], *ColTri.Vertices[2]);

			if (ColTri.Plane.Contains(BoxCenter)){
				for (dword j = 0; j < 3; j++){
					const Vertex& v0 = *ColTri.Vertices[j];
					const Vertex& v1 = *ColTri.Vertices[(j + 1) % 3];
					ColTri.Edges[j] = dcPlane(v0, v1, v1 - ColTri.Plane.Normal);
				}
				float DistSq = ColTri.Plane.Normal.DotProduct(BoxCenter - *ColTri.Vertices[0]);
				ColTri.ClosestPoint = BoxCenter - (ColTri.Plane.Normal * ColTri.DistSq);
				ColTri.DistSq = dcMath::Abs(ColTri.DistSq);
				
				/* Insert data */
				dword Index;
				for (Index = 0; Index < CollisionTriangles.GetLength(); Index++){
					if (ColTri.DistSq < CollisionTriangles[Index].DistSq){
						CollisionTriangles.InsertItem(Index, ColTri);
						break;
					}
				}
				if (Index == CollisionTriangles.GetLength()){
					CollisionTriangles.AddItem(ColTri);
				}
			}
		}

		if (CollisionTriangles.GetLength() != 0){
			/* Generating contacts */
			dword OutTriCount = 0;
			for (dword i = 0; i < CollisionTriangles.GetLength(); i++){
				dcPlane& TriPlane = *(dcPlane*)dGeomGetClassData(Geometry);
				TriPlane = CollisionTriangles[i].Plane;
				
				dword ContactCount = dCollideBP(Box, Geometry, 3, BoxContacts, sizeof(dContactGeom));

				for (dword j = 0; j < ContactCount; j++){
					Vertex& Pos = (Vertex&)BoxContacts[j].pos;
					Vertex& Normal = (Vertex&)BoxContacts[j].normal;
					float& Depth = BoxContacts[j].depth;
					
					for (dword k = 0; k < 3; k++){
						dcPlane& Edge = CollisionTriangles[i].Edges[k];
						const float PointDepth = Edge.Distance - Pos.DotProduct(Edge.Normal);
						if (PointDepth > 0.0f){
							Pos += Edge.Normal * PointDepth;
							Depth = 0.0f;
						}
					}
					dword Index;
					for (Index = 0; Index < OutTriCount; Index++){
						dContactGeom* Ref = CONTACT(Contacts, Index * Stride);
						float DistSq = ((Vertex&)Ref->pos - Pos).SquareMagnitude();
						if (DistSq < 0.1f){
							Vertex& RefNormal = (Vertex&)Ref->normal;
							RefNormal += Normal;
							RefNormal.Normalize();
						}
					}
					if (Index == OutTriCount){
						(*CONTACT(Contacts, OutTriCount * Stride)) = BoxContacts[j];
						OutTriCount++;
					}
				}
			}
			CollisionTriangles.Clear();

			return OutTriCount;
		}
		else return 0;
	}
	else return 0;
}

dword dcTriListCollider::CollideSphere(dxGeom* Sphere, dword Flags, dContactGeom* Contacts, dword Stride){
	dcAABBTreeCollider& Collider = AABBCollider;

	/* Get sphere */
	const Vertex& SphereCenter = *(Vertex*)dGeomGetPosition(Sphere);
	dReal SphereRadius = dGeomSphereGetRadius(Sphere);

	/* Make AABB */
	CollisionAABB Box;
	Box.mCenter = (Point&)SphereCenter;
	Box.mExtents.x = SphereRadius;
	Box.mExtents.y = SphereRadius;
	Box.mExtents.z = SphereRadius;

	/* Intersect */
	Collider.Collide((AABBNoLeafTree*)BVTree.GetTree(), Box);
	
	/* Retrieve data */
	dword TriangleIDCount = Collider.Contacts.GetLength();
	
	if (TriangleIDCount != 0){
		dcVector<dword>& TriangleIDs = Collider.Contacts;

		Callback(Sphere, TriangleIDs);

		/* Creating minimum contacts */
		dword OutTriCount = 0;
		for (dword i = 0; i < TriangleIDCount; i++){
			const dword& TriIndex = TriangleIDs[i];

			if (!Valid(Sphere, TriIndex)) continue;

			const Vertex& v0 = Vertices[Indices[TriIndex * 3 + 0]];
			const Vertex& v1 = Vertices[Indices[TriIndex * 3 + 1]];
			const Vertex& v2 = Vertices[Indices[TriIndex * 3 + 2]];

			dcPlane TriPlane(v0, v1, v2);

			if (!TriPlane.Contains(SphereCenter)){
				continue;
			}

			const Vertex& ContactNormal = TriPlane.Normal;
			const Vertex ContactPos = SphereCenter - ContactNormal * SphereRadius;
			const float ContactDepth = TriPlane.Distance - SphereCenter.DotProduct(TriPlane.Normal) + SphereRadius;

			if (ContactDepth >= 0){
				dword Index;
				for (Index = 0; Index < 3; Index++){
					const Vertex& v0 = Vertices[Indices[TriIndex * 3 + Index]];
					const Vertex& v1 = Vertices[Indices[TriIndex * 3 + (Index + 1) % 3]];
					dcPlane Plane(v0, v1, v1 - TriPlane.Normal);

					if (!Plane.Contains(ContactPos)){
						break;
					}
				}
				if (Index == 3){
					for (Index = 0; Index < OutTriCount; Index++){
						dContactGeom* RefContact = CONTACT(Contacts, Index * Stride);
						const Vertex& RefNormal = (Vertex&)RefContact->normal;

						if (TriPlane.Normal.DotProduct(RefNormal) > 0.9f){	// Coplanitary test
							RefContact->depth = dcMath::Max(RefContact->depth, ContactDepth);
							break;
						}
					}

					if (Index == OutTriCount){
						dContactGeom* Contact = CONTACT(Contacts, OutTriCount * Stride);
						((Vertex&)Contact->normal) = TriPlane.Normal;
						Contact->depth = ContactDepth;
						OutTriCount++;
					}
				}
			}
		}

		if (OutTriCount != 0){
			Vertex OutNormal(0, 0, 0);
		
			/* Combining contacts */
			for (dword i = 0; i < OutTriCount; i++){
				dContactGeom* Contact = CONTACT(Contacts, i * Stride);

				OutNormal += ((Vertex&)Contact->normal) * Contact->depth;
			}

			const float DepthSq = OutNormal.SquareMagnitude();

			/* Generating final contact */
			if (DepthSq > 0.0f){
				const float Depth = sqrtf(DepthSq);

				OutNormal /= Depth;	// Normalizing

				dContactGeom* Contact = Contacts;
				Contact->g1 = Sphere;
				Contact->g2 = Geometry;

				((Vertex&)Contact->pos) = SphereCenter - OutNormal * SphereRadius;
				((Vertex&)Contact->normal) = OutNormal;
				Contact->depth = Depth;
				return 1;
			}
			else return 0;
		}
		else return 0;
	}
	else return 0;
}

bool dcTriListCollider::GenerateTC(dxGeom* Object, const dcAABB& Box){
	TCData.Clear();

	AABBCollider.TCData = &TCData;
	AABBCollider.GenerateTC((AABBNoLeafTree*)BVTree.GetTree(), (const CollisionAABB&)Box);

	if (TCData.GetLength() != 0){
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

void dcTriListCollider::Callback(dxGeom* Object, dcVector<dword>& TriIndices){
	if (GeomData->ArrayCallback != null){
		GeomData->ArrayCallback(Geometry, Object, TriIndices);
	}
}

bool dcTriListCollider::Valid(dxGeom* Object, dword TriIndex){
	if (GeomData->Callback != null){
		return GeomData->Callback(Geometry, Object, TriIndex);
	}
	else return true;
}
