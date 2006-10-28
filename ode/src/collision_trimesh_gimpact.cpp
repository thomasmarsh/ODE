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

#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

void dxTriMeshData::Preprocess()
{
	// If this mesh has already been preprocessed, exit
//	if (UseFlags)
//		return;
//
//	udword numTris = Mesh.GetNbTriangles();
//	udword numEdges = numTris * 3;
//
//	UseFlags = new uint8[numTris];
//	memset(UseFlags, 0, sizeof(uint8) * numTris);
//
//	EdgeRecord* records = new EdgeRecord[numEdges];
//
//	// Make a list of every edge in the mesh
//	const IndexedTriangle* tris = Mesh.GetTris();
//    for (unsigned int i = 0; i < numTris; i++)
//	{
//		SetupEdge(&records[i*3],   0, i, tris->mVRef);
//		SetupEdge(&records[i*3+1], 1, i, tris->mVRef);
//		SetupEdge(&records[i*3+2], 2, i, tris->mVRef);
//
//		tris = (const IndexedTriangle*)(((uint8*)tris) + Mesh.GetTriStride());
//	}
//
//	// Sort the edges, so the ones sharing the same verts are beside each other
//	qsort(records, numEdges, sizeof(EdgeRecord), EdgeCompare);
//
//	// Go through the sorted list of edges and flag all the edges and vertices that we need to use
//	for (unsigned int i = 0; i < numEdges; i++)
//	{
//		EdgeRecord* rec1 = &records[i];
//		EdgeRecord* rec2 = 0;
//		if (i < numEdges - 1)
//			rec2 = &records[i+1];
//
//		if (rec2 &&
//			rec1->VertIdx1 == rec2->VertIdx1 &&
//			rec1->VertIdx2 == rec2->VertIdx2)
//		{
//			VertexPointers vp;
//			Mesh.GetTriangle(vp, rec1->TriIdx);
//
//			// Get the normal of the first triangle
//			Point triNorm = (*vp.Vertex[2] - *vp.Vertex[1]) ^ (*vp.Vertex[0] - *vp.Vertex[1]);
//			triNorm.Normalize();
//
//			// Get the vert opposite this edge in the first triangle
//			Point oppositeVert1 = GetOppositeVert(rec1, vp.Vertex);
//
//			// Get the vert opposite this edge in the second triangle
//			Mesh.GetTriangle(vp, rec2->TriIdx);
//			Point oppositeVert2 = GetOppositeVert(rec2, vp.Vertex);
//
//			float dot = triNorm.Dot((oppositeVert2 - oppositeVert1).Normalize());
//
//			// We let the dot threshold for concavity get slightly negative to allow for rounding errors
//			static const float kConcaveThresh = -0.000001f;
//
//			// This is a concave edge, leave it for the next pass
//			if (dot >= kConcaveThresh)
//				rec1->Concave = true;
//			// If this is a convex edge, mark its vertices and edge as used
//			else
//				UseFlags[rec1->TriIdx] |= rec1->Vert1Flags | rec1->Vert2Flags | rec1->EdgeFlags;
//
//			// Skip the second edge
//			i++;
//		}
//		// This is a boundary edge
//		else
//		{
//			UseFlags[rec1->TriIdx] |= rec1->Vert1Flags | rec1->Vert2Flags | rec1->EdgeFlags;
//		}
//	}
//
//	// Go through the list once more, and take any edge we marked as concave and
//	// clear it's vertices flags in any triangles they're used in
//	for (unsigned int i = 0; i < numEdges; i++)
//	{
//		EdgeRecord& er = records[i];
//
//		if (er.Concave)
//		{
//			for (unsigned int j = 0; j < numEdges; j++)
//			{
//				EdgeRecord& curER = records[j];
//
//				if (curER.VertIdx1 == er.VertIdx1 ||
//					curER.VertIdx1 == er.VertIdx2)
//					UseFlags[curER.TriIdx] &= ~curER.Vert1Flags;
//
//				if (curER.VertIdx2 == er.VertIdx1 ||
//					curER.VertIdx2 == er.VertIdx2)
//					UseFlags[curER.TriIdx] &= ~curER.Vert2Flags;
//			}
//		}
//	}
//
//	delete [] records;
}

dTriMeshDataID dGeomTriMeshDataCreate(){
    return new dxTriMeshData();
}

void dGeomTriMeshDataDestroy(dTriMeshDataID g){
    delete g;
}

void dGeomTriMeshSetLastTransform( dxGeom* g, dMatrix4 last_trans )
{
//	dAASSERT(g)
//    dUASSERT(g->type == dTriMeshClass, "geom not trimesh");

//    for (int i=0; i<16; i++)
//        (((dxTriMesh*)g)->last_trans)[ i ] = last_trans[ i ];

//    return;
}


//dReal* dGeomTriMeshGetLastTransform( dxGeom* g )
//{
//	dAASSERT(g)
//    dUASSERT(g->type == dTriMeshClass, "geom not trimesh");
//
//    return (dReal*)(((dxTriMesh*)g)->last_trans);
//}




void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void* in_data)
{
//    dUASSERT(g, "argument not trimesh data");
//
//    double *elem;
//
//    switch (data_id) {
//    case TRIMESH_FACE_NORMALS:
//	g->Normals = (dReal *) in_data;
//	break;
//
//    case TRIMESH_LAST_TRANSFORMATION:
//	elem = (double *) in_data;
//    for (int i=0; i<16; i++)
//        g->last_trans[i] = (dReal) elem[i];
//
//	break;
//    default:
//	dUASSERT(data_id, "invalid data type");
//	break;
//    }
//
//    return;

}



void*  dGeomTriMeshDataGet(dTriMeshDataID g, int data_id)
{
    dUASSERT(g, "argument not trimesh data");

//    switch (data_id) {
//    case TRIMESH_FACE_NORMALS:
//        return NULL;
//        break;
//
//    case TRIMESH_LAST_TRANSFORMATION:
//        return NULL;
//        break;
//    default:
//        dUASSERT(data_id, "invalid data type");
//        break;
//    }

    return NULL;
}


void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
                                  const void* Vertices, int VertexStride, int VertexCount,
                                  const void* Indices, int IndexCount, int TriStride,
                                  const void* Normals)
{
    dUASSERT(g, "argument not trimesh data");

    g->Build(Vertices, VertexStride, VertexCount,
             Indices, IndexCount, TriStride,
             Normals,
             true);
}


void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
                                 const void* Vertices, int VertexStride, int VertexCount,
                                 const void* Indices, int IndexCount, int TriStride)
{
    dGeomTriMeshDataBuildSingle1(g, Vertices, VertexStride, VertexCount,
                                 Indices, IndexCount, TriStride, (void*)NULL);
}


void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g,
                                  const void* Vertices, int VertexStride, int VertexCount,
                                 const void* Indices, int IndexCount, int TriStride,
				 const void* Normals)
{
    dUASSERT(g, "argument not trimesh data");

    g->Build(Vertices, VertexStride, VertexCount,
             Indices, IndexCount, TriStride,
             Normals,
             false);
}


void dGeomTriMeshDataBuildDouble(dTriMeshDataID g,
				 const void* Vertices, int VertexStride, int VertexCount,
                                 const void* Indices, int IndexCount, int TriStride) {
    dGeomTriMeshDataBuildDouble1(g, Vertices, VertexStride, VertexCount,
                                 Indices, IndexCount, TriStride, NULL);
}


void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
                                  const dReal* Vertices, int VertexCount,
                                 const int* Indices, int IndexCount,
                                 const int* Normals){
#ifdef dSINGLE
    dGeomTriMeshDataBuildSingle1(g,
				Vertices, 4 * sizeof(dReal), VertexCount,
				Indices, IndexCount, 3 * sizeof(unsigned int),
				Normals);
#else
    dGeomTriMeshDataBuildDouble1(g, Vertices, 4 * sizeof(dReal), VertexCount,
				Indices, IndexCount, 3 * sizeof(unsigned int),
				Normals);
#endif
}


void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
                                 const dReal* Vertices, int VertexCount,
                                 const int* Indices, int IndexCount) {
    dGeomTriMeshDataBuildSimple1(g,
                                 Vertices, VertexCount, Indices, IndexCount,
                                 (const int*)NULL);
}

void dGeomTriMeshDataPreprocess(dTriMeshDataID g)
{
    dUASSERT(g, "argument not trimesh data");
	g->Preprocess();
}

void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char** buf, int* bufLen)
{
    dUASSERT(g, "argument not trimesh data");
	*buf = NULL;
	*bufLen = 0;
}

void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf)
{
    dUASSERT(g, "argument not trimesh data");
//	g->UseFlags = buf;
}


// Trimesh

dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){
    type = dTriMeshClass;

    this->Data = Data;

    //Create trimesh

    gim_trimesh_create_from_data(&m_collision_trimesh,( vec3f *)(&Data->m_Vertices[0]), Data->m_VertexCount ,0, ( GUINT *)(&Data->m_Indices[0]), Data->m_TriangleCount*3,0,1);


	/* TC has speed/space 'issues' that don't make it a clear
	   win by default on spheres/boxes. */
	this->doSphereTC = true;
	this->doBoxTC = true;
	this->doCapsuleTC = true;

}

dxTriMesh::~dxTriMesh(){

    //Terminate Trimesh
    gim_trimesh_destroy(&m_collision_trimesh);
}


void dxTriMesh::ClearTCCache(){

}


int dxTriMesh::AABBTest(dxGeom* g, dReal aabb[6]){
    return 1;
}


void dxTriMesh::computeAABB()
{
    //update trimesh transform
    mat4f transform;
    IDENTIFY_MATRIX_4X4(transform);
    MakeMatrix(this, transform);
    gim_trimesh_set_tranform(&m_collision_trimesh,transform);

    //Update trimesh boxes
    gim_trimesh_update(&m_collision_trimesh);

    memcpy(aabb,&m_collision_trimesh.m_aabbset.m_global_bound,6*sizeof(GREAL));
}


void dxTriMeshData::UpdateData()
{
//  BVTree.Refit();
}


dGeomID dCreateTriMesh(dSpaceID space,
		       dTriMeshDataID Data,
		       dTriCallback* Callback,
		       dTriArrayCallback* ArrayCallback,
		       dTriRayCallback* RayCallback)
{
    dxTriMesh* Geom = new dxTriMesh(space, Data);
    Geom->Callback = Callback;
    Geom->ArrayCallback = ArrayCallback;
    Geom->RayCallback = RayCallback;

    return Geom;
}

void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->Callback = Callback;
}

dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	return ((dxTriMesh*)g)->Callback;
}

void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->ArrayCallback = ArrayCallback;
}

dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	return ((dxTriMesh*)g)->ArrayCallback;
}

void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->RayCallback = Callback;
}

dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	return ((dxTriMesh*)g)->RayCallback;
}

void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->Data = Data;
}

dTriMeshDataID dGeomTriMeshGetData(dGeomID g)
{
  dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
  return ((dxTriMesh*)g)->Data;
}



void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

	switch (geomClass)
	{
		case dSphereClass:
			((dxTriMesh*)g)->doSphereTC = (1 == enable);
			break;
		case dBoxClass:
			((dxTriMesh*)g)->doBoxTC = (1 == enable);
			break;
		case dCapsuleClass:
//		case dCCylinderClass:
			((dxTriMesh*)g)->doCapsuleTC = (1 == enable);
			break;
	}
}

int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

	switch (geomClass)
	{
		case dSphereClass:
			if (((dxTriMesh*)g)->doSphereTC)
				return 1;
			break;
		case dBoxClass:
			if (((dxTriMesh*)g)->doBoxTC)
				return 1;
			break;
		case dCapsuleClass:
			if (((dxTriMesh*)g)->doCapsuleTC)
				return 1;
			break;
	}
	return 0;
}

void dGeomTriMeshClearTCCache(dGeomID g){
    dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

    dxTriMesh* Geom = (dxTriMesh*)g;
    Geom->ClearTCCache();
}

/*
 * returns the TriMeshDataID
 */
dTriMeshDataID
dGeomTriMeshGetTriMeshDataID(dGeomID g)
{
    dxTriMesh* Geom = (dxTriMesh*) g;
    return Geom->Data;
}

// Getting data
void dGeomTriMeshGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2){
    dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

    dxTriMesh* Geom = (dxTriMesh*)g;

    const dVector3& Position = *(const dVector3*)dGeomGetPosition(g);
    const dMatrix3& Rotation = *(const dMatrix3*)dGeomGetRotation(g);

    dVector3 v[3];
	FetchTriangle(Geom, Index, Position, Rotation, v);

    if (v0){
        (*v0)[0] = v[0][0];
        (*v0)[1] = v[0][1];
        (*v0)[2] = v[0][2];
        (*v0)[3] = v[0][3];
    }
    if (v1){
        (*v1)[0] = v[1][0];
        (*v1)[1] = v[1][1];
        (*v1)[2] = v[1][2];
        (*v1)[3] = v[1][3];
    }
    if (v2){
        (*v2)[0] = v[2][0];
        (*v2)[1] = v[2][1];
        (*v2)[2] = v[2][2];
        (*v2)[3] = v[2][3];
    }
}

void dGeomTriMeshGetPoint(dGeomID g, int Index, dReal u, dReal v, dVector3 Out){
    dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

    dxTriMesh* Geom = (dxTriMesh*)g;

    const dVector3& Position = *(const dVector3*)dGeomGetPosition(g);
    const dMatrix3& Rotation = *(const dMatrix3*)dGeomGetRotation(g);

    dVector3 dv[3];
    FetchTriangle(Geom, Index, Position, Rotation, dv);

    GetPointFromBarycentric(dv, u, v, Out);
}

int dGeomTriMeshGetTriangleCount (dGeomID g)
{
    dxTriMesh* Geom = (dxTriMesh*)g;
    return 0;
}

void dGeomTriMeshDataUpdate(dTriMeshDataID g) {
    dUASSERT(g, "argument not trimesh data");
    g->UpdateData();
}

#endif
