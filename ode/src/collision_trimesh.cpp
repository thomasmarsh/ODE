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

// Trimesh data
dxTriMeshData::dxTriMeshData(){
	//
}

dxTriMeshData::~dxTriMeshData(){
	//
}

void dxTriMeshData::Build(const void* Vertices, int VertexStide, int VertexCount, const void* Indices, int IndexCount, int TriStride){
	Mesh.SetNbTriangles(IndexCount / 3);
	Mesh.SetNbVertices(VertexCount);
	Mesh.SetPointers((IndexedTriangle*)Indices, (Point*)Vertices);
	Mesh.SetStrides(TriStride, VertexStide);
	
	// Build tree
	BuildSettings Settings;
	Settings.mRules = SPLIT_BEST_AXIS;

	OPCODECREATE TreeBuilder;
	TreeBuilder.mIMesh = &Mesh;

	TreeBuilder.mSettings = Settings;
	TreeBuilder.mNoLeaf = true;
	TreeBuilder.mQuantized = false;

	TreeBuilder.mKeepOriginal = false;
	TreeBuilder.mCanRemap = false;

	BVTree.Build(TreeBuilder);
}

dTriMeshDataID dGeomTriMeshDataCreate(){
	return new dxTriMeshData();
}

void dGeomTriMeshDataDestroy(dTriMeshDataID g){
	delete g;
}

void dGeomTriMeshDataBuild(dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride){
	dUASSERT(g, "argument not trimesh data");

	g->Build(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride);
}

void dGeomTriMeshDataBuildSimple(dTriMeshDataID g, const dVector3* Vertices, int VertexCount, const int* Indices, int IndexCount){
	dGeomTriMeshDataBuild(g, Vertices, sizeof(dVector3), VertexCount, Indices, IndexCount, 3 * sizeof(int));
}

// Trimesh
PlanesCollider dxTriMesh::_PlanesCollider;
SphereCollider dxTriMesh::_SphereCollider;
OBBCollider dxTriMesh::_OBBCollider;
RayCollider dxTriMesh::_RayCollider;
AABBTreeCollider dxTriMesh::_AABBTreeCollider;

CollisionFaces dxTriMesh::Faces;

dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){
	type = dTriMeshClass;

	this->Data = Data;

	_RayCollider.SetDestination(&Faces);

	_PlanesCollider.SetTemporalCoherence(true);
	_SphereCollider.SetTemporalCoherence(true);
	_OBBCollider.SetTemporalCoherence(true);
	_AABBTreeCollider.SetTemporalCoherence(true);

	_SphereCollider.SetPrimitiveTests(false);
}

dxTriMesh::~dxTriMesh(){
	//
}

void dxTriMesh::ClearTCCache(){
	SphereTCCache.setSize(0);
	BoxTCCache.setSize(0);
}

int dxTriMesh::AABBTest(dxGeom* g, dReal aabb[6]){
	return 1;
}

void dxTriMesh::computeAABB(){
	aabb[0] = -dInfinity;
	aabb[1] = dInfinity;
	aabb[2] = -dInfinity;
	aabb[3] = dInfinity;
	aabb[4] = -dInfinity;
	aabb[5] = dInfinity;
}

dGeomID dCreateTriMesh(dSpaceID space, dTriMeshDataID Data, dTriCallback* Callback, dTriArrayCallback* ArrayCallback, dTriRayCallback* RayCallback){
	dxTriMesh* Geom = new dxTriMesh(space, Data);
	Geom->Callback = Callback;
	Geom->ArrayCallback = ArrayCallback;
	Geom->RayCallback = RayCallback;

	return Geom;
}

void dGeomTriMeshClearTC(dGeomID g){
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

	dxTriMesh* Geom = (dxTriMesh*)g;
	Geom->ClearTCCache();
}

// Getting data
void dGeomTriMeshGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2){
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

	dxTriMesh* Geom = (dxTriMesh*)g;

	const dVector3& Position = *(const dVector3*)dGeomGetPosition(g);
	const dMatrix3& Rotation = *(const dMatrix3*)dGeomGetRotation(g);

	dVector3 v[3];
	FetchTriangle(Geom, Index, Rotation, Position, v);

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
		(*v1)[3] = v[0][3];
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
	FetchTriangle(Geom, Index, Rotation, Position, dv);

	GetPointFromBarycentric(dv, u, v, Out);
}
