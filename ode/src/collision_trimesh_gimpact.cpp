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

#include <ode/collision.h>
#include <ode/rotation.h>
#include "config.h"
#include "matrix.h"
#include "odemath.h"


#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

#include "collision_util.h"
#include "collision_trimesh_gimpact.h"


void dxTriMeshData::build(const void *Vertices, int VertexStride, unsigned VertexCount,
    const void *Indices, unsigned IndexCount, int TriStride,
    const void *Normals,
    bool Single)
{
    dIASSERT(Vertices);
    dIASSERT(Indices);
    dIASSERT(VertexStride);
    dIASSERT(TriStride);
    dIASSERT(IndexCount);
    dIASSERT(IndexCount % 3 == 0);

    m_Vertices = (const uint8 *)Vertices;
    m_VertexStride = VertexStride;
    m_VertexCount = VertexCount;
    m_Indices = (const uint8 *)Indices;
    m_TriangleCount = IndexCount / 3;
    m_TriStride = TriStride;
    m_single = Single;
}


//////////////////////////////////////////////////////////////////////////

// Trimesh

dxTriMesh::~dxTriMesh()
{
    //Terminate Trimesh
    gim_trimesh_destroy(&m_collision_trimesh);
    gim_terminate_buffer_managers(m_buffer_managers);
}


/*virtual */
void dxTriMesh::computeAABB()
{
    //update trimesh transform
    mat4f transform;
    IDENTIFY_MATRIX_4X4(transform);
    MakeMatrix(this, transform);
    gim_trimesh_set_tranform(&m_collision_trimesh, transform);

    //Update trimesh boxes
    gim_trimesh_update(&m_collision_trimesh);

    GIM_AABB_COPY( &m_collision_trimesh.m_aabbset.m_global_bound, aabb );
}


void dxTriMesh::assignMeshData(dxTriMeshData *Data)
{
    // GIMPACT only supports stride 12, so we need to catch the error early.
    dUASSERT(
        Data->m_VertexStride == 3*sizeof(float) && Data->m_TriStride == 3*sizeof(int),
        "Gimpact trimesh only supports a stride of 3 float/int\n"
        "This means that you cannot use dGeomTriMeshDataBuildSimple() with Gimpact.\n"
        "Change the stride, or use Opcode trimeshes instead.\n"
    );

    dxTriMesh_Parent::assignMeshData(Data);

    //Create trimesh
    if ( Data->m_Vertices != NULL )
    {
        gim_trimesh_create_from_data(
            m_buffer_managers,
            &m_collision_trimesh,		        // gimpact mesh
            (vec3f *)(&Data->m_Vertices[0]),	// vertices
            Data->m_VertexCount,		        // nr of verts
            0,					                // copy verts?
            (GUINT32 *)(&Data->m_Indices[0]),	// indices
            Data->m_TriangleCount * 3,		    // nr of indices
            0,					                // copy indices?
            1					                // transformed reply
        );
    }
}


//////////////////////////////////////////////////////////////////////////

/*extern */
dTriMeshDataID dGeomTriMeshDataCreate()
{
    return new dxTriMeshData();
}

/*extern */
void dGeomTriMeshDataDestroy(dTriMeshDataID g)
{
    dxTriMeshData *data = g;
    delete data;
}

/*extern */
void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void *in_data) 
{
    dUASSERT(g, "The argument is not a trimesh data");

    //stub
}

/*extern */
void *dGeomTriMeshDataGet(dTriMeshDataID g, int data_id) 
{
    dUASSERT(g, "The argument is not a trimesh data");
    
    return NULL; // stub
}

/*extern */
void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
    const void* Vertices, int VertexStride, int VertexCount,
    const void* Indices, int IndexCount, int TriStride,
    const void* Normals)
{
    dUASSERT(g, "The argument is not a trimesh data");
    dAASSERT(Vertices);
    dAASSERT(Indices);

    dxTriMeshData *data = g;

    data->build(Vertices, VertexStride, VertexCount,
        Indices, IndexCount, TriStride,
        Normals,
        true);
}

/*extern */
void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g,
    const void* Vertices, int VertexStride, int VertexCount,
    const void* Indices, int IndexCount, int TriStride,
    const void* Normals)
{
    dUASSERT(g, "The argument is not a trimesh data");
    dAASSERT(Vertices);
    dAASSERT(Indices);

    dxTriMeshData *data = g;

    data->build(Vertices, VertexStride, VertexCount,
        Indices, IndexCount, TriStride,
        Normals,
        false);
}


/*extern */
void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char ** buf, int *bufLen)
{
    dUASSERT(g, "The argument is not a trimesh data");

    *buf = NULL;
    *bufLen = 0;
}

/*extern */
void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char *buf)
{
    dUASSERT(g, "The argument is not a trimesh data");
    dAASSERT(buf == NULL); // The buffer assignment is not implemented
}


//////////////////////////////////////////////////////////////////////////

/*extern */
dGeomID dCreateTriMesh(dSpaceID space,
    dTriMeshDataID Data,
    dTriCallback* Callback,
    dTriArrayCallback* ArrayCallback,
    dTriRayCallback* RayCallback)
{
    dxTriMesh *mesh = new dxTriMesh(space, Data, Callback, ArrayCallback, RayCallback);
    return mesh;
}


/*extern */
void dGeomTriMeshSetLastTransform(dGeomID g, const dMatrix4 last_trans ) 
{
    dAASSERT(g);
    dUASSERT(g->type == dTriMeshClass, "The geom is not a trimesh");

    //stub
}

/*extern */
const dReal *dGeomTriMeshGetLastTransform(dGeomID g)
{
    dAASSERT(g);
    dUASSERT(g->type == dTriMeshClass, "The geom is not a trimesh");

    return NULL; // stub
}


#endif // #if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

