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


#if dTRIMESH_ENABLED

#include "collision_trimesh_internal.h"
#include "odeou.h"


//////////////////////////////////////////////////////////////////////////

/*extern */
void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
    const dReal* Vertices, int VertexCount, 
    const dTriIndex* Indices, int IndexCount,
    const int *Normals)
{
#ifdef dSINGLE
    dGeomTriMeshDataBuildSingle1(g,
        Vertices, 4 * sizeof(dReal), VertexCount, 
        Indices, IndexCount, 3 * sizeof(dTriIndex),
        Normals);
#else
    dGeomTriMeshDataBuildDouble1(g, Vertices, 4 * sizeof(dReal), VertexCount, 
        Indices, IndexCount, 3 * sizeof(dTriIndex),
        Normals);
#endif
}


/*extern */
void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
    const void* Vertices, int VertexStride, int VertexCount, 
    const void* Indices, int IndexCount, int TriStride)
{
    dGeomTriMeshDataBuildSingle1(g, Vertices, VertexStride, VertexCount,
        Indices, IndexCount, TriStride, (const void *)NULL);
}

/*extern */
void dGeomTriMeshDataBuildDouble(dTriMeshDataID g,
    const void* Vertices, int VertexStride, int VertexCount, 
    const void* Indices, int IndexCount, int TriStride)
{
    dGeomTriMeshDataBuildDouble1(g, Vertices, VertexStride, VertexCount,
        Indices, IndexCount, TriStride, NULL);
}

/*extern */
void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
    const dReal* Vertices, int VertexCount, 
    const dTriIndex* Indices, int IndexCount)
{
    dGeomTriMeshDataBuildSimple1(g,
        Vertices, VertexCount, Indices, IndexCount,
        (int *)NULL);
}


/*extern */
void dGeomTriMeshDataPreprocess(dTriMeshDataID g)
{
    dUASSERT(g, "The argument is not a trimesh data");

    dxTriMeshData *data = g;
    data->preprocess();
}

/*extern */
void dGeomTriMeshDataUpdate(dTriMeshDataID g) 
{
    dUASSERT(g, "The argument is not a trimesh data");

    dxTriMeshData *data = g;
    data->updateData();
}


//////////////////////////////////////////////////////////////////////////

/*extern */
void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->assignCallback(Callback);
}

/*extern */
dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveCallback();
}

/*extern */
void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->assignArrayCallback(ArrayCallback);
}

/*extern */
dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveArrayCallback();
}

/*extern */
void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->assignRayCallback(Callback);
}

/*extern */
dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");	

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveRayCallback();
}

/*extern */
void dGeomTriMeshSetTriMergeCallback(dGeomID g, dTriTriMergeCallback* Callback)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->assignTriMergeCallback(Callback);
}

/*extern */
dTriTriMergeCallback* dGeomTriMeshGetTriMergeCallback(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");	

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveTriMergeCallback();
}

/*extern */
void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->assignMeshData(Data);
}

/*extern */
dTriMeshDataID dGeomTriMeshGetData(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveMeshData();
}


BEGIN_NAMESPACE_OU();
template<>
const int CEnumSortedElementArray<dxTriMesh::TRIMESHTC, dxTriMesh::TTC__MAX, int, 0x161003D5>::m_aetElementArray[] =
{
    dSphereClass, // TTC_SPHERE,
    dBoxClass, // TTC_BOX,
    dCapsuleClass, // TTC_CAPSULE,
};
END_NAMESPACE_OU();

static const CEnumSortedElementArray<dxTriMesh::TRIMESHTC, dxTriMesh::TTC__MAX, int, 0x161003D5> g_asiMeshTCGeomClasses;

/*extern */
void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);

    dxTriMesh::TRIMESHTC tc = g_asiMeshTCGeomClasses.Decode(geomClass);

    if (g_asiMeshTCGeomClasses.IsValidDecode(tc))
    {
        mesh->assignDoTC(tc, enable != 0);
    }
}

/*extern */
int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);

    dxTriMesh::TRIMESHTC tc = g_asiMeshTCGeomClasses.Decode(geomClass);

    bool result = g_asiMeshTCGeomClasses.IsValidDecode(tc) 
        && mesh->retrieveDoTC(tc);
    return result;
}


/*extern */
dTriMeshDataID dGeomTriMeshGetTriMeshDataID(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveMeshData();
}


/*extern */
void dGeomTriMeshClearTCCache(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->clearTCCache();
}


/*extern */
int dGeomTriMeshGetTriangleCount(dGeomID g)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    unsigned result = mesh->getMeshTriangleCount();
    return result;
}


/*extern */
void dGeomTriMeshGetTriangle(dGeomID g, int index, dVector3 *v0/*=NULL*/, dVector3 *v1/*=NULL*/, dVector3 *v2/*=NULL*/)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
    dUASSERT(v0 != NULL || v1 != NULL || v2 != NULL, "A meaningless call");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);

    dVector3 *pv[3] = { v0, v1, v2 };
    mesh->fetchMeshTransformedTriangle(pv, index);
}

/*extern */
void dGeomTriMeshGetPoint(dGeomID g, int index, dReal u, dReal v, dVector3 Out)
{
    dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);

    dVector3 dv[3];
    mesh->fetchMeshTransformedTriangle(dv, index);

    GetPointFromBarycentric(dv, u, v, Out);
}


#endif // #if dTRIMESH_ENABLED

