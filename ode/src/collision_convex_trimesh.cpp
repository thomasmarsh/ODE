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


typedef struct _sLocalContactData
{
    dVector3	vPos;
    dVector3	vNormal;
    dReal		fDepth;
    int			triIndex;
    int			nFlags; // 0 = filtered out, 1 = OK
}sLocalContactData;


#if dTRIMESH_ENABLED

#include "collision_util.h"
#include "collision_std.h"
#include "collision_trimesh_internal.h"
#if dLIBCCD_ENABLED
#include "collision_libccd.h"
#endif

int dCollideConvexTrimesh( dxGeom *o1, dxGeom *o2, int flags, dContactGeom* contacts, int skip )
{
    int contactcount = 0;
    dIASSERT( skip >= (int)sizeof( dContactGeom ) );
    dIASSERT( o1->type == dConvexClass );
    dIASSERT( o2->type == dTriMeshClass );
    dIASSERT ((flags & NUMC_MASK) >= 1);

#if dLIBCCD_ENABLED

#if dTRIMESH_OPCODE



#elif dTRIMESH_GIMPACT
    dxTriMesh *trimesh = (dxTriMesh *)o2;

    aabb3f test_aabb;
    test_aabb.minX = o1->aabb[0];
    test_aabb.maxX = o1->aabb[1];
    test_aabb.minY = o1->aabb[2];
    test_aabb.maxY = o1->aabb[3];
    test_aabb.minZ = o1->aabb[4];
    test_aabb.maxZ = o1->aabb[5];

    GDYNAMIC_ARRAY collision_result;
    GIM_CREATE_BOXQUERY_LIST(collision_result);

    gim_aabbset_box_collision(&test_aabb, &trimesh->m_collision_trimesh.m_aabbset, &collision_result);

    if (collision_result.m_size != 0)
    {
        GUINT32 * boxesresult = GIM_DYNARRAY_POINTER(GUINT32,collision_result);
        GIM_TRIMESH * ptrimesh = &trimesh->m_collision_trimesh;
        gim_trimesh_locks_work_data(ptrimesh);

        contactcount = dCollideConvexTrimeshTrianglesCCD(o1, o2, (int *)boxesresult, collision_result.m_size, flags, contacts, skip);

        gim_trimesh_unlocks_work_data(ptrimesh);
    }

    GIM_DYNARRAY_DESTROY(collision_result);
#endif // dTRIMESH_GIMPACT

#endif // dLIBCCD_ENABLED

    return contactcount;
}

#endif // dTRIMESH_ENABLED

