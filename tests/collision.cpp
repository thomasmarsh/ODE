#include <UnitTest++.h>
#include <ode/ode.h>

TEST(test_collision_trimesh_sphere)
{
    dInitODE();

    {
        const int VertexCount = 4;
        const int IndexCount = 2*3;
        // this is a square on the XY plane
        float vertices[VertexCount * 3] = {
            0,0,0,
            1,0,0,
            1,1,0,
            0,1,0
        };
        dTriIndex indices[IndexCount] = {
            0,1,2,
            0,2,3
        };
        
        dTriMeshDataID data = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildSingle(data,
                                    vertices,
                                    3 * sizeof(float),
                                    VertexCount,
                                    indices,
                                    IndexCount,
                                    3 * sizeof(dTriIndex));
        dGeomID trimesh = dCreateTriMesh(0, data, 0, 0, 0);
        const dReal radius = 4;
        dGeomID sphere = dCreateSphere(0, radius);
        dGeomSetPosition(sphere, 0,0,radius);
        dContactGeom cg[4];
        int nc;

        // check extreme case
        nc = dCollide(trimesh, sphere, 4, &cg[0], sizeof cg[0]);
        CHECK_EQUAL(1, nc);
        CHECK_EQUAL(0, cg[0].depth);
        
        // now translate both geoms
        dGeomSetPosition(trimesh, 10,30,40);
        dGeomSetPosition(sphere, 10,30,40+radius);
        // check extreme case, again
        nc = dCollide(trimesh, sphere, 4, &cg[0], sizeof cg[0]);
        CHECK_EQUAL(1, nc);
        CHECK_EQUAL(0, cg[0].depth);
        
        // and now, let's rotate the trimesh, 90 degrees on X
        dMatrix3 rot = { 1, 0, 0, 0,
                         0, 0, -1, 0,
                         0, 1, 0, 0 };
        dGeomSetPosition(trimesh, 10,30,40);
        dGeomSetRotation(trimesh, rot);
        
        dGeomSetPosition(sphere, 10,30-radius,40);
        // check extreme case, again
        nc = dCollide(trimesh, sphere, 4, &cg[0], sizeof cg[0]);
        CHECK_EQUAL(1, nc);
        CHECK_EQUAL(0, cg[0].depth);
    }
}
