#include <UnitTest++.h>
#include <ode/ode.h>
#include "common.h"

TEST(test_collision_sphere_point_depth)
{
    // Test case: sphere at the origin.
    {
        const dReal radius = 1;
        dGeomID sphere = dCreateSphere(0, radius);

        dGeomSetPosition(sphere, 0,0,0);

        // depth at center should equal radius
        CHECK_EQUAL(radius, dGeomSpherePointDepth(sphere, 0,0,0));

        // half-radius depth
        CHECK_EQUAL(0.5 * radius, dGeomSpherePointDepth(sphere, 0.5,   0,   0));
        CHECK_EQUAL(0.5 * radius, dGeomSpherePointDepth(sphere,   0, 0.5,   0));
        CHECK_EQUAL(0.5 * radius, dGeomSpherePointDepth(sphere,   0,   0, 0.5));
        CHECK_EQUAL(0.5 * radius, dGeomSpherePointDepth(sphere, -0.5,    0,    0));
        CHECK_EQUAL(0.5 * radius, dGeomSpherePointDepth(sphere,    0, -0.5,    0));
        CHECK_EQUAL(0.5 * radius, dGeomSpherePointDepth(sphere,    0,    0, -0.5));
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere, 0.3, 0.4,   0), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere,   0, 0.3, 0.4), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere, 0.4,   0, 0.3), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere, -0.3,  0.4,    0), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere,    0, -0.3,  0.4), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere,  0.4,    0, -0.3), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere,  0.3, -0.4,    0), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere,    0,  0.3, -0.4), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere, -0.4,    0,  0.3), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere, -0.3, -0.4,    0), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere,    0, -0.3, -0.4), 1e-12);
        CHECK_CLOSE(0.5 * radius, dGeomSpherePointDepth(sphere, -0.4,    0, -0.3), 1e-12);

        // 0.1 radius depth
        CHECK_CLOSE(0.1 * radius, dGeomSpherePointDepth(sphere, 0.9,   0,   0), 1e-12);
        CHECK_CLOSE(0.1 * radius, dGeomSpherePointDepth(sphere,   0, 0.9,   0), 1e-12);
        CHECK_CLOSE(0.1 * radius, dGeomSpherePointDepth(sphere,   0,   0, 0.9), 1e-12);
        CHECK_CLOSE(0.1 * radius, dGeomSpherePointDepth(sphere, -0.9,    0,    0), 1e-12);
        CHECK_CLOSE(0.1 * radius, dGeomSpherePointDepth(sphere,    0, -0.9,    0), 1e-12);
        CHECK_CLOSE(0.1 * radius, dGeomSpherePointDepth(sphere,    0,    0, -0.9), 1e-12);

        // on surface (zero depth)
        CHECK_EQUAL(0, dGeomSpherePointDepth(sphere, 1.0,   0,   0));
        CHECK_EQUAL(0, dGeomSpherePointDepth(sphere,   0, 1.0,   0));
        CHECK_EQUAL(0, dGeomSpherePointDepth(sphere,   0,   0, 1.0));
        CHECK_EQUAL(0, dGeomSpherePointDepth(sphere, -1.0,    0,    0));
        CHECK_EQUAL(0, dGeomSpherePointDepth(sphere,    0, -1.0,    0));
        CHECK_EQUAL(0, dGeomSpherePointDepth(sphere,    0,    0, -1.0));
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere, 0.6, 0.8,   0), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere,   0, 0.6, 0.8), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere, 0.8,   0, 0.6), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere, -0.6,  0.8,    0), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere,    0, -0.6,  0.8), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere,  0.8,    0, -0.6), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere,  0.6, -0.8,    0), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere,    0,  0.6, -0.8), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere, -0.8,    0,  0.6), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere, -0.6, -0.8,    0), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere,    0, -0.6, -0.8), 1e-12);
        CHECK_CLOSE(0, dGeomSpherePointDepth(sphere, -0.8,    0, -0.6), 1e-12);

        // 0.1 radius from surface (negative depth)
        CHECK_CLOSE(-0.1 * radius, dGeomSpherePointDepth(sphere, 1.1,   0,   0), 1e-12);
        CHECK_CLOSE(-0.1 * radius, dGeomSpherePointDepth(sphere,   0, 1.1,   0), 1e-12);
        CHECK_CLOSE(-0.1 * radius, dGeomSpherePointDepth(sphere,   0,   0, 1.1), 1e-12);
        CHECK_CLOSE(-0.1 * radius, dGeomSpherePointDepth(sphere, -1.1,    0,    0), 1e-12);
        CHECK_CLOSE(-0.1 * radius, dGeomSpherePointDepth(sphere,    0, -1.1,    0), 1e-12);
        CHECK_CLOSE(-0.1 * radius, dGeomSpherePointDepth(sphere,    0,    0, -1.1), 1e-12);

        // half-radius from surface (negative depth)
        CHECK_EQUAL(-0.5 * radius, dGeomSpherePointDepth(sphere, 1.5,   0,   0));
        CHECK_EQUAL(-0.5 * radius, dGeomSpherePointDepth(sphere,   0, 1.5,   0));
        CHECK_EQUAL(-0.5 * radius, dGeomSpherePointDepth(sphere,   0,   0, 1.5));
        CHECK_EQUAL(-0.5 * radius, dGeomSpherePointDepth(sphere, -1.5,    0,    0));
        CHECK_EQUAL(-0.5 * radius, dGeomSpherePointDepth(sphere,    0, -1.5,    0));
        CHECK_EQUAL(-0.5 * radius, dGeomSpherePointDepth(sphere,    0,    0, -1.5));
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere, 0.9, 1.2,   0), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere,   0, 0.9, 1.2), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere, 1.2,   0, 0.9), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere, -0.9,  1.2,    0), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere,    0, -0.9,  1.2), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere,  1.2,    0, -0.9), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere,  0.9, -1.2,    0), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere,    0,  0.9, -1.2), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere, -1.2,    0,  0.9), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere, -0.9, -1.2,    0), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere,    0, -0.9, -1.2), 1e-12);
        CHECK_CLOSE(-0.5, dGeomSpherePointDepth(sphere, -1.2,    0, -0.9), 1e-12);
    }
}
