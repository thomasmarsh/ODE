#include <ode/collision.h>
#include <ode/odemath.h>
#include <ccd/ccd.h>
#include <ccd/quat.h>
#include "collision_libccd.h"

struct _cyl_t {
    ccd_real_t radius, height;
    ccd_vec3_t pos;
    ccd_quat_t rot, rot_inv;
};
typedef struct _cyl_t cyl_t;

static void geomToCyl(const dGeomID geom, cyl_t *cyl)
{
    const dReal *ode_pos;
    dQuaternion ode_rot;

    dGeomCylinderGetParams(geom, &cyl->radius, &cyl->height);
    ode_pos = dGeomGetPosition(geom);
    dGeomGetQuaternion(geom, ode_rot);

    ccdVec3Set(&cyl->pos, ode_pos[0], ode_pos[1], ode_pos[2]);
    ccdQuatSet(&cyl->rot, ode_rot[1], ode_rot[2], ode_rot[3], ode_rot[0]);

    ccdQuatInvert2(&cyl->rot_inv, &cyl->rot);
}

static void supportCyl(const void *obj, const ccd_vec3_t *_dir,
                       ccd_vec3_t *v)
{
    const cyl_t *cyl = (const cyl_t *)obj;
    ccd_vec3_t dir;
    double zdist, rad;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &cyl->rot_inv);

    zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
    zdist = sqrt(zdist);
    if (ccdIsZero(zdist)){
        ccdVec3Set(v, 0., 0., ccdSign(ccdVec3Z(&dir)) * cyl->height * 0.5);
    }else{
        rad = cyl->radius / zdist;

        ccdVec3Set(v, rad * ccdVec3X(&dir),
                      rad * ccdVec3Y(&dir),
                      ccdSign(ccdVec3Z(&dir)) * cyl->height * 0.5);
    }

    // transform support vertex
    ccdQuatRotVec(v, &cyl->rot);
    ccdVec3Add(v, &cyl->pos);
}

static void centerCyl(const void *obj1, ccd_vec3_t *center)
{
    const cyl_t *cyl = (const cyl_t *)obj1;
    ccdVec3Copy(center, &cyl->pos);
}


int dCollideCylinderCylinder(dxGeom *o1, dxGeom *o2, int flags,
			                 dContactGeom *contact, int skip)
{
    ccd_t ccd;
    int res;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;
    int max_contacts = (flags & 0xffff);
    cyl_t cyl1, cyl2;

    if (max_contacts < 1)
        return 0;

    CCD_INIT(&ccd);
    ccd.support1 = supportCyl;
    ccd.support2 = supportCyl;
    ccd.center1  = centerCyl;
    ccd.center2  = centerCyl;
    ccd.mpr_tolerance = 1E-6;

    geomToCyl((dGeomID)o1, &cyl1);
    geomToCyl((dGeomID)o2, &cyl2);

    res = ccdMPRPenetration(&cyl1, &cyl2, &ccd, &depth, &dir, &pos);
    if (res == 0){
        contact->g1 = o1;
        contact->g2 = o2;

        contact->side1 = contact->side2 = -1;

        contact->depth = depth;

        contact->pos[0] = ccdVec3X(&pos);
        contact->pos[1] = ccdVec3Y(&pos);
        contact->pos[2] = ccdVec3Z(&pos);

        ccdVec3Scale(&dir, -1.);
        contact->normal[0] = ccdVec3X(&dir);
        contact->normal[1] = ccdVec3Y(&dir);
        contact->normal[2] = ccdVec3Z(&dir);

        return 1;
    }

    return 0;
}
