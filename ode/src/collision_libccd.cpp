/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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
#include <ccd/ccd.h>
#include <ccd/quat.h>
#include "config.h"
#include "odemath.h"
#include "collision_libccd.h"
#include "collision_std.h"
#if dTRIMESH_ENABLED
#include "collision_util.h"
#endif


struct _ccd_obj_t {
    ccd_vec3_t pos;
    ccd_quat_t rot, rot_inv;
};
typedef struct _ccd_obj_t ccd_obj_t;

struct _ccd_box_t {
    ccd_obj_t o;
    ccd_real_t dim[3];
};
typedef struct _ccd_box_t ccd_box_t;

struct _ccd_cap_t {
    ccd_obj_t o;
    ccd_real_t radius;
    ccd_vec3_t axis;
    ccd_vec3_t p1;
    ccd_vec3_t p2;
};
typedef struct _ccd_cap_t ccd_cap_t;

struct _ccd_cyl_t {
    ccd_obj_t o;
    ccd_real_t radius;
    ccd_vec3_t axis;
    ccd_vec3_t p1;
    ccd_vec3_t p2;
};
typedef struct _ccd_cyl_t ccd_cyl_t;

struct _ccd_sphere_t {
    ccd_obj_t o;
    ccd_real_t radius;
};
typedef struct _ccd_sphere_t ccd_sphere_t;

struct _ccd_convex_t {
    ccd_obj_t o;
    dxConvex *convex;
};
typedef struct _ccd_convex_t ccd_convex_t;

struct _ccd_triangle_t {
    ccd_obj_t o;
    ccd_vec3_t vertices[3];
};
typedef struct _ccd_triangle_t ccd_triangle_t;

/** Transforms geom to ccd struct */
static void ccdGeomToObj(const dGeomID g, ccd_obj_t *);
static void ccdGeomToBox(const dGeomID g, ccd_box_t *);
static void ccdGeomToCap(const dGeomID g, ccd_cap_t *);
static void ccdGeomToCyl(const dGeomID g, ccd_cyl_t *);
static void ccdGeomToSphere(const dGeomID g, ccd_sphere_t *);
static void ccdGeomToConvex(const dGeomID g, ccd_convex_t *);

/** Support functions */
static void ccdSupportBox(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportCap(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportCyl(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportSphere(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);

/** Center function */
static void ccdCenter(const void *obj, ccd_vec3_t *c);

/** General collide function */
static int ccdCollide(dGeomID o1, dGeomID o2, int flags,
                      dContactGeom *contact, int skip,
                      void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                      void *obj2, ccd_support_fn supp2, ccd_center_fn cen2);



static void ccdGeomToObj(const dGeomID g, ccd_obj_t *o)
{
    const dReal *ode_pos;
    dQuaternion ode_rot;

    ode_pos = dGeomGetPosition(g);
    dGeomGetQuaternion(g, ode_rot);

    ccdVec3Set(&o->pos, ode_pos[0], ode_pos[1], ode_pos[2]);
    ccdQuatSet(&o->rot, ode_rot[1], ode_rot[2], ode_rot[3], ode_rot[0]);

    ccdQuatInvert2(&o->rot_inv, &o->rot);
}

static void ccdGeomToBox(const dGeomID g, ccd_box_t *box)
{
    dVector3 dim;

    ccdGeomToObj(g, (ccd_obj_t *)box);

    dGeomBoxGetLengths(g, dim);
    box->dim[0] = dim[0] / 2.;
    box->dim[1] = dim[1] / 2.;
    box->dim[2] = dim[2] / 2.;
}

static void ccdGeomToCap(const dGeomID g, ccd_cap_t *cap)
{
    dReal r, h;
    ccdGeomToObj(g, (ccd_obj_t *)cap);

    dGeomCapsuleGetParams(g, &r, &h);
    cap->radius = r;
    ccdVec3Set(&cap->axis, 0.0, 0.0, h / 2);
    ccdQuatRotVec(&cap->axis, &cap->o.rot);
    ccdVec3Copy(&cap->p1, &cap->axis);
    ccdVec3Copy(&cap->p2, &cap->axis);
    ccdVec3Scale(&cap->p2, -1.0);
    ccdVec3Add(&cap->p1, &cap->o.pos);
    ccdVec3Add(&cap->p2, &cap->o.pos);
}

static void ccdGeomToCyl(const dGeomID g, ccd_cyl_t *cyl)
{
    dReal r, h;
    ccdGeomToObj(g, (ccd_obj_t *)cyl);

    dGeomCylinderGetParams(g, &r, &h);
    cyl->radius = r;
    ccdVec3Set(&cyl->axis, 0.0, 0.0, h / 2);
    ccdQuatRotVec(&cyl->axis, &cyl->o.rot);
    ccdVec3Copy(&cyl->p1, &cyl->axis);
    ccdVec3Copy(&cyl->p2, &cyl->axis);
    ccdVec3Normalize(&cyl->axis);
    ccdVec3Scale(&cyl->p2, -1.0);
    ccdVec3Add(&cyl->p1, &cyl->o.pos);
    ccdVec3Add(&cyl->p2, &cyl->o.pos);
}

static void ccdGeomToSphere(const dGeomID g, ccd_sphere_t *s)
{
    ccdGeomToObj(g, (ccd_obj_t *)s);
    s->radius = dGeomSphereGetRadius(g);
}

static void ccdGeomToConvex(const dGeomID g, ccd_convex_t *c)
{
    ccdGeomToObj(g, (ccd_obj_t *)c);
    c->convex = (dxConvex *)g;
}


static void ccdSupportBox(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_box_t *o = (const ccd_box_t *)obj;
    ccd_vec3_t dir;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &o->o.rot_inv);

    ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * o->dim[0],
        ccdSign(ccdVec3Y(&dir)) * o->dim[1],
        ccdSign(ccdVec3Z(&dir)) * o->dim[2]);

    // transform support vertex
    ccdQuatRotVec(v, &o->o.rot);
    ccdVec3Add(v, &o->o.pos);
}

static void ccdSupportCap(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_cap_t *o = (const ccd_cap_t *)obj;

    ccdVec3Copy(v, _dir);
    ccdVec3Scale(v, o->radius);

    if (ccdVec3Dot(_dir, &o->axis) > 0.0){
        ccdVec3Add(v, &o->p1);
    }else{
        ccdVec3Add(v, &o->p2);
    }

}

static void ccdSupportCyl(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_cyl_t *cyl = (const ccd_cyl_t *)obj;
    ccd_vec3_t dir;
    double len;

    double dot = ccdVec3Dot(_dir, &cyl->axis);
    if (dot > 0.0){
        ccdVec3Copy(v, &cyl->p1);
    } else{
        ccdVec3Copy(v, &cyl->p2);
    }
    ccdVec3Copy(&dir, &cyl->axis);
    ccdVec3Scale(&dir, -dot);
    ccdVec3Add(&dir, _dir);
    len = CCD_SQRT(ccdVec3Len2(&dir));
    if (!ccdIsZero(len)) {
        // project dir onto cylinder's 'top'/'bottom' plane
        ccdVec3Normalize(&dir);
        ccdVec3Scale(&dir, cyl->radius);
        ccdVec3Add(v, &dir);
    }

}

static void ccdSupportSphere(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_sphere_t *s = (const ccd_sphere_t *)obj;

    ccdVec3Copy(v, _dir);
    ccdVec3Scale(v, s->radius);
    dIASSERT(dFabs(CCD_SQRT(ccdVec3Len2(_dir)) - REAL(1.0)) < 1e-6); // ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Len2(_dir)));

    ccdVec3Add(v, &s->o.pos);
}

static void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_convex_t *c = (const ccd_convex_t *)obj;
    ccd_vec3_t dir, p;
    ccd_real_t maxdot, dot;
    size_t i;
    const dReal *curp;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &c->o.rot_inv);

    maxdot = -CCD_REAL_MAX;
    curp = c->convex->points;
    for (i = 0; i < c->convex->pointcount; i++, curp += 3){
        ccdVec3Set(&p, curp[0], curp[1], curp[2]);
        dot = ccdVec3Dot(&dir, &p);
        if (dot > maxdot){
            ccdVec3Copy(v, &p);
            maxdot = dot;
        }
    }


    // transform support vertex
    ccdQuatRotVec(v, &c->o.rot);
    ccdVec3Add(v, &c->o.pos);
}

static void ccdCenter(const void *obj, ccd_vec3_t *c)
{
    const ccd_obj_t *o = (const ccd_obj_t *)obj;
    ccdVec3Copy(c, &o->pos);
}

static int ccdCollide(
    dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip,
    void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
    void *obj2, ccd_support_fn supp2, ccd_center_fn cen2)
{
    ccd_t ccd;
    int res;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;
    int max_contacts = (flags & 0xffff);

    if (max_contacts < 1)
        return 0;

    CCD_INIT(&ccd);
    ccd.support1 = supp1;
    ccd.support2 = supp2;
    ccd.center1  = cen1;
    ccd.center2  = cen2;
    ccd.max_iterations = 500;
    ccd.mpr_tolerance = 1E-6;


    if (flags & CONTACTS_UNIMPORTANT){
        if (ccdMPRIntersect(obj1, obj2, &ccd)){
            return 1;
        }else{
            return 0;
        }
    }

    res = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
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

int dCollideCylinderCylinder(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cyl_t cyl1, cyl2;

    ccdGeomToCyl(o1, &cyl1);
    ccdGeomToCyl(o2, &cyl2);

    return ccdCollide(o1, o2, flags, contact, skip,
        &cyl1, ccdSupportCyl, ccdCenter,
        &cyl2, ccdSupportCyl, ccdCenter);
}

int dCollideBoxCylinderCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cyl_t cyl;
    ccd_box_t box;

    ccdGeomToBox(o1, &box);
    ccdGeomToCyl(o2, &cyl);

    return ccdCollide(o1, o2, flags, contact, skip,
        &box, ccdSupportBox, ccdCenter,
        &cyl, ccdSupportCyl, ccdCenter);
}

int dCollideCapsuleCylinder(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cap_t cap;
    ccd_cyl_t cyl;

    ccdGeomToCap(o1, &cap);
    ccdGeomToCyl(o2, &cyl);

    return ccdCollide(o1, o2, flags, contact, skip,
        &cap, ccdSupportCap, ccdCenter,
        &cyl, ccdSupportCyl, ccdCenter);
}

int dCollideConvexBoxCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_box_t box;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToBox(o2, &box);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &box, ccdSupportBox, ccdCenter);
}

int dCollideConvexCapsuleCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cap_t cap;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToCap(o2, &cap);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &cap, ccdSupportCap, ccdCenter);
}

int dCollideConvexSphereCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_sphere_t sphere;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToSphere(o2, &sphere);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &sphere, ccdSupportSphere, ccdCenter);
}

int dCollideConvexCylinderCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cyl_t cyl;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToCyl(o2, &cyl);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &cyl, ccdSupportCyl, ccdCenter);
}

int dCollideConvexConvexCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_convex_t c1, c2;

    ccdGeomToConvex(o1, &c1);
    ccdGeomToConvex(o2, &c2);

    return ccdCollide(o1, o2, flags, contact, skip,
        &c1, ccdSupportConvex, ccdCenter,
        &c2, ccdSupportConvex, ccdCenter);
}


#if dTRIMESH_ENABLED

static 
void ccdSupportTriangle(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_triangle_t* o = (ccd_triangle_t *) obj;
    ccd_real_t maxdot, dot;
    maxdot = -CCD_REAL_MAX;
    for (int i = 0; i < 3; i++) {
        dot = ccdVec3Dot(_dir, &o->vertices[i]);
        if (dot > maxdot) {
            ccdVec3Copy(v, &o->vertices[i]);
            maxdot = dot;
        }
    }
}

const static float CONTACT_DEPTH_EPSILON = 0.0001f;
const static float CONTACT_POS_EPSILON = 0.0001f;
const static float CONTACT_PERTURBATION_ANGLE = 0.001f;

static 
int addUniqueContact(dContactGeom *contacts, dContactGeom *c, int contactcount, int maxcontacts, int flags, int skip)
{
    dReal minDepth = dInfinity;
    int index = contactcount;
    for (int k = 0; k < contactcount; k++) {
        dContactGeom* pc = SAFECONTACT(flags, contacts, k, skip);
        if (fabs(c->pos[0] - pc->pos[0]) < CONTACT_POS_EPSILON
            && fabs(c->pos[1] - pc->pos[1]) < CONTACT_POS_EPSILON
            && fabs(c->pos[2] - pc->pos[2]) < CONTACT_POS_EPSILON) {
            index = pc->depth + CONTACT_DEPTH_EPSILON < c->depth ? k : maxcontacts;
            break;
        }
        if (contactcount == maxcontacts && pc->depth < minDepth && pc->depth < c->depth) {
            minDepth = pc->depth;
            index = k;
        }
    }
    if (index < maxcontacts) {
        dContactGeom* contact = SAFECONTACT(flags, contacts, index, skip);
        contact->g1 = c->g1;
        contact->g2 = c->g2;
        contact->depth = c->depth;
        contact->side1 = c->side1;
        contact->side2 = c->side2;
        contact->pos[0] = c->pos[0];
        contact->pos[1] = c->pos[1];
        contact->pos[2] = c->pos[2];
        contact->normal[0] = c->normal[0];
        contact->normal[1] = c->normal[1];
        contact->normal[2] = c->normal[2];
        contactcount = index == contactcount ? contactcount + 1 : contactcount;
    }
    return contactcount;
}

static 
int addPerturbedContacts(dxGeom* o1, dxGeom* o2, int flags, dContactGeom *contacts, int skip,
        ccd_convex_t* c1, ccd_triangle_t* c2, dVector3* triangle, dContactGeom* contact, int contactcount)
{
    int maxcontacts = (flags & 0xffff);
    dVector3 upAxis, cross;
    dVector3 pos;
    
    pos[0] = contact->pos[0];
    pos[1] = contact->pos[1];
    pos[2] = contact->pos[2];

    upAxis[0] = 0;
    upAxis[1] = 1;
    upAxis[2] = 0;
    if (fabs(dVector3Dot(contact->normal, upAxis)) > 0.7) {
        upAxis[0] = 0;
        upAxis[1] = 0;
        upAxis[2] = 1;
    }

    dVector3Cross(contact->normal, upAxis, cross);
    if (dSafeNormalize3(cross)) {

        dVector3Cross(cross, contact->normal, upAxis);
        if (dSafeNormalize3(upAxis)) {

            for (int k = 0; k < 4; k++) {
                dContactGeom perturbedContact;
                dQuaternion q1, q2, qr;
                dQFromAxisAndAngle(q1, upAxis[0], upAxis[1], upAxis[2], k % 2 == 0 ? CONTACT_PERTURBATION_ANGLE : -CONTACT_PERTURBATION_ANGLE);
                dQFromAxisAndAngle(q2, cross[0], cross[1], cross[2], k / 2 == 0 ? CONTACT_PERTURBATION_ANGLE : -CONTACT_PERTURBATION_ANGLE);
                dQMultiply0(qr, q1, q2);
                
                for (int j = 0; j < 3; j++) {
                    dVector3 p, perturbed;
                    dVector3Subtract(triangle[j], pos, p);
                    dQuatTransform(qr, p, perturbed);
                    dVector3Add(perturbed, pos, perturbed);
                    
                    c2->vertices[j].v[0] = perturbed[0];
                    c2->vertices[j].v[1] = perturbed[1];
                    c2->vertices[j].v[2] = perturbed[2];
                }

                if (ccdCollide(o1, o2, flags, &perturbedContact, skip, c1, ccdSupportConvex, ccdCenter, c2, ccdSupportTriangle, ccdCenter) == 1) {
                    perturbedContact.side2 = contact->side2;
                    contactcount = addUniqueContact(contacts, &perturbedContact, contactcount, maxcontacts, flags, skip);
                }
            }
        }
    }

    return contactcount;
}

/*extern */
int dCollideConvexTrimeshTrianglesCCD(dxGeom *o1, dxGeom *o2, int* indices, int numindices, int flags, dContactGeom *contacts, int skip)
{
    ccd_convex_t c1;
    ccd_triangle_t c2;
    dVector3 triangle[3];
    int maxcontacts = (flags & 0xffff);
    int contactcount = 0;
    ccdGeomToConvex(o1, &c1);
    ccdGeomToObj(o2, (ccd_obj_t *)&c2);

    for (int i = 0; i < numindices; i++) {
        dContactGeom tempcontact;
        dGeomTriMeshGetTriangle(o2, indices[i], &triangle[0], &triangle[1], &triangle[2]);

        for (int j = 0; j < 3; j++) {
            c2.vertices[j].v[0] = triangle[j][0];
            c2.vertices[j].v[1] = triangle[j][1];
            c2.vertices[j].v[2] = triangle[j][2];
        }

        if (ccdCollide(o1, o2, flags, &tempcontact, skip, &c1, ccdSupportConvex, ccdCenter, &c2, ccdSupportTriangle, ccdCenter) == 1) {
            tempcontact.side2 = i;
            contactcount = addUniqueContact(contacts, &tempcontact, contactcount, maxcontacts, flags, skip);

            if ((flags & CONTACTS_UNIMPORTANT)) {
                break;
            }
        }
    }

    if (contactcount == 1 && !(flags & CONTACTS_UNIMPORTANT)) {
        dContactGeom* contact = SAFECONTACT(flags, contacts, 0, skip);
        dGeomTriMeshGetTriangle(o2, contact->side2, &triangle[0], &triangle[1], &triangle[2]);
        contactcount = addPerturbedContacts(o1, o2, flags, contacts, skip, &c1, &c2, triangle, contact, contactcount);
    }

    return contactcount;
}

#endif // dTRIMESH_ENABLED
