
#ifndef dCylinder_h
#define dCylinder_h


struct dxCylinder;
extern int dCylinderClass;


dxGeom *dCreateCylinder (dSpaceID space, dReal r, dReal lz);
void dGeomCylinderSetParams (dGeomID g, dReal radius, dReal length);

void dGeomCylinderGetParams (dGeomID g, dReal *radius, dReal *length);


#endif
