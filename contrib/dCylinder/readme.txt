Cylinder class includes collisions
Cylinder - Box Cylinder-Cylinder Cylinder - Sphere and Cylinder-Plain
I also included Cylinder -  Tri-Mesh in my version of Triangle
collider.

Cylinder aligned along axis - Y when created. (Not like Capped
Cylinder which aligned along axis - Z).

Interface is just the same as  Capped Cylinder has.

Use functions which have one "C" instead of double "C".

to create:
dGeomID dCreateCylinder (dSpaceID space, dReal radius, dReal length);

to set params:
void dGeomCylinderSetParams (dGeomID cylinder,
                              dReal radius, dReal length);


to get params:
void dGeomCylinderGetParams (dGeomID cylinder,
                              dReal *radius, dReal *length);

Return in radius and length the parameters of the given cylinder.

Identification number of the class:
 dCylinderClass

 I do not include a function that sets inertia tensor for cylinder.
 One may use existing ODE functions dMassSetCappedCylinder or dMassSetBox.
 To set exact tensor for cylinder use dMassSetParameters.
 Remember cylinder aligned along axis - Y.
 
 ///////////////////////////////////////////////////////////////////////////
 Konstantin Slipchenko
  October 9, 2002