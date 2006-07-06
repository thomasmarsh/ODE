README.TXT

dHeightfield collider for ODE

 Created by Martijn Buijs
 April 2006


Introduction

 dHeightfield is a heightfield collider for ODE, based on the "Terrain & Cone" contrib by Benoit Chaperot.
 It removes the terrain dimension limitations (supports non-square fields & grid spacing, non-power of two
 dimensions) and allows deformable terrains. It should work for animated water surfaces as well -- the reason
 I named it "dHeightfield" rather than "dTerrain". A bunch of fixes are included (AABB patch, callback patch),
 and the geom creation interface has been changed to mimic the trimesh collider.


Notes:

* This collider is NOT final. It wes tested under a limited number of situations. The Short sample
  path has not been tested, allthough it should work okay. Optimizations may be made. I encourage
  anyone using this collider to take a brief look at the code.
* Works fine alongside the Terrain & Cone contrib.
* Can confirm it works fine with sphere, box, capsule, cone colliders, not sure about rays.
* The short sample data path has not been tested, but I think it should work ok.
* I used the word "depth" for the Z dimension throughout the code to avoid confusion with the
  vertical "height" dimension. It is unrelated to other uses of the word troughout ODE.
* The vWidth and vDepth parameters in the dGeomHeightfieldDataBuild* are the world space dimensions
  of the heightfield. For example, if you have 30 meter sample data then vWidth = vWidthSamples * 30.
* The surface thickness can be specified, rather than treating the parameter as flag and setting the
  AABB minimum height bound to -dInfinity. This allows bodies to move underneath the terrain (for
  example in tunnels and overhangs modeled with trimesh colliders).
* With dGeomHeightfieldDataSetBounds the min and max height bounds can be set, this is usefull if
  you are using the callback path (and thus the bounds cannot be automatically computed) or have
  dynamic heightfields. When using the callback, the min and max bounds are initially set to
  inifinity, set them manually to speed things up.
* The min/max values passed to dGeomHeightfieldDataSetBounds are in sample space. It is scaled and
  offset (just like all other sample data) internally.


Open issues/wishlist/todo list:

* needs some work to make heightfields work with Z-up systems
* 'real' surface thickness (right now it only adds to the bottom of the AABB)
* a bit flag map to disable samples (to create holes in terrains)
* allow variable sample interpolation methods (preferably per callback)
* allow arbitrary heightfield orientations
* configuration independent floating point sample path
* DrawStuff draw function
* test_heightfield demo program


Bugs:

* There's one extra row of quads along the border of finite heightfields (presumably to make terrains
  seamlessly tileable?). This should be easy to fix by removing that feature. Rather than dealing with
  it internally, the sample data itself should be made tileable. I believe that is more correct.


New data types:

 dGeomHeightfieldDataID


New functions:

 dHeightfieldDataID dGeomHeightfieldDataCreate()

 void dGeomHeightfieldDataBuildCallback(dHeightfieldDataID d,
 					void *pUserData, dHeightfieldGetHeight *Callback,
 					int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
 					dReal vScale, dReal vOffset,
 					int nWrapMode, dReal vThickness)

 void dGeomHeightfieldDataBuildByte(dHeightfieldDataID d,
 					unsigned char *pHeightData, int bCopyHeightData,
 					int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
 					dReal vScale, dReal vOffset,
 					int nWrapMode, dReal vThickness)

 void dGeomHeightfieldDataBuildShort(dHeightfieldDataID d,
 					unsigned char *pHeightData, int bCopyHeightData,
 					int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
 					dReal vScale, dReal vOffset,
 					int nWrapMode, dReal vThickness)

 void dGeomHeightfieldDataBuildFloat(dHeightfieldDataID d,
 					dReal *pHeightData, int bCopyHeightData,
 					int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
 					dReal vScale, dReal vOffset,
 					int nWrapMode, dReal vThickness)

 void dGeomHeightfieldDataSetBounds(dHeightfieldDataID d, dReal vMinHeight, dReal vMaxHeight)

 void dGeomHeightfieldDataDestroy(dHeightfieldDataID d)

 dGeomID dCreateHeightfield(dSpaceID space, dHeightfieldDataID Data, int bPlaceable)

 void dGeomHeightfieldSetHeightfieldData(dGeomID g, dHeightfieldDataID Data)

 dHeightfieldDataID dGeomHeightfieldGetHeightfieldData(dGeomID g)

 dReal dGeomHeightfieldPointDepth(dGeomID g, dReal x, dReal y, dReal z)


Usage (ODE 0.5):
 
1) Add to folder "ode\src":

    dHeightfield.h
    dHeightfield.cpp

2) Add in "include\ode\collision.h":

    enum {
      dSphereClass = 0,
      dBoxClass,
      dCCylinderClass,
      dCylinderClass,
      dPlaneClass,
      dRayClass,
      dGeomTransformClass,
      dTriMeshClass,
    
      dHeightfieldClass, // <------- add this line
    
      dFirstSpaceClass,
      dSimpleSpaceClass = dFirstSpaceClass,
      dHashSpaceClass,
      dQuadTreeSpaceClass,
    
      dLastSpaceClass = dQuadTreeSpaceClass,
    
      dFirstUserClass,
      dLastUserClass = dFirstUserClass + dMaxUserClasses - 1,
      dGeomNumClasses
    };
    
    // <------- and from here...
    struct dxHeightfieldData;
    typedef struct dxHeightfieldData* dHeightfieldDataID;
    typedef float dHeightfieldGetHeight(void* pUserData, int x, int y);
    
    dHeightfieldDataID dGeomHeightfieldDataCreate();

    void dGeomHeightfieldDataBuildCallback(dHeightfieldDataID d,
         void *pUserData, dHeightfieldGetHeight *Callback,
         int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
         dReal vScale, dReal vOffset,
         int nWrapMode, dReal vThickness);
   
    void dGeomHeightfieldDataBuildByte(dHeightfieldDataID d,
         unsigned char *pHeightData, int bCopyHeightData,
         int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
         dReal vScale, dReal vOffset,
         int nWrapMode, dReal vThickness);
   
    void dGeomHeightfieldDataBuildShort(dHeightfieldDataID d,
         unsigned char *pHeightData, int bCopyHeightData,
         int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
         dReal vScale, dReal vOffset,
         int nWrapMode, dReal vThickness);
   
    void dGeomHeightfieldDataBuildFloat(dHeightfieldDataID d,
         dReal *pHeightData, int bCopyHeightData,
         int nWidthSamples, int nDepthSamples, dReal vWidth, dReal vDepth,
         dReal vScale, dReal vOffset,
         int nWrapMode, dReal vThickness);
         
    void dGeomHeightfieldDataSetBounds(dHeightfieldDataID d, dReal vMinHeight, dReal vMaxHeight);
    
    void dGeomHeightfieldDataDestroy(dHeightfieldDataID d);
    
    dGeomID dCreateHeightfield(dSpaceID space, dHeightfieldDataID Data, int bPlaceable);
    
    void dGeomHeightfieldSetHeightfieldData(dGeomID g, dHeightfieldDataID Data);
    
    dHeightfieldDataID dGeomHeightfieldGetHeightfieldData(dGeomID g);
    
    dReal dGeomHeightfieldPointDepth(dGeomID g, dReal x, dReal y, dReal z);
    // <------- to here

3) Add in "include\ode\odemath.h" (this can be skipped if the "Terrain & Cone" contrib is already applied):

    #define dOP(a,op,b,c) \
      (a)[0] = ((b)[0]) op ((c)[0]); \
      (a)[1] = ((b)[1]) op ((c)[1]); \
      (a)[2] = ((b)[2]) op ((c)[2]);
    #define dOPC(a,op,b,c) \
      (a)[0] = ((b)[0]) op (c); \
      (a)[1] = ((b)[1]) op (c); \
      (a)[2] = ((b)[2]) op (c);
    #define dOPE(a,op,b) \
      (a)[0] op ((b)[0]); \
      (a)[1] op ((b)[1]); \
      (a)[2] op ((b)[2]);
    #define dOPEC(a,op,c) \
      (a)[0] op (c); \
      (a)[1] op (c); \
      (a)[2] op (c);
    #define dLENGTH(a) \
     (dSqrt( ((a)[0])*((a)[0]) + ((a)[1])*((a)[1]) + ((a)[2])*((a)[2]) ));
    #define dLENGTHSQUARED(a) \
     (((a)[0])*((a)[0]) + ((a)[1])*((a)[1]) + ((a)[2])*((a)[2]));
     
4) Add in "ode\src\collision_kernel.cpp" line 137:

    setCollider (dHeightfieldClass,dSphereClass,&dCollideHeightfield);
    setCollider (dHeightfieldClass,dBoxClass,&dCollideHeightfield);
    setCollider (dHeightfieldClass,dCCylinderClass,&dCollideHeightfield);
    setCollider (dHeightfieldClass,dRayClass,&dCollideHeightfield);
    setCollider (dHeightfieldClass,dConeClass,&dCollideHeightfield);

5) Add in "ode\src\collision_std.h":

    int dCollideHeightfield(dxGeom *o1, dxGeom *o2, int flags,dContactGeom *contact, int skip);

6) Add "dHeightfield.cpp" to the ODE makefile, workspace etc.


Acknowledgements:
  
 Based on Terrain & Cone by:
  Benoit CHAPEROT 2003-2004
  http://www.jstarlab.com

 Geoff Carlton
  AABB patch for Terrain & Cone
  dTerrainZ height callback


Contact:
 
  buijs512 AT planet DOT nl
  http://home.planet.nl/~buijs512/



END OF FILE
