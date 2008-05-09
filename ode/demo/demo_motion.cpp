/*
    This demo shows how to use dContactMotionN in a lifting platform.
 */
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif


// some constants

#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 8		// maximum number of contact points per body
#define USE_GEOM_OFFSET 1

// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static int write_world = 0;
static int show_body = 0;

static dGeomID platform, ground;
static dReal platpos = 0;
const dReal maxplatpos = 30;
const dReal platspeed = 0.2;

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    dMatrix3 RI;
    static const dReal ss[3] = {0.02,0.02,0.02};

    dContact contact[MAX_CONTACTS];
    int numc = dCollide (o1, o2, MAX_CONTACTS,
                        &contact[0].geom, sizeof(dContact));
    if (numc)
        dRSetIdentity(RI);

    bool isplatform = (o1 == platform) || (o2 == platform);

    for (int i=0; i< numc; i++) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.0;
        
        if (isplatform) {
            contact[i].surface.mode |= dContactMotionN;
            contact[i].surface.motionN = platspeed;
            /* Note: for arbitrary contact directions, you
                need to project the bodyless geom's velocity against
                the contact normal.
                For example, assuming o1 is the platform:
                    motionN = -dDOT(normal, platvel);
                and for o2 = platform:
                    motionN = dDOT(normal, platvel);
                Where:
                    normal = contac[i].geom.normal (always points into o1)
                    platvel = velocity vector of the platform
                Note that for arbitrary movement you will also have to
                use dContactFDir1, dContactMotion1 and dContactMotion2.
            */
        }

        dJointID c = dJointCreateContact (world,contactgroup,contact+i);
        dJointAttach (c, dGeomGetBody(o1), dGeomGetBody(o2));
        if (show_contacts) 
            dsDrawBox (contact[i].geom.pos, RI, ss);
    }
}


// start simulation - set viewpoint

static float xyz[3] = {2.1106f,-1.3007,2.f};
static float hpr[3] = {150.f,-13.5000f,0.0000f};

static void start()
{
  //dAllocateODEDataForThread(dAllocateMaskAll);
  dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   b for box.\n");
  printf ("   s for sphere.\n");
  printf ("   c for capsule.\n");
  printf ("   y for cylinder.\n");
  printf ("To toggle showing the geom AABBs, press a.\n");
  printf ("To toggle showing the contact points, press t.\n");
  printf ("To toggle dropping from random position/orientation, press r.\n");
  printf ("To save the current state to 'state.dif', press 1.\n");
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed

static void command (int cmd)
{
  size_t i;
  int j,k;
  dReal sides[3];
  dMass m;
  int setBody;
  
  cmd = locase (cmd);
  if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'y')
  {
    setBody = 0;
    if (num < NUM) {
      i = num;
      num++;
    }
    else {
      i = nextobj;
      nextobj++;
      if (nextobj >= num) nextobj = 0;

      // destroy the body and geoms for slot i
      dBodyDestroy (obj[i].body);
      for (k=0; k < GPB; k++) {
	if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
      }
      memset (&obj[i],0,sizeof(obj[i]));
    }

    obj[i].body = dBodyCreate (world);
    for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

    dMatrix3 R;
    if (random_pos) 
      {
	dBodySetPosition (obj[i].body,
			  dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2 + platpos);
	dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			    dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
      }
    else 
      {
	dReal maxheight = 0;
	for (k=0; k<num; k++) 
	  {
	    const dReal *pos = dBodyGetPosition (obj[k].body);
	    if (pos[2] > maxheight) maxheight = pos[2];
	  }
	dBodySetPosition (obj[i].body, 0,0,maxheight+platpos + 2);
	dRSetIdentity (R);
      }
    dBodySetRotation (obj[i].body,R);
    dBodySetData (obj[i].body,(void*) i);

    if (cmd == 'b') {
      dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
      obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
    }
    else if (cmd == 'c') {
      sides[0] *= 0.5;
      dMassSetCapsule (&m,DENSITY,3,sides[0],sides[1]);
      obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
    }
    else if (cmd == 'y') {
      dMassSetCylinder (&m,DENSITY,3,sides[0],sides[1]);
      obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
    }
    else if (cmd == 's') {
      sides[0] *= 0.5;
      dMassSetSphere (&m,DENSITY,sides[0]);
      obj[i].geom[0] = dCreateSphere (space,sides[0]);
    }

    if (!setBody)
     for (k=0; k < GPB; k++) {
      if (obj[i].geom[k]) dGeomSetBody (obj[i].geom[k],obj[i].body);
     }

    dBodySetMass (obj[i].body,&m);
  }
  else if (cmd == 'a') {
    show_aabb ^= 1;
  }
  else if (cmd == 't') {
    show_contacts ^= 1;
  }
  else if (cmd == 'r') {
    random_pos ^= 1;
  }
  else if (cmd == '1') {
    write_world = 1;
  }
  else if (cmd == ' ') {
    platpos = 0;
  }
}


// draw a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
  int i;
	
  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    dsDrawBox (pos,R,sides);
  }
  else if (type == dSphereClass) {
    dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
  }
  else if (type == dCapsuleClass) {
    dReal radius,length;
    dGeomCapsuleGetParams (g,&radius,&length);
    dsDrawCapsule (pos,R,length,radius);
  }
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
  else if (type == dGeomTransformClass) {
    dGeomID g2 = dGeomTransformGetGeom (g);
    const dReal *pos2 = dGeomGetPosition (g2);
    const dReal *R2 = dGeomGetRotation (g2);
    dVector3 actual_pos;
    dMatrix3 actual_R;
    dMULTIPLY0_331 (actual_pos,R,pos2);
    actual_pos[0] += pos[0];
    actual_pos[1] += pos[1];
    actual_pos[2] += pos[2];
    dMULTIPLY0_333 (actual_R,R,R2);
    drawGeom (g2,actual_pos,actual_R,0);
  }
  if (show_body) {
    dBodyID body = dGeomGetBody(g);
    if (body) {
      const dReal *bodypos = dBodyGetPosition (body); 
      const dReal *bodyr = dBodyGetRotation (body); 
      dReal bodySides[3] = { 0.1, 0.1, 0.1 };
      dsSetColorAlpha(0,1,0,1);
      dsDrawBox(bodypos,bodyr,bodySides); 
    }
  }
  if (show_aabb) {
    // draw the bounding box for this geom
    dReal aabb[6];
    dGeomGetAABB (g,aabb);
    dVector3 bbpos;
    for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }
}


// simulation loop

static void updatecam()
{
    xyz[2] = 2 + platpos;
    dsSetViewpoint (xyz,hpr);
}

static void simLoop (int pause)
{
    const dReal stepsize = 0.02;

  dsSetColor (0,0,2);
  dSpaceCollide (space,0,&nearCallback);
  if (!pause) {
    platpos += platspeed * stepsize;
    if (platpos > maxplatpos)
        platpos = 0;
    dGeomSetPosition(platform, 0,0,platpos);
    updatecam();
    dWorldQuickStep (world,0.02);
    //dWorldStep (world,0.02);
  }

  if (write_world) {
    FILE *f = fopen ("state.dif","wt");
    if (f) {
      dWorldExportDIF (world,f,"X");
      fclose (f);
    }
    write_world = 0;
  }
  
  // remove all contact joints
  dJointGroupEmpty (contactgroup);

  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (! dBodyIsEnabled (obj[i].body)) {
	dsSetColor (1,0.8,0);
      }
      else {
	dsSetColor (1,1,0);
      }
      drawGeom (obj[i].geom[j],0,0,show_aabb);
    }
  }
  dsSetColor (1,0,0);
  drawGeom (platform,0,0,show_aabb);
}


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
  if(argc==2)
    {
        fn.path_to_textures = argv[1];
    }

  // create world
  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);

  dWorldSetLinearDamping(world, 0.00001);
  dWorldSetAngularDamping(world, 0.005);
  dWorldSetMaxAngularSpeed(world, 200);

  dWorldSetContactSurfaceLayer (world,0.001);
  ground = dCreatePlane (space,0,0,1,0);
  memset (obj,0,sizeof(obj));

    // create lift platform
    platform = dCreateBox(space, 2, 2, 1);

    dGeomSetCategoryBits(ground, 1ul);
    dGeomSetCategoryBits(platform, 2ul);
    dGeomSetCollideBits(ground, ~2ul);
    dGeomSetCollideBits(platform, ~1ul);

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
