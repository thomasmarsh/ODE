Benoit CHAPEROT 2003 www.jstarlab.com
Support for terrain and cones, collision and drawing.

For terrains, z axis is assumed to point upwards.
Terrains are defined by a height field, and repeat themselves infinitely in the x and y directions.
Terrains can potentially collide with everything that collides with planes and rays; 
see the switch statement in dTerrain.cpp line 230.
Terrains are non movable.

Cones currently collides only with terrain and planes and rays.

You will need to complete the following operations (with ODE 0.039):

*** add to folder ode\src:

dCone.cpp
dTerrain.cpp 
collision_std_internal.h

*** add to drawstuff\src\drawstuff.cpp:

static void drawCone(float l, float r)
{
  int i;
  float tmp,ny,nz,a,ca,sa;
  const int n = 24;	// number of sides to the cone (divisible by 4)

  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw top
  glShadeModel (GL_FLAT);
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,l);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (ny*r,nz*r,0);
    glVertex3d (ny*r,nz*r,0);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,0);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*r,nz*r,0);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();
}

void dsDrawCone (const float pos[3], const float R[12], float length, float radius)
{
	if (current_state != 2) dsError ("drawing function called outside simulation loop");
	setupDrawingMode();
	glShadeModel (GL_SMOOTH);
	setTransform (pos,R);
	drawCone (length,radius);
	glPopMatrix();
	
	if (use_shadows) {
		setShadowDrawingMode();
		setShadowTransform();
		setTransform (pos,R);
		drawCone (length,radius);
		glPopMatrix();
		glPopMatrix();
		glDepthRange (0,1);
	}
}

void dsDrawConeD (const double pos[3], const double R[12], float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCone(pos2,R2,length,radius);
}

static float GetHeight(int x,int y,int nNumNodesPerSide,float *pHeights)
{
	int nNumNodesPerSideMask = nNumNodesPerSide - 1;
	return pHeights[	(((unsigned int)(y) & nNumNodesPerSideMask) * nNumNodesPerSide)
					+	 ((unsigned int)(x) & nNumNodesPerSideMask)];
}

void dsDrawTerrain(int x,int y,float vLength,float vNodeLength,int nNumNodesPerSide,float *pHeights)
{
	float A[3],B[3],C[3],D[3];
	float R[12];
	memset(R,0,sizeof(R));
	R[0] = 1.f;
	R[5] = 1.f;
	R[10] = 1.f;
	float pos[3];
	pos[0] = pos[1] = pos[2] = 0.f;
	float vx,vy;
	vx = vLength * x;
	vy = vLength * y;
	
	int i;
	for (i=0;i<nNumNodesPerSide;i++)
	{
		for (int j=0;j<nNumNodesPerSide;j++)
		{
			A[0] = i * vNodeLength + vx;
			A[1] = j * vNodeLength + vy;
			A[2] = GetHeight(i,j,nNumNodesPerSide,pHeights);
			B[0] = (i+1) * vNodeLength + vx;
			B[1] = j * vNodeLength + vy;
			B[2] = GetHeight(i+1,j,nNumNodesPerSide,pHeights);
			C[0] = i * vNodeLength + vx;
			C[1] = (j+1) * vNodeLength + vy;
			C[2] = GetHeight(i,j+1,nNumNodesPerSide,pHeights);
			D[0] = (i+1) * vNodeLength + vx;
			D[1] = (j+1) * vNodeLength + vy;
			D[2] = GetHeight(i+1,j+1,nNumNodesPerSide,pHeights);
			dsDrawTriangle(pos,R,C,A,B,1);
			dsDrawTriangle(pos,R,D,C,B,1);
		}
	}
}

*** add to include\drawstuff\drawstuff.h:
void dsDrawCone (const float pos[3], const float R[12],	float length, float radius);
void dsDrawConeD (const double pos[3], const double R[12], float length, float radius);
void dsDrawTerrain(int x,int y,float vLength,float vNodeLength,int nNumNodesPerSide,float *pHeights);

*** add in include\ode\collision.h line 77:
/* class numbers - each geometry object needs a unique number */
enum {
  dSphereClass = 0,
  dBoxClass,
  dCCylinderClass,
  dCylinderClass,
  dPlaneClass,
  dRayClass,
  dGeomTransformClass,
  dTriMeshClass,

  dTerrainClass,	//here
  dConeClass,		//here

  dFirstSpaceClass,
  dSimpleSpaceClass = dFirstSpaceClass,
  dHashSpaceClass,
  dQuadTreeSpaceClass,

  dLastSpaceClass = dQuadTreeSpaceClass,

  dFirstUserClass,
  dLastUserClass = dFirstUserClass + dMaxUserClasses - 1,
  dGeomNumClasses
};

dGeomID dCreateTerrain (dSpaceID space, dReal *pHeights,dReal vLength,int nNumNodesPerSide);
dReal dGeomTerrainPointDepth (dGeomID ccylinder, dReal x, dReal y, dReal z);

dGeomID dCreateCone(dSpaceID space, dReal radius, dReal length);
void dGeomConeSetParams (dGeomID cone, dReal radius, dReal length);
void dGeomConeGetParams (dGeomID cone, dReal *radius, dReal *length);
dReal dGeomConePointDepth(dGeomID g, dReal x, dReal y, dReal z);

*** add in include\ode\geom.h:
dGeomID dCreateTerrain(dSpaceID space, dReal *pHeights,dReal vLength,int nNumNodesPerSide);

*** add in include\ode\odemath.h:
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

*** add in ode\src\collision_kernel.cpp line 137:
  setCollider (dTerrainClass,dSphereClass,&dCollideTerrain);
  setCollider (dTerrainClass,dBoxClass,&dCollideTerrain);
  setCollider (dTerrainClass,dCCylinderClass,&dCollideTerrain);
  setCollider (dTerrainClass,dRayClass,&dCollideTerrain);
  setCollider (dTerrainClass,dConeClass,&dCollideTerrain);
  setCollider (dConeClass,dPlaneClass,&dCollideConePlane);

*** add in ode\src\collision_std.h:
int dCollideTerrain(dxGeom *o1, dxGeom *o2, int flags,dContactGeom *contact, int skip);
int dCollideConePlane (dxGeom *o1, dxGeom *o2, int flags,dContactGeom *contact, int skip);
int dCollideRayCone (dxGeom *o1, dxGeom *o2, int flags,dContactGeom *contact, int skip);

***
*** ok, maybe now you want to try the terrain;
***
*** in ode\test\testboxstack.cpp;
*** add line 41 (top of file):

const int TERRAINNODES = 4;
dReal pTerrainHeights[TERRAINNODES*TERRAINNODES];
const dReal vTerrainLength = 4.f;
const dReal vTerrainHeight = 0.5f;
dGeomID terrain = NULL;

*** add line 365 (in simLoop()):

dAASSERT(terrain);
dsSetColor (0,1,0);
dsDrawTerrain(0,0,vTerrainLength,vTerrainLength/TERRAINNODES,TERRAINNODES,pTerrainHeights);
dsDrawTerrain(0,-1,vTerrainLength,vTerrainLength/TERRAINNODES,TERRAINNODES,pTerrainHeights);
dsDrawTerrain(-1,0,vTerrainLength,vTerrainLength/TERRAINNODES,TERRAINNODES,pTerrainHeights);
dsDrawTerrain(-1,-1,vTerrainLength,vTerrainLength/TERRAINNODES,TERRAINNODES,pTerrainHeights);
dsSetColor (1,1,0);

*** add line 416 (in main(), just before dsSimulationLoop()):

for (int i=0;i<TERRAINNODES*TERRAINNODES;i++)	pTerrainHeights[i] = vTerrainHeight * dRandReal();
terrain = dCreateTerrain(space,pTerrainHeights,vTerrainLength,TERRAINNODES);



