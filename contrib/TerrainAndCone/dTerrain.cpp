//Benoit CHAPEROT 2003 www.jstarlab.com
//some code inspired by Magic Software
#include <ode/common.h>
#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_std_internal.h"
#include "collision_util.h"
//#include <drawstuff/drawstuff.h>
#include "windows.h"
#include "ode\ode.h"

#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))
#define MAXCONTACT 10
#define TERRAINTOL 0.0f

static bool IsAPowerOfTwo(int f)
{
	while ((f&1) != 1)	
		f >>= 1;

	return (f == 1);
}

static int GetPowerOfTwo(int f)
{
	int n = 0;
	while ((f&1) != 1)
	{
		n++;
		f >>= 1;
	}
	
	return n;
}

dxTerrain::dxTerrain (dSpaceID space, dReal *pHeights,dReal vLength,int nNumNodesPerSide) :
dxGeom (space,0)
{
	dIASSERT(IsAPowerOfTwo(nNumNodesPerSide));
	dIASSERT(pHeights);
	dIASSERT(vLength > 0.f);
	dIASSERT(nNumNodesPerSide > 0);
	type = dTerrainClass;
	m_vLength = vLength;
	m_pHeights = new dReal[nNumNodesPerSide * nNumNodesPerSide];
	dIASSERT(m_pHeights);
	m_nNumNodesPerSide = nNumNodesPerSide;
	m_vNodeLength = m_vLength / m_nNumNodesPerSide;
	m_nNumNodesPerSideShift = GetPowerOfTwo(m_nNumNodesPerSide);
	m_nNumNodesPerSideMask  = m_nNumNodesPerSide - 1;
	m_vMinHeight = dInfinity;
	m_vMaxHeight = -dInfinity;

	for (int i=0;i<nNumNodesPerSide * nNumNodesPerSide;i++)
	{
		m_pHeights[i] = pHeights[i];
		if (m_pHeights[i] < m_vMinHeight)	m_vMinHeight = m_pHeights[i];
		if (m_pHeights[i] > m_vMaxHeight)	m_vMaxHeight = m_pHeights[i];
	}
}

dxTerrain::~dxTerrain()
{
	dIASSERT(m_pHeights);
	delete [] m_pHeights;
}

void dxTerrain::computeAABB()
{
	aabb[0] = -dInfinity;
	aabb[1] = dInfinity;
	aabb[2] = -dInfinity;
	aabb[3] = dInfinity;
	aabb[4] = m_vMinHeight;
	aabb[5] = m_vMaxHeight;
}

dReal dxTerrain::GetHeight(int x,int y)
{
	return m_pHeights[	(((unsigned int)(y) & m_nNumNodesPerSideMask) << m_nNumNodesPerSideShift)
					+	 ((unsigned int)(x) & m_nNumNodesPerSideMask)];
}
/*
void dxTerrain::Draw(int x,int y)
{
	dVector3 A,B,C,D;	//,E,F,G,N;
	dMatrix3 R;
	dRSetIdentity(R);
	dVector3 pos;
	pos[0] = pos[1] = pos[2] = 0.f;
	dReal vx,vy;
	vx = m_vLength * x;
	vy = m_vLength * y;
	
	int i;
	for (i=0;i<m_nNumNodesPerSide;i++)
	{
		for (int j=0;j<m_nNumNodesPerSide;j++)
		{
			A[0] = i * m_vNodeLength + vx;
			A[1] = j * m_vNodeLength + vy;
			A[2] = GetHeight(i,j);
			B[0] = (i+1) * m_vNodeLength + vx;
			B[1] = j * m_vNodeLength + vy;
			B[2] = GetHeight(i+1,j);
			C[0] = i * m_vNodeLength + vx;
			C[1] = (j+1) * m_vNodeLength + vy;
			C[2] = GetHeight(i,j+1);
			D[0] = (i+1) * m_vNodeLength + vx;
			D[1] = (j+1) * m_vNodeLength + vy;
			D[2] = GetHeight(i+1,j+1);
			dsDrawTriangle(pos,R,C,A,B);
			dsDrawTriangle(pos,R,D,C,B);
		}
	}
}
*/
dReal dxTerrain::GetHeight(dReal x,dReal y)
{
	int nX		= int(floor(x / m_vNodeLength));
	int nY		= int(floor(y / m_vNodeLength));
	dReal dx	= (x - (dReal(nX) * m_vNodeLength)) / m_vNodeLength;
	dReal dy	= (y - (dReal(nY) * m_vNodeLength)) / m_vNodeLength;
	dIASSERT((dx >= 0.f) && (dx <= 1.f));
	dIASSERT((dy >= 0.f) && (dy <= 1.f));

	dReal z,z0;
	
	if (dx + dy < 1.f)
	{
		z0	= GetHeight(nX,nY);
		z	= z0	
			+ (GetHeight(nX+1,nY) - z0) * dx
			+ (GetHeight(nX,nY+1) - z0) * dy;
	}
	else
	{
		z0	= GetHeight(nX+1,nY+1);
		z	= z0	
			+ (GetHeight(nX+1,nY) - z0) * (1.f - dy)
			+ (GetHeight(nX,nY+1) - z0) * (1.f - dx);
	}

	return z;	
}

bool dxTerrain::IsOnTerrain(int nx,int ny,int w,dReal *pos)
{
	dVector3 Min,Max;
	Min[0] = nx * m_vNodeLength;
	Min[1] = ny * m_vNodeLength;
	Max[0] = (nx+1) * m_vNodeLength;
	Max[1] = (ny+1) * m_vNodeLength;
	dReal Tol = m_vNodeLength * TERRAINTOL;
	
	if ((pos[0]<Min[0]-Tol) || (pos[0]>Max[0]+Tol))
		return false;

	if ((pos[1]<Min[1]-Tol) || (pos[1]>Max[1]+Tol))
		return false;

	dReal dx	= (pos[0] - (dReal(nx) * m_vNodeLength)) / m_vNodeLength;
	dReal dy	= (pos[1] - (dReal(ny) * m_vNodeLength)) / m_vNodeLength;

	if ((w == 0) && (dx + dy > 1.f+TERRAINTOL))
		return false;

	if ((w == 1) && (dx + dy < 1.f-TERRAINTOL))
		return false;

	return true;
}

dGeomID dCreateTerrain(dSpaceID space, dReal *pHeights,dReal vLength,int nNumNodesPerSide)
{
	return new dxTerrain(space, pHeights,vLength,nNumNodesPerSide);
}

dReal dGeomTerrainPointDepth (dGeomID g, dReal x, dReal y, dReal z)
{
	dUASSERT (g && g->type == dTerrainClass,"argument not a terrain");
	dxTerrain *t = (dxTerrain*) g;
	return t->GetHeight(x,y) - z;
}

int CompareContactGeom(const void *p1,const void *p2)
{
	dContactGeom *pC1 = (dContactGeom*)p1;
	dContactGeom *pC2 = (dContactGeom*)p2;
	if (pC1->depth > pC2->depth)	return -1;
	if (pC2->depth > pC1->depth)	return 1;
	return 0;
}

typedef dReal dGetDepthFn(dGeomID g, dReal x, dReal y, dReal z);
#define RECOMPUTE_RAYNORMAL
//#define DO_RAYDEPTH

#define DMESS(A)	\
			dMessage(0,"Contact Plane (%d %d %d) %.5e %.5e (%.5e %.5e %.5e)(%.5e %.5e %.5e)).",	\
					x,y,A,	\
					pContact[numContacts].depth,	\
					dGeomSphereGetRadius(o2),		\
					pContact[numContacts].pos[0],	\
					pContact[numContacts].pos[1],	\
					pContact[numContacts].pos[2],	\
					pContact[numContacts].normal[0],	\
					pContact[numContacts].normal[1],	\
					pContact[numContacts].normal[2]);

int dxTerrain::dCollideTerrainUnit(
	int x,int y,dxGeom *o2,int numMaxContacts,
	int flags,dContactGeom *pContact)
{
	dColliderFn *CollideRayN;
	dColliderFn *CollideNPlane;
	dGetDepthFn *GetDepth;
	int numContacts = 0;
	int numPlaneContacts = 0;
	int i;
	dContactGeom PlaneContact[MAXCONTACT];

	if (numContacts == numMaxContacts)
		return numContacts;
	
	switch (o2->type)
	{
	case dSphereClass:
		CollideRayN		= dCollideRaySphere;
		CollideNPlane	= dCollideSpherePlane;
		GetDepth		= dGeomSpherePointDepth;
		break;
	case dBoxClass:
		CollideRayN		= dCollideRayBox;
		CollideNPlane	= dCollideBoxPlane;
		GetDepth		= dGeomBoxPointDepth;
		break;
	case dCCylinderClass:
		CollideRayN		= dCollideRayCCylinder;
		CollideNPlane	= dCollideCCylinderPlane;
		GetDepth		= dGeomCCylinderPointDepth;
		break;
	case dRayClass:
		CollideRayN		= NULL;
		CollideNPlane	= dCollideRayPlane;
		GetDepth		= NULL;
		break;
	case dConeClass:
		CollideRayN		= dCollideRayCone;
		CollideNPlane	= dCollideConePlane;
		GetDepth		= dGeomConePointDepth;
		break;
	default:
		dIASSERT(0);
	}

	dReal Plane[4],lBD,lCD,lBC;
	dVector3 A,B,C,D,BD,CD,BC,AB,AC;
	A[0] = x * m_vNodeLength;
	A[1] = y * m_vNodeLength;
	A[2] = GetHeight(x,y);
	B[0] = (x+1) * m_vNodeLength;
	B[1] = y * m_vNodeLength;
	B[2] = GetHeight(x+1,y);
	C[0] = x * m_vNodeLength;
	C[1] = (y+1) * m_vNodeLength;
	C[2] = GetHeight(x,y+1);
	D[0] = (x+1) * m_vNodeLength;
	D[1] = (y+1) * m_vNodeLength;
	D[2] = GetHeight(x+1,y+1);

	dOP(BC,-,C,B);
	lBC = dLENGTH(BC);
	dOPEC(BC,/=,lBC);

	dOP(BD,-,D,B);
	lBD = dLENGTH(BD);
	dOPEC(BD,/=,lBD);

	dOP(CD,-,D,C);
	lCD = dLENGTH(CD);
	dOPEC(CD,/=,lCD);

	dOP(AB,-,B,A);
	dNormalize3(AB);

	dOP(AC,-,C,A);
	dNormalize3(AC);

	if (CollideRayN)
	{
#ifdef RECOMPUTE_RAYNORMAL
		dVector3 E,F;
		dVector3 CE,FB,AD;
		dVector3 Normal[3];
		E[0] = (x+2) * m_vNodeLength;
		E[1] = y * m_vNodeLength;
		E[2] = GetHeight(x+2,y);
		F[0] = x * m_vNodeLength;
		F[1] = (y+2) * m_vNodeLength;
		F[2] = GetHeight(x,y+2);
		dOP(AD,-,D,A);
		dNormalize3(AD);
		dOP(CE,-,E,C);
		dNormalize3(CE);
		dOP(FB,-,B,F);
		dNormalize3(FB);

		//BC
		dCROSS(Normal[0],=,AD,BC);
		dNormalize3(Normal[0]);

		//BD
		dCROSS(Normal[1],=,CE,BD);
		dNormalize3(Normal[1]);

		//CD
		dCROSS(Normal[2],=,FB,CD);
		dNormalize3(Normal[2]);
#endif		
		int nA[3],nB[3];
		dContactGeom ContactA[3],ContactB[3];
		dxRay rayBC(0,lBC);	
		dGeomRaySet(&rayBC, B[0], B[1], B[2], BC[0], BC[1], BC[2]);
		nA[0] = CollideRayN(&rayBC,o2,flags,&ContactA[0],sizeof(dContactGeom));
		dGeomRaySet(&rayBC, C[0], C[1], C[2], -BC[0], -BC[1], -BC[2]);
		nB[0] = CollideRayN(&rayBC,o2,flags,&ContactB[0],sizeof(dContactGeom));
		
		dxRay rayBD(0,lBD);	
		dGeomRaySet(&rayBD, B[0], B[1], B[2], BD[0], BD[1], BD[2]);
		nA[1] = CollideRayN(&rayBD,o2,flags,&ContactA[1],sizeof(dContactGeom));
		dGeomRaySet(&rayBD, D[0], D[1], D[2], -BD[0], -BD[1], -BD[2]);
		nB[1] = CollideRayN(&rayBD,o2,flags,&ContactB[1],sizeof(dContactGeom));
	
		dxRay rayCD(0,lCD);	
		dGeomRaySet(&rayCD, C[0], C[1], C[2], CD[0], CD[1], CD[2]);
		nA[2] = CollideRayN(&rayCD,o2,flags,&ContactA[2],sizeof(dContactGeom));
		dGeomRaySet(&rayCD, D[0], D[1], D[2], -CD[0], -CD[1], -CD[2]);
		nB[2] = CollideRayN(&rayCD,o2,flags,&ContactB[2],sizeof(dContactGeom));
	
		for (i=0;i<3;i++)
		{
			if (nA[i] & nB[i])
			{
				pContact[numContacts].pos[0] = (ContactA[i].pos[0] + ContactB[i].pos[0])/2;
				pContact[numContacts].pos[1] = (ContactA[i].pos[1] + ContactB[i].pos[1])/2;
				pContact[numContacts].pos[2] = (ContactA[i].pos[2] + ContactB[i].pos[2])/2;
#ifdef RECOMPUTE_RAYNORMAL
				pContact[numContacts].normal[0] = -Normal[i][0];
				pContact[numContacts].normal[1] = -Normal[i][1];
				pContact[numContacts].normal[2] = -Normal[i][2];
#else
				pContact[numContacts].normal[0] = (ContactA[i].normal[0] + ContactB[i].normal[0])/2;	//0.f;
				pContact[numContacts].normal[1] = (ContactA[i].normal[1] + ContactB[i].normal[1])/2;	//0.f;
				pContact[numContacts].normal[2] = (ContactA[i].normal[2] + ContactB[i].normal[2])/2;	//-1.f;
				dNormalize3(pContact[numContacts].normal);
#endif
#ifdef DO_RAYDEPTH
				dxRay rayV(0,1000.f);
				dGeomRaySet(&rayV,	pContact[numContacts].pos[0],
									pContact[numContacts].pos[1],
									pContact[numContacts].pos[2],
									-pContact[numContacts].normal[0],
									-pContact[numContacts].normal[1],
									-pContact[numContacts].normal[2]);
		
				dContactGeom ContactV;
				if (CollideRayN(&rayV,o2,flags,&ContactV,sizeof(dContactGeom)))
				{
					pContact[numContacts].depth = ContactV.depth;
					numContacts++;	
				}
#else
				pContact[numContacts].depth =  GetDepth(o2,
				pContact[numContacts].pos[0],
				pContact[numContacts].pos[1],
				pContact[numContacts].pos[2]);
				numContacts++;
#endif
				if (numContacts == numMaxContacts)
					return numContacts;

			}
		}
	}

	dCROSS(Plane,=,AB,AC);
	dNormalize3(Plane);
	Plane[3] = Plane[0] * A[0] + Plane[1] * A[1] + Plane[2] * A[2];
	dxPlane planeABC(0,Plane[0],Plane[1],Plane[2],Plane[3]);
	numPlaneContacts = CollideNPlane(o2,&planeABC,flags,PlaneContact,sizeof(dContactGeom));

	for (i=0;i<numPlaneContacts;i++)
	{
		if (IsOnTerrain(x,y,0,PlaneContact[i].pos))
		{
			pContact[numContacts].pos[0] = PlaneContact[i].pos[0];
			pContact[numContacts].pos[1] = PlaneContact[i].pos[1];
			pContact[numContacts].pos[2] = PlaneContact[i].pos[2];
			pContact[numContacts].normal[0] = -PlaneContact[i].normal[0];
			pContact[numContacts].normal[1] = -PlaneContact[i].normal[1];
			pContact[numContacts].normal[2] = -PlaneContact[i].normal[2];
			pContact[numContacts].depth = PlaneContact[i].depth;

			//DMESS(0);
			numContacts++;

			if (numContacts == numMaxContacts)
					return numContacts;
		}
	}

	dCROSS(Plane,=,CD,BD);
	dNormalize3(Plane);
	Plane[3] = Plane[0] * D[0] + Plane[1] * D[1] + Plane[2] * D[2];
	dxPlane planeDCB(0,Plane[0],Plane[1],Plane[2],Plane[3]);
	numPlaneContacts = CollideNPlane(o2,&planeDCB,flags,PlaneContact,sizeof(dContactGeom));

	for (i=0;i<numPlaneContacts;i++)
	{
		if (IsOnTerrain(x,y,1,PlaneContact[i].pos))
		{
			pContact[numContacts].pos[0] = PlaneContact[i].pos[0];
			pContact[numContacts].pos[1] = PlaneContact[i].pos[1];
			pContact[numContacts].pos[2] = PlaneContact[i].pos[2];
			pContact[numContacts].normal[0] = -PlaneContact[i].normal[0];
			pContact[numContacts].normal[1] = -PlaneContact[i].normal[1];
			pContact[numContacts].normal[2] = -PlaneContact[i].normal[2];
			pContact[numContacts].depth = PlaneContact[i].depth;
			//DMESS(1);
			numContacts++;

			if (numContacts == numMaxContacts)
					return numContacts;
		}
	}

	return numContacts;
}

int dCollideTerrain(dxGeom *o1, dxGeom *o2, int flags,dContactGeom *contact, int skip)
{
	dIASSERT (skip >= (int)sizeof(dContactGeom));
	dIASSERT (o1->type == dTerrainClass);
	int i,j;

	if ((flags & 0xffff) > MAXCONTACT)
		flags = (flags & 0xffff0000) | MAXCONTACT;

	if ((flags & 0xffff) == 0)
		flags = (flags & 0xffff0000) | 1;

	dxTerrain *terrain = (dxTerrain*) o1;
	dVector3 AabbTop;
	AabbTop[0] = (o2->aabb[0]+o2->aabb[1]) / 2;
	AabbTop[1] = (o2->aabb[2]+o2->aabb[3]) / 2;
	AabbTop[2] = o2->aabb[5];
	dReal AabbTopDepth = terrain->GetHeight(AabbTop[0],AabbTop[1]) - AabbTop[2];
	if (AabbTopDepth > 0.f)
	{
		contact->depth = AabbTopDepth;
		dReal MaxDepth = (o2->aabb[5]-o2->aabb[4]) / 2;
		if (contact->depth > MaxDepth)
			contact->depth = MaxDepth;
		contact->g1 = o1;
		contact->g2 = o2;
		dOPE(contact->pos,=,AabbTop);
		contact->normal[0] = 0.f;
		contact->normal[1] = 0.f;
		contact->normal[2] = -1.f;
		return 1;
	}
		
	dContactGeom TerrainContact[MAXCONTACT];
	int nMinX	= int(floor(o2->aabb[0] / terrain->m_vNodeLength));
	int nMaxX	= int(floor(o2->aabb[1] / terrain->m_vNodeLength)) + 1;
	int nMinY	= int(floor(o2->aabb[2] / terrain->m_vNodeLength));
	int nMaxY	= int(floor(o2->aabb[3] / terrain->m_vNodeLength)) + 1;

	//TO FIX
	int numTerrainContacts = 0;
	
	for (i=nMinX;i<nMaxX;i++)
	{
		for (j=nMinY;j<nMaxY;j++)
		{
			numTerrainContacts += terrain->dCollideTerrainUnit(
				i,j,o2,MAXCONTACT - numTerrainContacts,
				flags,&TerrainContact[numTerrainContacts]);

		}
	}

	dIASSERT(numTerrainContacts <= MAXCONTACT);

	qsort(&TerrainContact[0],numTerrainContacts,sizeof(dContactGeom),CompareContactGeom);
	
	if (numTerrainContacts > (flags & 0xffff))
		numTerrainContacts = (flags & 0xffff);

	for (i=0; i<numTerrainContacts; i++) 
	{
		*(CONTACT(contact,i*skip)) = TerrainContact[i];
		CONTACT(contact,i*skip)->g1 = o1;
		CONTACT(contact,i*skip)->g2 = o2;
		CONTACT(contact,i*skip)->depth /= numTerrainContacts;
	}

	return numTerrainContacts;
}

/*
void dDrawTerrain(dGeomID g,int x,int y)
{
	dUASSERT (g && g->type == dTerrainClass,"argument not a terrain");
	dxTerrain *t = (dxTerrain*) g;
	t->Draw(x,y);
}
*/