// dHeightfield Collider
//  Martijn Buijs 2006 http://home.planet.nl/~buijs512/
// Based on Terrain & Cone contrib by:
//  Benoit CHAPEROT 2003-2004 http://www.jstarlab.com
//  Some code inspired by Magic Software

#include <ode/common.h>
#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"
#include "heightfield.h"

#ifdef dTRIMESH_ENABLED
#include "collision_trimesh_internal.h"
#endif // dTRIMESH_ENABLED

#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))
#define MAXCONTACT 10
#define TERRAINTOL 0.0f

#if _MSC_VER <= 1200
#define dMIN(A,B)  ((A)>(B) ? B : A)
#define dMAX(A,B)  ((A)>(B) ? A : B)
#else
#define dMIN(A,B)  std::min(A,B)
#define dMAX(A,B)  std::max(A,B)
#endif

// Three-way MIN and MAX
#define dMIN3(A,B,C)	( (A)<(B) ? dMIN((A),(C)) : dMIN((B),(C)) )
#define dMAX3(A,B,C)	( (A)>(B) ? dMAX((A),(C)) : dMAX((B),(C)) )


//////// dxHeightfieldData /////////////////////////////////////////////////////////////

// dxHeightfieldData constructor
dxHeightfieldData::dxHeightfieldData()
{
 //
}


// build Heightfield data
void dxHeightfieldData::SetData(int nWidthSamples, int nDepthSamples,
                                dReal vWidth, dReal vDepth,
                                dReal vScale, dReal vOffset,
                                int nWrapMode, dReal vThickness)
{
	dIASSERT(vWidth > 0.0f);
	dIASSERT(vDepth > 0.0f);
	dIASSERT(nWidthSamples > 0);
	dIASSERT(nDepthSamples > 0);

	// x,z bounds
	m_vWidth = vWidth;
	m_vDepth = vDepth;

	// scale and offset
	m_vScale = vScale;
	m_vOffset = vOffset;

	// number of vertices per side
	m_nWidthSamples = nWidthSamples;
	m_nDepthSamples = nDepthSamples;

	m_vSampleWidth = m_vWidth / m_nWidthSamples;
	m_vSampleDepth = m_vDepth / m_nDepthSamples;

	// infinite min height bounds
	m_vThickness = vThickness;

	// finite or repeated terrain
	m_nWrapMode = nWrapMode;
}


// recomputes heights bounds
void dxHeightfieldData::ComputeHeightBounds()
{
	static int i;
	static dReal h;
	static unsigned char *data_byte;
	static short *data_short;
	static dReal *data_float;

	switch ( m_nGetHeightMode )
	{

	// callback
	case 0:
		// change nothing, keep using default or user specified bounds
		return;

	// byte
	case 1:
		data_byte = (unsigned char*)m_pHeightData;
		m_vMinHeight = dInfinity;
		m_vMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = data_byte[i];
			if (h < m_vMinHeight)	m_vMinHeight = h;
			if (h > m_vMaxHeight)	m_vMaxHeight = h;
		}

		break;

	// short
	case 2:
		data_short = (short*)m_pHeightData;
		m_vMinHeight = dInfinity;
		m_vMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = data_short[i];
			if (h < m_vMinHeight)	m_vMinHeight = h;
			if (h > m_vMaxHeight)	m_vMaxHeight = h;
		}

		break;

	// float
	case 3:
		data_float = (dReal*)m_pHeightData;
		m_vMinHeight = dInfinity;
		m_vMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = data_float[i];
			if (h < m_vMinHeight)	m_vMinHeight = h;
			if (h > m_vMaxHeight)	m_vMaxHeight = h;
		}

		break;

	}

	// scale and offset
	m_vMinHeight *= m_vScale;
	m_vMaxHeight *= m_vScale;
	m_vMinHeight += m_vOffset;
	m_vMaxHeight += m_vOffset;

	// add thickness
	m_vMinHeight -= m_vThickness;
}


// returns whether point is over terrain?
bool dxHeightfieldData::IsOnHeightfield( int nx, int nz, int w, dReal *pos )
{
	dVector3 Min,Max;
	Min[0] = nx * m_vSampleWidth;
	Min[2] = nz * m_vSampleDepth;
	Max[0] = (nx+1) * m_vSampleWidth;
	Max[2] = (nz+1) * m_vSampleDepth;
	dReal TolX = m_vSampleWidth * TERRAINTOL;
	dReal TolZ = m_vSampleDepth * TERRAINTOL;

	if ((pos[0]<Min[0]-TolX) || (pos[0]>Max[0]+TolX))	return false;
	if ((pos[2]<Min[2]-TolZ) || (pos[2]>Max[2]+TolZ))	return false;

	dReal dx	= (pos[0] - (dReal(nx) * m_vSampleWidth)) / m_vSampleWidth;
	dReal dz	= (pos[2] - (dReal(nz) * m_vSampleDepth)) / m_vSampleDepth;

	if ((w == 0) && (dx + dz > 1.f+TERRAINTOL))	return false;
	if ((w == 1) && (dx + dz < 1.f-TERRAINTOL))	return false;

	return true;
}


// returns height at given sample coordinates
dReal dxHeightfieldData::GetHeight( int x, int z )
{
	static dReal h;
	static unsigned char *data_byte;
	static short *data_short;
	static dReal *data_float;

	if (m_nWrapMode == 0)
	{
		if (x < 0) x = 0;
		if (z < 0) z = 0;
		if (x > m_nWidthSamples-1) x = m_nWidthSamples-1;
		if (z > m_nDepthSamples-1) z = m_nDepthSamples-1;
	}
	else
	{
		x %= m_nWidthSamples;
		z %= m_nDepthSamples;
		if (x < 0) x+=m_nWidthSamples;
		if (z < 0) z+=m_nDepthSamples;
	}

	switch (m_nGetHeightMode)
	{

	// callback (dReal)
	case 0:
 		h = (*m_pGetHeightCallback)(m_pUserData, x, z);
		break;

	// byte
	case 1:
		data_byte = (unsigned char*)m_pHeightData;
		h = data_byte[x+(z * m_nWidthSamples)];
		break;

	// short
	case 2:
		data_short = (short*)m_pHeightData;
		h = data_short[x+(z * m_nWidthSamples)];
		break;

	// dReal
	case 3:
		data_float = (dReal*)m_pHeightData;
		h = data_float[x+(z * m_nWidthSamples)];
		break;
	}

	return (h * m_vScale) + m_vOffset;
}


// returns height at given coordinates
dReal dxHeightfieldData::GetHeight( dReal x, dReal z )
{
	int nX	= int(floor(x / m_vSampleWidth));
	int nZ	= int(floor(z / m_vSampleDepth));
	dReal dx	= (x - (dReal(nX) * m_vSampleWidth)) / m_vSampleWidth;
	dReal dz	= (z - (dReal(nZ) * m_vSampleDepth)) / m_vSampleDepth;
	dIASSERT((dx >= 0.f) && (dx <= 1.f));
	dIASSERT((dz >= 0.f) && (dz <= 1.f));

	dReal y, y0;

	if (dx + dz < 1.f)
	{
		y0	= GetHeight(nX,nZ);
		y	= y0	+ (GetHeight(nX+1,nZ) - y0) * dx +
			        (GetHeight(nX,nZ+1) - y0) * dz;
	}	else	{
		y0	= GetHeight(nX+1,nZ+1);
		y	= y0	+ (GetHeight(nX+1,nZ) - y0) * (1.f - dz) +
			        (GetHeight(nX,nZ+1) - y0) * (1.f - dx);
	}

	return y;
}


// dxHeightfieldData destructor
dxHeightfieldData::~dxHeightfieldData()
{
	if (m_bCopyHeightData)
	{
		dIASSERT(m_pHeightData);
		delete [] m_pHeightData;
	}
}


//////// dxHeightfield /////////////////////////////////////////////////////////////////


// dxHeightfield constructor
dxHeightfield::dxHeightfield( dSpaceID space,
							  dHeightfieldDataID data,
							  int bPlaceable )			: dxGeom( space, bPlaceable )
{
	type = dHeightfieldClass;
	this->m_p_data = data;
}


// compute axis aligned bounding box
void dxHeightfield::computeAABB()
{
	const dxHeightfieldData *d = m_p_data;

	if ( d->m_nWrapMode == 0 )
	{
		// Finite
		if ( gflags & GEOM_PLACEABLE )
		{
			dReal dx[3], dy[6], dz[3];

            dx[0] = ( final_posr->R[ 0] * d->m_vWidth );
            dx[1] = ( final_posr->R[ 4] * d->m_vWidth );
            dx[2] = ( final_posr->R[ 8] * d->m_vWidth );

            dy[0] = ( final_posr->R[ 1] * d->m_vMinHeight );
            dy[1] = ( final_posr->R[ 5] * d->m_vMinHeight );
            dy[2] = ( final_posr->R[ 9] * d->m_vMinHeight );
            dy[3] = ( final_posr->R[ 1] * d->m_vMaxHeight );
            dy[4] = ( final_posr->R[ 5] * d->m_vMaxHeight );
            dy[5] = ( final_posr->R[ 9] * d->m_vMaxHeight );

            dz[0] = ( final_posr->R[ 2] * d->m_vDepth );
            dz[1] = ( final_posr->R[ 6] * d->m_vDepth );
            dz[2] = ( final_posr->R[10] * d->m_vDepth );

			// X extents
			aabb[0] = final_posr->pos[0] +
				dMIN3( dMIN( REAL( 0.0 ), dx[0] ), dMIN( dy[0], dy[3] ), dMIN( REAL( 0.0 ), dz[0] ) );
			aabb[1] = final_posr->pos[0] +
				dMAX3( dMAX( REAL( 0.0 ), dx[0] ), dMAX( dy[0], dy[3] ), dMAX( REAL( 0.0 ), dz[0] ) );

			// Y extents
			aabb[2] = final_posr->pos[1] +
				dMIN3( dMIN( REAL( 0.0 ), dx[1] ), dMIN( dy[1], dy[4] ), dMIN( REAL( 0.0 ), dz[1] ) );
			aabb[3] = final_posr->pos[1] +
				dMAX3( dMAX( REAL( 0.0 ), dx[1] ), dMAX( dy[1], dy[4] ), dMAX( REAL( 0.0 ), dz[1] ) );

			// Z extents
			aabb[4] = final_posr->pos[2] +
				dMIN3( dMIN( REAL( 0.0 ), dx[2] ), dMIN( dy[2], dy[5] ), dMIN( REAL( 0.0 ), dz[2] ) );
			aabb[5] = final_posr->pos[2] +
				dMAX3( dMAX( REAL( 0.0 ), dx[2] ), dMAX( dy[2], dy[5] ), dMAX( REAL( 0.0 ), dz[2] ) );
		}
		else
		{
			aabb[0] = 0;
			aabb[1] = d->m_vWidth;
			aabb[2] = d->m_vMinHeight;
			aabb[3] = d->m_vMaxHeight;
			aabb[4] = 0;
			aabb[5] = d->m_vDepth;
		}
	}
	else
	{
		// Infinite
		if ( gflags & GEOM_PLACEABLE )
		{
			aabb[0] = -dInfinity;
			aabb[1] = dInfinity;
			aabb[2] = -dInfinity;
			aabb[3] = dInfinity;
			aabb[4] = -dInfinity;
			aabb[5] = dInfinity;
		}
		else
		{
			aabb[0] = -dInfinity;
			aabb[1] = dInfinity;
			aabb[2] = d->m_vMinHeight;
			aabb[3] = d->m_vMaxHeight;
			aabb[4] = -dInfinity;
			aabb[5] = dInfinity;
		}
	}
}


// dxHeightfield destructor
dxHeightfield::~dxHeightfield()
{
	//
}


//////// Heightfield data interface ////////////////////////////////////////////////////


dHeightfieldDataID dGeomHeightfieldDataCreate()
{
	return new dxHeightfieldData();
}


void dGeomHeightfieldDataBuildCallback( dHeightfieldDataID d,
									    void* pUserData, dHeightfieldGetHeight* pCallback,
										dReal width, dReal depth, int widthSamples, int depthSamples,
										dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "argument not Heightfield data" );
	dIASSERT( pCallback );

	// callback
	d->m_nGetHeightMode = 0;
	d->m_pUserData = pUserData;
	d->m_pGetHeightCallback = pCallback;

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, bWrap, thickness );

	// default bounds
	d->m_vMinHeight = -dInfinity;
	d->m_vMaxHeight = dInfinity;
}


void dGeomHeightfieldDataBuildByte( dHeightfieldDataID d,
                                    unsigned char *pHeightData, int bCopyHeightData,
									dReal width, dReal depth, int widthSamples, int depthSamples,
									dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, bWrap, thickness );
	d->m_nGetHeightMode = 1;
	d->m_bCopyHeightData = bCopyHeightData;

	if ( d->m_bCopyHeightData == 0 )
	{
		// Data is referenced only.
		d->m_pHeightData = pHeightData;
	}
	else
	{
		// We own the height data, allocate storage
		d->m_pHeightData = new unsigned char[ d->m_nWidthSamples * d->m_nDepthSamples ];
		dIASSERT( d->m_pHeightData );
		
		// Copy data.
		memcpy( d->m_pHeightData, pHeightData, 
			sizeof( unsigned char ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}


void dGeomHeightfieldDataBuildShort( dHeightfieldDataID d,
									 short* pHeightData, int bCopyHeightData,
									 dReal width, dReal depth, int widthSamples, int depthSamples,
									 dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, bWrap, thickness );
	d->m_nGetHeightMode = 2;
	d->m_bCopyHeightData = bCopyHeightData;

	if ( d->m_bCopyHeightData == 0 )
	{
		// Data is referenced only.
		d->m_pHeightData = pHeightData;
	}
	else
	{
		// We own the height data, allocate storage
		d->m_pHeightData = new short[ d->m_nWidthSamples * d->m_nDepthSamples ];
		dIASSERT( d->m_pHeightData );
		
		// Copy data.
		memcpy( d->m_pHeightData, pHeightData, 
			sizeof( short ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}


void dGeomHeightfieldDataBuildFloat( dHeightfieldDataID d,
                                     dReal *pHeightData, int bCopyHeightData,
									 dReal width, dReal depth, int widthSamples, int depthSamples,
									 dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, bWrap, thickness );
	d->m_nGetHeightMode = 3;
	d->m_bCopyHeightData = bCopyHeightData;

	if ( d->m_bCopyHeightData == 0 )
	{
		// Data is referenced only.
		d->m_pHeightData = pHeightData;
	}
	else
	{
		// We own the height data, allocate storage
		d->m_pHeightData = new dReal[ d->m_nWidthSamples * d->m_nDepthSamples ];
		dIASSERT( d->m_pHeightData );
		
		// Copy data.
		memcpy( d->m_pHeightData, pHeightData, 
			sizeof( dReal ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}




void dGeomHeightfieldDataSetBounds( dHeightfieldDataID d, dReal minHeight, dReal maxHeight )
{
	dUASSERT(d, "Argument not Heightfield data");
	d->m_vMinHeight = ( minHeight * d->m_vScale ) + d->m_vOffset - d->m_vThickness;
	d->m_vMaxHeight = ( maxHeight * d->m_vScale ) + d->m_vOffset;
}


void dGeomHeightfieldDataDestroy( dHeightfieldDataID d )
{
	dUASSERT(d, "argument not Heightfield data");
	delete d;
}


//////// Heightfield geom interface ////////////////////////////////////////////////////


dGeomID dCreateHeightfield( dSpaceID space, dHeightfieldDataID data, int bPlaceable )
{
	return new dxHeightfield( space, data, bPlaceable );
}


void dGeomHeightfieldSetHeightfieldData( dGeomID g, dHeightfieldDataID d )
{
	dxHeightfield* geom = (dxHeightfield*) g;
	geom->data = d;
}


dHeightfieldDataID dGeomHeightfieldGetHeightfieldData( dGeomID g )
{
	dxHeightfield* geom = (dxHeightfield*) g;
	return geom->m_p_data;
}


/* Disabled by David Walters, 7/7/2006 as this function
// doesn't respect geom placement.
dReal dGeomHeightfieldPointDepth( dGeomID g, dReal x, dReal y, dReal z )
{
	dUASSERT (g && g->type == dHeightfieldClass,"argument not a dHeightfield");
	dxHeightfield *t = (dxHeightfield*) g;
	return t->m_p_data->GetHeight( x, z ) - y;
}*/

//////// dxHeightfield /////////////////////////////////////////////////////////////////


// Typdef for generic 'get point depth' function
typedef dReal dGetDepthFn( dGeomID g, dReal x, dReal y, dReal z );

#define RECOMPUTE_RAYNORMAL

#define DMESS(A)	\
			dMessage(0,"Contact Plane (%d %d %d) %.5e %.5e (%.5e %.5e %.5e)(%.5e %.5e %.5e)).",	\
					x,z,A,	\
					pContact->depth,	\
					dGeomSphereGetRadius(o2),		\
					pContact->pos[0],	\
					pContact->pos[1],	\
					pContact->pos[2],	\
					pContact->normal[0],	\
					pContact->normal[1],	\
					pContact->normal[2]);

/*
(y is up)

A-B-E.x
|/|
C-D
|
F
.
z
*/

int dxHeightfield::dCollideHeightfieldUnit( int x, int z, dxGeom* o2, int numMaxContacts,
	                                        int flags, dContactGeom* contact, int skip )
{
	dColliderFn *CollideRayN;
	dColliderFn *CollideNPlane;
	dGetDepthFn *GetDepth;
	int numContacts = 0;
	int numPlaneContacts = 0;
	int i;

	if ( numContacts == numMaxContacts )
		return numContacts;

	dContactGeom PlaneContact[MAXCONTACT];
	flags = (flags & 0xffff0000) | MAXCONTACT;

	switch (o2->type)
	{
	case dRayClass:
		CollideRayN		= NULL;
		CollideNPlane	= dCollideRayPlane;
		GetDepth		= NULL;
		break;
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
	case dCapsuleClass:
		CollideRayN		= dCollideRayCapsule;
		CollideNPlane	= dCollideCapsulePlane;
		GetDepth		= dGeomCapsulePointDepth;
		break;
	case dCylinderClass:
		CollideRayN		= dCollideRayCylinder;
		CollideNPlane	= dCollideCylinderPlane;
		GetDepth		= NULL;// TODO: dGeomCylinderPointDepth;
		break;
	case dConvexClass:
		CollideRayN		= dCollideRayConvex;
		CollideNPlane	= dCollideConvexPlane;
		GetDepth		= NULL;// TODO: dGeomConvexPointDepth;
		break;

	default:
		dIASSERT(0);

	}

	dReal Plane[4],lBD,lCD,lBC;
	dVector3 A,B,C,D,BD,CD,BC,AB,AC;
	A[0] = x * m_p_data->m_vSampleWidth;
	A[2] = z * m_p_data->m_vSampleDepth;
	A[1] = m_p_data->GetHeight(x,z);

	B[0] = (x+1) * m_p_data->m_vSampleWidth;
	B[2] = z * m_p_data->m_vSampleDepth;
	B[1] = m_p_data->GetHeight(x+1,z);

	C[0] = x * m_p_data->m_vSampleWidth;
	C[2] = (z+1) * m_p_data->m_vSampleDepth;
	C[1] = m_p_data->GetHeight(x,z+1);

	D[0] = (x+1) * m_p_data->m_vSampleWidth;
	D[2] = (z+1) * m_p_data->m_vSampleDepth;
	D[1] = m_p_data->GetHeight(x+1,z+1);

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

	if ( CollideRayN )
	{

#ifdef RECOMPUTE_RAYNORMAL

		dVector3 E,F;
		dVector3 CE,FB,AD;
		dVector3 Normal[3];
		E[0] = (x+2) * m_p_data->m_vSampleWidth;
		E[2] = z * m_p_data->m_vSampleDepth;
		E[1] = m_p_data->GetHeight(x+2,z);
		F[0] = x * m_p_data->m_vSampleWidth;
		F[2] = (z+2) * m_p_data->m_vSampleDepth;
		F[1] = m_p_data->GetHeight(x,z+2);
		dOP(AD,-,D,A);
		dNormalize3(AD);
		dOP(CE,-,E,C);
		dNormalize3(CE);
		dOP(FB,-,B,F);
		dNormalize3(FB);

		//BC
		dCROSS(Normal[0],=,BC,AD);
		dNormalize3(Normal[0]);

		//BD
		dCROSS(Normal[1],=,BD,CE);
		dNormalize3(Normal[1]);

		//CD
		dCROSS(Normal[2],=,CD,FB);
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
				dContactGeom *pContact = CONTACT(contact,numContacts*skip);
				pContact->pos[0] = (ContactA[i].pos[0] + ContactB[i].pos[0])/2;
				pContact->pos[1] = (ContactA[i].pos[1] + ContactB[i].pos[1])/2;
				pContact->pos[2] = (ContactA[i].pos[2] + ContactB[i].pos[2])/2;

#ifdef RECOMPUTE_RAYNORMAL

				pContact->normal[0] = -Normal[i][0];
				pContact->normal[1] = -Normal[i][1];
				pContact->normal[2] = -Normal[i][2];

#else // RECOMPUTE_RAYNORMAL

				pContact->normal[0] = (ContactA[i].normal[0] + ContactB[i].normal[0])/2;	//0.f;
				pContact->normal[1] = (ContactA[i].normal[1] + ContactB[i].normal[1])/2;	//0.f;
				pContact->normal[2] = (ContactA[i].normal[2] + ContactB[i].normal[2])/2;	//-1.f;
				dNormalize3(pContact->normal);

#endif // RECOMPUTE_RAYNORMAL



				//
				// Find Contact Penetration Depth
				//

				if ( GetDepth )
				{
					pContact->depth = GetDepth( o2, 
						pContact->pos[0], pContact->pos[1], pContact->pos[2] );

					numContacts++;
				}
				else
				{
					// We don't have a GetDepth function, so do a ray cast instead.
					// NOTE: This isn't ideal, and a GetDepth function should be 
					// written for all geom classes.
					dxRay rayV( 0, 1000.f );
					dGeomRaySet( &rayV, pContact->pos[0], pContact->pos[1], pContact->pos[2],
										-pContact->normal[0], -pContact->normal[1], -pContact->normal[2] );

					dContactGeom ContactV;
					if ( CollideRayN( &rayV, o2, flags, &ContactV, sizeof( dContactGeom ) ) )
					{
						pContact->depth = ContactV.depth;
						numContacts++;
					}
				}

				if (numContacts == numMaxContacts)
					return numContacts;

			}
		}
	}

	dCROSS(Plane,=,AC,AB);
	dNormalize3(Plane);
	Plane[3] = Plane[0] * A[0] + Plane[1] * A[1] + Plane[2] * A[2];
	dxPlane planeABC(0,Plane[0],Plane[1],Plane[2],Plane[3]);
	numPlaneContacts = CollideNPlane(o2,&planeABC,flags,PlaneContact,sizeof(dContactGeom));

	for ( i = 0; i < numPlaneContacts; i++ )
	{
		if ( m_p_data->IsOnHeightfield( x, z, 0, PlaneContact[i].pos ) )
		{
			dContactGeom *pContact = CONTACT(contact,numContacts*skip);
			pContact->pos[0] = PlaneContact[i].pos[0];
			pContact->pos[1] = PlaneContact[i].pos[1];
			pContact->pos[2] = PlaneContact[i].pos[2];
			pContact->normal[0] = -PlaneContact[i].normal[0];
			pContact->normal[1] = -PlaneContact[i].normal[1];
			pContact->normal[2] = -PlaneContact[i].normal[2];
			pContact->depth = PlaneContact[i].depth;

			numContacts++;

			if ( numContacts == numMaxContacts )
				return numContacts;
		}
	}

	dCROSS(Plane,=,BD,CD);
	dNormalize3(Plane);
	Plane[3] = Plane[0] * D[0] + Plane[1] * D[1] + Plane[2] * D[2];
	dxPlane planeDCB(0,Plane[0],Plane[1],Plane[2],Plane[3]);
	numPlaneContacts = CollideNPlane(o2,&planeDCB,flags,PlaneContact,sizeof(dContactGeom));

	for ( i = 0; i < numPlaneContacts; i++ )
	{
		if ( m_p_data->IsOnHeightfield( x, z, 1, PlaneContact[i].pos ) )
		{
			dContactGeom *pContact = CONTACT(contact,numContacts*skip);
			pContact->pos[0] = PlaneContact[i].pos[0];
			pContact->pos[1] = PlaneContact[i].pos[1];
			pContact->pos[2] = PlaneContact[i].pos[2];
			pContact->normal[0] = -PlaneContact[i].normal[0];
			pContact->normal[1] = -PlaneContact[i].normal[1];
			pContact->normal[2] = -PlaneContact[i].normal[2];
			pContact->depth = PlaneContact[i].depth;

			numContacts++;

			if ( numContacts == numMaxContacts )
				return numContacts;
		}
	}

	return numContacts;
}


int dCollideHeightfield( dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip )
{
	dIASSERT (skip >= (int)sizeof(dContactGeom));
	dIASSERT (o1->type == dHeightfieldClass);
	int i,j;

	if ((flags & 0xffff) == 0)
		flags = (flags & 0xffff0000) | 1;

	int numMaxTerrainContacts = (flags & 0xffff);
	dxHeightfield *terrain = (dxHeightfield*) o1;

	dVector3 posbak;
	dMatrix3 Rbak;
	dReal aabbbak[6];
	int gflagsbak;

	dVector3 pos0,pos1;
	dMatrix3 R1;
	int numTerrainContacts = 0;

	if (terrain->gflags & GEOM_PLACEABLE)
	{
		dOP(pos0,-,o2->final_posr->pos,terrain->final_posr->pos);
		dMULTIPLY1_331(pos1,terrain->final_posr->R,pos0);
		dMULTIPLY1_333(R1,terrain->final_posr->R,o2->final_posr->R);
		dVector3Copy(o2->final_posr->pos,posbak);
		dMatrix3Copy(o2->final_posr->R,Rbak);
		dVector3Copy(pos1,o2->final_posr->pos);
		dMatrix3Copy(R1,o2->final_posr->R);
		memcpy(aabbbak,o2->aabb,sizeof(dReal)*6);
		gflagsbak = o2->gflags;
		o2->computeAABB();
	}

	int nMinX	= int(floor(o2->aabb[0] / terrain->m_p_data->m_vSampleWidth));
	int nMaxX	= int(floor(o2->aabb[1] / terrain->m_p_data->m_vSampleWidth)) + 1;
	int nMinZ	= int(floor(o2->aabb[4] / terrain->m_p_data->m_vSampleDepth));
	int nMaxZ	= int(floor(o2->aabb[5] / terrain->m_p_data->m_vSampleDepth)) + 1;


	if ( terrain->m_p_data->m_nWrapMode == 0 )
	{
		nMinX = dMAX( nMinX, 0 );
		nMaxX = dMIN( nMaxX, terrain->m_p_data->m_nWidthSamples );
		nMinZ = dMAX( nMinZ, 0 );
		nMaxZ = dMIN( nMaxZ, terrain->m_p_data->m_nDepthSamples );

		if ((nMinX >= nMaxX) || (nMinZ >= nMaxZ))
			goto dCollideHeightfieldExit;
	}


	dVector3 AabbTop;
	AabbTop[0] = (o2->aabb[0]+o2->aabb[1]) / 2;
	AabbTop[2] = (o2->aabb[4]+o2->aabb[5]) / 2;
	AabbTop[1] = o2->aabb[3];
	if (o2->type != dRayClass)
	{
		dReal AabbTopDepth = terrain->m_p_data->GetHeight(AabbTop[0],AabbTop[2]) - AabbTop[1];
		if (AabbTopDepth > 0.f)
		{
			contact->depth = AabbTopDepth;
			dReal MaxDepth = (o2->aabb[3]-o2->aabb[2]) / 2;
			if (contact->depth > MaxDepth)	contact->depth = MaxDepth;
			contact->g1 = o1;
			contact->g2 = o2;
			dOPE(contact->pos,=,AabbTop);
			contact->normal[0] = 0.f;
			contact->normal[1] = -1.f;
			contact->normal[2] = 0.f;

			numTerrainContacts = 1;
			goto dCollideHeightfieldExit;
		}
	}

	for (i=nMinX;i<nMaxX;i++)
	{
		for (j=nMinZ;j<nMaxZ;j++)
		{
			numTerrainContacts += terrain->dCollideHeightfieldUnit(
				i,j,o2,numMaxTerrainContacts - numTerrainContacts,
				flags,CONTACT(contact,numTerrainContacts*skip),skip	);
		}
	}

	dIASSERT(numTerrainContacts <= numMaxTerrainContacts);

	for (i=0; i<numTerrainContacts; i++)
	{
		CONTACT(contact,i*skip)->g1 = o1;
		CONTACT(contact,i*skip)->g2 = o2;
	}

dCollideHeightfieldExit:

	if ( terrain->gflags & GEOM_PLACEABLE )
	{
		dVector3Copy( posbak, o2->final_posr->pos );
		dMatrix3Copy( Rbak, o2->final_posr->R );
		memcpy( o2->aabb, aabbbak, sizeof(dReal)*6 );
		o2->gflags = gflagsbak;

		for ( i = 0; i < numTerrainContacts; i++ )
		{
			dOPE(pos0,=,CONTACT(contact,i*skip)->pos);
			dMULTIPLY0_331(CONTACT(contact,i*skip)->pos,terrain->final_posr->R,pos0);
			dOP(CONTACT(contact,i*skip)->pos,+,CONTACT(contact,i*skip)->pos,terrain->final_posr->pos);

			dOPE(pos0,=,CONTACT(contact,i*skip)->normal);
			dMULTIPLY0_331(CONTACT(contact,i*skip)->normal,terrain->final_posr->R,pos0);
		}
	}

	return numTerrainContacts;
}


