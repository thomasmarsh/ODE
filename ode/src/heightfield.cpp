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

#if dTRIMESH_ENABLED
#include "collision_trimesh_internal.h"
#endif // dTRIMESH_ENABLED

//#define CONTACT(p, skip) ((dContactGeom*) (((char*)p) + (skip)))
#define HEIGHTFIELDMAXCONTACTPERCELL 10
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


//////// Local Build Option ////////////////////////////////////////////////////

// Uncomment this #define to use the (0,0) corner of the geom as the origin,
// rather than the center. This was the way the original heightfield worked,
// but as it does not match the way all other geometries work, so for constancy it
// was changed to work like this.

// #define DHEIGHTFIELD_CORNER_ORIGIN

// use optimised version of colliding against a terrain triangles
#define _PLANECONSTRUCTIONOPTIMISATION

// No clue about why colliding against TRIANGLE_BORDER is that important ?
// It's really slowing down the thing 
// and not proven to be necessary in all tests I could do.
#define HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY

#ifdef HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY

// If colliding against TRIANGLE_BORDER ray,
// recompute normal using neighbor grid cells..
// to build larger triangles to get normals from
#define RECOMPUTE_RAYNORMAL

#endif //HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY

//////// dxHeightfieldData /////////////////////////////////////////////////////////////

// dxHeightfieldData constructor
dxHeightfieldData::dxHeightfieldData()
{
 //
}


// build Heightfield data
void dxHeightfieldData::SetData( int nWidthSamples, int nDepthSamples,
                                 dReal fWidth, dReal fDepth,
                                 dReal fScale, dReal fOffset, dReal fThickness,
								 int bWrapMode )
{
	dIASSERT( fWidth > REAL( 0.0 ) );
	dIASSERT( fDepth > REAL( 0.0 ) );
	dIASSERT( nWidthSamples > 0 );
	dIASSERT( nDepthSamples > 0 );

	// x,z bounds
	m_fWidth = fWidth;
	m_fDepth = fDepth;

	// cache half x,z bounds
	m_fHalfWidth = fWidth / REAL( 2.0 );
	m_fHalfDepth = fDepth / REAL( 2.0 );

	// scale and offset
	m_fScale = fScale;
	m_fOffset = fOffset;

	// infinite min height bounds
	m_fThickness = fThickness;

	// number of vertices per side
	m_nWidthSamples = nWidthSamples;
	m_nDepthSamples = nDepthSamples;

	m_fSampleWidth = m_fWidth / ( m_nWidthSamples - 1 );
	m_fSampleDepth = m_fDepth / ( m_nDepthSamples - 1 );

	// finite or repeated terrain?
	m_bWrapMode = bWrapMode;
}


// recomputes heights bounds
void dxHeightfieldData::ComputeHeightBounds()
{
	static int i;
	static dReal h;
	static unsigned char *data_byte;
	static short *data_short;
	static float *data_float;
	static double *data_double;

	switch ( m_nGetHeightMode )
	{

	// callback
	case 0:
		// change nothing, keep using default or user specified bounds
		return;

	// byte
	case 1:
		data_byte = (unsigned char*)m_pHeightData;
		m_fMinHeight = dInfinity;
		m_fMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = data_byte[i];
			if (h < m_fMinHeight)	m_fMinHeight = h;
			if (h > m_fMaxHeight)	m_fMaxHeight = h;
		}

		break;

	// short
	case 2:
		data_short = (short*)m_pHeightData;
		m_fMinHeight = dInfinity;
		m_fMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = data_short[i];
			if (h < m_fMinHeight)	m_fMinHeight = h;
			if (h > m_fMaxHeight)	m_fMaxHeight = h;
		}

		break;

	// float
	case 3:
		data_float = (float*)m_pHeightData;
		m_fMinHeight = dInfinity;
		m_fMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = data_float[i];
			if (h < m_fMinHeight)	m_fMinHeight = h;
			if (h > m_fMaxHeight)	m_fMaxHeight = h;
		}

		break;

	// double
	case 4:
		data_double = (double*)m_pHeightData;
		m_fMinHeight = dInfinity;
		m_fMaxHeight = -dInfinity;

		for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
		{
			h = static_cast< dReal >( data_double[i] );
			if (h < m_fMinHeight)	m_fMinHeight = h;
			if (h > m_fMaxHeight)	m_fMaxHeight = h;
		}

		break;

	}

	// scale and offset
	m_fMinHeight *= m_fScale;
	m_fMaxHeight *= m_fScale;
	m_fMinHeight += m_fOffset;
	m_fMaxHeight += m_fOffset;

	// add thickness
	m_fMinHeight -= m_fThickness;
}


// returns whether point is over terrain?
bool dxHeightfieldData::IsOnHeightfield  ( const dReal * const CellOrigin, const dReal * const pos,  const bool isABC) const
{
    {
	    const dReal MaxX = CellOrigin[0] + m_fSampleWidth;
	    const dReal TolX = m_fSampleWidth * TERRAINTOL;
	    if ((pos[0]<CellOrigin[0]-TolX) || (pos[0]>MaxX+TolX))	
            return false;
    }

    {
        const dReal MaxZ = CellOrigin[2] + m_fSampleDepth;
        const dReal TolZ = m_fSampleDepth * TERRAINTOL;
        if ((pos[2]<CellOrigin[2]-TolZ) || (pos[2]>MaxZ+TolZ))	
            return false;
    }

	// add X percentage position on cell and Z percentage position on cell
    const dReal pctTotal = (pos[0] - CellOrigin[0]) / m_fSampleWidth 
                         + (pos[2] - CellOrigin[2]) / m_fSampleDepth;

	if (isABC)
    {
        if (pctTotal >= 1.0 + TERRAINTOL)	
            return false;
        else	
            return true;
    }
    else if (pctTotal <= 1.0 - TERRAINTOL)	
    {
        return false;
    }
	return true;
}



// returns height at given sample coordinates
dReal dxHeightfieldData::GetHeight( int x, int z )
{
	static dReal h;
	static unsigned char *data_byte;
	static short *data_short;
	static float *data_float;
	static double *data_double;

	if ( m_bWrapMode == 0 )
	{
		// Finite
		if ( x < 0 ) x = 0;
		if ( z < 0 ) z = 0;
		if ( x > m_nWidthSamples - 1 ) x = m_nWidthSamples - 1;
		if ( z > m_nDepthSamples - 1 ) z = m_nDepthSamples - 1;
	}
	else
	{
		// Infinite
		x %= m_nWidthSamples - 1;
		z %= m_nDepthSamples - 1;
		if ( x < 0 ) x += m_nWidthSamples - 1;
		if ( z < 0 ) z += m_nDepthSamples - 1;
	}

	switch ( m_nGetHeightMode )
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

	// float
	case 3:
		data_float = (float*)m_pHeightData;
		h = data_float[x+(z * m_nWidthSamples)];
		break;

	// double
	case 4:
		data_double = (double*)m_pHeightData;
		h = static_cast< dReal >( data_double[x+(z * m_nWidthSamples)] );
		break;
	}

	return (h * m_fScale) + m_fOffset;
}


// returns height at given coordinates
dReal dxHeightfieldData::GetHeight( dReal x, dReal z )
{
	int nX	= int( floor( x / m_fSampleWidth ) );
	int nZ	= int( floor( z / m_fSampleDepth ) );

	dReal dx = ( x - ( dReal( nX ) * m_fSampleWidth ) ) / m_fSampleWidth;
	dReal dz = ( z - ( dReal( nZ ) * m_fSampleDepth ) ) / m_fSampleDepth;


	//dIASSERT( ( dx + dEpsilon >= 0.0f ) && ( dx - dEpsilon <= 1.0f ) );
	//dIASSERT( ( dz + dEpsilon >= 0.0f ) && ( dz - dEpsilon <= 1.0f ) );

	dReal y, y0;

	if ( dx + dz < REAL( 1.0 ) )
	{
		y0 = GetHeight( nX, nZ );

		y = y0 + ( GetHeight( nX + 1, nZ ) - y0 ) * dx
			   + ( GetHeight( nX, nZ + 1 ) - y0 ) * dz;
	}
	else
	{
		y0 = GetHeight( nX + 1, nZ + 1 );

		y = y0	+ ( GetHeight( nX + 1, nZ ) - y0 ) * ( 1.0f - dz ) +
			      ( GetHeight( nX, nZ + 1 ) - y0 ) * ( 1.0f - dx );
	}

	return y;
}


// dxHeightfieldData destructor
dxHeightfieldData::~dxHeightfieldData()
{
	static unsigned char *data_byte;
	static short *data_short;
	static float *data_float;
	static double *data_double;

	dIASSERT( m_pHeightData );

	if ( m_bCopyHeightData )
	{
		switch ( m_nGetHeightMode )
		{

		// callback
		case 0:
			// do nothing
			break;

		// byte
		case 1:
			data_byte = (unsigned char*)m_pHeightData;
			delete [] data_byte;
			break;

		// short
		case 2:
			data_short = (short*)m_pHeightData;
			delete [] data_short;
			break;

		// float
		case 3:
			data_float = (float*)m_pHeightData;
			delete [] data_float;
			break;

		// double
		case 4:
			data_double = (double*)m_pHeightData;
			delete [] data_double;
			break;

		}
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
    this->sliding_plane = new dxPlane(0, 0, 0, 0, 0);
    dGeomDisable (sliding_plane);
}


// compute axis aligned bounding box
void dxHeightfield::computeAABB()
{
	const dxHeightfieldData *d = m_p_data;

	if ( d->m_bWrapMode == 0 )
	{
		// Finite
		if ( gflags & GEOM_PLACEABLE )
		{
			dReal dx[6], dy[6], dz[6];

			// Y-axis
			dy[0] = ( final_posr->R[ 1] * d->m_fMinHeight );
			dy[1] = ( final_posr->R[ 5] * d->m_fMinHeight );
			dy[2] = ( final_posr->R[ 9] * d->m_fMinHeight );
			dy[3] = ( final_posr->R[ 1] * d->m_fMaxHeight );
			dy[4] = ( final_posr->R[ 5] * d->m_fMaxHeight );
			dy[5] = ( final_posr->R[ 9] * d->m_fMaxHeight );

#ifdef DHEIGHTFIELD_CORNER_ORIGIN

			// X-axis
			dx[0] = 0;	dx[3] = ( final_posr->R[ 0] * d->m_fWidth );
			dx[1] = 0;	dx[4] = ( final_posr->R[ 4] * d->m_fWidth );
			dx[2] = 0;	dx[5] = ( final_posr->R[ 8] * d->m_fWidth );

			// Z-axis
			dz[0] = 0;	dz[3] = ( final_posr->R[ 2] * d->m_fDepth );
			dz[1] = 0;	dz[4] = ( final_posr->R[ 6] * d->m_fDepth );
			dz[2] = 0;	dz[5] = ( final_posr->R[10] * d->m_fDepth );

#else // DHEIGHTFIELD_CORNER_ORIGIN

			// X-axis
			dx[0] = ( final_posr->R[ 0] * -d->m_fHalfWidth );
			dx[1] = ( final_posr->R[ 4] * -d->m_fHalfWidth );
			dx[2] = ( final_posr->R[ 8] * -d->m_fHalfWidth );
			dx[3] = ( final_posr->R[ 0] * d->m_fHalfWidth );
			dx[4] = ( final_posr->R[ 4] * d->m_fHalfWidth );
			dx[5] = ( final_posr->R[ 8] * d->m_fHalfWidth );

			// Z-axis
			dz[0] = ( final_posr->R[ 2] * -d->m_fHalfDepth );
			dz[1] = ( final_posr->R[ 6] * -d->m_fHalfDepth );
			dz[2] = ( final_posr->R[10] * -d->m_fHalfDepth );
			dz[3] = ( final_posr->R[ 2] * d->m_fHalfDepth );
			dz[4] = ( final_posr->R[ 6] * d->m_fHalfDepth );
			dz[5] = ( final_posr->R[10] * d->m_fHalfDepth );

#endif // DHEIGHTFIELD_CORNER_ORIGIN

			// X extents
			aabb[0] = final_posr->pos[0] +
				dMIN3( dMIN( dx[0], dx[3] ), dMIN( dy[0], dy[3] ), dMIN( dz[0], dz[3] ) );
			aabb[1] = final_posr->pos[0] +
				dMAX3( dMAX( dx[0], dx[3] ), dMAX( dy[0], dy[3] ), dMAX( dz[0], dz[3] ) );

			// Y extents
			aabb[2] = final_posr->pos[1] +
				dMIN3( dMIN( dx[1], dx[4] ), dMIN( dy[1], dy[4] ), dMIN( dz[1], dz[4] ) );
			aabb[3] = final_posr->pos[1] +
				dMAX3( dMAX( dx[1], dx[4] ), dMAX( dy[1], dy[4] ), dMAX( dz[1], dz[4] ) );

			// Z extents
			aabb[4] = final_posr->pos[2] +
				dMIN3( dMIN( dx[2], dx[5] ), dMIN( dy[2], dy[5] ), dMIN( dz[2], dz[5] ) );
			aabb[5] = final_posr->pos[2] +
				dMAX3( dMAX( dx[2], dx[5] ), dMAX( dy[2], dy[5] ), dMAX( dz[2], dz[5] ) );
		}
		else
		{

#ifdef DHEIGHTFIELD_CORNER_ORIGIN

			aabb[0] = 0;					aabb[1] = d->m_fWidth;
			aabb[2] = d->m_fMinHeight;		aabb[3] = d->m_fMaxHeight;
			aabb[4] = 0;					aabb[5] = d->m_fDepth;

#else // DHEIGHTFIELD_CORNER_ORIGIN

			aabb[0] = -d->m_fHalfWidth;		aabb[1] = +d->m_fHalfWidth;
			aabb[2] = d->m_fMinHeight;		aabb[3] = d->m_fMaxHeight;
			aabb[4] = -d->m_fHalfDepth;		aabb[5] = +d->m_fHalfDepth;

#endif // DHEIGHTFIELD_CORNER_ORIGIN

		}
	}
	else
	{
		// Infinite
		if ( gflags & GEOM_PLACEABLE )
		{
			aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
			aabb[2] = -dInfinity;			aabb[3] = +dInfinity;
			aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
		}
		else
		{
			aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
			aabb[2] = d->m_fMinHeight;		aabb[3] = d->m_fMaxHeight;
			aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
		}
	}

}


// dxHeightfield destructor
dxHeightfield::~dxHeightfield()
{
	delete sliding_plane;//
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
	dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
	dIASSERT( depthSamples >= 2 );

	// callback
	d->m_nGetHeightMode = 0;
	d->m_pUserData = pUserData;
	d->m_pGetHeightCallback = pCallback;

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );

	// default bounds
	d->m_fMinHeight = -dInfinity;
	d->m_fMaxHeight = dInfinity;
}


void dGeomHeightfieldDataBuildByte( dHeightfieldDataID d,
                                    const unsigned char *pHeightData, int bCopyHeightData,
									dReal width, dReal depth, int widthSamples, int depthSamples,
									dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );
	dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
	dIASSERT( depthSamples >= 2 );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
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
		memcpy( (void*)d->m_pHeightData, pHeightData,
			sizeof( unsigned char ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}


void dGeomHeightfieldDataBuildShort( dHeightfieldDataID d,
									 const short* pHeightData, int bCopyHeightData,
									 dReal width, dReal depth, int widthSamples, int depthSamples,
									 dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );
	dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
	dIASSERT( depthSamples >= 2 );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
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
		memcpy( (void*)d->m_pHeightData, pHeightData,
			sizeof( short ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}


void dGeomHeightfieldDataBuildSingle( dHeightfieldDataID d,
                                     const float *pHeightData, int bCopyHeightData,
									 dReal width, dReal depth, int widthSamples, int depthSamples,
									 dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );
	dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
	dIASSERT( depthSamples >= 2 );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
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
		d->m_pHeightData = new float[ d->m_nWidthSamples * d->m_nDepthSamples ];
		dIASSERT( d->m_pHeightData );

		// Copy data.
		memcpy( (void*)d->m_pHeightData, pHeightData,
			sizeof( float ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}

void dGeomHeightfieldDataBuildDouble( dHeightfieldDataID d,
                                     const double *pHeightData, int bCopyHeightData,
									 dReal width, dReal depth, int widthSamples, int depthSamples,
									 dReal scale, dReal offset, dReal thickness, int bWrap )
{
	dUASSERT( d, "Argument not Heightfield data" );
	dIASSERT( pHeightData );
	dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
	dIASSERT( depthSamples >= 2 );

	// set info
	d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
	d->m_nGetHeightMode = 4;
	d->m_bCopyHeightData = bCopyHeightData;

	if ( d->m_bCopyHeightData == 0 )
	{
		// Data is referenced only.
		d->m_pHeightData = pHeightData;
	}
	else
	{
		// We own the height data, allocate storage
		d->m_pHeightData = new double[ d->m_nWidthSamples * d->m_nDepthSamples ];
		dIASSERT( d->m_pHeightData );

		// Copy data.
		memcpy( (void*)d->m_pHeightData, pHeightData,
			sizeof( double ) * d->m_nWidthSamples * d->m_nDepthSamples );
	}

	// Find height bounds
	d->ComputeHeightBounds();
}




void dGeomHeightfieldDataSetBounds( dHeightfieldDataID d, dReal minHeight, dReal maxHeight )
{
	dUASSERT(d, "Argument not Heightfield data");
	d->m_fMinHeight = ( minHeight * d->m_fScale ) + d->m_fOffset - d->m_fThickness;
	d->m_fMaxHeight = ( maxHeight * d->m_fScale ) + d->m_fOffset;
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

//////// dxHeightfield /////////////////////////////////////////////////////////////////


// Typdef for generic 'get point depth' function
typedef dReal dGetDepthFn( dGeomID g, dReal x, dReal y, dReal z );
 

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


int dxHeightfield::dCollideHeightfieldZone( const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, const int numMaxContacts,
                                           int flags, dContactGeom* contact, 
                                           int skip )
{

    // get All Planes that could collide against.
    dColliderFn *geomRayNCollider;
    dColliderFn *geomNPlaneCollider;
    dGetDepthFn *geomNDepthGetter;

    switch (o2->type)
    {
    case dRayClass:
        geomRayNCollider		= NULL;
        geomNPlaneCollider		= dCollideRayPlane;
        geomNDepthGetter		= NULL;
        break;

    case dSphereClass:
        geomRayNCollider		= dCollideRaySphere;
        geomNPlaneCollider		= dCollideSpherePlane;
        geomNDepthGetter		= dGeomSpherePointDepth;
        break;

    case dBoxClass:
        geomRayNCollider		= dCollideRayBox;
        geomNPlaneCollider		= dCollideBoxPlane;
        geomNDepthGetter		= dGeomBoxPointDepth;
        break;

    case dCapsuleClass:
        geomRayNCollider		= dCollideRayCapsule;
        geomNPlaneCollider		= dCollideCapsulePlane;
        geomNDepthGetter		= dGeomCapsulePointDepth;
        break;

    case dCylinderClass:
        geomRayNCollider		= dCollideRayCylinder;
        geomNPlaneCollider		= dCollideCylinderPlane;
        geomNDepthGetter		= NULL;// TODO: dGeomCylinderPointDepth;
        break;

    case dConvexClass:
        geomRayNCollider		= dCollideRayConvex;
        geomNPlaneCollider		= dCollideConvexPlane;
        geomNDepthGetter		= NULL;// TODO: dGeomConvexPointDepth;
        break;

#if dTRIMESH_ENABLED

    case dTriMeshClass:
        geomRayNCollider		= dCollideRayTrimesh;
        geomNPlaneCollider		= dCollideTrimeshPlane;
        geomNDepthGetter		= NULL;// N/A?
        break;

#endif // dTRIMESH_ENABLED

    default:
        dIASSERT(0);	// Shouldn't ever get here.
        break;

    }



   
    dReal Plane[4];
    dVector3 A,B,C,D;
    dVector3 BD,CD,BC,AB,AC;
    int numTerrainContacts = 0;
    dGeomEnable (sliding_plane);
    int i;
    dContactGeom *pContact = 0;

    //dContactGeom *PlaneContact = new dContactGeom[numMaxContacts];
    //dContactGeom *PlaneContact = (dContactGeom*)new char[numMaxContacts*skip];
    //flags = (flags & 0xffff0000) | numMaxContacts;

    dContactGeom *PlaneContact = new dContactGeom[HEIGHTFIELDMAXCONTACTPERCELL];    
    flags = (flags & 0xffff0000) | HEIGHTFIELDMAXCONTACTPERCELL;

    // localize and const for faster access
    const dReal cfSampleWidth = m_p_data->m_fSampleWidth;
    const dReal cfSampleDepth = m_p_data->m_fSampleDepth;

#ifdef HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY
    const bool isColliderRayEnabled = geomRayNCollider != NULL;
    int nA[3],nB[3];
    dContactGeom ContactA[3],ContactB[3], ContactV;
    dxRay tempRay(0, 1);
    // taking advantage of grid square, 
    // so those length always the same
    const dReal lWidthSq  = cfSampleWidth*cfSampleWidth;
    const dReal lDepthSq  = cfSampleDepth*cfSampleDepth;
    const dReal lDiagSq   = lWidthSq + lDepthSq;

    const dReal lDiagDblDepthSq  = isColliderRayEnabled ? lWidthSq   + 4*lDepthSq : 0.0f;
    const dReal lDiagDblWidthSq  = isColliderRayEnabled ? 4*lWidthSq + lDepthSq   : 0.0f;
    // if CollideRayN...
    #ifdef RECOMPUTE_RAYNORMAL
        dVector3 E,F;
        dVector3 CE,FB,AD;
        dVector3 Normal[3];
    #endif
#endif //HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY

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

    C[0] = minX*cfSampleWidth - cfSampleWidth;
    for ( int x = minX; x < maxX; ++x )
    {
        // Re-use previous results to set A,B
        // using CD in next loop
        // so we compute CD once here at start, 
        // as if it was AB.

        // y values
        C[1] = m_p_data->GetHeight(x,   minZ);
        D[1] = m_p_data->GetHeight(x+1, minZ);

        C[2] = minZ*cfSampleDepth;
        D[2] = C[2];

        // x values
        // those are set once as they do not move in next loop
        C[0] += cfSampleWidth;
        D[0] = C[0] + cfSampleWidth;
        A[0] = C[0];
        B[0] = D[0];

        // calculate CD once at start
        // then reuse previous normalized CD
        dOP(CD,-,D,C);

#ifdef RECOMPUTE_RAYNORMAL
        if (isColliderRayEnabled)
            F[1] = m_p_data->GetHeight(x, minZ+1);
#endif // RECOMPUTE_RAYNORMAL

        for ( int z = minZ; z < maxZ; ++z )
        {
            // AB = previous  CD
            // [0] doesn't change as it's X
            A[2] = C[2];
            B[2] = D[2];

            A[1] = C[1];
            B[1] = D[1];

            // CD advance
            C[2] = C[2] + cfSampleDepth;
            D[2] = C[2];
#ifdef RECOMPUTE_RAYNORMAL
            C[1] = isColliderRayEnabled ? F[1] : m_p_data->GetHeight(x,  z+1);
#else
            C[1] = m_p_data->GetHeight(x,  z+1);
#endif // RECOMPUTE_RAYNORMAL
            D[1] = m_p_data->GetHeight(x+1,z+1);

            // compute 4 Vector of ABCD square 
           
            dOP(BD,-,D,B);

            dOP(AC,-,C,A);

            // using previous result
            // as AB now was CD in previous loop
            //  AB <= CD
            dVector3Copy(CD, AB);
    
            dOP(CD,-,D,C);

            // compute Diagonal Vector
            dOP(BC,-,C,B);


            // First plane.
            // PLANE ABC
           dIASSERT (numTerrainContacts != numMaxContacts );
            {
                dCROSS(Plane,=,AC,AB);

                //dNormalize3 (Plane);
                const dReal dlength = dRecipSqrt(dLENGTHSQUARED(Plane));
                Plane[0] *= dlength;
                Plane[1] *= dlength;
                Plane[2] *= dlength;

                Plane[3] =  Plane[0] * A[0] + Plane[1] * A[1] + Plane[2] * A[2];

                // PLANE ABC
                // should call that, but it renormalizes the normal once again (uselessly costly) 
                //dGeomPlaneSetParams (sliding_plane, Plane[0],Plane[1],Plane[2],Plane[3]);

                sliding_plane->p[0] = Plane[0];
                sliding_plane->p[1] = Plane[1];
                sliding_plane->p[2] = Plane[2];
                sliding_plane->p[3] = Plane[3];
                dGeomMoved (sliding_plane);

                //equivalent to that, but saving creation and deletion
                //dxPlane planeABC(0,Plane[0],Plane[1],Plane[2],Plane[3]);

                const int numPlaneContacts = geomNPlaneCollider(o2,sliding_plane,flags,PlaneContact,sizeof(dContactGeom));

                for ( i = 0; i < numPlaneContacts; i++ )
                {
                    const dVector3 &pCPos = PlaneContact[i].pos;
                    if (  m_p_data->IsOnHeightfield( A, pCPos, true) )
                    {
                        pContact = CONTACT(contact,numTerrainContacts*skip);
                       
                        pContact->pos[0] = pCPos[0];
                        pContact->pos[1] = pCPos[1];
                        pContact->pos[2] = pCPos[2];

                        //why this is inverted ?
                        pContact->normal[0] = -PlaneContact[i].normal[0];
                        pContact->normal[1] = -PlaneContact[i].normal[1];
                        pContact->normal[2] = -PlaneContact[i].normal[2];

                        pContact->depth = PlaneContact[i].depth;

                        numTerrainContacts++;

                        if ( numTerrainContacts == numMaxContacts )
                            break;
                    }
                }

                if (numTerrainContacts == numMaxContacts )
                    break;
            } 
            // Second plane.
            // PLANE DCB
            dIASSERT (numTerrainContacts != numMaxContacts );
            {
                dCROSS(Plane,=,BD,CD);

                //dNormalize3 (Plane);
                const dReal dlength = dRecipSqrt(dLENGTHSQUARED(Plane));
                Plane[0] *= dlength;
                Plane[1] *= dlength;
                Plane[2] *= dlength;

                Plane[3] = Plane[0] * D[0] + Plane[1] * D[1] + Plane[2] * D[2];

                // should call that, but it renormalizes the normal once again (uselessly costly) 
                //dGeomPlaneSetParams (sliding_plane, Plane[0],Plane[1],Plane[2],Plane[3]);

                 sliding_plane->p[0] = Plane[0];
                 sliding_plane->p[1] = Plane[1];
                 sliding_plane->p[2] = Plane[2];
                 sliding_plane->p[3] = Plane[3];
                 dGeomMoved (sliding_plane);

                //equivalent to that, but saving creation and deletion
                //dxPlane planeDCB(0,Plane[0],Plane[1],Plane[2],Plane[3]);

                const int numPlaneContacts = geomNPlaneCollider(o2,sliding_plane,flags,PlaneContact,sizeof(dContactGeom));

                for ( i = 0; i < numPlaneContacts; i++ )
                {
                    const dVector3 &pCPos = PlaneContact[i].pos;
                    if (  m_p_data->IsOnHeightfield( A, pCPos, false) )
                    {
                        pContact = CONTACT(contact,numTerrainContacts*skip);

                        pContact->pos[0] = pCPos[0];
                        pContact->pos[1] = pCPos[1];
                        pContact->pos[2] = pCPos[2];

                        //why this is inverted ?
                        pContact->normal[0] = -PlaneContact[i].normal[0];
                        pContact->normal[1] = -PlaneContact[i].normal[1];
                        pContact->normal[2] = -PlaneContact[i].normal[2];

                        pContact->depth = PlaneContact[i].depth;

                        numTerrainContacts++;

                        if ( numTerrainContacts == numMaxContacts )
                            break;
                    }
                }

                if ( numTerrainContacts == numMaxContacts )
                    break;
            }

#ifdef HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY
            dIASSERT (numTerrainContacts != numMaxContacts );
            if ( isColliderRayEnabled)
            {

#ifdef RECOMPUTE_RAYNORMAL

                E[0] = B[0] + cfSampleWidth;
                E[1] = m_p_data->GetHeight(x+2,z);
                E[2] = A[2];

                F[0] = A[0];
                F[1] = m_p_data->GetHeight(x,z+2);
                F[2] = C[2] + cfSampleDepth;

                dOP(AD,-,D,A);
                //dOPEC(AD, /=, dSqrt(lDiagSq         + AD[1]*AD[1]));
                //dNormalize3(AD);

                dOP(CE,-,E,C);
                //dOPEC(CE, /=, dSqrt(lDiagDblWidthSq + CE[1]*CE[1]));
                //dNormalize3(CE);

                dOP(FB,-,B,F);
                //dOPEC(FB, /=, dSqrt(lDiagDblDepthSq + FB[1]*FB[1]));
                //dNormalize3(FB);


#endif



#define dGeomRaySetNoNormalize(myRay, MyPoint, MyVector) { \
                            \
                            dVector3Copy (MyPoint, myRay.final_posr->pos); \
                            myRay.final_posr->R[0*4+2] = MyVector[0]; \
                            myRay.final_posr->R[1*4+2] = MyVector[1]; \
                            myRay.final_posr->R[2*4+2] = MyVector[2]; \
                            dGeomMoved (&myRay); \
                };

#define NUM_RAYCHECK 3

#ifdef RECOMPUTE_RAYNORMAL
                //BC
                dCROSS(Normal[0],=,BC,AD);
                dNormalize3(Normal[0]);
#endif
                // ray BC and CB
                const dReal lBC = dSqrt(lDiagSq + BC[1]*BC[1]);
                tempRay.length = lBC;
                dGeomRaySet(&tempRay, B[0], B[1], B[2], BC[0], BC[1], BC[2]);
                //dGeomRaySetNoNormalize(tempRay, B, BC);
                nA[0] = geomRayNCollider(&tempRay,o2,flags,&ContactA[0],sizeof(dContactGeom));
                //dGeomRaySet(&tempRay, C[0], C[1], C[2], -BC[0], -BC[1], -BC[2]);
                dGeomRaySetNoNormalize(tempRay, C, -BC);
                nB[0] = geomRayNCollider(&tempRay,o2,flags,&ContactB[0],sizeof(dContactGeom));

                if (NUM_RAYCHECK >= 2)
                {

#ifdef RECOMPUTE_RAYNORMAL
                    //BD
                    dCROSS(Normal[1],=,BD,CE);
                    dNormalize3(Normal[1]);
#endif



                    const dReal lBD = dSqrt(lDepthSq + BD[1]*BD[1]);
                    // ray BD and DB
                    tempRay.length = lBD;
                    dGeomRaySet(&tempRay, B[0], B[1], B[2], BD[0], BD[1], BD[2]);
                    //dGeomRaySetNoNormalize(tempRay, B, BD);
                    nA[1] = geomRayNCollider(&tempRay,o2,flags,&ContactA[1],sizeof(dContactGeom));
                    //dGeomRaySet(&tempRay, D[0], D[1], D[2], -BD[0], -BD[1], -BD[2]);
                    dGeomRaySetNoNormalize(tempRay, D, -BD);
                    nB[1] = geomRayNCollider(&tempRay,o2,flags,&ContactB[1],sizeof(dContactGeom));

                }
                if (NUM_RAYCHECK >= 3)
                {
#ifdef RECOMPUTE_RAYNORMAL
                    //CD
                    dCROSS(Normal[2],=,CD,FB);
                    dNormalize3(Normal[2]);
#endif

                    const dReal lCD = dSqrt(lWidthSq + CD[1]*CD[1]);
                    // ray CD and DC
                    tempRay.length = lCD;
                    dGeomRaySet(&tempRay, C[0], C[1], C[2], CD[0], CD[1], CD[2]);
                    //dGeomRaySetNoNormalize(tempRay, C, CD);
                    nA[2] = geomRayNCollider(&tempRay,o2,flags,&ContactA[2],sizeof(dContactGeom));
                    //dGeomRaySet(&tempRay, D[0], D[1], D[2], -CD[0], -CD[1], -CD[2]);
                    dGeomRaySetNoNormalize(tempRay, D, -CD);
                    nB[2] = geomRayNCollider(&tempRay,o2,flags,&ContactB[2],sizeof(dContactGeom));
                }

                for ( i = 0; i < NUM_RAYCHECK; i++ )
                {
                    if ( nA[i] & nB[i] )
                    {
                        pContact = CONTACT( contact,numTerrainContacts*skip );

                        pContact->pos[0] = (ContactA[i].pos[0] + ContactB[i].pos[0]) * 0.5f;
                        pContact->pos[1] = (ContactA[i].pos[1] + ContactB[i].pos[1]) * 0.5f;
                        pContact->pos[2] = (ContactA[i].pos[2] + ContactB[i].pos[2]) * 0.5f;

#ifdef RECOMPUTE_RAYNORMAL

                        pContact->normal[0] = -Normal[i][0];
                        pContact->normal[1] = -Normal[i][1];
                        pContact->normal[2] = -Normal[i][2];

#else // RECOMPUTE_RAYNORMAL

                        pContact->normal[0] = (ContactA[i].normal[0] + ContactB[i].normal[0]) * 0.5f;
                        pContact->normal[1] = (ContactA[i].normal[1] + ContactB[i].normal[1]) * 0.5f;
                        pContact->normal[2] = (ContactA[i].normal[2] + ContactB[i].normal[2]) * 0.5f;
                        dNormalize3(pContact->normal);

#endif // RECOMPUTE_RAYNORMAL



                        //
                        // Find Contact Penetration Depth
                        //

                        if ( geomNDepthGetter )
                        {
                            pContact->depth = geomNDepthGetter( o2,
                                pContact->pos[0], pContact->pos[1], pContact->pos[2] );
                            numTerrainContacts++;
                        }
                        else
                        {
                            // We don't have a GetDepth function, so do a ray cast instead.
                            // NOTE: This isn't ideal, and a GetDepth function should be
                            // written for all geom classes.
                            tempRay.length = 1000.f;

                            //dGeomRaySet( &tempRay, pContact->pos[0], pContact->pos[1], pContact->pos[2],
                            //    -pContact->normal[0], -pContact->normal[1], -pContact->normal[2] );
                            dGeomRaySetNoNormalize(tempRay, pContact->pos, -pContact->normal);

                            if ( geomRayNCollider( &tempRay, o2, flags, &ContactV, sizeof( dContactGeom ) ) )
                            {
                                pContact->depth = ContactV.depth;
                                numTerrainContacts++;
                            }
                        }

                        if ( numTerrainContacts == numMaxContacts )
                            break;

                    }
                }
                if ( numTerrainContacts == numMaxContacts )
                    break;
            }
#endif //HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY
        }
        if (numTerrainContacts == numMaxContacts )
            break;
    }
    dGeomDisable (sliding_plane);
    //delete [] PlaneContact;
    return numTerrainContacts;
}
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

    dContactGeom PlaneContact[HEIGHTFIELDMAXCONTACTPERCELL];
	flags = (flags & 0xffff0000) | HEIGHTFIELDMAXCONTACTPERCELL;

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
#ifdef dTRIMESH_ENABLED
#if dTRIMESH_ENABLED

	case dTriMeshClass:
		CollideRayN		= dCollideRayTrimesh;
		CollideNPlane	= dCollideTrimeshPlane;
		GetDepth		= NULL;// N/A?
		break;
#endif
#endif // dTRIMESH_ENABLED

	default:
		dIASSERT(0);	// Shouldn't ever get here.
		break;

	}

#ifndef HEIGHTFIELD_TRIANGLE_BORDER_AS_RAY
    CollideRayN		= NULL;
#endif

	dReal Plane[4],lBD,lCD,lBC;
	dVector3 A,B,C,D,BD,CD,BC,AB,AC;
	A[0] = x * m_p_data->m_fSampleWidth;
	A[2] = z * m_p_data->m_fSampleDepth;
	A[1] = m_p_data->GetHeight(x,z);

	B[0] = (x+1) * m_p_data->m_fSampleWidth;
	B[2] = z * m_p_data->m_fSampleDepth;
	B[1] = m_p_data->GetHeight(x+1,z);

	C[0] = x * m_p_data->m_fSampleWidth;
	C[2] = (z+1) * m_p_data->m_fSampleDepth;
	C[1] = m_p_data->GetHeight(x,z+1);

	D[0] = (x+1) * m_p_data->m_fSampleWidth;
	D[2] = (z+1) * m_p_data->m_fSampleDepth;
	D[1] = m_p_data->GetHeight(x+1,z+1);

	dOP(BC,-,C,B);
	//dOPEC(BC,/=,lBC);

    dOP(BD,-,D,B);
	//dOPEC(BD,/=,lBD);

	dOP(CD,-,D,C);
	//dOPEC(CD,/=,lCD);

	dOP(AB,-,B,A);
	//dNormalize3(AB);

	dOP(AC,-,C,A);
	//dNormalize3(AC);

	dCROSS(Plane,=,AC,AB);
	dNormalize3(Plane);
	Plane[3] = Plane[0] * A[0] + Plane[1] * A[1] + Plane[2] * A[2];
	dxPlane planeABC(0,Plane[0],Plane[1],Plane[2],Plane[3]);
	numPlaneContacts = CollideNPlane(o2,&planeABC,flags,PlaneContact,sizeof(dContactGeom));

	for ( i = 0; i < numPlaneContacts; i++ )
    { 
        if ( m_p_data->IsOnHeightfield( A, PlaneContact[i].pos, true) )
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
        if ( m_p_data->IsOnHeightfield( A, PlaneContact[i].pos, false ) )
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

    if ( CollideRayN && numContacts == 0)
    {

#ifdef RECOMPUTE_RAYNORMAL

        dVector3 E,F;
        dVector3 CE,FB,AD;
        dVector3 Normal[3];
        E[0] = (x+2) * m_p_data->m_fSampleWidth;
        E[2] = z * m_p_data->m_fSampleDepth;
        E[1] = m_p_data->GetHeight(x+2,z);
        F[0] = x * m_p_data->m_fSampleWidth;
        F[2] = (z+2) * m_p_data->m_fSampleDepth;
        F[1] = m_p_data->GetHeight(x,z+2);
        dOP(AD,-,D,A);
        //dNormalize3(AD);
        dOP(CE,-,E,C);
        //dNormalize3(CE);
        dOP(FB,-,B,F);
        //dNormalize3(FB);

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

        lCD = dLENGTH(CD);
        lBC = dLENGTH(BC);
        lBD = dLENGTH(BD);

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

        for ( i = 0; i < 3; i++ )
        {
            if ( nA[i] & nB[i] )
            {
                dContactGeom *pContact = CONTACT( contact,numContacts*skip );

                pContact->pos[0] = (ContactA[i].pos[0] + ContactB[i].pos[0])/2;
                pContact->pos[1] = (ContactA[i].pos[1] + ContactB[i].pos[1])/2;
                pContact->pos[2] = (ContactA[i].pos[2] + ContactB[i].pos[2])/2;

#ifdef RECOMPUTE_RAYNORMAL

                pContact->normal[0] = -Normal[i][0];
                pContact->normal[1] = -Normal[i][1];
                pContact->normal[2] = -Normal[i][2];

#else // RECOMPUTE_RAYNORMAL

                pContact->normal[0] = (ContactA[i].normal[0] + ContactB[i].normal[0])/2;
                pContact->normal[1] = (ContactA[i].normal[1] + ContactB[i].normal[1])/2;
                pContact->normal[2] = (ContactA[i].normal[2] + ContactB[i].normal[2])/2;
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

	return numContacts;
}


int dCollideHeightfield( dxGeom *o1, dxGeom *o2, int flags, dContactGeom* contact, int skip )
{
	dIASSERT( skip >= (int)sizeof(dContactGeom) );
	dIASSERT( o1->type == dHeightfieldClass );
	int i;

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


	//
	// Transform O2 into Heightfield Space
	//

	// Backup original o2 position, rotation and AABB.
	dVector3Copy( o2->final_posr->pos, posbak );
	dMatrix3Copy( o2->final_posr->R, Rbak );
	memcpy( aabbbak, o2->aabb, sizeof( dReal ) * 6 );
	gflagsbak = o2->gflags;

	if ( terrain->gflags & GEOM_PLACEABLE )
	{
		// Transform o2 into heightfield space.
		dOP( pos0, -, o2->final_posr->pos, terrain->final_posr->pos );
		dMULTIPLY1_331( pos1, terrain->final_posr->R, pos0 );
		dMULTIPLY1_333( R1, terrain->final_posr->R, o2->final_posr->R );

		// Update o2 with transformed position and rotation.
		dVector3Copy( pos1, o2->final_posr->pos );
		dMatrix3Copy( R1, o2->final_posr->R );
	}

#ifndef DHEIGHTFIELD_CORNER_ORIGIN
	o2->final_posr->pos[ 0 ] += terrain->m_p_data->m_fHalfWidth;
	o2->final_posr->pos[ 2 ] += terrain->m_p_data->m_fHalfDepth;
#endif // DHEIGHTFIELD_CORNER_ORIGIN

	// Rebuild AABB for O2
	o2->computeAABB();



	//
	// Collide
	//

	int nMinX = int(floor(o2->aabb[0] / terrain->m_p_data->m_fSampleWidth));
	int nMaxX = int(floor(o2->aabb[1] / terrain->m_p_data->m_fSampleWidth)) + 1;
	int nMinZ = int(floor(o2->aabb[4] / terrain->m_p_data->m_fSampleDepth));
	int nMaxZ = int(floor(o2->aabb[5] / terrain->m_p_data->m_fSampleDepth)) + 1;


	if ( terrain->m_p_data->m_bWrapMode == 0 )
	{
		nMinX = dMAX( nMinX, 0 );
		nMaxX = dMIN( nMaxX, terrain->m_p_data->m_nWidthSamples - 1 );
		nMinZ = dMAX( nMinZ, 0 );
		nMaxZ = dMIN( nMaxZ, terrain->m_p_data->m_nDepthSamples - 1 );

		if ((nMinX >= nMaxX) || (nMinZ >= nMaxZ))
			goto dCollideHeightfieldExit;
	}


	dVector3 AabbTop;
	AabbTop[0] = (o2->aabb[0]+o2->aabb[1]) / 2;
	AabbTop[2] = (o2->aabb[4]+o2->aabb[5]) / 2;
	AabbTop[1] = o2->aabb[3];

	if ( o2->type != dRayClass )
	{
		dReal AabbTopDepth = terrain->m_p_data->GetHeight( AabbTop[0],AabbTop[2] ) - AabbTop[1];
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


#ifdef _PLANECONSTRUCTIONOPTIMISATION

    numTerrainContacts  = terrain->dCollideHeightfieldZone(
        nMinX,nMaxX,nMinZ,nMaxZ,o2,numMaxTerrainContacts - numTerrainContacts,
        flags,CONTACT(contact,numTerrainContacts*skip),skip	);

#else //_PLANECONSTRUCTIONOPTIMISATION

    int j;
	// Collide against all potential collision cells.
	for ( i = nMinX; i < nMaxX; ++i )
	for ( j = nMinZ; j < nMaxZ; ++j )
	{ 
		numTerrainContacts += terrain->dCollideHeightfieldUnit(
			i,j,o2,numMaxTerrainContacts - numTerrainContacts,
			flags,CONTACT(contact,numTerrainContacts*skip),skip	);
	}

#endif //_PLANECONSTRUCTIONOPTIMISATION
	dIASSERT( numTerrainContacts <= numMaxTerrainContacts );

	for ( i = 0; i < numTerrainContacts; ++i )
	{
		CONTACT(contact,i*skip)->g1 = o1;
		CONTACT(contact,i*skip)->g2 = o2;
	}


//------------------------------------------------------------------------------

dCollideHeightfieldExit:


	// Restore o2 position, rotation and AABB
	dVector3Copy( posbak, o2->final_posr->pos );
	dMatrix3Copy( Rbak, o2->final_posr->R );
	memcpy( o2->aabb, aabbbak, sizeof(dReal)*6 );
	o2->gflags = gflagsbak;


	//
	// Transform Contacts to World Space
	//

	if ( terrain->gflags & GEOM_PLACEABLE )
	{
		for ( i = 0; i < numTerrainContacts; ++i )
		{
			dOPE( pos0, =, CONTACT(contact,i*skip)->pos );

#ifndef DHEIGHTFIELD_CORNER_ORIGIN
			pos0[ 0 ] -= terrain->m_p_data->m_fHalfWidth;
			pos0[ 2 ] -= terrain->m_p_data->m_fHalfDepth;
#endif // !DHEIGHTFIELD_CORNER_ORIGIN

			dMULTIPLY0_331( CONTACT(contact,i*skip)->pos, terrain->final_posr->R, pos0 );

			dOP( CONTACT(contact,i*skip)->pos, +, CONTACT(contact,i*skip)->pos, terrain->final_posr->pos );
			dOPE( pos0, =, CONTACT(contact,i*skip)->normal );

			dMULTIPLY0_331( CONTACT( contact, i*skip )->normal, terrain->final_posr->R, pos0 );
		}
	}
#ifndef DHEIGHTFIELD_CORNER_ORIGIN
	else
	{
		for ( i = 0; i < numTerrainContacts; ++i )
		{
			CONTACT(contact,i*skip)->pos[ 0 ] -= terrain->m_p_data->m_fHalfWidth;
			CONTACT(contact,i*skip)->pos[ 2 ] -= terrain->m_p_data->m_fHalfDepth;
		}
	}
#endif // !DHEIGHTFIELD_CORNER_ORIGIN

	// Return contact count.
	return numTerrainContacts;
}


