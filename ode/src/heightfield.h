// dHeightfield Collider
//  Martijn Buijs 2006 http://home.planet.nl/~buijs512/
// Based on Terrain & Cone contrib by:
//  Benoit CHAPEROT 2003-2004 http://www.jstarlab.com

#ifndef _DHEIGHTFIELD_H_
#define _DHEIGHTFIELD_H_

#include <ode/common.h>
#include "collision_kernel.h"

// Heightfield Data structure
struct dxHeightfieldData
{
	dReal m_vWidth;            // world space heightfield dimension on X axis
	dReal m_vDepth;            // world space heightfield dimension on Z axis
	dReal m_vMinHeight;        // min sample height value (scaled and offset)
	dReal m_vMaxHeight;        // max sample height value (scaled and offset)
	dReal m_vSampleWidth;      // sample spacing on X axis (== m_vWidth / m_nWidthSamples)
	dReal m_vSampleDepth;      // sample spacing on Z axis (== m_vDepth / m_nDepthSamples)
	dReal m_vThickness;        // surface thickness (currently only added to bottom AABB)
	dReal m_vScale;            // sample value multiplier
	dReal m_vOffset;           // vertical sample offset
	int	m_nWidthSamples;       // number of samples on X axis
	int	m_nDepthSamples;       // number of samples on Z axis
	int m_bCopyHeightData;     // copy sample data flag
	int	m_nWrapMode;           // heightfield wrapping mode (0=finite, 1=infinite)
	int m_nGetHeightMode;      // getheight mode (0=callback, 1=byte, 2=short, 3=float)
	void *m_pHeightData;       // sample data array
	void *m_pUserData;         // callback user data

	dHeightfieldGetHeight* m_pGetHeightCallback;

	dxHeightfieldData();
	~dxHeightfieldData();

	void SetData( int nWidthSamples, int nDepthSamples,
				  dReal vWidth, dReal vDepth,
				  dReal vScale, dReal vOffset,
				  int bFinite, dReal vThickness );

	void ComputeHeightBounds();

	bool IsOnHeightfield(int nx, int nz, int w, dReal *pos);
	dReal GetHeight(int x, int z);
	dReal GetHeight(dReal x, dReal z);
};


// Heightfield geom structure
struct dxHeightfield : public dxGeom
{
	dxHeightfieldData* m_p_data;

	dxHeightfield( dSpaceID space, dHeightfieldDataID data, int bPlaceable );
	~dxHeightfield();

	void computeAABB();

	int dCollideHeightfieldUnit( int x, int z, dxGeom *o2, int numMaxContacts, 
		int flags, dContactGeom *contact, int skip );
};


#endif //_DHEIGHTFIELD_H_
