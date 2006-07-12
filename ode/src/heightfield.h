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
	dReal m_vWidth;            // World space heightfield dimension on X axis
	dReal m_vDepth;            // World space heightfield dimension on Z axis
	dReal m_vMinHeight;        // Min sample height value (scaled and offset)
	dReal m_vMaxHeight;        // Max sample height value (scaled and offset)
	dReal m_vSampleWidth;      // Vertex count on X axis edge (== m_vWidth / (m_nWidthSamples-1))
	dReal m_vSampleDepth;      // Vertex count on Z axis edge (== m_vDepth / (m_nDepthSamples-1))
	dReal m_vThickness;        // Surface thickness (added to bottom AABB)
	dReal m_vScale;            // Sample value multiplier
	dReal m_vOffset;           // Vertical sample offset
	int	m_nWidthSamples;       // Vertex count on X axis edge (number of samples)
	int	m_nDepthSamples;       // Vertex count on Z axis edge (number of samples)
	int m_bCopyHeightData;     // Copy sample data flag
	int	m_nWrapMode;           // Heightfield wrapping mode (0=finite, 1=infinite)
	int m_nGetHeightMode;      // GetHeight mode ( 0=callback, 1=byte, 2=short, 3=float )
	const void* m_pHeightData; // Sample data array
	void* m_pUserData;         // Callback user data

	dHeightfieldGetHeight* m_pGetHeightCallback;		// Callback pointer.

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
