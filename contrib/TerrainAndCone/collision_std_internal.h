//Benoit CHAPEROT 2003 www.jstarlab.com
#ifndef _ODE_COLLISION_STD_INTERNAL_H_
#define _ODE_COLLISION_STD_INTERNAL_H_

#include <ode/common.h>
#include "collision_kernel.h"

struct dxSphere : public dxGeom {
  dReal radius;		// sphere radius
  dxSphere (dSpaceID space, dReal _radius);
  void computeAABB();
};


struct dxBox : public dxGeom {
  dVector3 side;	// side lengths (x,y,z)
  dxBox (dSpaceID space, dReal lx, dReal ly, dReal lz);
  void computeAABB();
};


struct dxCCylinder : public dxGeom {
  dReal radius,lz;	// radius, length along z axis
  dxCCylinder (dSpaceID space, dReal _radius, dReal _length);
  void computeAABB();
};


struct dxPlane : public dxGeom {
  dReal p[4];
  dxPlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d);
  void computeAABB();
};

struct dxCylinder : public dxGeom {
  dReal radius,lz;	// radius, length along z axis
  dxCylinder (dSpaceID space, dReal _radius, dReal _length);
  void computeAABB();
};

struct dxCone : public dxGeom {
  dReal radius,lz;
  dxCone(dSpaceID space, dReal _radius,dReal _length);
  ~dxCone();
  void computeAABB();
};

struct dxRay : public dxGeom {
  dReal length;
  dxRay (dSpaceID space, dReal _length);
  void computeAABB();
};

struct dxTerrain : public dxGeom {
  dReal m_vLength;
  dReal *m_pHeights;
  dReal m_vMinHeight;
  dReal m_vMaxHeight;
  dReal m_vNodeLength;
  int	m_nNumNodesPerSide;
  int	m_nNumNodesPerSideShift;
  int	m_nNumNodesPerSideMask;
  dxTerrain(dSpaceID space, dReal *pHeights,dReal vLength,int nNumNodesPerSide);
  ~dxTerrain();
  void computeAABB();
  dReal GetHeight(dReal x,dReal y);
  dReal GetHeight(int x,int y);
  int dCollideTerrainUnit(int x,int y,dxGeom *o2,int numMaxContacts,int flags,dContactGeom *pContact);
  bool IsOnTerrain(int nx,int ny,int w,dReal *pos);
};

#endif //_ODE_COLLISION_STD_INTERNAL_H_