#ifndef __DXTRILIST_INCLUDED__
#define __DXTRILIST_INCLUDED__

class dcTriListCollider;

struct dxTriList{
	dReal p[4];						// dxPlane
	dTriCallback* Callback;
	dTriArrayCallback* ArrayCallback;
	dcTriListCollider* Collider;
};

#endif	//__DXTRILIST_INCLUDED__