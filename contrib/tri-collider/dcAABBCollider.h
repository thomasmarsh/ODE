#ifndef __DCAABBTREECOLLIDER_INCLUDED__
#define __DCAABBTREECOLLIDER_INCLUDED__

class OPCODE_API dcAABBTreeCollider{
	CollisionAABB Box;

	Point* LeafVerts[3];
	udword LeafIndex;
public:
	dword BoxBoxTestCount;
	dword BoxTriTestCount;

	/* In */
	const dcArray<Vertex>& Vertices;
	const dcArray<dword>& Indices;

	/* In/Out */
	dcVector<const AABBNoLeafNode*>* TCData;

	/* Out */
	dcVector<dword> Contacts;

	/* Constructor/destructor */
	dcAABBTreeCollider(const dcArray<Vertex>& Vertices, const dcArray<dword>& Indices);
	~dcAABBTreeCollider();

	/* Collide */
	void Collide(const AABBNoLeafTree* Tree, const CollisionAABB& Box);
	void GenerateTC(const AABBNoLeafTree* Tree, const CollisionAABB& Box);
private:
	void _CollideTriBox();
	void _Collide(const AABBNoLeafNode* a);

	void _GenerateTC(const AABBNoLeafNode* a);

	/* General */
	bool BoxBoxOverlap(const Point& a, const Point& Pa);
	bool TriBoxOverlap();
	
	void InitQuery(const CollisionAABB& Box);
};
	
#endif // __DCAABBTREECOLLIDER_INCLUDED__
