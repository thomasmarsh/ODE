#ifndef __DCOBBCOLLIDER_INCLUDED__
#define __DCOBBCOLLIDER_INCLUDED__


class OPCODE_API dcOBBTreeCollider{
	Point Box;

	float BBx1;
	float BBy1;
	float BBz1;

	float BB_1;
	float BB_2;
	float BB_3;
	float BB_4;
	float BB_5;
	float BB_6;
	float BB_7;
	float BB_8;
	float BB_9;

	Matrix3x3 mAR;
	Matrix3x3 mR0to1;
	Matrix3x3 mR1to0;
	Point mT0to1;
	Point mT1to0;

	Point LeafVerts[3];
	udword LeafIndex;

public:
	/* In */
	const dcVector3*& Vertices;
	const int*& Indices;

	dArray<const AABBNoLeafNode*>* TCData;

	/* Out */
	dArray<int> Contacts;

	/* Constructor/destructor */
	dcOBBTreeCollider(const dcVector3*& Vertices, const int*& Indices);
	~dcOBBTreeCollider();

	/* Collision queries */
	void Collide(const AABBNoLeafTree* Tree, const Point& Box, const Matrix4x4& BoxMatrix);
private:
	void _CollideTriBox();
	void _Collide(const AABBNoLeafNode* a);
	
	bool BoxBoxOverlap(const Point& a, const Point& Pa);
	bool TriBoxOverlap();
	
	void InitQuery(const Point& Box, const Matrix4x4& BoxMatrix);
};
	
#endif // __DCCOBBCOLLIDER_INCLUDED__
