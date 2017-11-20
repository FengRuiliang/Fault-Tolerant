#pragma once
#include "HE_mesh/Vec.h"
typedef trimesh::vec3  Vec3f;

class CutLine
{
public:
	CutLine() {};
	CutLine(Vec3f p1, Vec3f p2)
	{
		position_vert[0] = p1; position_vert[1] = p2;
	}
	~CutLine()
	{}
	Vec3f position_vert[2];
};
class CutPoint
{
public:CutPoint(Vec3f a) { pos_ = a; };
	   ~CutPoint() {};
	   CutLine* setEdge(CutLine* e) { return pedge_ = e; }
	   Vec3f getPosition() { return pos_; }
private:
	CutLine* pedge_{ NULL };
	Vec3f pos_;
};


