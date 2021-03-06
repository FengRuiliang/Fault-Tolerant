#pragma once
#include "HE_mesh/Vec.h"
#include <vector>
typedef trimesh::vec3  Vec3f;
class CutLine;
class CutPoint
{
public:CutPoint(Vec3f a) { pos_ = a; };
	   ~CutPoint() {};
	   CutLine* setInEdge(CutLine* e) { in_edges_.push_back(e); return e; }
	   CutLine* setOutEdge(CutLine*e) { out_edges_.push_back(e); return e; }
	   Vec3f getPosition()const { return pos_; }
	   std::vector<CutLine*>& getInEdges() { return in_edges_; }
	   std::vector<CutLine*>& getOutEdges() { return out_edges_; }
	   int getEdgeSize() { return in_edges_.size() + out_edges_.size(); }
private:
	Vec3f pos_;
	std::vector<CutLine*> in_edges_;
	std::vector<CutLine*> out_edges_;
	
};

class CutLine
{
public:
	CutLine() {};
	CutLine(Vec3f p1, Vec3f p2)
	{
		position_vert[0] = p1; position_vert[1] = p2;
	}
	CutLine(CutPoint* p1, CutPoint* p2)
	{
		cut_point_[0] = p1;
		cut_point_[1] = p2;
		position_vert[0] = p1->getPosition();
		position_vert[1] = p2->getPosition();
		order_point_[0] = p1->getPosition() < p2->getPosition() ? p1 : p2;
		order_point_[1] = p1->getPosition() > p2->getPosition() ? p1 : p2;
	}
	~CutLine()
	{}
	Vec3f position_vert[2];
	CutPoint* order_point_[2];
	CutPoint* cut_point_[2];
	
	CutLine* pnext_{NULL};
	CutLine* pprev_{ NULL };
	float angle_{0};
	bool visit{ false };
	bool isoutedge{ false };
	Vec3f cross_point_;
};



