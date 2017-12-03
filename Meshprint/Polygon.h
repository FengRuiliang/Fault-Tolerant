#pragma once
#include <vector>
#include <set>
#include "Cutline.h"
#define  LIMIT 1e-3
struct comPoints
{
	bool  operator ()(CutPoint* a, CutPoint* b)const
	{
		if (a->getPosition().x() - b->getPosition().x() < -LIMIT)
		{
			return true;
		}
		else if (a->getPosition().x() - b->getPosition().x() < LIMIT)
		{
			return a->getPosition().y() - b->getPosition().y()<-LIMIT;
		}
		return false;
	}
};
struct comPointsLarge
{
	bool  operator ()(CutPoint* a, CutPoint* b)const
	{
		if (a->getPosition().x() - b->getPosition().x() < -5*LIMIT)
		{
			return true;
		}
		else if (a->getPosition().x() - b->getPosition().x() < 5*LIMIT)
		{
			return a->getPosition().y() - b->getPosition().y() < -5*LIMIT;
		}
		return false;
	}
};


struct comPointsHuge
{
	bool  operator ()(CutPoint* a, CutPoint* b)const
	{
		if (a->getPosition().x() - b->getPosition().x() < -17 * LIMIT)
		{
			return true;
		}
		else if (a->getPosition().x() - b->getPosition().x() < 17 * LIMIT)
		{
			return a->getPosition().y() - b->getPosition().y() < -17 * LIMIT;
		}
		return false;
	}
};



class Polygon
{
public:
	Polygon();
	~Polygon();
	CutLine* insertEdge(CutLine * e);
	CutLine* insertEdge(Vec3f a, Vec3f b);
	int FindIntersection();
	void ConnectCutline();
	int num_of_edges() { return edges.size(); }
	int num_of_points() { return points.size(); }
private:
	std::vector<CutLine*> edges;
	std::set<CutPoint*,comPoints> points;

	float angleWithXAxis(Vec3f dir);
	void UpdateStructure(CutPoint* ptr_point_, std::vector<CutLine *>& str_line_, std::vector<CutLine *>& left_line_, std::vector<CutLine *>& righ_line_, std::vector<CutLine *>& cros_line_);
public:
	void storePathToPieces(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_list_,int id);
	void FindNewEvent(CutLine * down, CutLine * up, CutPoint * point, std::vector<CutPoint*> queue);
private:
};

