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

class Polygon
{
public:
	Polygon();
	~Polygon();
	CutLine* insertEdge(CutLine * e);
	CutLine* insertEdge(Vec3f a, Vec3f b);
	int sweepPolygon();
	int num_of_edges() { return edges.size(); }
	int num_of_points() { return points.size(); }
private:
	std::vector<CutLine*> edges;
	std::set<CutPoint*,comPoints> points;

	float angleWithXAxis(Vec3f dir);
public:
	void storePathToPieces(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_list_,int id);
};

