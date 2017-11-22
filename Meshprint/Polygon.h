#pragma once
#include <vector>
#include <set>
#include "Cutline.h"
struct comPoints
{
	bool  operator ()(CutPoint* a, CutPoint* b)const
	{
		if (a->getPosition().x() - b->getPosition().x() < -1e-4)
		{
			return true;
		}
		else if (a->getPosition().x() - b->getPosition().x() < 1e-4)
		{
			if (a->getPosition().y() - b->getPosition().y() < -1e-4)
			{
				return true;
			}
			else if (a->getPosition().y() - b->getPosition().y() < 1e-4)
			{
				return a->getPosition().z() - b->getPosition().z() < -1e-4;
			}
			return false;
		}
		return false;
	}
};
bool sortByAngle(const CutLine* a, const CutLine* b)
{
	return a->angle_ > b->angle_;
}
class Polygon
{
public:
	Polygon();
	~Polygon();
	CutLine* insertEdge(CutLine * e);
	CutLine* insertEdge(Vec3f a, Vec3f b);
	void sweepPolygon();
private:
	std::vector<CutLine*> edges;
	std::set<CutPoint*,comPoints> points;

	float angleWithXAxis(Vec3f dir);
};

