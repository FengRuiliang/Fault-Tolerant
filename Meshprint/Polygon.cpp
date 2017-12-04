#include "Polygon.h"
#include "Cutline.h"
#include "HE_mesh/Vec.h"
#include <algorithm>
#include "clipper.hpp"
typedef trimesh::vec3  Vec3f;

Polygon::Polygon()
{
}


Polygon::~Polygon()
{
}
CutLine* Polygon::insertEdge(CutLine* e)
{
	edges.push_back(e);
	return e;
}

CutLine* Polygon::insertEdge(Vec3f a, Vec3f b)
{
	
	CutPoint*   p1 = new CutPoint(a);
	CutPoint*	p2 = new CutPoint(b);
	auto pair1_ = points.insert(p1);
	auto pair2_ = points.insert(p2);
	CutLine* e = new CutLine(*pair1_.first, *pair2_.first);
	(*pair1_.first)->setOutEdge(e);
	(*pair2_.first)->setInEdge(e);
	if (!pair1_.second)
		delete p1;
	if (!pair2_.second)
		delete p2;
	edges.push_back(e);
	return e;
}

void Polygon::sweepPolygon()
{
	std::set<CutLine*> crossEdges;
	for (auto iter = points.begin(); iter != points.end(); iter++)
	{

		if ((*iter)->getInEdges().size() == 1 && (*iter)->getOutEdges().size() == 1)
		{
			(*iter)->getInEdges()[0]->pnext_ = (*iter)->getOutEdges()[0];
		}
		else if ((*iter)->getInEdges().size() > 0 && (*iter)->getOutEdges().size() > 0)
		{
			std::vector<CutLine*> lines;
			for (auto iterIn = (*iter)->getInEdges().begin(); iterIn != (*iter)->getInEdges().end(); iterIn++)
			{
				(*iterIn)->angle_ = angleWithXAxis((*iterIn)->position_vert[0] - (*iterIn)->position_vert[1]);
				lines.push_back(*iterIn);
			}
			for (auto iterOut = (*iter)->getOutEdges().begin(); iterOut != (*iter)->getOutEdges().end(); iterOut++)
			{
				(*iterOut)->angle_ = angleWithXAxis((*iterOut)->position_vert[0] - (*iterOut)->position_vert[1]);
				lines.push_back(*iterOut);
			}
			std::sort(lines.begin(), lines.end(), sortByAngle);//ccw
			while (lines.size()>1)
			{
				int count = lines.size();
				while (count > 0 &&
					(lines.back()->cut_point_[1] != *iter ||
						lines[lines.size() - 1]->cut_point_[0] != *iter))
				{
					lines.insert(lines.begin(), lines.back());
					lines.pop_back();
					count--;
				}
				if (count!=0)
				{
					CutLine* in_ = lines.back();
					lines.pop_back();
					in_->pnext_ = lines.back();
					lines.pop_back();
				}
				
			}			
		}
	}
}

inline float Polygon::angleWithXAxis(Vec3f dir)
{
	if (dir.x() == 0.0)
	{
		if (dir.y() > 0)
		{
			return 90;
		}
		else
		{
			return 270;
		}
	}
	else if (dir.x() < 0)
	{
		return atan(dir.y() / dir.x()) + 180;
	}
	else if (dir.y() < 0)
	{
		return atan(dir.y() / dir.x()) + 360;
	}
	else
		return atan(dir.y() / dir.x());
}

void Polygon::Voronoi()
{

}
