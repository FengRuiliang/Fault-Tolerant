#include "Polygon.h"
#include "Cutline.h"
#include "HE_mesh/Vec.h"
#include <algorithm>
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
}

CutLine* Polygon::insertEdge(Vec3f a, Vec3f b)
{
	CutLine* e = new CutLine(a, b);
	CutPoint*   p1 = new CutPoint(a);
	CutPoint*	p2 = new CutPoint(b);
	points.push_back(p1);
	points.push_back(p2);
	edges.push_back(e);
	p1->setEdge(e);
	p2->setEdge(e);
}

void Polygon::sweepPolygon()
{
	std::sort(points.begin(), points.end(), sortByX);
	for (auto iterP = points.begin(); iterP != points.end();)
	{
		std::set<HE_edge*, comHE>::iterator iter = segments.begin();
		for (auto iterSEG = segments.begin(); iterSEG != segments.end();)
		{

			(*iterSEG)->is_selected_ = false;
			Vec3f& rP_ = (*iterSEG)->pvert_->position().x() > (*iterSEG)->start_->position().x() ? (*iterSEG)->pvert_->position() : (*iterSEG)->start_->position();
			Vec3f& lP_ = (*iterSEG)->pvert_->position().x() < (*iterSEG)->start_->position().x() ? (*iterSEG)->pvert_->position() : (*iterSEG)->start_->position();
			if ((*iterP)->position().x() - rP_.x() > -1e-5)//É¾³ý??
			{
				iterSEG = segments.erase(iterSEG);
			}
			else
			{
				if ((rP_ - lP_).cross((*iterP)->position() - lP_).z() > 0)// is up the segment
				{
					iter = iterSEG;
				}
				iterSEG++;
			}
		}
		(*iter)->is_selected_ = 1;
		(*++iter)->is_selected_ = 1;
		--iter;

		Trapezoidal(iter, iterP);
		if (sss == 110)
		{
			//break;
		}
	}


}
bool Polygon::sortByX(CutPoint* a,CutPoint* b)
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
