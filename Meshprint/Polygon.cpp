#include "Polygon.h"
#include "Cutline.h"
#include "HE_mesh/Vec.h"
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
}

void Polygon::sweepPolygon()
{

}
void Polygon::sortByX(CutPoint* a,CutPoint* b)
{
	if (a->pos_.x() - b->position().x() < -1e-4)
	{
		return true;
	}
	else if (a->position().x() - b->position().x() < 1e-4)
	{
		if (a->position().y() - b->position().y() < -1e-4)
		{
			return true;
		}
		else if (a->position().y() - b->position().y() < 1e-4)
		{
			return a->position().z() - b->position().z() < -1e-4;
		}
		return false;
	}
	return false;
}
