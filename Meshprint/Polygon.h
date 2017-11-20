#pragma once
#include <vector>
#include "Cutline.h"
class Polygon
{
public:
	Polygon();
	~Polygon();
	CutLine* insertEdge(CutLine * e);
	CutLine* insertEdge(Vec3f a, Vec3f b);
	void sweepPolygon();
	void sortByX(CutPoint * a, CutPoint * b);
private:
	std::vector<CutLine*> edges;
	std::vector<CutPoint*> points;

};

