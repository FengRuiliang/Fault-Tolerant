#pragma once
#include "globalFunctions.h"
class cutLine
{
public:
	cutLine() {};
	cutLine(point p1, point p2)
	{
		position_vert[0] = p1; position_vert[1] = p2;
		sweep_point_ = p1;
		sweep_point_Last_ = p1;
		/*	edgeid_vert[0] = edge1; edgeid_vert[1] = edge2;*/
	}
	cutLine(point p1, point p2, int xID, int yID)
	{
		position_vert[0] = p1; position_vert[1] = p2;
		x_field_ = xID;
		y_field_ = yID;
		/*	edgeid_vert[0] = edge1; edgeid_vert[1] = edge2;*/
	}
	~cutLine()
	{}

	point position_vert[2];
	int edgeid_vert[2];

public:
	Vec3f sweep_point_;
	Vec3f sweep_point_Last_;
	int x_field_;
	int y_field_;
};

