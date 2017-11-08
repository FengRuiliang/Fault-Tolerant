#include "Cubes.h"

class Box;

void Cubes::insertToBox(Box box)
{
	std::set<Box, sortBox>::iterator box_iter_ = boxes_.insert(box).first;
}

Cubes::Box::Box()
{
	idX_ = 0;	idY = 0;	idZ_ = 0;	zheight_ = 0;	xmin_ = 0.0;	ymin_ = 0.0;	zmin_ = 0.0;
}

Cubes::Box::Box(int x, int y, int z, Vec3f hei)
{
	idX_ = x;
	idY = y;
	idZ_ = z;
	xlength = hei[0];
	ywidth = hei[1];
	zheight_ = hei[2];
	xmin_ = x*xlength;
	ymin_ = y*ywidth;
	zmin_ = z*zheight_;
}

Cubes::Box::Box(int x, int y, int z)
{
	idX_ = x;
	idY = y;
	idZ_ = z;
}

void Cubes::Box::setCube(float x, float y, float z, float h)
{
	xmin_ = x; ymin_ = y; zmin_ = z; zheight_ = h;
}

Vec4f Cubes::Box::GetCoordinate()
{
	return Vec4f(xmin_, ymin_, zmin_, zheight_);
}

std::vector<int>  Cubes::Box::getID() const
{
	std::vector<int> id = { idX_,idY,idZ_ };
	return id;
}
