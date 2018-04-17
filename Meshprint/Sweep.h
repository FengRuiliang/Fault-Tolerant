#pragma once
#include "HE_mesh/Mesh3D.h"
using namespace trimesh;
struct SweepLine;
struct SweepPoint
{
	Vec3f position_;
	SweepLine* line_[2];
};
struct SweepLine
{
	SweepPoint* left_;
	SweepPoint* right_;
};

class Sweep
{
public:
	Sweep();
	~Sweep();
private:
	std::vector<SweepPoint*> points_;
	std::vector<SweepLine*> lines_;

public:
	Sweep(std::vector<Vec3f> input);

};

