#pragma once
#include "HE_mesh/Vec.h"
#include "HE_mesh/Mesh3D.h"
#include "globalFunctions.h"
using namespace std;
class Grid
{
public:
	Grid();
	Grid(int x, int y);
	~Grid();
	std::map<int, std::set<int>> chain;
	int x_id_, y_id;
	float size_;
};

