#pragma once
#include "HE_mesh/Mesh3D.h"
#include "HE_mesh/Vec.h"
#include "MSAABB.h"
class Mesh3D;

class OCTREE
{
private:
	struct node 
	{
		node* child[8];
	};
	void BuildOctree(std::vector<HE_face *> * fList);
public:
	OCTREE();
	OCTREE(Mesh3D* mesh);
	~OCTREE();
};

