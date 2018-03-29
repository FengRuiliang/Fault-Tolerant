#pragma once
#include "HE_mesh/Mesh3D.h"
#include "HE_mesh/Vec.h"
class Support
{
public:
	Support();
	Support(Mesh3D * mesh);
	~Support();
	void project_on_ground();
private:
	Mesh3D* target_mesh;
public:
	std::map<int,Mesh3D> supp_aeras;
};

