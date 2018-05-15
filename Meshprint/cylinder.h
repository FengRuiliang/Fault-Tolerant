#pragma once
#include "HE_mesh/Mesh3D.h"
class cylinder
{
public:
	cylinder();
	~cylinder();
	Mesh3D creat_cylinder(Vec3f s, Vec3f e,float d);
private:
	Vec3f start_center, end_center;
	float diameter;
	Mesh3D cylinder_mesh;
};

