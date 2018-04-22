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
	void support_point_sampling(int counter_);
	std::pair<float, float> get_dense(int angle);
private:
	Mesh3D* target_mesh;
	int limit_angle_ = { 30 };

public:
	std::map<int,Mesh3D> supp_aeras;
	std::vector<Vec3f> sample_points_;	Mesh3D sp_mesh;
};

