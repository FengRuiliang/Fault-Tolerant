#pragma once
#include "HE_mesh/Mesh3D.h"
#include "HE_mesh/Vec.h"
class Support
{
public:
	Support();
	Support(Mesh3D * mesh);
	~Support();
	void sup_face_dfs(HE_face * facet, Mesh3D* mesh_);
	void find_support_area();
	void support_point_sampling(int counter_);
	std::pair<float, float> get_dense(int angle);
private:
	Mesh3D* target_mesh;
	int limit_angle_ = { 30 };
	std::vector<bool> face_selected_;
	std::vector<bool> face_marked_;


public:
	std::vector<Mesh3D*> sup_ptr_aera_list_;
	std::vector<Vec3f> sample_points_;
	Mesh3D sp_mesh;
};

