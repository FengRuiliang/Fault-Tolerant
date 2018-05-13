#pragma once
#include "HE_mesh/Mesh3D.h"
#include "HE_mesh/Vec.h"
#include "Library/clipper.hpp"
class Support
{
public:
	Support();
	Support(Mesh3D * mesh);
	~Support();
	void sup_face_dfs(HE_face * facet, std::vector<HE_face*>* faces, int angle_id_);
	void angle_dfs(HE_face * facet, Mesh3D * mesh, int angle_id_);
	void sup_mesh_dfs(HE_face * facet, Mesh3D * mesh);
	void find_support_area();
	void support_point_sampling(int counter_);
	ClipperLib::IntPoint get_dense(int angle);
private:
	Mesh3D* target_mesh;
	int limit_angle_ = { 30 };
	std::vector<bool> face_selected_;
	std::vector<bool> face_marked_;
	Mesh3D* wholemesh;// find big support area

public:
	std::map<int, std::vector<Mesh3D*>> sup_ptr_aera_list_;
	std::vector<Mesh3D*> sup_areas_;
	std::map<int,std::vector<Vec3f>> sample_points_;
	Mesh3D sp_mesh;
private:
	void sam_project_to_mesh(std::map<int, std::vector<Vec3f>> sample_points_);
	int pso_target_ = { 0 };
};

