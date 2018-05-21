#pragma once
#include "HE_mesh/Mesh3D.h"
#include "HE_mesh/Vec.h"
#include "Library/clipper.hpp"
using namespace ClipperLib;
extern Paths test_path;
class Support
{
public:
	Support();
	Support(Mesh3D * mesh);
	~Support();
	void sup_face_dfs(HE_face * facet, std::vector<HE_face*>& faces, int angle_id_, std::vector<int>& vec_id);

	void sup_face_dfs(HE_face * facet, std::vector<HE_face*>& faces);


	void angle_dfs(HE_face * facet, Mesh3D * mesh, int angle_id_);
	void angle_dfs(HE_face* facet, std::vector<HE_face*>& re_faces, int angle_id_);
	void sup_mesh_dfs(HE_face* facet, Mesh3D* mesh, int angle_id);
	void find_support_area();
	void support_point_sampling(int counter_);
	
	void exportcylinder(const char * fouts);
	

	

private:
	Mesh3D* target_mesh;
	int limit_angle_ = { 30 };
	std::vector<bool> face_selected_;
	std::vector<bool> face_marked_;


public:

	std::vector<Mesh3D*> component;
	std::vector<Vec3f> sample_points_;
	std::vector<std::map<int, std::vector<Mesh3D*>>> component_regions_mesh;
	Mesh3D sp_mesh;
	std::vector<int> vec_angle;

private:
	void sam_project_to_mesh(std::vector<Vec3f> sample_points_);
	Mesh3D* wholemesh;// find big support area
					 // local minimal point
	int pso_target_ = { 0 };
};
namespace SupportLib
{
	Vec2f get_dense(int angle);
	void single_area_sampling(Mesh3D * mesh, Vec2f dense, std::vector<Vec3f>& last_loop_point, Vec2f center=Vec2f(0,0));
	void single_area_sampling(Mesh3D * mesh, Vec2f dense, std::vector<Vec3f>& last_loop_point, Paths& last_polygons, Vec2f center = Vec2f(0, 0));
	std::vector<Vec3f> compute_local_low_point(Mesh3D *mesh);
}
