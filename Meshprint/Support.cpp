#include "Support.h"
#include <algorithm>


#define PI 3.1415926
Support::Support()
{
}
Support::Support(Mesh3D* mesh)
{
	target_mesh = mesh;

}

Support::~Support()
{
}

void Support::project_on_ground()
{
	Vec3f perpendicular(0.0, 0.0, 1.0);
	for (auto iter=target_mesh->get_faces_list()->begin();iter!=target_mesh->get_faces_list()->end();iter++)
	{
		int supp_angle = acos((*iter)->normal() * perpendicular) * 180 / PI;
		if (supp_angle>120)
		{

			std::vector<HE_vert*> verts,input;
			(*iter)->face_verts(verts);
			for (auto iterV=verts.begin();iterV!=verts.end();iterV++)
			{
			input.push_back(supp_aeras[(180 - supp_angle)/10].InsertVertex((*iterV)->position()));

			}
			supp_aeras[(180 - supp_angle)/10].InsertFaceSup(input);
		}		
	}
	for (auto iterM=supp_aeras.begin();iterM!=supp_aeras.end();iterM++)
	{
		iterM->second.UpdateMeshSup();


// 		continue;
// 		for (int k=0;k<flist.size();k++)
// 		{
// 			flist[k]->vertices_[0] -= Vec3f(0.0, 0.0, flist[k]->vertices_[0].z());
// 			flist[k]->vertices_[1] -= Vec3f(0.0, 0.0, flist[k]->vertices_[1].z());
// 			flist[k]->vertices_[2] -= Vec3f(0.0, 0.0, flist[k]->vertices_[2].z());
// 		}
	}

}
void Support::support_point_sampling()
{
	// 对每个三角面片的支撑点采样
	auto face_list_ = *(target_mesh->get_faces_list());
	Vec3f perpendicular(0.0, 0.0, 1.0);
	for (int i=0;i<face_list_.size();i++)
	{
		int supp_angle =180- acos(face_list_[i]->normal() *perpendicular) * 180 / PI;
		if (supp_angle<20)
		{
			std::pair<float, float> dense(2.0, 2.0);
			std::vector<Vec3f>& v=face_list_[i]->vertices_;
			


			float x_min_ =std::min(std::min(v[0].x(), v[1].x()),v[2].x());
			float x_cor_ = x_min_ / dense.first;

		}
	}




	for (auto miter = supp_aeras.begin(); miter != supp_aeras.end(); miter++)
	{
		auto boundary_loop_ = miter->second.GetBLoop();
		std::pair<float, float> dense(2.0, 2.0);
		std::vector<HE_vert*> sweep_point_;
		std::vector<HE_edge*> sweep_segment_;
		for (int i = 0; i < boundary_loop_.size(); i++)
		{
			for (int j=0;j<boundary_loop_[i].size();j++)
			{
				sweep_point_.push_back(boundary_loop_[i][j]->start_);
				sweep_point_.push_back(boundary_loop_[i][j]->pvert_);
			}
		}

		std::sort(sweep_point_,)
		
	}
}