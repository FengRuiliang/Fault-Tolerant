#include "Support.h"
#include <algorithm>
#include <PSO/pso.h>
#include "Library/clipper.hpp"
#include "Library/Octree.h"
#include <qdebug.h>
#define PI 3.1415926
Support::Support()
{
}
Support::Support(Mesh3D* mesh)
{
	target_mesh = mesh;
	sp_mesh.LoadFromOBJFile(".\\Resources\\models\\sp_sim.obj");
	sp_mesh.scalemesh(0.2);
}

Support::~Support()
{
}
void Support::sup_face_dfs(HE_face* facet, Mesh3D* mesh_)
{
	face_selected_[facet->id()] = true;
	std::vector<HE_vert*> verts, input;
	facet->face_verts(verts);
	for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
	{
		input.push_back(mesh_->InsertVertex((*iterV)->position()));
	}
	mesh_->InsertFaceSup(input);
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		if (facet != NULL && !face_selected_[facet->id()] && face_marked_[facet->id()])
		{
			sup_face_dfs(facet, mesh_);
		}
		cur = cur->pnext_;
	} while (cur != sta);
}

void Support::find_support_area()
{
	Vec3f perpendicular(0.0, 0.0, 1.0);
	face_selected_.resize(target_mesh->num_of_face_list(), false);
	face_marked_.resize(target_mesh->num_of_face_list(), false);
	auto face_list_ = *target_mesh->get_faces_list();
	for (int i = 0; i < face_list_.size(); i++)
	{
		if (180 - acos(face_list_[i]->normal() * perpendicular) * 180 / PI < 30)
		{
			float t = 0.0;
			for (int j = 0; j < 3; j++)
				t = std::max(t, face_list_[i]->vec_ptr_vert_[j]->position().z());
			if (t > 1e1)
			{
				face_marked_[i] = true;
			}
		}
	}
	for (int i = 0; i < face_list_.size(); i++)
	{
		if (face_marked_[i] && !face_selected_[i])
		{
			Mesh3D* mesh_ = new Mesh3D;
			sup_face_dfs(face_list_[i], mesh_);
			if (mesh_->num_of_face_list() < 3)
			{
				delete mesh_;
				mesh_ = NULL;
			}
			else
			{
				mesh_->UpdateMeshSup();
				sup_ptr_aera_list_.push_back(mesh_);
			}
		}
	}




}
void Support::support_point_sampling(int counter_)
{
#define UNIFORM (int)2
#define SPARSE (int)1
#define OPTIMAL (int)0

	sample_points_.clear();
	std::pair<float, float> dense(1.0, 1.0);
	Vec3f perpendicular(0.0, 0.0, 1.0);
	for (int i = 0; i < sup_ptr_aera_list_.size(); i++)
	{
		std::vector<HE_face*> face_list_ = *(sup_ptr_aera_list_[i]->get_faces_list());

		for (int j = 0; j < face_list_.size(); j++)
		{
			if (counter_ % 3 == SPARSE)
			{
				dense = get_dense(180 - acos(face_list_[i]->normal() * perpendicular) * 180 / PI);
				dense = make_pair(2.0, 2.0);
			}
			std::vector<Vec3f> verts;
			HE_edge* sta = face_list_[j]->pedge_;
			HE_edge* cur = sta;
			do
			{
				verts.push_back(cur->pvert_->position());
				cur = cur->pnext_;
			} while (cur != sta);

			sort(verts.begin(), verts.end());
			float sta_x_ = ((int)(verts[0].x() / dense.first))*dense.first;
			if (sta_x_ < verts[0].x())
				sta_x_ += dense.first;
			for (; sta_x_ < verts[1].x(); sta_x_ += dense.first)
			{
				Vec3f p1, p2;
				p1 = ((sta_x_ - verts[0].x()) / (verts[1].x() - verts[0].x()))*(verts[1] - verts[0]) + verts[0];
				p2 = ((sta_x_ - verts[0].x()) / (verts[2].x() - verts[0].x()))*(verts[2] - verts[0]) + verts[0];
				float sta_y_ = ((int)(std::min(p1.y(), p2.y()) / dense.second))*dense.second;
				if (sta_y_ < std::min(p1.y(), p2.y()))
					sta_y_ += dense.second;

				for (; sta_y_ < std::max(p1.y(), p2.y()); sta_y_ += dense.second)
				{
					Vec3f p = ((sta_y_ - p1.y()) / (p2.y() - p1.y()))*(p2 - p1) + p1;
					sample_points_[i].push_back(p);
				}
			}
			for (; sta_x_ < verts[2].x(); sta_x_ += dense.first)
			{
				Vec3f p1, p2;
				p1 = ((sta_x_ - verts[1].x()) / (verts[2].x() - verts[1].x()))*(verts[2] - verts[1]) + verts[1];
				p2 = ((sta_x_ - verts[0].x()) / (verts[2].x() - verts[0].x()))*(verts[2] - verts[0]) + verts[0];
				float sta_y_ = ((int)(std::min(p1.y(), p2.y()) / dense.second))*dense.second;
				if (sta_y_ < std::min(p1.y(), p2.y()))
					sta_y_ += dense.second;
				for (; sta_y_ < std::max(p1.y(), p2.y()); sta_y_ += dense.second)
				{
					Vec3f p = ((sta_y_ - p1.y()) / (p2.y() - p1.y()))*(p2 - p1) + p1;
					sample_points_[i].push_back(p);
				}
			}
		}
	}


	if (counter_ % 3 == OPTIMAL)
	{
		for (int i = 0; i < sup_ptr_aera_list_.size(); i++)
		{
			auto loop_list_ = sup_ptr_aera_list_[i]->GetBLoop();
			ClipperLib::Paths polygon;
			std::vector<std::pair<int, int>> points;
			polygon.resize(loop_list_.size());
			for (int j = 0; j < loop_list_.size(); j++)
			{
				for (int k = 0; k < loop_list_[j].size(); k++)
				{
					polygon[j] << ClipperLib::IntPoint((int)(loop_list_[j][k]->pvert_->position().x()*1e3),
						(int)(loop_list_[j][k]->pvert_->position().y()*1e3));
				}
			}
			for (int j = 0; j < sample_points_[i].size(); j++)
			{
				points.push_back(make_pair((int)(sample_points_[i][j].x()*1e3),
					(int)(sample_points_[i][j].y()*1e3)));
			}
			PSO pso_solver_(points, polygon, make_pair((int)1000, (int)1000), 100);
			pso_solver_.settings.goal = sample_points_[i].size()-1;
			auto pos_ = pso_solver_.pso_solve();
			sample_points_[i].clear();
			for (int j = 0; j < pos_.size(); j++)
			{
				Vec3f p(pos_[j].first / 1e3, pos_[j].second / 1e3, 0.0);
				sample_points_[i].push_back(p);
			}
		}
	
	}	
	sam_project_to_mesh(sample_points_);
	
	qDebug() << counter_ << sample_points_[0].size();
}
std::pair<float, float> Support::get_dense(int angle)
{
	std::pair<float, float>d_;
	if (angle < 15)
	{
		d_.first = 2.0;
		if (angle < 5)
		{
			d_.second = 2.0;
		}
		else if (angle < 10)
		{
			d_.second = 2.5;
		}
		else if (angle < 15)
		{
			d_.second = 8.0;
		}
	}
	else
	{
		d_.first = 2.5;

		if (angle < 18)
		{
			d_.second = 10.0;
		}
		else
			d_.second = 15.0;
	}
	return d_;
}

void Support::sam_project_to_mesh(std::map<int, std::vector<Vec3f>> points_)
{
	int num_of_sam = 0;
	for (int i = 0; i < sup_ptr_aera_list_.size(); i++)
	{
		MeshOctree octree;
		octree.BuildOctree(sup_ptr_aera_list_[i]);
		std::vector<Vec3f> re_sample_p;
		for (int j = 0; j < points_[i].size(); j++)
		{
			re_sample_p.push_back(octree.InteractPoint(points_[i][j], Vec3f(0, 0, 1)));
		}
		num_of_sam += re_sample_p.size();
		sample_points_[i] = re_sample_p;
	}
}
