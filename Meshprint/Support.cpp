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
	IntPoint dense(1000, 1000);
	for (int i = 0; i < sup_ptr_aera_list_.size(); i++)
	{
		auto loop_list_ = sup_ptr_aera_list_[i]->GetBLoop();
		using namespace ClipperLib;
		ClipperLib::Paths polygon;
		polygon.resize(loop_list_.size());
		IntPoint p;
		for (int j = 0; j < loop_list_.size(); j++)
		{
			for (int k = 0; k < loop_list_[j].size(); k++)
			{
				p.X = (int)(loop_list_[j][k]->pvert_->position().x()*1e3);
				p.Y = (int)(loop_list_[j][k]->pvert_->position().y()*1e3);
				polygon[j] << p;
			}
		}

		std::vector<Vec3f> box = sup_ptr_aera_list_[i]->getBoundingBox();
		int min_x_, min_y_, max_x_, max_y_;
		min_x_ = ((int)box[1].x() - 2) * 1000;
		min_y_ = ((int)box[1].y() - 2) * 1000;
		max_x_ = ((int)box[0].x() + 2) * 1000;
		max_y_ = ((int)box[0].x() + 2) * 1000;
		std::vector<IntPoint> points;
		Path rec(4);
		Clipper solver;

		Paths solution;
		for (p.X = min_x_; p.X < max_x_; p.X += dense.X)
		{
			for (p.Y = min_y_; p.Y < max_y_; p.Y += dense.Y)
			{

				if (PointInPolygon(p, polygon[0]) == 0)
				{
					rec[0].X = p.X - dense.X / 2;
					rec[0].Y = p.Y - dense.Y / 2;
					rec[1].X = p.X + dense.X / 2;
					rec[1].Y = p.Y - dense.Y / 2;
					rec[2].X = p.X + dense.X / 2;
					rec[2].Y = p.Y + dense.Y / 2;
					rec[3].X = p.X - dense.X / 2;
					rec[3].Y = p.Y + dense.Y / 2;
					solver.Clear();
					solver.AddPaths(polygon, ptClip, true);
					solver.AddPath(rec, ptSubject, true);
					solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
					if (solution.size())
					{
						points << p;
						Vec3f v(p.X / 1e3, p.Y / 1e3, 0.0);
						sample_points_[i].push_back(v);
					}
				}
				else
				{
					points << p;
					Vec3f v(p.X / 1e3, p.Y / 1e3, 0.0);
					sample_points_[i].push_back(v);
				}


			}

		}
		if (counter_ % 2 == OPTIMAL)
		{
			PSO pso_solver_;
			pso_solver_.remain_paths_ = polygon;
			pso_solver_.settings.clamp_pos[0] = IntPoint(max_x_, max_y_);
			pso_solver_.settings.clamp_pos[1] = IntPoint(min_x_, min_y_);
			pso_solver_.settings.size = 100;
			pso_solver_.settings.steps = 10;
			pso_solver_.dense_ = dense;
			pso_solver_.pso_swarm_init();	
			auto pos_ = pso_solver_.pso_solve();
		

			sample_points_[i].clear();
			for (int j = 0; j < pos_.size(); j++)
			{
				Vec3f p(pos_[j].first / 1e3, pos_[j].second / 1e3, 10);
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
		octree.BuildOctree(target_mesh);
		std::vector<Vec3f> re_sample_p;
		for (int j = 0; j < points_[i].size(); j++)
		{
			re_sample_p.push_back(octree.InteractPoint(points_[i][j], Vec3f(0, 0, 1)));
		}
		num_of_sam += re_sample_p.size();
		sample_points_[i] = re_sample_p;
	}
}
