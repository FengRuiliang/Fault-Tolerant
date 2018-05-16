#include "Support.h"
#include <algorithm>
#include <PSO/pso.h>
#include "Library/clipper.hpp"
#include "Library/Octree.h"
#include <qdebug.h>
#include <QFileDialog>
#include <fstream>
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
void Support::sup_face_dfs(HE_face* facet, std::vector<HE_face*>* faces, int angle_id_)
{
	face_selected_[facet->id()] = true;
	std::vector<HE_vert*> verts, input;
	facet->face_verts(verts);
	faces->push_back(facet);
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		int angle_ = 180 - acos(facet->normal()*Vec3f(0, 0, 1.0)) * 180 / PI;
		if ((int)angle_ / 5 == angle_id_&&facet != NULL && !face_selected_[facet->id()])
		{
			sup_face_dfs(facet, faces, angle_id_);
		}
		cur = cur->pnext_;
	} while (cur != sta);
}


void Support::angle_dfs(HE_face* facet, Mesh3D* mesh, int angle_id_)
{
	facet->selected_ = true;
	std::vector<HE_vert*> verts, input;
	facet->face_verts(verts);
	for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
	{
		input.push_back(mesh->InsertVertex((*iterV)->position()));
	}
	mesh->InsertFaceSup(input);
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		if (facet!=NULL&&!facet->selected())
		{
			int angle_ = 180 - acos(facet->normal()*Vec3f(0, 0, 1.0)) * 180 / PI;
			if (angle_<(angle_id_+1)*5+0.5&&angle_>=angle_id_*5)
			{
				angle_dfs(facet, mesh, angle_id_);
			}
		}
		cur = cur->pnext_;
	} while (cur != sta);
}

void Support::sup_mesh_dfs(HE_face* facet, Mesh3D* mesh)
{
	facet->selected_ = true;
	std::vector<HE_vert*> verts, input;
	facet->face_verts(verts);
	for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
	{
		input.push_back(mesh->InsertVertex((*iterV)->position()));
	}
	mesh->InsertFaceSup(input);

	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		if (cur->ppair_->pface_ != NULL && !cur->ppair_->pface_->selected_)
		{
			sup_mesh_dfs(cur->ppair_->pface_, mesh);
		}
		cur = cur->pnext_;
	} while (cur != sta);
}

void Support::find_support_area()
{
	Vec3f perpendicular(0.0, 0.0, 1.0);
	auto face_list_ = target_mesh->get_faces_list();
	face_selected_.resize(face_list_->size());
	Mesh3D wholemesh;// find big support area

	for (int i = 0; i < face_list_->size(); i++)
	{
		int angle_ = 180 - acos(face_list_->at(i)->normal()*perpendicular) * 180 / PI;
		if (angle_ < 30)
		{
			std::vector<HE_vert*> verts, input;
			face_list_->at(i)->face_verts(verts);
			for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
			{
				input.push_back(wholemesh.InsertVertex((*iterV)->position()));
			}
			wholemesh.InsertFaceSup(input);
		}
	}
	wholemesh.UpdateMeshSup();
	// find connected component
	face_list_ = wholemesh.get_faces_list();
	for (int i = 0; i < face_list_->size(); i++)
	{
		if (!face_list_->at(i)->selected_)
		{

			Mesh3D* mesh_a_ = new Mesh3D;
			sup_mesh_dfs(face_list_->at(i), mesh_a_);
			if (mesh_a_->num_of_face_list() != 1)
			{
				sup_areas_.push_back(mesh_a_);
			}
			else
			{
				delete mesh_a_;
			}
		}
		face_list_->at(i)->selected_;
	}
	// for every support connected component
	for (int i = 0; i < sup_areas_.size(); i++)
	{
		if (i != 0)
		{
			continue;
		}
		//detect all the  facet support angle  and put them into the corresponding mesh
		std::map<int, Mesh3D> map_one_mesh;
		sup_areas_[i]->UpdateMeshSup();
		face_list_ = sup_areas_[i]->get_faces_list();
		for (int id=0;id<6;id++)
		{
			for (int j = 0; j < face_list_->size(); j++)
			{
				if (face_list_->at(j)->selected())
				{
					continue;
				}
				int angle = 180 - acos(face_list_->at(j)->normal()*perpendicular) * 180 / PI;
				if (angle < (id + 1) * 5 + 0.5&&angle > id * 5-0.5)
				{
					Mesh3D* me = new Mesh3D;
					angle_dfs(face_list_->at(j), me, id);
					if (me->num_of_face_list() == 1)
					{
						face_list_->at(j)->selected_ = false;
						delete me;
					}
					else
					{
						me->UpdateMeshSup();
						sup_ptr_aera_list_[id].push_back(me);
					}

				}

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
	IntPoint dense(2000, 2000),dense_high;
	MeshOctree wholeoctree;
	wholeoctree.BuildOctree(sup_areas_[0]);


	if (counter_%3==OPTIMAL)
	{
		qDebug() << "optimal";
		PSO pso_solver;
		pso_solver.meshs_ = sup_ptr_aera_list_;
		std::vector<Vec3f> box = sup_areas_[0]->getBoundingBox();
		pso_solver.settings.clamp_pos[0].X = ((int)box[1].x() - 2) * 1000;
		pso_solver.settings.clamp_pos[0].Y = ((int)box[1].y() - 2) * 1000;
		pso_solver.settings.clamp_pos[1].X = ((int)box[0].x() - 2) * 1000;
		pso_solver.settings.clamp_pos[1].Y = ((int)box[0].y() - 2) * 1000;
		pso_solver.settings.size = 100;
		pso_solver.settings.steps = 100;
		pso_solver.dense.X = 2000;
		pso_solver.dense.Y = 2000;
		pso_solver.pso_swarm_init();
		auto pos_ = pso_solver.pso_solve();
		sam_project_to_mesh(pos_);
	}
	else
	{
		Paths lastclipper;
		Clipper tsolver;
		int min_x_, min_y_, max_x_, max_y_;
		std::vector<Vec3f> box = sup_areas_[0]->getBoundingBox();
		min_x_ = ((int)box[1].x() - 2) * 1000;
		min_y_ = ((int)box[1].y() - 2) * 1000;
		max_x_ = ((int)box[0].x() + 2) * 1000;
		max_y_ = ((int)box[0].y() + 2) * 1000;
		for (auto iter = sup_ptr_aera_list_.begin(); iter != sup_ptr_aera_list_.end(); iter++)
		{
			//for every component
			if (counter_ % 3 == SPARSE)
			{
				dense = get_dense(iter->first * 5);
				dense_high = get_dense(iter->first * 5 + 5);
			}

			for (int i = 0; i < iter->second.size(); i++)
			{
				tsolver.Clear();
				auto loop_list_ = iter->second[i]->GetBLoop();
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
				Clipper solver;
				Paths sub,rec_union;
				solver.AddPaths(lastclipper, ptClip, true);
				solver.AddPaths(polygon, ptSubject, true);
				solver.Execute(ctDifference, sub, pftNonZero, pftNonZero);

				Path rec(4);
				for (p.X = 0; p.X < max_x_; p.X += dense.X)
				{
					for (p.Y = 0; p.Y < max_y_; p.Y += dense.Y)
					{
						solver.Clear();
						Paths solution;
						rec[0].X = p.X - dense.X / 2;
						rec[0].Y = p.Y - dense.Y / 2;
						rec[1].X = p.X + dense.X / 2;
						rec[1].Y = p.Y - dense.Y / 2;
						rec[2].X = p.X + dense.X / 2;
						rec[2].Y = p.Y + dense.Y / 2;
						rec[3].X = p.X - dense.X / 2;
						rec[3].Y = p.Y + dense.Y / 2;
						solver.AddPaths(sub, ptClip, true);
						solver.AddPath(rec, ptSubject, true);
						solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
						if (solution.size())
						{
							Vec3f  intersectP = wholeoctree.InteractPoint(Vec3f(p.X / 1000, p.Y / 1000, 0), Vec3f(0, 0, 1));
							sample_points_[iter->first].push_back(intersectP);
							rec[0].X = p.X - dense_high.X / 2;
							rec[0].Y = p.Y - dense_high.Y / 2;
							rec[1].X = p.X + dense_high.X / 2;
							rec[1].Y = p.Y - dense_high.Y / 2;
							rec[2].X = p.X + dense_high.X / 2;
							rec[2].Y = p.Y + dense_high.Y / 2;
							rec[3].X = p.X - dense_high.X / 2;
							rec[3].Y = p.Y + dense_high.Y / 2;



							rec_union<<rec;
						}
					}
					for (p.Y = -dense.Y; p.Y >=min_y_; p.Y -= dense.Y)
					{
						solver.Clear();
						Paths solution;
						rec[0].X = p.X - dense.X / 2;
						rec[0].Y = p.Y - dense.Y / 2;
						rec[1].X = p.X + dense.X / 2;
						rec[1].Y = p.Y - dense.Y / 2;
						rec[2].X = p.X + dense.X / 2;
						rec[2].Y = p.Y + dense.Y / 2;
						rec[3].X = p.X - dense.X / 2;
						rec[3].Y = p.Y + dense.Y / 2;
						solver.AddPaths(sub, ptClip, true);
						solver.AddPath(rec, ptSubject, true);
						solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
						if (solution.size())
						{
							Vec3f  intersectP = wholeoctree.InteractPoint(Vec3f(p.X / 1000, p.Y / 1000, 0), Vec3f(0, 0, 1));
							sample_points_[iter->first].push_back(intersectP);
							rec[0].X = p.X - dense_high.X / 2;
							rec[0].Y = p.Y - dense_high.Y / 2;
							rec[1].X = p.X + dense_high.X / 2;
							rec[1].Y = p.Y - dense_high.Y / 2;
							rec[2].X = p.X + dense_high.X / 2;
							rec[2].Y = p.Y + dense_high.Y / 2;
							rec[3].X = p.X - dense_high.X / 2;
							rec[3].Y = p.Y + dense_high.Y / 2;
							rec_union << rec;
						}
					}
				}
				for (p.X = -dense.X; p.X >=min_x_; p.X -= dense.X)
				{
					for (p.Y = 0; p.Y < max_y_; p.Y += dense.Y)
					{
						solver.Clear();
						Paths solution;
						rec[0].X = p.X - dense.X / 2;
						rec[0].Y = p.Y - dense.Y / 2;
						rec[1].X = p.X + dense.X / 2;
						rec[1].Y = p.Y - dense.Y / 2;
						rec[2].X = p.X + dense.X / 2;
						rec[2].Y = p.Y + dense.Y / 2;
						rec[3].X = p.X - dense.X / 2;
						rec[3].Y = p.Y + dense.Y / 2;
						solver.AddPaths(sub, ptClip, true);
						solver.AddPath(rec, ptSubject, true);
						solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
						if (solution.size())
						{
							Vec3f  intersectP = wholeoctree.InteractPoint(Vec3f(p.X / 1000, p.Y / 1000, 0), Vec3f(0, 0, 1));
							sample_points_[iter->first].push_back(intersectP);
							rec[0].X = p.X - dense_high.X / 2;
							rec[0].Y = p.Y - dense_high.Y / 2;
							rec[1].X = p.X + dense_high.X / 2;
							rec[1].Y = p.Y - dense_high.Y / 2;
							rec[2].X = p.X + dense_high.X / 2;
							rec[2].Y = p.Y + dense_high.Y / 2;
							rec[3].X = p.X - dense_high.X / 2;
							rec[3].Y = p.Y + dense_high.Y / 2;
							rec_union << rec;
						}
					}
					for (p.Y = -dense.Y; p.Y >= min_y_; p.Y -= dense.Y)
					{
						solver.Clear();
						Paths solution;
						rec[0].X = p.X - dense.X / 2;
						rec[0].Y = p.Y - dense.Y / 2;
						rec[1].X = p.X + dense.X / 2;
						rec[1].Y = p.Y - dense.Y / 2;
						rec[2].X = p.X + dense.X / 2;
						rec[2].Y = p.Y + dense.Y / 2;
						rec[3].X = p.X - dense.X / 2;
						rec[3].Y = p.Y + dense.Y / 2;
						solver.AddPaths(sub, ptClip, true);
						solver.AddPath(rec, ptSubject, true);
						solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
						if (solution.size())
						{
							Vec3f  intersectP = wholeoctree.InteractPoint(Vec3f(p.X / 1000, p.Y / 1000, 0), Vec3f(0, 0, 1));
							sample_points_[iter->first].push_back(intersectP);
							rec[0].X = p.X - dense_high.X / 2;
							rec[0].Y = p.Y - dense_high.Y / 2;
							rec[1].X = p.X + dense_high.X / 2;
							rec[1].Y = p.Y - dense_high.Y / 2;
							rec[2].X = p.X + dense_high.X / 2;
							rec[2].Y = p.Y + dense_high.Y / 2;
							rec[3].X = p.X - dense_high.X / 2;
							rec[3].Y = p.Y + dense_high.Y / 2;
							rec_union << rec;
						}
					}
				}

				tsolver.Clear();
				tsolver.AddPaths(lastclipper, ptSubject, true);
				tsolver.AddPaths(rec_union, ptClip, true);
				tsolver.Execute(ctUnion, lastclipper, pftNonZero, pftNonZero);
			}
		}

	}
	int num_of_sam = 0;
	for (int i=0;i<sup_ptr_aera_list_.size();i++)
	{

		num_of_sam += sample_points_[i].size();
	}
	qDebug() << "the total number of sample point is "<<num_of_sam;
}



IntPoint Support::get_dense(int angle)
{
	IntPoint d_;
	if (angle < 15)
	{
		d_.X = 2000;

		if (angle < 5)
		{
			d_.Y = 2000;
		}
		else if (angle < 10)
		{
			d_.Y = 2500;
		}
		else if (angle < 15)
		{
			d_.Y = 8000;
		}
	}
	else
	{
		d_.X = 2500;

		if (angle < 18)
		{
			d_.Y = 10000;
		}
		else
			d_.Y = 15000;
	}
	return d_;
}

void Support::sam_project_to_mesh(std::vector<Vec3f> points_)
{
	int num_of_sam = 0;

	MeshOctree octree;
	octree.BuildOctree(sup_areas_[0]);std::vector<Vec3f> re_sample_p;
	for (int i=0;i<points_.size();i++)
	{
		sample_points_[0].push_back(octree.InteractPoint(points_[i], Vec3f(0, 0, 1)));
	}
}
void Support::exportcylinder(const char* fouts)
{
	std::ofstream fout(fouts);
	fout.precision(16);
	MeshOctree oct_obj;
	oct_obj.BuildOctree(target_mesh);
	fout << "ENTITY/OBJ" << endl;

	for (int i=0;i<sample_points_.size();i++)
	{
		for (int j=0;j<sample_points_[i].size();j++)
		{
			
			Vec3f lp = oct_obj.InteractPoint(sample_points_[i][j], Vec3f(0, 0, -1));
			Vec3f c = lp - sample_points_[i][j];
			fout << "OBJ=SOLCYL/ORIGIN," << sample_points_[i][j].x()<< "," << sample_points_[i][j].y() << "," << sample_points_[i][j].z() 
				<< ",HEIGHT,$" << endl << (sample_points_[i][j] - lp).length() << ",DIAMTR," << 1.0 << ",AXIS," << c.x() << "," << c.y() << "," << c.z() << endl;

		}
	}
	fout << "HALT" << endl;
	fout.close();
	
}
