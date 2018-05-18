#include "Support.h"
#include <algorithm>
#include <PSO/pso.h>
#include "Library/clipper.hpp"
#include "Library/Octree.h"
#include <qdebug.h>
#include <QFileDialog>
#include <fstream>
#include "Library/IntervalTree.h"
#include "Library/space2dKDTree.h"
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
	delete wholemesh;
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
	
	wholemesh = new Mesh3D;
	for (int i = 0; i < face_list_->size(); i++)
	{
		int angle_ = 180 - acos(face_list_->at(i)->normal()*perpendicular) * 180 / PI;
		if (angle_ < 30&&face_list_->at(i)->center().z()>1e-1)
		{
			face_list_->at(i)->selected_ = true;
			std::vector<HE_vert*> verts, input;
			face_list_->at(i)->face_verts(verts);
			for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
			{
				input.push_back(wholemesh->InsertVertex((*iterV)->position()));
			}
			wholemesh->InsertFaceSup(input);
		}
	}
	wholemesh->UpdateMeshSup();
	
	// find connected component
	face_list_ = wholemesh->get_faces_list();
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
		//detect all the  facet support angle  and put them into the corresponding mesh
		std::map<int, std::vector<Mesh3D*>> regions_;
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
						regions_[id].push_back(me);
					}

				}

			}

		}
		component_regions_.push_back(regions_);
	}
}
void Support::support_point_sampling(int counter_)
{
#define UNIFORM (int)1
#define SPARSE (int)2
#define OPTIMAL (int)0

	sample_points_.clear();
	
	// add local minimal support point
	
	if (counter_%3==OPTIMAL)
	{
		qDebug() << "optimal";
		PSO pso_solver;
		//pso_solver.meshs_ = regions_;
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
	else if (counter_%3==UNIFORM)
	{
		Vec2f dense(2.0, 2.0);
		for (int i=0;i<sup_areas_.size();i++)
		{
			std::vector<Vec3f> last_loop_point = compute_local_low_point(sup_areas_[i]);
			single_area_sampling(sup_areas_[i], dense,last_loop_point);
			sample_points_.insert(sample_points_.end(), last_loop_point.begin(), last_loop_point.end());
		}
	}
	else if (counter_ % 3 == SPARSE)
	{
		for (int i=0;i<component_regions_.size();i++)
		{
			std::vector<Vec3f> single_point;
			for (int j=0;j<component_regions_[i].size();j++)
			{
				Vec2f dense = get_dense(j * 5);
				
				for (int k=0;k<component_regions_[i][j].size();k++)
				{
					
					single_area_sampling(component_regions_[i][j][k], dense,single_point);
				}
			}
		}

	}
	
	sam_project_to_mesh(sample_points_);
}



Vec2f Support::get_dense(int angle)
{
	Vec2f d_;
	if (angle < 15)
	{
		d_.x() = 2.0;

		if (angle < 5)
		{
			d_.y() = 2.0;
		}
		else if (angle < 10)
		{
			d_.y() = 2.5;
		}
		else if (angle < 15)
		{
			d_.y() = 8.0;
		}
	}
	else
	{
		d_.x() = 2.5;

		if (angle < 18)
		{
			d_.y() = 10.0;
		}
		else
			d_.y() = 15.0;
	}
	return d_;
}

void Support::sam_project_to_mesh(std::vector<Vec3f> points_)
{
	int num_of_sam = 0;
	sample_points_.clear();
	MeshOctree octree;
	octree.BuildOctree(wholemesh);
	for (int i=0;i<points_.size();i++)
	{
		sample_points_.push_back(octree.InteractPoint(points_[i], Vec3f(0, 0, 1)));
	}
}
void Support::exportcylinder(const char* fouts)
{
	std::ofstream fout(fouts);
	fout.precision(16);
	MeshOctree oct_obj;
	oct_obj.BuildOctree(target_mesh);
	fout << "ENTITY/OBJ" << endl;


		for (int j=0;j<sample_points_.size();j++)
		{
			
			Vec3f lp = oct_obj.InteractPoint(sample_points_[j], Vec3f(0, 0, -1));
			Vec3f c = lp - sample_points_[j];
			fout << "OBJ=SOLCYL/ORIGIN," << sample_points_[j].x()<< "," << sample_points_[j].y() << "," << sample_points_[j].z() 
				<< ",HEIGHT,$" << endl << (sample_points_[j] - lp).length() << ",DIAMTR," << 1.0 << ",AXIS," << c.x() << "," << c.y() << "," << c.z() << endl;

		}
	fout << "HALT" << endl;
	fout.close();
	
}



void Support::single_area_sampling(Mesh3D* mesh, Vec2f dense, std::vector<Vec3f>& last_loop_point)
{
	Paths ori;
	auto belist = mesh->GetBLoop();
	for (int i = 0; i < belist.size(); i++)
	{
		Path loop;
		for (int j = 0; j < belist[i].size(); j++)
		{
			loop << IntPoint(belist[i][j]->pvert_->position().x() * 1000, belist[i][j]->pvert_->position().y() * 1000);
		}
		ori << loop;
	}
	
	 Paths clip;
	 for (int i = 0; i < last_loop_point.size(); i++)
	 {
		 Path rectangle;
		 rectangle << IntPoint((last_loop_point[i].x() - dense.x()) * 1000, (last_loop_point[i].y() - dense.y()) * 1000)
			 << IntPoint((last_loop_point[i].x() + dense.x()) * 1000, (last_loop_point[i].y() - dense.y()) * 1000)
			 << IntPoint((last_loop_point[i].x() + dense.x()) * 1000, (last_loop_point[i].y() + dense.y()) * 1000)
			 << IntPoint((last_loop_point[i].x() - dense.x()) * 1000, (last_loop_point[i].y() + dense.y()) * 1000);
		 clip << rectangle;
	 } 
	 Clipper sol;
	 sol.AddPaths(clip, ptClip, true);
	 sol.Execute(ctUnion, clip, pftNonZero, pftNonZero);

	
	 Paths subject;
	 sol.Clear();
	 sol.AddPaths(clip, ptClip, true);
	 sol.AddPaths(ori, ptSubject, true); 
	 sol.Execute(ctDifference, subject, pftNonZero, pftNonZero);
	 if (subject.size() == 0)
	 {
		 return;
	 }
	 std::vector<Vec3f> box = mesh->getBoundingBox();
	 int min_x_ = (int)((box[1].x() + 1000 * dense.x()) / dense.x()) - 1000;
	 int min_y_ = (int)((box[1].y() + 1000 * dense.y()) / dense.y()) - 1000;
	 int max_x_ = (int)((box[0].x() + 1000 * dense.x()) / dense.x()) - 1000;
	 int max_y_ = (int)((box[0].y() + 1000 * dense.y()) / dense.y()) - 1000;
	 for (int x_ = min_x_; x_ <= max_x_ + 1; x_++)
	 {
		 for (int y_ = min_y_; y_ <= max_y_ + 1; y_++)
		 {
			 IntPoint p(x_*dense.x() * 1000, y_*dense.y() * 1000);
			 bool in_polygons(true);
			 for (int i = 0; i < subject.size(); i++)
			 {
				 if (Orientation(subject[i]))
				 {
					 in_polygons = in_polygons&&PointInPolygon(p, subject[i]);
				 }
				 else
				 {
					 in_polygons = in_polygons&&!PointInPolygon(p, subject[i]);
				 }

			 }
			 if (in_polygons)
			 {
				 last_loop_point.push_back(Vec3f(p.X / 1000, p.Y / 1000, 0));
			 }
			 else
			 {
				
				 IntPoint pr[4] = {p,p,p,p};
				 pr[0].X -=dense.x() / 2 * 1000; 
				 pr[1].Y -=dense.y() / 2 * 1000;
				 pr[2].X +=dense.x() / 2 * 1000;
				 pr[3].Y += dense.y() / 2 * 1000;
				 for (int r = 0; r < 4; r++)
				 {
					 in_polygons = true;
					 for (int i = 0; i < subject.size(); i++)
					 {
						 if (Orientation(subject[i]))
						 {
							 in_polygons = in_polygons&&PointInPolygon(pr[r], subject[i]);
						 }
						 else
						 {
							 in_polygons = in_polygons && !PointInPolygon(pr[r], subject[i]);
						 }
					 }
					 if (in_polygons)
					 {
						 last_loop_point.push_back(Vec3f((float)pr[r].X / 1000, (float)pr[r].Y / 1000, 0));
					 }
				 }
			 }
		 }
	 }
 }


 std::vector<Vec3f> Support::compute_local_low_point(Mesh3D* mesh)
 {
	 std::vector<Vec3f> local_minimal_point;
	 auto vList = *(mesh->get_vertex_list());
	 mesh->UpdateMesh();
	 for (int j = 0; j < vList.size(); j++)
	 {
		 if (vList[j]->boundary_flag_ == 0)
		 {
			 continue;
		 }
		 std::vector<size_t> va = vList[j]->neighborIdx;
		 int k = 0;
		 for (k = 0; k < va.size(); k++)
		 {
			 if (vList[va[k]]->position().z() < vList[j]->position().z())
			 {

				 break;
			 }
		 }
		 if (k == va.size())
		 {
			 local_minimal_point.push_back(vList[j]->position());
		 }
	 }
	 return local_minimal_point;
 }