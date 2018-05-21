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
#include <iomanip>
#define PI 3.1415926
Paths test_path;

Support::Support()
{
}
Support::Support(Mesh3D* mesh)
{
	target_mesh = mesh;
	sp_mesh.LoadFromOBJFile(".\\Resources\\models\\sp_sim.obj");
	sp_mesh.scalemesh(0.3);
}

Support::~Support()
{
	delete wholemesh;
}
void Support::sup_face_dfs(HE_face* facet, std::vector<HE_face*>& faces, int angle_id_,std::vector<int>& vec_id)
{
	facet->selected_ = true;
	vec_id[facet->id()] = angle_id_;

	faces.push_back(facet);
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		if (facet != NULL && !facet->selected())
		{
			int angle_ = 180 - acos(facet->normal()*Vec3f(0, 0, 1.0)) * 180 / PI;
			if (angle_ < 30 && angle_ < (angle_id_ + 1) * 5 + 0.5&&angle_ >= angle_id_ * 5)
			{
				sup_face_dfs(facet, faces, angle_id_,vec_id);
			}
		}
		cur = cur->pnext_;
	} while (cur != sta);
}
void Support::sup_face_dfs(HE_face* facet, std::vector<HE_face*>& faces)
{
	facet->selected_ = false;
	faces.push_back(facet);
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		if (facet != NULL && facet->selected())
		{
			sup_face_dfs(facet, faces);
		}
		cur = cur->pnext_;
	} while (cur != sta);
}


void Support::angle_dfs(HE_face* facet, Mesh3D* mesh, int angle_id_)
{
	facet->selected_ = false;
	facet->com_flag = angle_id_;
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
		if (facet!=NULL&&facet->selected())
		{
			int angle_ = 180 - acos(facet->normal()*Vec3f(0, 0, 1.0)) * 180 / PI;
			if (angle_<(angle_id_+1)*5/*+0.5*/&&angle_>=angle_id_*5)
			{
				angle_dfs(facet, mesh, angle_id_);
			}
		}
		cur = cur->pnext_;
	} while (cur != sta);
}
void Support::angle_dfs(HE_face* facet,std::vector<HE_face*>& re_faces, int angle_id_)
{
	facet->selected_ = false;
	//facet->com_flag = angle_id_;
	re_faces.push_back(facet);
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		if (facet != NULL&&facet->selected() && facet->com_flag == angle_id_)
		{
			angle_dfs(facet, re_faces, angle_id_);
		}
		cur = cur->pnext_;
	} while (cur != sta);
}


void Support::sup_mesh_dfs(HE_face* facet, Mesh3D* mesh,int angle_id)
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
	HE_face* curf;
	do
	{
		curf = cur->ppair_->pface_;
		if (curf!= NULL && !curf->selected_&&curf->com_flag==angle_id)
		{
			sup_mesh_dfs(curf, mesh,angle_id);
		}
		cur = cur->pnext_;
	} while (cur != sta);
}

void Support::find_support_area()
{
	Vec3f perpendicular(0.0, 0.0, 1.0);
	
	wholemesh = new Mesh3D;
	std::vector<HE_face*> sele_f;
	{
		auto face_list_ = target_mesh->get_faces_list();
		for (int i = 0; i < face_list_->size(); i++)
		{
			int angle_ = 180 - acos(face_list_->at(i)->normal()*perpendicular) * 180 / PI;
			if (angle_ < 30 && face_list_->at(i)->center().z()>1e-1)
			{
				face_list_->at(i)->selected_ = true;
				sele_f.push_back(face_list_->at(i));

				std::vector<HE_vert*> verts, input;
				face_list_->at(i)->face_verts(verts);
				for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
				{
					input.push_back(wholemesh->InsertVertex((*iterV)->position()));
				}
				wholemesh->InsertFaceSup(input);

			}
		}
	}
	
	wholemesh->UpdateMeshSup();

	for (int i=0;i<sele_f.size();i++)
	{
		if (sele_f[i]->selected_)
		{
			std::vector<HE_face*> con_faces;
			sup_face_dfs(sele_f[i], con_faces);//mark facet as unselected
			if (con_faces.size() == 1)
			{
				con_faces[0]->selected_ = false;
			}
			else
			{
				//for every component
		
				std::map<int, std::vector<HE_face*>> map_id_fs;
				for (int j=0;j<con_faces.size();j++)
				{
					//con_faces[j]->selected_ = true;	
					int region_id = (180 - acos(con_faces[j]->normal()*perpendicular) * 180 / PI)/5;
					con_faces[j]->com_flag = region_id;
					map_id_fs[region_id].push_back(con_faces[j]);
				}
				//mark off different region
				for (int id = 5; id >= 0; id--)
				{
					for (auto iter = map_id_fs[id].begin(); iter != map_id_fs[id].end(); iter++)
					{

						HE_edge* s_ = (*iter)->pedge_;
						HE_edge* c_ = s_;
						int is_same = 0;
						do
						{
							if (c_->ppair_->pface_->com_flag == id)
							{
								is_same++;
							}
							c_ = c_->pnext_;
						} while (c_ != s_);
						if (!is_same)
						{
							(*iter)->com_flag--;
							map_id_fs[id - 1].push_back(*iter);
						}
					}
				}	
				for (auto iter = map_id_fs[-1].begin(); iter != map_id_fs[-1].end(); iter++)
				{
					HE_edge* s_ = (*iter)->pedge_;
					HE_edge* c_ = s_;
					int ids[6] = { 0,0,0,0,0,0 };
					do
					{
						ids[c_->ppair_->pface_->com_flag]++;
						c_ = c_->pnext_;
					} while (c_ != s_);
					int max_id, max_val = -1;
					for (int k = 0; k < 6; k++)
					{
						if (ids[k] > max_val)
						{
							max_id = k;
							max_val = ids[k];
						}
					}
					(*iter)->com_flag = max_id;

				}
				// generate one component
				Mesh3D* mesh_component = new Mesh3D;
				for (int j = 0; j < con_faces.size(); j++)
				{
					// find connected component
					std::vector<HE_vert*> verts, input;
					con_faces[j]->face_verts(verts);
					for (auto iterV = verts.begin(); iterV != verts.end(); iterV++)
					{
						input.push_back(mesh_component->InsertVertex((*iterV)->position()));
					}
					mesh_component->InsertFaceSup(input)->com_flag=con_faces[j]->com_flag;

				}
				mesh_component->UpdateMeshSup();
				component.push_back(mesh_component);

				//generate different region
				auto fs = *(mesh_component->get_faces_list());
				std::map<int, std::vector<Mesh3D*>> id_meshs;
				for (int id = 0; id < 6; id++)
				{
					for (auto iterf = fs.begin(); iterf != fs.end(); iterf++)
					{
						if (!(*iterf)->selected_&&(*iterf)->com_flag==id)
						{
							Mesh3D* m_=new Mesh3D;
							sup_mesh_dfs(*iterf, m_, id);
							id_meshs[id].push_back(m_);
							m_->UpdateMeshSup();
						}
					}
				}
				component_regions_mesh.push_back(id_meshs);
			}
		}
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
		pso_solver.component_regions_mesh = component_regions_mesh;
		pso_solver.component = component;
		pso_solver.settings.size = 100;
		pso_solver.settings.steps = 1000;
		pso_solver.pso_swarm_init();
		sample_points_ = pso_solver.solution.resualt;
		//sample_points_ = pso_solver.pso_solve();
	}
	else if (counter_%3==UNIFORM)
	{
		qDebug() << "uniform";
		Vec2f dense(2.0, 2.0);
		for (int i=0;i<component.size();i++)
		{
			std::vector<Vec3f> last_sampling = SupportLib::compute_local_low_point(component[i]);
			SupportLib::single_area_sampling(component[i], dense,last_sampling);
			sample_points_.insert(sample_points_.end(), last_sampling.begin(), last_sampling.end());
		}
	}
	else if (counter_ % 3 == SPARSE)
	{
		qDebug() << "sparse:";
		for (int i=0;i<1/*component_regions_mesh.size()*/;i++)
		{
			std::vector<Vec3f> last_sampling;
			last_sampling = SupportLib::compute_local_low_point(component[i]);
			Paths last_polygon;
			for (int j=0;j<component_regions_mesh[i].size();j++)
			{
				Vec2f dense = SupportLib::get_dense(j * 5);
				
				for (int k=0;k<component_regions_mesh[i][j].size();k++)
				{
					
					SupportLib::single_area_sampling(component_regions_mesh[i][j][k], dense,last_sampling, last_polygon);
				}
				
			}
			sample_points_.insert(sample_points_.end(), last_sampling.begin(), last_sampling.end());
			
		}
		
	}
	qDebug() << "sample point number is:"<<sample_points_.size();
	sam_project_to_mesh(sample_points_);

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
	fout.precision(4);
	MeshOctree oct_obj;
	oct_obj.BuildOctree(target_mesh);
	//fout<<fixed<<::setprecision(4);
	fout << "ENTITY/OBJ" << endl;


		for (int j=0;j<sample_points_.size();j++)
		{
			
			Vec3f lp = oct_obj.InteractPoint(sample_points_[j], Vec3f(0, 0, -1));
			Vec3f c = lp - sample_points_[j];
			fout << "OBJ=SOLCYL/ORIGIN," << sample_points_[j].x()<< "," << sample_points_[j].y() << "," << sample_points_[j].z() 
				<< ",HEIGHT,$" << endl << (sample_points_[j] - lp).length() << ",DIAMTR," << 1.5 << ",AXIS," << c.x() << "," << c.y() << "," << c.z() << endl;

		}
	fout << "HALT" << endl;
	fout.close();
	
}


Vec2f SupportLib::get_dense(int angle)
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

void SupportLib::single_area_sampling(Mesh3D* mesh, Vec2f dense, std::vector<Vec3f>& last_loop_point,Vec2f center)
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
	//test_path = ori;
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

	
	 PolyTree polytree;
	 Paths subject;
	 sol.Clear();
	 sol.AddPaths(clip, ptClip, true);
	 sol.AddPaths(ori, ptSubject, true);
	 sol.Execute(ctDifference, polytree, pftNonZero, pftNonZero);
	 PolyTreeToPaths(polytree, subject);
	 if (subject.size()== 0)
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
			 IntPoint p((x_*dense.x() + center.x()) * 1000, (y_*dense.y() + center.y()) * 1000);
			 PolyNode* polynode = polytree.GetFirst();
			 while (polynode)
			 {
				 //do stuff with polynode here
				 bool in_polygons = true;
				 if (polynode->IsHole())
				 {
					 in_polygons = in_polygons && !PointInPolygon(p, polynode->Contour);
				 }
				 else
					 in_polygons = in_polygons && PointInPolygon(p, polynode->Contour);

				 for (int ii = 0; ii < polynode->ChildCount(); ii++)
				 {
					 if (polynode->Childs[ii]->IsHole())
					 {
						 in_polygons = in_polygons && !PointInPolygon(p, polynode->Childs[ii]->Contour);
					 }
					 else
						 in_polygons = in_polygons && PointInPolygon(p, polynode->Childs[ii]->Contour);
				 }
				 if (in_polygons)
				 {
					 last_loop_point.push_back(Vec3f(p.X / 1000, p.Y / 1000, 0));
				 }
				 else
				 {
					 IntPoint pr[4] = { p,p,p,p };
					 pr[0].X -= dense.x() / 2 * 1000;
					 pr[1].Y -= dense.y() / 2 * 1000;
					 pr[2].X += dense.x() / 2 * 1000;
					 pr[3].Y += dense.y() / 2 * 1000;
					 for (int r = 0; r < 4; r++)
					 {
						 in_polygons = true;
						 if (polynode->IsHole())
						 {
							 in_polygons = in_polygons && !PointInPolygon(pr[r], polynode->Contour);
						 }
						 else
							 in_polygons = in_polygons && PointInPolygon(pr[r], polynode->Contour);
						 for (int ii = 0; ii < polynode->ChildCount(); ii++)
						 {
							 if (polynode->Childs[ii]->IsHole())
							 {
								 in_polygons = in_polygons && !PointInPolygon(p, polynode->Childs[ii]->Contour);
							 }
							 else
								 in_polygons = in_polygons && PointInPolygon(p, polynode->Childs[ii]->Contour);
						 }
						 if (in_polygons)
						 {
							 last_loop_point.push_back(Vec3f((float)pr[r].X / 1000, (float)pr[r].Y / 1000, 0));
						 }
					 }

				 }


				 polynode = polynode->GetNext();
			 }
			
		 }
	 }
 }
 void SupportLib::single_area_sampling(Mesh3D* mesh, Vec2f dense, std::vector<Vec3f>& last_loop_point,Paths& last_polygons,Vec2f center)
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

	 test_path = ori;
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
	 test_path = clip;
	 sol.Clear();
	 sol.AddPaths(last_polygons, ptClip, true);
	 sol.AddPaths(ori, ptClip, true);
	 sol.Execute(ctUnion, ori, pftNonZero, pftNonZero);

	 PolyTree polytree;
	 Paths subject;
	 sol.Clear();
	 sol.AddPaths(clip, ptClip, true);
	 sol.AddPaths(ori, ptSubject, true);
	 sol.Execute(ctDifference, polytree, pftNonZero, pftNonZero);
	 PolyTreeToPaths(polytree, subject);
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
			 IntPoint p((x_*dense.x() + center.x()) * 1000, (y_*dense.y() + center.y()) * 1000);
			 PolyNode* polynode = polytree.GetFirst();
			 while (polynode)
			 {
				 //do stuff with polynode here
				 bool in_polygons = true;
				 if (polynode->IsHole())
				 {
					 in_polygons = in_polygons && !PointInPolygon(p, polynode->Contour);
				 }
				 else
					 in_polygons = in_polygons && PointInPolygon(p, polynode->Contour);

				 for (int ii = 0; ii < polynode->ChildCount(); ii++)
				 {
					 if (polynode->Childs[ii]->IsHole())
					 {
						 in_polygons = in_polygons && !PointInPolygon(p, polynode->Childs[ii]->Contour);
					 }
					 else
						 in_polygons = in_polygons && PointInPolygon(p, polynode->Childs[ii]->Contour);
				 }
				 if (in_polygons)
				 {
					 last_loop_point.push_back(Vec3f(p.X / 1000, p.Y / 1000, 0));
				 }
				 else
				 {
					 IntPoint pr[4] = { p,p,p,p };
					 pr[0].X -= dense.x() / 2 * 1000;
					 pr[1].Y -= dense.y() / 2 * 1000;
					 pr[2].X += dense.x() / 2 * 1000;
					 pr[3].Y += dense.y() / 2 * 1000;
					 for (int r = 0; r < 4; r++)
					 {
						 in_polygons = true;
						 if (polynode->IsHole())
						 {
							 in_polygons = in_polygons && !PointInPolygon(pr[r], polynode->Contour);
						 }
						 else
							 in_polygons = in_polygons && PointInPolygon(pr[r], polynode->Contour);
						 for (int ii = 0; ii < polynode->ChildCount(); ii++)
						 {
							 if (polynode->Childs[ii]->IsHole())
							 {
								 in_polygons = in_polygons && !PointInPolygon(p, polynode->Childs[ii]->Contour);
							 }
							 else
								 in_polygons = in_polygons && PointInPolygon(p, polynode->Childs[ii]->Contour);
						 }
						 if (in_polygons)
						 {
							 last_loop_point.push_back(Vec3f((float)pr[r].X / 1000, (float)pr[r].Y / 1000, 0));
						 }
					 }

				 }


				 polynode = polynode->GetNext();
			 }

		 }
	 }
 }


 std::vector<Vec3f> SupportLib::compute_local_low_point(Mesh3D* mesh)
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


