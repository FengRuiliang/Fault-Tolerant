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
std::map<int, Paths> test_path;

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
}
void Support::sup_face_dfs(HE_face* facet, std::vector<HE_face*>& faces, int angle_id_, std::vector<int>& vec_id)
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
				sup_face_dfs(facet, faces, angle_id_, vec_id);
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
		if (facet != NULL&&facet->selected())
		{
			int angle_ = 180 - acos(facet->normal()*Vec3f(0, 0, 1.0)) * 180 / PI;
			if (angle_ < (angle_id_ + 1) * 5/*+0.5*/ && angle_ >= angle_id_ * 5)
			{
				angle_dfs(facet, mesh, angle_id_);
			}
		}
		cur = cur->pnext_;
	} while (cur != sta);
}
void Support::angle_dfs(HE_face* facet, std::vector<HE_face*>& re_faces, int angle_id_)
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


void Support::sup_mesh_dfs(HE_face* facet, Mesh3D* mesh, int angle_id)
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
		if (curf != NULL && !curf->selected_&&curf->com_flag == angle_id)
		{
			sup_mesh_dfs(curf, mesh, angle_id);
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
			if (angle_ < 30 && face_list_->at(i)->center().z()>5e-1)
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
	for (int i = 0; i < sele_f.size(); i++)
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
				for (int j = 0; j < con_faces.size(); j++)
				{
					//con_faces[j]->selected_ = true;	
					int region_id = (180 - acos(con_faces[j]->normal()*perpendicular) * 180 / PI) / 5;
					if (region_id == 5)
					{
						region_id--;
					}
					con_faces[j]->com_flag = region_id;
					map_id_fs[region_id].push_back(con_faces[j]);
				}
				//mark off different region
				for (int id = 4; id >= 0; id--)
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
					int ids[COUNTOFANGLE] = { 0,0,0,0,0 };
					do
					{
						ids[c_->ppair_->pface_->com_flag]++;
						c_ = c_->pnext_;
					} while (c_ != s_);
					int max_id, max_val = -1;
					for (int k = 0; k < 5; k++)
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
					mesh_component->InsertFaceSup(input)->com_flag = con_faces[j]->com_flag;

				}
				mesh_component->UpdateMeshSup();
				component.push_back(mesh_component);

				//generate different region
				auto fs = *(mesh_component->get_faces_list());
				std::map<int, std::vector<Mesh3D*>> id_meshs;
				for (int id = 0; id < COUNTOFANGLE; id++)
				{
					for (auto iterf = fs.begin(); iterf != fs.end(); iterf++)
					{
						if (!(*iterf)->selected_ && (*iterf)->com_flag == id)
						{
							Mesh3D* m_ = new Mesh3D;
							sup_mesh_dfs(*iterf, m_, id);
							id_meshs[id].push_back(m_);
							m_->UpdateMeshSup();
						}
					}
				}
				//id_meshs.clear();
				//id_meshs[0].push_back(mesh_component);
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
	auto local_sup_point = SupportLib::compute_local_low_point(target_mesh);
	std::vector<MeshOctree> octree;
	octree.resize(component.size());
	for (int i = 0; i < component.size(); i++)
	{
		octree[i].BuildOctree(component[i]);
	}

	map<int, std::vector<Vec3f>> component_local_minimal_point;
	for (auto iter = local_sup_point.begin(); iter != local_sup_point.end(); iter++)
	{
		bool is_in_component_ = false;
		for (int i = 0; i < component.size(); i++)
		{
			Vec3f p = octree[i].InteractPoint(*iter - Vec3f(0.0, 0.0, (*iter).z()), Vec3f(0, 0, 1));
			if (p.z() < 999.0f)
			{
				is_in_component_ = true;
				component_local_minimal_point[i].push_back(p);
				Vec3f p = octree[i].InteractPoint(*iter - Vec3f(2.0, 0.0, 0.0), Vec3f(0, 0, 1));
				if (p.z() < 999.0)
					component_local_minimal_point[i].push_back(p);
				p = octree[i].InteractPoint(*iter - Vec3f(0.0, 2.0, 0.0), Vec3f(0, 0, 1));
				if (p.z() < 999.0)
					component_local_minimal_point[i].push_back(p);
				p = octree[i].InteractPoint(*iter + Vec3f(2.0, 0.0, 0.0), Vec3f(0, 0, 1));

				if (p.z() < 999.0)
					component_local_minimal_point[i].push_back(p);
				p = octree[i].InteractPoint(*iter + Vec3f(0.0, 2.0, 0.0), Vec3f(0, 0, 1));
				if (p.z() < 999.0)
					component_local_minimal_point[i].push_back(p);
			}
		}
		if (is_in_component_ == false)
		{
			sample_points_[-1].insert(*iter);
		}
	}

	std::map<int, std::set<Vec3f>> d_sample_points;


	component_local_minimal_point.clear();
	if (counter_ % 3 == OPTIMAL)
	{
		qDebug() << "optimal";
		test_path.clear();
		PSO pso_solver;
		pso_solver.component_local_sup_point = component_local_minimal_point;
		pso_solver.component_regions_mesh = component_regions_mesh;
		pso_solver.component = component;
		pso_solver.settings.size = 100;
		pso_solver.settings.steps = 2000;
		pso_solver.pso_swarm_init();
		//sample_points_ = pso_solver.solution.resualt;
		d_sample_points = pso_solver.pso_solve();
	}
	else if (counter_ % 3 == UNIFORM)
	{

		qDebug() << "uniform";
		test_path.clear();
		Vec2f dense(2.0, 2.0);

		for (int i = 0; i < component.size(); i++)
		{
			std::map<int, std::set<Vec3f>> temp_int_set;
			for (int j = 0; j < component_local_minimal_point[i].size(); j++)
			{
				temp_int_set[0].insert(component_local_minimal_point[i][j]);
			}
			SupportLib::single_area_sampling(component[i], dense, temp_int_set, 0);
			for (int i = 0; i < COUNTOFANGLE; i++)
			{
				for (auto iter = temp_int_set[i].begin(); iter != temp_int_set[i].end(); iter++)
				{
					d_sample_points[i].insert(*iter);
				}

			}
		}
	}
	else if (counter_ % 3 == SPARSE)
	{
		qDebug() << "sparse:";
		test_path.clear();
		
		for (int i = 0; i < component_regions_mesh.size(); i++)
		{
		
			std::map<int, std::set<Vec3f>> temp_int_set;
			for (int j = 0; j < component_local_minimal_point[i].size(); j++)
			{
				temp_int_set[0].insert(component_local_minimal_point[i][j]);
			}
			for (int j = 0; j < component_regions_mesh[i].size(); j++)
			{

				Vec2f dense = SupportLib::get_dense(j);
				for (int k = 0; k < component_regions_mesh[i][j].size(); k++)
				{
					SupportLib::single_area_sampling(component_regions_mesh[i][j][k], dense, temp_int_set, j);
				}
			}
			for (int i = 0; i < COUNTOFANGLE; i++)
			{
				for (auto iter = temp_int_set[i].begin(); iter != temp_int_set[i].end(); iter++)
				{
					d_sample_points[i].insert(*iter);
				}
			}
		}
	}
	Support::sam_project_to_mesh(d_sample_points);
	for (int i = 0; i < COUNTOFANGLE; i++)
	{
		Clipper sov;Vec2f dense = SupportLib::get_dense(i);
		for (int j=0;j<=i;j++)
		{
			for (auto iter = d_sample_points[j].begin(); iter != d_sample_points[j].end(); iter++)
			{
				Path rectangle;
				rectangle << IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000);
				sov.AddPath(rectangle, ptClip, true);
			}
		
		}
		sov.Execute(ctUnion, test_path[i], pftNonZero, pftNonZero);
	}
}

void Support::sam_project_to_mesh(std::map<int, std::set<Vec3f>> points_)
{

	MeshOctree octree;
	octree.BuildOctree(wholemesh);
	for (int i = 0; i < COUNTOFANGLE; i++)
	{
		sample_points_[i].clear();
		for (auto iter = points_[i].begin(); iter != points_[i].end(); iter++)
		{
			Vec3f p = octree.InteractPoint(*iter, Vec3f(0, 0, 1));
			if (p.z() < 999.0)
			{
				sample_points_[i].insert(p);
			}
		}
	}

}
void Support::exportcylinder(const char* fouts)
{
	std::ofstream fout(fouts);
	fout.precision(4);
	MeshOctree oct_obj;
	oct_obj.BuildOctree(target_mesh);
	//fout<<fixed<<::setprecision(4);
	fout << "ENTITY/CONE" << endl;
	for (int i = -1; i < COUNTOFANGLE; i++)
	{
		for (auto iter = sample_points_[i].begin(); iter != sample_points_[i].end(); iter++)
		{
			Vec3f ins_p = oct_obj.InteractPoint(*iter, Vec3f(0, 0, -1));
			Vec3f axis = ins_p - (*iter);
			//fout << "OBJ=SOLCYL/ORIGIN," << (*iter).x() << "," << (*iter).y() << "," << (*iter).z() << ",HEIGHT,$" << endl 
			//	<< axis.length() << ",DIAMTR," << 1.0 << ",AXIS," << axis.x() << "," <<axis.y() << "," << axis.z() << endl;
			fout << "CONE=SOLCON/ORIGIN," << (*iter).x() << "," << (*iter).y() << "," << (*iter).z() << ",HEIGHT,$" << endl;
			fout << axis.length() << ",DIAMTR," << 0.8 << "," << 1.2 << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << endl;
		}
	}
	fout << "HALT" << endl;
	fout.close();

}


Vec2f SupportLib::get_dense(int angle)
{
	
	switch (angle)
	{
	case 0:
		return Vec2f(2.0, 2.0);
	case 1:
		return Vec2f(2.0, 2.5);
	case 2:
		return Vec2f(2.0, 8.0);
	case 3:
		return Vec2f(2.5, 10.0);
	case 4:
		return Vec2f(2.5, 15.0);
	default:
		return Vec2f(2.0, 2.0);
	}

}

float SupportLib::single_area_sampling(Mesh3D* mesh, Vec2f dense, std::map<int, std::set<Vec3f>>& last_loop_point, int angle_id, Vec2f center)
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
	for (int i = 0; i < COUNTOFANGLE; i++)
	{
		for (auto iter = last_loop_point[i].begin(); iter != last_loop_point[i].end(); iter++)
		{
			Path rectangle;
			rectangle << IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
				<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
				<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000)
				<< IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000);
			clip << rectangle;
		}

	}

	Clipper sol;
	//clip.clear();//for display
	sol.AddPaths(clip, ptClip, true);
	sol.Execute(ctUnion, clip, pftNonZero, pftNonZero);


	PolyTree polytree;
	Paths subject;
	sol.Clear();
	sol.AddPaths(clip, ptClip, true);
	sol.AddPaths(ori, ptSubject, true);
	sol.Execute(ctDifference, polytree, pftNonZero, pftNonZero);
	PolyTreeToPaths(polytree, subject);
	if (subject.size() == 0)
	{
		clip.clear();
		for (int i = 0; i < COUNTOFANGLE; i++)
		{
			for (auto iter = last_loop_point[i].begin(); iter != last_loop_point[i].end(); iter++)
			{
				Path rectangle;
				rectangle << IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000);
				clip << rectangle;
			}

		}
		return 0.0;
	}
	std::vector<Vec3f> box = mesh->getBoundingBox();
	int min_x_ = (int)((box[1].x() + 1000 * dense.x()) / dense.x()) - 1000;
	int min_y_ = (int)((box[1].y() + 1000 * dense.y()) / dense.y()) - 1000;
	int max_x_ = (int)((box[0].x() + 1000 * dense.x()) / dense.x()) - 1000;
	int max_y_ = (int)((box[0].y() + 1000 * dense.y()) / dense.y()) - 1000;
	float area_rate = 0;
	for (int x_ = min_x_; x_ <= max_x_ + 1; x_++)
	{
		for (int y_ = min_y_; y_ <= max_y_ + 1; y_++)
		{
			IntPoint p((x_*dense.x() + center.x()) * 1000, (y_*dense.y() + center.y()) * 1000);
			//////////////////////////////////////////////////////////////////////////
			int count = 0;
			for (int i = 0; i < subject.size(); i++)
			{

				if (PointInPolygon(p, subject[i]) == 1)//means that the ray shoot odd number
				{
					count++;
				}
			}
			if (count % 2 == 1)
			{
				last_loop_point[angle_id].insert(Vec3f((float)p.X / 1000, (float)p.Y / 1000, 0));

			}
			else
			{
				Path rectangle;
				Paths insec;
				rectangle << IntPoint(p.X - dense.x() / 2 * 1000, p.Y - dense.y() / 2 * 1000)
					<< IntPoint(p.X + dense.x() / 2 * 1000, p.Y - dense.y() / 2 * 1000)
					<< IntPoint(p.X + dense.x() / 2 * 1000, p.Y + dense.y() / 2 * 1000)
					<< IntPoint(p.X - dense.x() / 2 * 1000, p.Y + dense.y() / 2 * 1000);
				Clipper com_insec_;
				com_insec_.AddPath(rectangle, ptClip, true);
				com_insec_.AddPaths(subject, ptSubject, true);
				com_insec_.Execute(ctIntersection, insec, pftNonZero, pftNonZero);
				float area = 0.0;
				for (auto iter = insec.begin(); iter != insec.end(); iter++)
				{
					area += Area(*iter);
				}
				area=area/(1e6*dense.x()*dense.y());
				area_rate = area_rate > area ? area_rate : area;
			}		
		}
	}
	return area_rate;
}


std::set<Vec3f> SupportLib::compute_local_low_point(Mesh3D* mesh)
{
	std::set<Vec3f> local_minimal_point;
	auto vList = *(mesh->get_vertex_list());
	mesh->UpdateMesh();
	for (int j = 0; j < vList.size(); j++)
	{
		if (vList[j]->position().z() < 5e-1)
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
		if (k == va.size() && vList[j]->pedge_->pface_->normal().z() < 0)
		{

			local_minimal_point.insert(vList[j]->position());
		}
	}
	return local_minimal_point;
}


