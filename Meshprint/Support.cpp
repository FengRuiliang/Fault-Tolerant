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
std::vector<Mesh3D*> component;
std::vector<Paths> covers;
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
	std::map<int, std::set<Vec3f>> d_sample_points;
	map<int, std::vector<Vec3f>> component_local_minimal_point;

	if (counter_ % 3 == OPTIMAL)
	{
		test_path.clear();
		qDebug() << "optimal";
		compute_intersection_line();
		for (int i=0;i<covers.size();i++)
		{
			test_path[i]=covers[i];
		}
		GARandomSeed((unsigned)time(NULL));
		// Declare variables for the GA parameters and set them to some default values.
		int popsize = 500;
		int ngen = 20000;
		float pmut = 0.001;
		float pcross = 0.9;
		float Objective(GAGenome &);
		GA1DBinaryStringGenome genome((unsigned int)covers.size(), Objective);
		// Now that we have the genome, we create the genetic algorithm and set
		// its parameters - number of generations, mutation probability, and crossover
		// probability.  And finally we tell it to evolve itself.
		GASimpleGA ga(genome);
		ga.populationSize(popsize);
		ga.nGenerations(ngen);
		ga.pMutation(pmut);
		ga.pCrossover(pcross);
		ga.evolve();
		cout << "The GA found:\n" << ga.statistics().bestIndividual() << "\n";
		GA1DBinaryStringGenome & genomeout = (GA1DBinaryStringGenome &)ga.statistics().bestIndividual();
		for (int i = 0; i < genomeout.length(); i++) {
			if (genomeout.gene(i))
			{
				for (size_t j = 0; j < covers[i].size(); j++)
				{
					Vec3f p((covers[i][j][0].X + covers[i][j][1].X) / 2000, (covers[i][j][0].Y + covers[i][j][3].Y) / 2000, 0.0);
					d_sample_points[0].insert(p);

				}
			}
		}
	}
	else if (counter_ % 3 == UNIFORM)
	{

		qDebug() << "uniform";
	
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




	Clipper show_solver;
	test_path.clear();
	for (int i = 0; i < COUNTOFANGLE; i++)
	{
		show_solver.Clear();
		Paths ith_clipper;
		Vec2f dense = SupportLib::get_dense(i);
		for (int j = 0; j <= i; j++)
		{
			
			for (auto iter = d_sample_points[j].begin(); iter != d_sample_points[j].end(); iter++)
			{
				Path rec;
				float X = (*iter).x(), Y = (*iter).y();
				rec << IntPoint((X - dense.x() / 2) * 1000, (Y - dense.y() / 2) * 1000)
					<< IntPoint((X + dense.x() / 2) * 1000, (Y - dense.y() / 2) * 1000)
					<< IntPoint((X + dense.x() / 2) * 1000, (Y + dense.y() / 2) * 1000)
					<< IntPoint((X - dense.x() / 2) * 1000, (Y + dense.y() / 2) * 1000);
				ith_clipper << rec;
			}
			
			
		}
		show_solver.AddPaths(ith_clipper, ptClip, true);
		show_solver.Execute(ctUnion, test_path[i], pftNonZero, pftNonZero);

	}



	Support::sam_project_to_mesh(d_sample_points);
	num = 0;
	for (int i = -1; i < COUNTOFANGLE; i++)
	{
		num += sample_points_[i].size();
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
	Paths subject;
	auto belist = mesh->GetBLoop();
	for (int i = 0; i < belist.size(); i++)
	{
		Path loop;
		for (int j = 0; j < belist[i].size(); j++)
		{
			loop << IntPoint(belist[i][j]->pvert_->position().x() * 1000, belist[i][j]->pvert_->position().y() * 1000);
		}
		subject << loop;
	}
	

	float max_different = 0;
	Paths clip;
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
				Path rec;
				rec << IntPoint(p.X - dense.x() / 2 * 1000, p.Y - dense.y() / 2 * 1000)
					<< IntPoint(p.X + dense.x() / 2 * 1000, p.Y - dense.y() / 2 * 1000)
					<< IntPoint(p.X + dense.x() / 2 * 1000, p.Y + dense.y() / 2 * 1000)
					<< IntPoint(p.X - dense.x() / 2 * 1000, p.Y + dense.y() / 2 * 1000);
				clip << rec;
			}
		} 
	}
	Clipper differ;
	differ.AddPaths(clip, ptClip, true);
	differ.AddPaths(subject, ptSubject, true);
	differ.Execute(ctDifference, clip, pftNonZero, pftNonZero);
	for (auto iter = clip.begin(); iter != clip.end(); iter++)
	{
		float a= Area(*iter)/1e6;
		max_different = max_different > a ? max_different : a;
	}
	return max_different;
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

int SupportLib::sam_project_to_mesh(std::map<int, std::set<Vec3f>> in_, std::map<int, std::set<Vec3f>>&out_,Mesh3D* mesh,float& height_)
{
	int num = 0;
	MeshOctree octree;
	octree.BuildOctree(mesh);
	for (int i = 0; i < COUNTOFANGLE; i++)
	{
		for (auto iter = in_[i].begin(); iter != in_[i].end(); iter++)
		{
			Vec3f p = octree.InteractPoint(*iter, Vec3f(0, 0, 1));
			if (p.z() < 999.0)
			{
				num++;
				height_ += p.z();
				out_[i].insert(p);
			}
		}
	}
	return num;
}

float Objective(GAGenome &g)
{
	float score = 0.0;
	GA1DBinaryStringGenome & genome = (GA1DBinaryStringGenome &)g;
	Paths subject, clip;
	for (int i = 0; i < genome.length(); i++) {
		if (genome.gene(i))
		{
			score += covers[i].size();
			clip.insert(clip.end(), covers[i].begin(), covers[i].end());
		}
	}
	for (int i = 0; i < component.size(); i++)
	{

		auto belist = component[i]->GetBLoop();
		for (int i = 0; i < belist.size(); i++)
		{
			Path loop;
			for (int j = 0; j < belist[i].size(); j++)
			{
				loop << IntPoint(belist[i][j]->pvert_->position().x() * 1000, belist[i][j]->pvert_->position().y() * 1000);
			}
			subject << loop;
		}
	}
	Clipper solver;
	solver.AddPaths(clip, ptClip, true);
	solver.Execute(ctUnion, clip, pftNonZero, pftNonZero);
	solver.Clear();
	solver.AddPaths(clip, ptClip, true);
	solver.AddPaths(subject, ptSubject, true);
	solver.Execute(ctDifference, subject, pftNonZero, pftNonZero);
	float max_area = 0;
	for (int i = 0; i < subject.size(); i++)
	{
		max_area = max_area > Area(subject[i]) ? max_area : Area(subject[i]);
	}
	if (max_area > 4e6)
	{
		return 1e6;
	}
	else
		return score;
}


void Support::compute_intersection_line()
{
	covers.clear();
	for (int i = 0; i < component_regions_mesh.size(); i++)
	{
		for (int j = 0; j < component_regions_mesh[i].size(); j++)
		{
			float gap = SupportLib::get_dense(j).x();
			float ver = SupportLib::get_dense(j).y();
			for (int k = 0; k < component_regions_mesh[i][j].size(); k++)
			{
				auto blist = component_regions_mesh[i][j][k]->GetBLoop();
				std::map<int, std::vector<Vec2f>>ins_container;
				std::set<int> container_first;
				float line_width_ = 1.0;
				for (int ii = 0; ii < blist.size(); ii++)
				{
					if (blist[ii].size() < 7)
					{
						continue;
					}
					for (int jj = 0; jj < blist[ii].size(); jj++)
					{
						Vec3f a, b, v; Vec2f p_vec2;
						a = blist[ii][jj]->start_->position();
						b = blist[ii][jj]->pvert_->position();
						v = b - a;
						float y_min_ = a.y() <= b.y() ? a.y() : b.y();
						float y_max_ = a.y() >= b.y() ? a.y() : b.y();
						int min_hei_id_ = (int)((y_min_ + line_width_ * 1000) / line_width_) - 1000;
						int max_hei_id_ = (int)((y_max_ + line_width_ * 1000) / line_width_) - 1000;
						for (int y_id_ = min_hei_id_ + 1; y_id_ <= max_hei_id_; y_id_++)//取上不取下
						{
							if (v.y() == 0) { continue; }
							else
							{
								p_vec2.x() = a.x() + (y_id_*line_width_ - a.y()) / v.y()*v.x();
								p_vec2.y() = a.y() + (y_id_*line_width_ - a.y()) / v.y()*v.y();
								container_first.insert(y_id_);
								ins_container[y_id_].push_back(p_vec2);
							}
						}
					}
				}
				for (auto iter=container_first.begin();iter!=container_first.end();iter++)
				{
					std::sort(ins_container[*iter].begin(), ins_container[*iter].end());
					for (int jj = 0; jj+ 1 < ins_container[*iter].size(); jj += 2)
					{
						vector<Vec2f> sup_sample;
						sup_sample.push_back(ins_container[*iter][jj]);
						while ((ins_container[*iter][jj + 1].x() - sup_sample.back().x()) > gap)
						{
							sup_sample.push_back(sup_sample.back() + Vec2f(gap, 0));

						}
						if ((ins_container[*iter][jj + 1].x() - sup_sample.back().x()) > 0.6*gap)
						{
							sup_sample.push_back(ins_container[*iter][jj + 1]);
						}
						else
						{
							float dis=ins_container[*iter][jj + 1].x() - sup_sample.back().x();
							for (int kk = 0; kk < sup_sample.size(); kk++)
							{
								sup_sample[kk] += Vec2f(0,0.5*dis);
							}
						}
						Paths cover;
						for (int kk = 0; kk < sup_sample.size(); kk++)
						{
							Path rec;
							IntPoint p(sup_sample[kk].x() * 1000, sup_sample[kk].y() * 1000);
							rec << IntPoint(p.X - gap * 1000 / 2, p.Y - ver * 1000 / 2)
								<< IntPoint(p.X + gap * 1000 / 2, p.Y - ver * 1000 / 2)
								<< IntPoint(p.X + gap * 1000 / 2, p.Y + ver * 1000 / 2)
								<< IntPoint(p.X - gap * 1000 / 2, p.Y + ver * 1000 / 2);
							cover << rec;
						}
						covers.push_back(cover);//每一条线段都看作是粒子的一个维度。
					}
				}
			}
		}
	}
}

void Support::compute_local_minimal_point()
{
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



	component_local_minimal_point.clear();
}