#include "Support.h"
#include <algorithm>
#include "Sweep.h"
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
		int supp_angle =180- acos((*iter)->normal() * perpendicular) * 180 / PI;
		if (supp_angle<60)
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
	}

}
void Support::support_point_sampling(int counter_)
{
	sample_points_.clear();qDebug() << counter_;
//do sampling for every triangle
	auto face_list_ = *(target_mesh->get_faces_list());
	Vec3f perpendicular(0.0, 0.0, 1.0);
	for (int i=0;i<face_list_.size();i++)
	{
		int supp_angle =180- acos(face_list_[i]->normal() *perpendicular) * 180 / PI;
		if (supp_angle<60)
		{
			
			std::pair<float, float> dense;
			if (counter_%3==1)
			{// uniform sampling
				dense.first = 2.0;
				dense.second = 2.0;
			}
			else if (counter_%3==2)
			{
				dense = get_dense(supp_angle);
			}
			else
			{
				//best sampling


			}
			std::vector<Vec3f> verts = face_list_[i]->vertices_;

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

				for (;sta_y_<std::max(p1.y(),p2.y());sta_y_+=dense.second)
				{
					Vec3f p = ((sta_y_ - p1.y()) / (p2.y() - p1.y()))*(p2 - p1)+p1;
					sample_points_.push_back(p);
				}
			}
			for (;sta_x_<verts[2].x();sta_x_+=dense.first)
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
					sample_points_.push_back(p);
				}

			}
		}
	}
	qDebug() << " the size of support is:" << sample_points_.size();
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
		else if (angle < 18)
		{
			d_.second = 10.0;
		}
		else
			d_.second = 15.0;
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