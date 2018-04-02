#include "Support.h"


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