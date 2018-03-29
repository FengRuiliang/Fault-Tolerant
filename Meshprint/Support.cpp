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
		if (supp_angle>155)
		{

			std::vector<HE_vert*> verts;
			(*iter)->face_verts(verts);
			supp_aeras[(180 - supp_angle)/2].InsertFace(verts);
		}		
	}
	return;
	for (auto iterM=supp_aeras.begin();iterM!=supp_aeras.end();iterM++)
	{
		iterM->second.UpdateMesh();
		std::vector<HE_face*> flist=*(iterM->second.get_faces_list());
		for (int k=0;k<flist.size();k++)
		{
			flist[k]->vertices_[0] -= Vec3f(0.0, 0.0, flist[k]->vertices_[0].z());
			flist[k]->vertices_[1] -= Vec3f(0.0, 0.0, flist[k]->vertices_[1].z());
			flist[k]->vertices_[2] -= Vec3f(0.0, 0.0, flist[k]->vertices_[2].z());
		}
	}

}