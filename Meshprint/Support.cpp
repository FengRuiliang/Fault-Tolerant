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
	Vec3f point_up(0.0, 0.0, 1.0);
	for (auto iter=target_mesh->get_faces_list()->begin();iter!=target_mesh->get_faces_list()->end();iter++)
	{
		if (acos((*iter)->normal() * point_up)*PI / 180>160.0);
		{

		}
	}
}