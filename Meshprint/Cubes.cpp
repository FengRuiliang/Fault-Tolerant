#include "Cubes.h"

class Box;

Cubes::Cubes()
{
	ptr_faces_ = NULL;
	
}


Cubes::Cubes(Mesh3D* mesh)
{
	mesh_in_ = mesh;
	ptr_faces_ = mesh->get_faces_list();
}
Cubes::Cubes(std::vector<HE_face*>* faces)
{
	mesh_in_ = NULL;
	ptr_faces_ = faces;
	
	
}


Cubes::Box* Cubes::insertToBox(Vec3f pos, HE_face * facet)
{

	int id_x_ = round((pos.x() + GetUnit()[0] * 10000) / GetUnit()[0] - 0.5) - 10000;
	int id_y_ = round((pos.y() + GetUnit()[1] * 10000) / GetUnit()[1] - 0.5) - 10000;
	int id_z_ = round((pos.z() + GetUnit()[2] * 10000) / GetUnit()[2] - 0.5) - 10000;
	Box b_(id_x_, id_y_, id_z_, GetUnit());
	std::set<Box, sortBox>::iterator box_iter_ = boxes_.insert(b_).first;
	return &b_;
}


Cubes::~Cubes()
{
	boxes_.clear();
	ptr_faces_ = NULL;
	mesh_in_ = NULL;
}


void Cubes::insertArcToBox(std::vector<cutLine>* pieces_)
{
	for (int i=0;i<pieces_->size();i++)
	{

	}
}

void Cubes::StoreBox()
{

//	setUnit(mesh_in_->average_edge_length());//5 triangles
	for (std::vector<HE_face*>::iterator iterF = ptr_faces_->begin(); iterF != ptr_faces_->end(); iterF++)
	{
		HE_edge* start_ = (*iterF)->pedge_;
		HE_edge* cur_ = start_;
		do
		{
			insertToBox(cur_->pvert_->position(), *iterF);
			cur_ = cur_->pnext_;
		} while (cur_ != start_&&cur_ != NULL);
	}

}

Cubes::Box::Box()
{
	idX_ = 0;	idY = 0;	idZ_ = 0;	zheight_ = 0;	xmin_ = 0.0;	ymin_ = 0.0;	zmin_ = 0.0;
	facets = NULL;
}

Cubes::Box::Box(int x, int y, int z, Vec3f hei)
{
	idX_ = x;
	idY = y;
	idZ_ = z;
	xlength = hei[0];
	ywidth = hei[1];
	zheight_ = hei[2];
	xmin_ = x*xlength;
	ymin_ = y*ywidth;
	zmin_ = z*zheight_;
	facets=NULL;
}

void Cubes::Box::setCube(float x, float y, float z, float h)
{
	xmin_ = x; ymin_ = y; zmin_ = z; zheight_ = h;
}

Vec4f Cubes::Box::GetCoordinate()
{
	return Vec4f(xmin_, ymin_, zmin_, zheight_);
}

std::vector<int>  Cubes::Box::getID()
{
	std::vector<int> id = { idX_,idY,idZ_ };
	return id;
}


void Cubes::Box::insertF(HE_face * facet)
{
	if (facets==NULL)
	{
		facets = new std::vector<HE_face *>;
	}
	facets->push_back(facet);
}

void Cubes::Box::insertP(Vec3f point)
{
	if (crosspoint==NULL)
	{
		crosspoint = new std::set<Vec3f, sortVec>;
	}
	crosspoint->insert(point);
}

Cubes::Box::~Box()
{
	SafeDelete(facets);
}
