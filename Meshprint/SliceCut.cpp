#pragma once
#include "SliceCut.h"
#include "QDebug"
#include "Library/Polygon.h"
#define MAX_FLOAT_VALUE (static_cast<float>(10e10))
#define MIN_FLOAT_VALUE	(static_cast<float>(-10e10))
//CHANGE
// type definitions
typedef std::vector<HE_vert* >::iterator VERTEX_ITER;
typedef std::vector<HE_face* >::iterator FACE_ITER;
typedef std::vector<HE_edge* >::iterator EDGE_ITER;

typedef std::vector<HE_vert* >::reverse_iterator VERTEX_RITER;
typedef std::vector<HE_face* >::reverse_iterator FACE_RITER;
typedef std::vector<HE_edge* >::reverse_iterator EDGE_RITER;
typedef std::pair<HE_vert*, HE_vert* > PAIR_VERTEX;


SliceCut::~SliceCut()
{
	clearcut();
	pieces_list_ = NULL;
	storage_Face_list_ = NULL;
// 	clip_list_ = NULL;
}

std::vector<int> * SliceCut::StoreFaceIntoSlice()
{

	//qDebug() << mesh_in_->get_faces_list()->size();
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	const std::vector<HE_vert *>& vertice = *(mesh_in_->get_vertex_list());
	storage_Face_list_ = new std::vector<int>[num_pieces_];// new #2/thickness_
	
	//qDebug() << storage_Face_list_->size()<<2/thickness_+1;
	for (auto iter_Face= faces.begin();iter_Face!= faces.end();iter_Face++ )
	{
		double max_height = MIN_FLOAT_VALUE;
		double min_height = MAX_FLOAT_VALUE;
	
		for (int i=0;i<3;i++)
		{
			min_height = min(min_height, (*iter_Face)->vec_ptr_vert_[i]->position().z());
			max_height = max(max_height, (*iter_Face)->vec_ptr_vert_[i]->position().z());
		}
		if (max_height-min_height<1e-3)// 22/01/2017
		{
			continue;
		}
		//the num of layer equal to 
		int j = min_height / thickness_;
		if (j*thickness_ != min_height)j++;
		for (;j* thickness_<max_height ;j++)
		{
			storage_Face_list_[j].push_back((*iter_Face)->id());	
		}
	}
	return storage_Face_list_;
}

int SliceCut::isEdgeInFace(HE_vert* pvert1, HE_vert* pvert2, HE_face* pface){
	HE_edge* temp_edge = mesh_in_->getedgemap()[PAIR_VERTEX(pvert1, pvert2)];
	if (mesh_in_->getedgemap()[PAIR_VERTEX(pvert1, pvert2)]->pface_ == pface)
	{
		return temp_edge->id();
	}
	temp_edge = temp_edge->ppair_;
	if (temp_edge->pface_ == pface)
	{
		return temp_edge->id();
	}
	return -1;
}

HE_edge* SliceCut::getLeftEdge(HE_face* face_,float height_)
{
	HE_edge* sta_ = face_->pedge_;
	HE_edge* cur_ = face_->pedge_;
	do 
	{
		if (cur_->pvert_->position().z()<=height_&&cur_->start_->position().z()>=height_)
		{
			return cur_;
		}
		cur_ = cur_->pnext_;
	} while (cur_!=sta_);
	return NULL;
}

void SliceCut:: clearcut()
{
	if (pieces_list_==NULL)
	{
		return;
	}
	delete[]pieces_list_;
	delete[] storage_Face_list_;
	pieces_list_ = NULL;
	storage_Face_list_ = NULL;

	
}

std::vector<int> SliceCut::sortVertInFace(int faceid)
{
	// get sorted vertexes in a face
	HE_face* curFace = mesh_in_->get_face(faceid);
	HE_edge* pedge = curFace->pedge_;
	//each face with 3 vertexes
	std::vector<int> vertex_list;
	double max_height = MIN_FLOAT_VALUE;
	double min_height = MIN_FLOAT_VALUE;
	do
	{
		int vertid = pedge->pvert_->id();
		vertex_list.push_back(vertid);
		pedge = pedge->pnext_;
	} while (pedge != curFace->pedge_);
#define it_rotate_vert(n) mesh_in_->get_vertex_list()->at(vertex_list[n])->position()
#define switch_vert(a,b) tempid = vertex_list[a]; vertex_list[a] = vertex_list[b]; vertex_list[b] = tempid;
	if (vertex_list.size() == 3)
	{
		int tempid;
		if (it_rotate_vert(0).z() < it_rotate_vert(2).z())
		{
			switch_vert(0, 2);
		}
		if (it_rotate_vert(1).z() < it_rotate_vert(2).z())
		{
			switch_vert(1, 2);
		}
		else if (it_rotate_vert(0).z() < it_rotate_vert(1).z())
		{
			switch_vert(0, 1);
		}
	}

	return vertex_list;
}

void SliceCut::CutInPieces()
{
//#define  SHOWLINES
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	const std::vector<HE_vert *>& verts = *(mesh_in_->get_vertex_list());
	if (pieces_list_ != NULL)
	{
		delete[]pieces_list_;
	}
	pieces_list_ = new std::vector<std::vector<std::pair<Vec3f, Vec3f>>>[num_pieces_];

	for (int i = 0; i < num_pieces_; i++)
	{
		Polygon polygon_;
		std::vector<int>&slice_faces_ = storage_Face_list_[i];
		float cur_height_ = i*thickness_;
#ifdef SHOWLINES
		int count_ = 0;
		std::vector<std::pair<Vec3f, Vec3f>> cir;
		for (int j = 0; j < slice_faces_.size(); j++)
		{
			auto t = cutFacet(faces[slice_faces_[j]], cur_height_);
			CutLine* et_ = polygon_.insertEdge(t.first, t.second);
			if (et_ != NULL)
			{
				
				cir.push_back(std::pair<Vec3f, Vec3f>(et_->position_vert[0], et_->position_vert[1]));
				pieces_list_[i].push_back(cir);
			}
		}
		qDebug() << count_;
		
#else
		std::vector<std::pair<Vec3f, Vec3f>> cir;
		for (int j = 0; j < slice_faces_.size(); j++)
		{
			auto t = cutFacet(faces[slice_faces_[j]], cur_height_);
			polygon_.insertEdge(t.first, t.second);
		}
		polygon_.ConnectCutline();
		//polygon_.FindIntersection();
		polygon_.storePathToPieces(pieces_list_, i);
		//qDebug() << i << pieces_list_[i].size();
#endif
	}
}

static bool sortCutLineByZ(const CutLine* a,const CutLine* b )
{
	return abs((a->position_vert[0] - a->position_vert[1]).z()) <
		abs((b->position_vert[0] - b->position_vert[1]).z());
}

std::pair<Vec3f,Vec3f> SliceCut::cutFacet(HE_face* facet,float cur_height_)
{
	std::vector<Vec3f> p;
	p.push_back(facet->vec_ptr_vert_[0]->position());
	p.push_back(facet->vec_ptr_vert_[1]->position());
	p.push_back(facet->vec_ptr_vert_[2]->position());
	std::vector<CutLine*> e_;
	e_.push_back(new CutLine(p[0],p[1]));
	e_.push_back(new CutLine(p[1], p[2]));
	e_.push_back(new CutLine(p[2], p[0]));
	for (int i = 0; i < 3; i++)
		e_[i]->pnext_ = e_[(i + 1) % 3];
	std::sort(e_.begin(), e_.end(), sortCutLineByZ);
	Vec3f pos1,pos2;
	Vec3f dir = e_[2]->position_vert[1] - e_[2]->position_vert[0];
	if (dir.z()<0)
	{
		pos1 = e_[2]->position_vert[0] + (cur_height_ - e_[2]->position_vert[0].z())/(dir.z())*dir;
		if (e_[2]->pnext_->position_vert[1].z()<cur_height_)
		{
			pos2 = e_[2]->pnext_->pnext_->position_vert[0] +
				(e_[2]->pnext_->pnext_->position_vert[1] - e_[2]->pnext_->pnext_->position_vert[0])*
				(cur_height_ - e_[2]->pnext_->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->pnext_->position_vert[1]- e_[2]->pnext_->pnext_->position_vert[0]).z();
		}
		else if (e_[2]->pnext_->position_vert[1].z()==cur_height_)
		{
			pos2 = e_[2]->pnext_->position_vert[1];
		}
		else
		{
			pos2 = e_[2]->pnext_->position_vert[0] +
				
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0])*
				(cur_height_ - e_[2]->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0]).z();
		}
	}
	else
	{
		pos2 = e_[2]->position_vert[0] + (cur_height_ - e_[2]->position_vert[0].z())/dir.z()*dir;
		if (e_[2]->pnext_->position_vert[1].z()<cur_height_)
		{
			pos1 = e_[2]->pnext_->position_vert[0] +
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0])*
				(cur_height_ - e_[2]->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0]).z()	;
		}
		else if (e_[2]->pnext_->position_vert[1].z()==cur_height_)
		{
			pos1 = e_[2]->pnext_->position_vert[1];
		}
		else
		{
			pos1 = e_[2]->pnext_->pnext_->position_vert[0] +
				(e_[2]->pnext_->pnext_->position_vert[1] - e_[2]->pnext_->pnext_->position_vert[0])*
				(cur_height_ - e_[2]->pnext_->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->pnext_->position_vert[1] - e_[2]->pnext_->pnext_->position_vert[0]).z();
		}
	}
	for (int i=0;i<3;i++)
	{
		delete e_[i];
	}
	return std::pair<Vec3f, Vec3f>(pos1, pos2);
}

std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* SliceCut::GetPieces()
{
	
	 return pieces_list_; 
}