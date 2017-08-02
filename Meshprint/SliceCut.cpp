#pragma once
#include "SliceCut.h"
#include "QDebug"

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
	ClearSlice();
}


std::vector<int> * SliceCut::StoreFaceIntoSlice()
{
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	storage_Face_list_ = new std::vector<int>[num_pieces_];// new #2/thickness_
	
	for (auto iter_Face= faces.begin();iter_Face!= faces.end();iter_Face++ )
	{
		std::vector<HE_vert*> vertex_list = (*iter_Face)->vertices_;
		double max_height = MIN_FLOAT_VALUE;
		double min_height = MAX_FLOAT_VALUE;

		for (int i = 0; i < 3; i++)
		{
			min_height = min(min_height, vertex_list[i]->position().z());
			max_height = max(max_height, vertex_list[i]->position().z());
		}
		if (max_height == min_height)// 22/01/2017
		{
			continue;
		}
		//the num of layer equal to 
		for (int j= min_height / thickness_;j<=max_height / thickness_;j++)
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

std::vector<Vec3f> SliceCut::sortVertInFace(HE_face* face_)
{

	std::vector<Vec3f> vertex_list;
	 face_->vertices_;
	int min_id_;
	double min_height = MIN_FLOAT_VALUE;
	for (int i = 0; i < 3; i++)
	{
		vertex_list.push_back(face_->vertices_[i]->position());
		if (min_height < vertex_list[i].z())
		{
			min_height = vertex_list[i].z();
			min_id_ = i;
		}
	}
	switch (min_id_)
	{
	case 0:
		break;
	case 1:
		SWAP(vertex_list[0], vertex_list[1],Vec3f);
		SWAP(vertex_list[1], vertex_list[2], Vec3f);
		break;
	case 2:
		SWAP(vertex_list[0], vertex_list[2], Vec3f);
		SWAP(vertex_list[1], vertex_list[2], Vec3f);
		break;
	default:
		break;
	}
	 return vertex_list;
}

void SliceCut:: ClearSlice()
{
	if (pieces_list_==NULL)
	{
		return;
	}
	for (size_t i=0;i<num_pieces_;i++)
	{
		pieces_list_[i].clear();
		storage_Face_list_[i].clear();
	}
	delete[]pieces_list_;
	delete[] storage_Face_list_;
	pieces_list_ = NULL;
	storage_Face_list_ = NULL;
}

void SliceCut::CutInPieces()
{
	pieces_list_ = new std::vector<std::vector<cutLine>*>[num_pieces_];
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	const std::vector<HE_vert *>& verts = *(mesh_in_->get_vertex_list());
	for (size_t i = 0; i < num_pieces_; i++)
	{
		circle_list_ = new std::vector<cutLine>;
		std::vector<int>&slice_faces_ = storage_Face_list_[i];
		float cur_height_ = i*thickness_;
		for (int j = 0; j < slice_faces_.size(); j++)
		{
			HE_face* f_ = faces[slice_faces_[j]];
			std::vector<Vec3f> v = sortVertInFace(f_);
			Vec3f pos1, pos2;
			if (v[2].z() > cur_height_)
			{
				CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_), v[2] - v[0], v[0], pos1);
			}
			else
			{
				CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_), v[1] - v[2], v[2], pos1);
			}
			if (v[1].z() > cur_height_)
			{
				CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_), v[1] - v[0], v[0], pos2);
			}
			else
			{
				CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_), v[2] - v[1], v[1], pos2);
			}
			pos1.z() = cur_height_;
			pos2.z() = cur_height_;
			cutLine new_cutline(pos1, pos2);
			circle_list_->push_back(new_cutline);
		}
		pieces_list_[i].push_back(circle_list_);
	}
}

