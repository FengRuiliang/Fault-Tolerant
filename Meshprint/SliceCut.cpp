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
		double max_height = MIN_FLOAT_VALUE;
		double min_height = MAX_FLOAT_VALUE;
		HE_edge* sta_ = (*iter_Face)->pedge_;
		HE_edge* cur_ = sta_;
		do 
		{
			min_height = min(min_height, cur_->pvert_->position().z());
			max_height = max(max_height, cur_->pvert_->position().z());
			cur_ = cur_->pnext_;
		} while (cur_!=sta_);
		if (max_height == min_height)// 22/01/2017
		{
			continue;
		}
		//the num of layer equal to 
		for (int j= min_height / thickness_+1;j<=max_height / thickness_;j++)
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
		if (cur_->pvert_->position().z()<height_&&cur_->start_->position().z()>height_)
		{
			return cur_;
		}
		cur_ = cur_->pnext_;
	} while (cur_!=sta_);
	return NULL;
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
	cutline_list_ = new std::vector<HE_edge *>[num_pieces_];
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	const std::vector<HE_vert *>& verts = *(mesh_in_->get_vertex_list());
	for (size_t i = 0; i < num_pieces_; i++)
	{
		circle_list_ = new std::vector<cutLine>;
		std::vector<int>& slice_faces_ = storage_Face_list_[i];
		float cur_height_ = i*thickness_;
		for (int j = 0; j < slice_faces_.size(); j++)
		{
			HE_face* f_ = faces[slice_faces_[j]];
			HE_edge* e_ = getLeftEdge(f_,cur_height_);
			if (e_==NULL)
			{
				qDebug() << "e is null";
				continue;
			}
			Vec3f pos1, pos2;
			CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
				e_->pvert_->position()-e_->start_->position(), e_->pvert_->position(), pos1);
			do
			{
				e_ = e_->pnext_;
			} while (e_->pvert_->position().z()<=cur_height_);
			

			CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
				e_->pvert_->position() - e_->start_->position(), e_->pvert_->position(), pos2);
			insertCutline(pos1, pos2,i);
			cutLine new_cutline(pos1, pos2);
			circle_list_->push_back(new_cutline);
		}
		pieces_list_[i].push_back(circle_list_);
	}
}


void SliceCut::insertCutline(Vec3f p1, Vec3f p2, int id)
{
	cutLine l_(p1, p2);

	for (int i=0;i<cutline_list_[id].size();i++)
	{
		if (cutline_list_[id][i].front().position_vert[0]==p2)
		{
			cutline_list_[id][i].insert(cutline_list_[id][i].begin(), l_);
		}
		else if (cutline_list_[id][i].back().position_vert[1] == p1)
		{
			cutline_list_[id][i].push_back(l_);
		}
	}
}

