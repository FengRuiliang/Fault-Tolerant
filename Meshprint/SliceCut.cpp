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
		if (cur_->pvert_->position().z()<=height_&&cur_->start_->position().z()>=height_)
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
	delete[]pieces_list_;
	delete[] storage_Face_list_;
	pieces_list_ = NULL;
	storage_Face_list_ = NULL;
	
}

void SliceCut::CutInPieces()
{
	pieces_list_ = new std::vector<std::vector<cutLine>*>[num_pieces_];
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	for (size_t i = 0; i < num_pieces_; i++)
	{
		std::vector<int>& slice_faces_ = storage_Face_list_[i];
		float cur_height_ = i*thickness_;
		std::vector<std::pair<Vec3f,Vec3f>> chain_boundary_;
		for (int j=0;j<slice_faces_.size();j++)
		{
			Vec3f p[3];
			for (int m = 0; m < 3; m++)
			{
				p[m] = faces[slice_faces_[j]]->vertices_[m]->position();
			}
			Vec3f pos1, pos2;
			for (int j = 0; j < slice_faces_.size(); j++)
			{
				if (p[0].z() >= cur_height_)
				{
					if (p[1].z() >= cur_height_)
					{
						CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
							p[2] - p[1], p[2], pos1);
						CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
							p[0] - p[2], p[0], pos2);
					}
					else
					{
						if (p[2].z() >= cur_height_)
						{
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[1] - p[0], p[0], pos1);
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[2] - p[1], p[1], pos2);
						}
						else
						{
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[1] - p[0], p[0], pos1);
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[2] - p[0], p[0], pos2);

						}
					}
				}
				else
				{
					if (p[1].z() >= cur_height_)
					{
						if (p[2].z() >= cur_height_)
						{
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[0] - p[2], p[2], pos1);
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[1] - p[0], p[0], pos2);
						}
						else
						{
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[2] - p[1], p[2], pos1);
							CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
								p[1] - p[0], p[0], pos2);
						}
					}
					else
					{
						CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
							p[0] - p[2], p[2], pos1);
						CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), Vec3f(0.0, 0.0, cur_height_),
							p[2] - p[1], p[1], pos2);
					}
				}
			chain_boundary_.push_back(std::pair<Vec3f,Vec3f>(pos1, pos2));
			}			
		}
		SweepLine sweep_line_(chain_boundary_);
	}
}


