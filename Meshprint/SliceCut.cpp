#pragma once
#include "SliceCut.h"
#include "QDebug"
#include "Polygon.h"
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
			min_height = min(min_height, (*iter_Face)->vertices_[i].z());
			max_height = max(max_height, (*iter_Face)->vertices_[i].z());
		}
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
		//std::vector<std::pair<Vec3f, Vec3f>> circle_;
		for (auto iter = slice_faces_.begin(); iter != slice_faces_.end(); iter++)
		{
			auto t = cutFacet(faces[*iter], cur_height_);
			polygon_.insertEdge(t.first, t.second);
			//circle_.push_back(t);
		}
		//pieces_list_[i].push_back(circle_);
		qDebug() << i << slice_faces_.size()
			<< polygon_.num_of_edges()
			<< polygon_.num_of_points()
			<< polygon_.sweepPolygon();
		polygon_.storePathToPieces(pieces_list_, i);
	}
}




bool compvec3fz(const Vec3f a, const Vec3f b)
{
	return a.z() < b.z();
}

void SliceCut::cutThrouthVertex()
{
	std::vector<HE_vert*> queue_event_ = *mesh_in_->get_vertex_list();
	std::vector<HE_face*> faces_ = *mesh_in_->get_faces_list();
	sort(queue_event_.begin(), queue_event_.end(), sortByZB);
	std::vector<int> layer_faces_(mesh_in_->num_of_face_list());
	int nu = 0;
	while (!queue_event_.empty())
	{
		nu++;
		std::vector<CutLine> chain_boundary_;
		HE_vert* cur_vert_ = queue_event_.back();
		HE_vert* sta_vert_ = cur_vert_;
		int hei_ = cur_vert_->position().z() * 100;
		int thi_ = thickness_ * 100;
		float cur_hei_ = (float)(hei_ -hei_%thi_) / 100;
		do 
		{
			cur_vert_ = queue_event_.back();
			for (int i = 0; i < cur_vert_->mergeFace.size(); i++)
			{
				layer_faces_[cur_vert_->mergeFace[i]] = 0;
			}
			for (int j = 0; j < cur_vert_->splitFace.size(); j++)
			{
				layer_faces_[cur_vert_->splitFace[j]] = 1;
			}
			queue_event_.pop_back();
		} while (!queue_event_.empty()&&sta_vert_->position().z()-queue_event_.back()->position().z()<1e-3);

		for (int k=0;k<layer_faces_.size();k++)
		{
			if (layer_faces_[k])
			{
				CutLine l;
// 				if (cutFacet(faces_[k], cur_vert_->position().z()-0.001,l))
// 				{chain_boundary_.push_back(l );
// 				}
				
			}
		}
		SweepLine sweep_line_(chain_boundary_);
		sweep_line_.polygonization();
		sweep_line_.getContuor(cut_list_);
	}
}

void SliceCut::sweepPline()
{

// 	std::vector<HE_vert*> queue_event_ = *mesh_in_->get_vertex_list();
// 	sort(queue_event_.begin(), queue_event_.end(), sortByZB);
// 	while (!queue_event_.empty())
// 	{
// 		HE_vert* cur_vert_= queue_event_.back();
// 		std::vector<cutLine*> lines_;
// 		float cur_hei_ = cur_vert_->position().z() - (float)((int)(cur_vert_->position().z() * 100) % (int)(thickness_ * 100)) / 100;
// 		for (int i=0;i<cur_vert_->splitFace.size();i++)
// 		{
// 			std::pair<Vec3f, Vec3f> p = cutFacet(cur_vert_->splitFace[i], cur_hei_);
// 			std::vector<std::pair<Vec3f, Vec3f>> lines_;
// 			auto iter = lines_.begin();
// 			for (;iter!=lines_.end();iter++)
// 			{
// 				if ((iter->second-p.first).length()<1e-3)
// 				{
// 					lines_.insert(iter, p);
// 				}
// 				else if ((iter->first - p.second).length()<1e-3)
// 				{
// 					lines_.insert(iter-1, p);
// 				}
// 			}
// 			if (iter==lines_.end())
// 			{
// 				lines_.push_back(p);
// 			}
// 		}
// 		if (!cur_vert_->mergeFace.empty())
// 		{
// 			//replace all merge face by split facet
// 
// 		}
// 		else
// 		{
// 			//insert new loop
// 		}
// 
// 		//////////////////////////////////////////////////////////////////////////
// 
// 		if (!cur_vert_->mergeFace.empty() || !cur_vert_->splitFace.empty())
// 		{
// 			
// 		}
// 
// 	}
// 		



}

static bool sortCutLineByZ(const CutLine* a,const CutLine* b )
{
	return abs((a->position_vert[0] - a->position_vert[1]).z()) <
		abs((b->position_vert[0] - b->position_vert[1]).z());
}
std::pair<Vec3f,Vec3f> SliceCut::cutFacet(HE_face* facet,float cur_height_)
{
	std::vector<Vec3f>& p = facet->vertices_;
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
				(cur_height_ - e_[2]->pnext_->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->pnext_->position_vert[1]- e_[2]->pnext_->pnext_->position_vert[0]).z()*
				(e_[2]->pnext_->pnext_->position_vert[1] - e_[2]->pnext_->pnext_->position_vert[0]);
		}
		else
		{
			pos2 = e_[2]->pnext_->position_vert[0] +
				(cur_height_ - e_[2]->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0]).z()*
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0]);
		}
	}
	else
	{
		pos2 = e_[2]->position_vert[0] + (cur_height_ - e_[2]->position_vert[0].z())/dir.z()*dir;
		if (e_[2]->pnext_->position_vert[1].z() < cur_height_)
		{
			pos1 = e_[2]->pnext_->position_vert[0] +
				(cur_height_ - e_[2]->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0]).z()*
				(e_[2]->pnext_->position_vert[1] - e_[2]->pnext_->position_vert[0]);
		}
		else
		{
			pos1 = e_[2]->pnext_->pnext_->position_vert[0] +
				(cur_height_ - e_[2]->pnext_->pnext_->position_vert[0].z())/
				(e_[2]->pnext_->pnext_->position_vert[1] - e_[2]->pnext_->pnext_->position_vert[0]).z()*
				(e_[2]->pnext_->pnext_->position_vert[1] - e_[2]->pnext_->pnext_->position_vert[0]);
		}
	}
	return std::pair<Vec3f, Vec3f>(pos1, pos2);
}


std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* SliceCut::GetPieces()
{
	
	 return pieces_list_; 
}

HE_edge* SliceCut::InsertEdgeBySweep(HE_vert* vstart, HE_vert* vend)
{
	if (vstart == NULL || vend == NULL)
	{
		return NULL;
	}
}
