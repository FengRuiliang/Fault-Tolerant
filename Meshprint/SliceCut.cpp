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
			min_height = min(min_height, (*iter_Face)->vertices_[i]->position().z());
			max_height = max(max_height, (*iter_Face)->vertices_[i]->position().z());
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

		std::vector<int>&slice_faces_ = storage_Face_list_[i];
		//qDebug() <<i<< slice_faces_.size();
		float cur_height_ = i*thickness_;
		HE_edge * cur_edge_ = NULL;
		HE_edge * longest_edge_ = NULL;
		HE_edge * cut_edge = NULL;
		std::vector<int> vert_sort;

		// find the longest edge which is been acrossed;
		std::vector<HE_edge *> layer_hedge_;
		
		for (auto iter = slice_faces_.begin(); iter != slice_faces_.end(); iter++)
		{
			//circle_list_ = new std::vector<cutLine>;//new a vector storage
			//pieces_list_[i].push_back(circle_list_);
			if (*iter == -1)
			{
				continue;
			}
			cur_edge_ = faces.at(*iter)->pedge_;
			vert_sort = sortVertInFace(*iter);//sort the three vertex in z position of No *iter face.
											  //lowest point shall  not higher than current height
											  //qDebug() << *iter;
			if (verts.at(vert_sort[2])->position().z() > cur_height_ || (faces.at(*iter)->normal().x() == 0 & faces.at(*iter)->normal().y() == 0))//in this layer,eliminate the higher one
			{
				*iter = -1;
				continue;
			}
			else
			{
				std::vector<std::pair<Vec3f, Vec3f>> circle_list_;
				if (cur_edge_->pvert_->id() == vert_sort[0])//当前边的end是最高点
				{
					if (cur_edge_->pprev_->pvert_->id() == vert_sort[1])//但前边的start是中间高度点。
					{
						longest_edge_ = cur_edge_->pnext_;
					}
					else
					{
						longest_edge_ = cur_edge_;
					}

				}
				else if (cur_edge_->pvert_->id() == vert_sort[1])
				{
					longest_edge_ = cur_edge_->pprev_;
				}
				else
				{
					if (cur_edge_->pprev_->pvert_->id() == vert_sort[1])
					{
						longest_edge_ = cur_edge_->pnext_;
					}
					else
					{
						longest_edge_ = cur_edge_;
					}

				}
				cur_edge_ = longest_edge_;
				if (cur_edge_ != NULL)
				{
					int index = 0;
					do
					{
						index++;
						Vec3f cur_vector_(cur_edge_->pvert_->position() - cur_edge_->pprev_->pvert_->position());

						point pos1, pos2;
						if (cur_vector_.z() == 0)//it means the face is lay flat in the cut face.
						{
							cur_edge_ = cur_edge_->pnext_->ppair_;//28/11/2016
							continue;
						}
						else
						{
							//if (cur_edge_->pvert_->position().z() > cur_height_)//the cur_edge_ is up vector
							if (cur_vector_.z() > 0)// modify at 2017/2/28
							{
								cut_edge = cur_edge_;
								pos2 = cur_edge_->pvert_->position() - cur_vector_*(cur_edge_->pvert_->position().z() - cur_height_) / cur_vector_.z();
								if (cur_edge_->pnext_->pvert_->position().z() > cur_height_)
								{
									cur_edge_ = cur_edge_->pprev_;
									cur_vector_ = (cur_edge_->pvert_->position() - cur_edge_->pprev_->pvert_->position());
									pos1 = cur_edge_->pvert_->position() - cur_vector_*(cur_edge_->pvert_->position().z() - cur_height_) / cur_vector_.z();

								}
								else if (cur_edge_->pnext_->pvert_->position().z() < cur_height_)
								{
									cur_edge_ = cur_edge_->pnext_;
									cur_vector_ = (cur_edge_->pvert_->position() - cur_edge_->pprev_->pvert_->position());
									pos1 = cur_edge_->pvert_->position() - cur_vector_*(cur_edge_->pvert_->position().z() - cur_height_) / cur_vector_.z();

								}
								else
								{
									pos1 = cur_edge_->pnext_->pvert_->position();
									cur_edge_ = cur_edge_->pnext_;//2016/11/28
								}

							}
							else  //(cur_edge_->pvert_->position().z() <=cur_height_),the cur_edge_ is down vector
							{

								pos1 = cur_edge_->pvert_->position() - cur_vector_*(cur_edge_->pvert_->position().z() - cur_height_) / cur_vector_.z();

								if (cur_edge_->pnext_->pvert_->position().z() < cur_height_)
								{

									cur_edge_ = cur_edge_->pprev_;
									cur_vector_ = (cur_edge_->pvert_->position() - cur_edge_->pprev_->pvert_->position());

									pos2 = cur_edge_->pvert_->position() - cur_vector_*(cur_edge_->pvert_->position().z() - cur_height_) / cur_vector_.z();
									cut_edge = cur_edge_;
								}
								else if (cur_edge_->pnext_->pvert_->position().z() > cur_height_)
								{
									cur_edge_ = cur_edge_->pnext_;
									cur_vector_ = (cur_edge_->pvert_->position() - cur_edge_->pprev_->pvert_->position());
									pos2 = cur_edge_->pvert_->position() - cur_vector_*(cur_edge_->pvert_->position().z() - cur_height_) / cur_vector_.z();
									cut_edge = cur_edge_;
								}
								else// cur_edge_->pnext_->pvert_->position().z()== cur_height_
								{
									pos2 = cur_edge_->pnext_->pvert_->position();
									if (cur_edge_->pvert_->position().z() == cur_height_)
									{
										cur_edge_ = cur_edge_->pprev_;
									}
									else
									{
										cur_edge_ = cur_edge_->pnext_;
									}

									cut_edge = cur_edge_;
								}

							}
							cur_edge_ = cut_edge->ppair_;
						}
						if (cur_edge_->pface_ == NULL)//means reach the boundary 28/11/2016
						{
							//qDebug()<<" faces.at(*iter)"<< faces.at(*iter) <<
							break;
						}
						std::vector<int>::iterator p = std::find(slice_faces_.begin(), slice_faces_.end(), cur_edge_->pface_->id());
						if (p == slice_faces_.end())
						{
							break;
							qDebug() << i << index;
						}
						//if (p != slice_faces_.end())
						{

							*p = -1;
						}

						//qDebug() << "slice" << i;
						if (pos1 == pos2)
						{
							continue;
						}
						circle_list_.push_back(std::pair<Vec3f,Vec3f>(pos1, pos2));
					} while (cur_edge_->id() != longest_edge_->id() && cur_edge_->pface_ != NULL);
				}
				pieces_list_[i].push_back(circle_list_);
			}

		}
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
		std::vector<cutLine> chain_boundary_;
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
// 		qDebug() << nu;
// 		if (nu==127||nu==250||nu==255||nu==272||nu==451||nu==572||nu==772||nu==835||nu==943||nu==947||nu==963)
// 		{
// 			qDebug() << nu;
// 			continue;
// 		}
		for (int k=0;k<layer_faces_.size();k++)
		{
			if (layer_faces_[k])
			{
				cutLine l;
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

std::pair<Vec3f,Vec3f> SliceCut::cutFacet(HE_face* facet,float cur_height_)
{
	Vec3f p[3];
	for (int m = 0; m < 3; m++)
	{
		p[m] = facet->vertices_[m];
	}
	Vec3f pos1, pos2;
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
	return std::pair<Vec3f, Vec3f>(pos1, pos2);
}

void SliceCut::cutFacetInmiddlePoint()
{
// 	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
// 	for (int i = 0; i < faces.size(); i++)
// 	{
// 		if (faces[i]->normal().x() == 0 && faces[i]->normal().y() == 0)
// 		{
// 			continue;
// 		}
// 		std::vector<Vec3f>  v_;
// 		Vec3f pos_;
// 		for (int j = 0; j < 3; j++)
// 		{
// 			v_.push_back(faces[i]->vertices_[j]->position_);
// 		}
// 		sort(v_.begin(), v_.end(), compvec3fz);
// 		CalPlaneLineIntersectPoint(Vec3f(0.0, 0.0, 1.0), v_[1],
// 			v_[2] - v_[0], v_[0], pos_);
// 		if ((pos_ - v_[1]).cross(faces[i]->normal()).z() > 0)
// 			cut_list_[v_[1].z()].push_back(std::pair<Vec3f, Vec3f>(v_[1], pos_));
// 		else
// 			cut_list_[v_[1].z()].push_back(std::pair<Vec3f, Vec3f>(pos_, v_[1]));
// 
// 		thickf_.insert(v_[1].z());
//	}
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
