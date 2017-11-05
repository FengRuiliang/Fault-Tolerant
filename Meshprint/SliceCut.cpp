#pragma once
#include "SliceCut.h"
#include "QDebug"
#include <omp.h>
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
	clip_list_ = NULL;
}

std::vector<int> * SliceCut::storeMeshIntoSlice()
{

	//qDebug() << mesh_in_->get_faces_list()->size();
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	const std::vector<HE_vert *>& vertice = *(mesh_in_->get_vertex_list());
	storage_Face_list_ = new std::vector<int>[num_pieces_];// new #2/thickness_
	pieces_list_ = new std::vector<std::vector<cutLine>*>[num_pieces_];
	
	//qDebug() << storage_Face_list_->size()<<2/thickness_+1;
	for (auto iter_Face= faces.begin();iter_Face!= faces.end();iter_Face++ )
	{
		
		//minimal 2 and maximal 0
		float min_z_ = vertice.at(sortVertInFace((*iter_Face)->id()).at(2))->position().z();
		float max_z_ = vertice.at(sortVertInFace((*iter_Face)->id()).at(0))->position().z();
		if (min_z_==max_z_)// 22/01/2017
		{
			continue;
		}
		//the num of layer equal to 
		for (int j= min_z_ / thickness_;j<=max_z_ / thickness_;j++)
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

void SliceCut:: clearcut()
{
	if (pieces_list_==NULL)
	{
		return;
	}
	for (size_t i=0;i<num_pieces_;i++)
	{
		for (size_t j=0;j<pieces_list_[i].size();j++)
		{
			pieces_list_[i][j]->clear();
			delete pieces_list_[i][j];
		}
	}
	delete[]pieces_list_;
	delete[] storage_Face_list_;
	delete[] clip_list_;
}

void SliceCut::CutInPieces()
{
	const std::vector<HE_face *>& faces = *(mesh_in_->get_faces_list());
	const std::vector<HE_vert *>& verts = *(mesh_in_->get_vertex_list());

#pragma omp parallel for
	for (int i = 0; i <num_pieces_; i++)
	{
	
		std::vector<int>&slice_faces_ = storage_Face_list_[i];
		//qDebug() <<i<< slice_faces_.size();
		float cur_height_ = i*thickness_;
		HE_edge * cur_edge_=NULL;
		HE_edge * longest_edge_=NULL;
		HE_edge * cut_edge = NULL;
		std::vector<int> vert_sort;
	
		// find the longest edge which is been acrossed;
		std::vector<HE_edge *> layer_hedge_;
		std::vector<cutLine>* circle_list_;
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
			if (verts.at(vert_sort[2])->position().z()>cur_height_|| (faces.at(*iter)->normal().x()==0& faces.at(*iter)->normal().y()==0))//in this layer,eliminate the higher one
			{
				*iter=-1;
				continue;
			}
			else
			{
				//if (circle_list_!=NULL)
				{
					circle_list_ = new std::vector<cutLine>;//new a vector storage
					pieces_list_[i].push_back(circle_list_);
				}
				
				if (cur_edge_->pvert_->id()==vert_sort[0])//当前边的end是最高点
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
				else if (cur_edge_->pvert_->id()==vert_sort[1])
				{
					longest_edge_ = cur_edge_->pprev_;
				}
				else
				{
					if (cur_edge_->pprev_->pvert_->id()==vert_sort[1])
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
							if (cur_vector_.z() > 0 )// modify at 2017/2/28
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
						if (cur_edge_->pface_==NULL)//means reach the boundary 28/11/2016
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
						if (pos1==pos2)
						{
							continue;
						}
 						circle_list_->push_back(cutLine(pos1,pos2));
					} while (cur_edge_->id() != longest_edge_->id() && cur_edge_ ->pface_!= NULL);
				}
			}

		}
	}
}

void SliceCut::clipPolygon()
{
	if (clip_list_!=NULL)
	{
		delete[] clip_list_;
	}
	clip_list_ = new std::vector<std::vector<Vec3f>>[num_pieces_];
	for (int i=num_pieces_-1;i>1;i--)
	{
		//std::vector < std::vector<cutLine>*>*pieces_list_;

		std::vector<std::vector<cutLine>*>& slice = pieces_list_[i];
		std::vector<std::vector<cutLine>*>& slice_down_= pieces_list_[i-1];
		Paths sub(slice.size());
		Paths clip(slice_down_.size());
		Paths solution;
		for (int j=0;j<slice.size();j++)
		{
			std::vector<cutLine>* loop = slice[j];
			for (int k=0;k<loop->size();k++)
			{
				int x_ = loop->at(k).position_vert[0].x()*1e6;
				int y_ = loop->at(k).position_vert[0].y()*1e6;
				sub[j] << IntPoint(x_, y_);
			}
		}
		for (int j = 0; j < slice_down_.size(); j++)
		{
			std::vector<cutLine>* loop = slice_down_[j];
			for (int k = 0; k < loop->size(); k++)
			{
				int x_ = loop->at(k).position_vert[0].x()*1e6;
				int y_ = loop->at(k).position_vert[0].y()*1e6;
				clip[j] << IntPoint(x_, y_);
			}
		}
		Clipper c;
		c.AddPaths(sub, ptSubject, true);
// 		for (int m=0;m< slice_down_.size();m++)
// 		{
// 		
// 			c.Clear();
// 			c.AddPaths(solution, ptSubject, true);
// 		}
		c.AddPaths(clip, ptClip, true);
		c.Execute(ctDifference, solution, pftNonZero, pftNonZero);

		for (int j = 0; j < solution.size(); j++)
		{
			std::vector<Vec3f> loop;
			for (int k = 0; k < solution[j].size(); k++)
			{
				Vec3f p(solution[j][k].X*1e-6, solution[j][k].Y*1e-6, i*thickness_);
				loop.push_back(p);
			}
			clip_list_[i].push_back(loop);
		}
	}
}


void SliceCut::storeClipIntoCube()
{
	Cubes cubes_support_;
	cubes_support_.setUnit(spot_size_, spot_size_, thickness_);
	for (int i = 0; i < num_pieces_; i++)
	{
		
		for (int j=0;j<clip_list_[i].size();j++)
		{
			std::multiset<Vec3f, comVec3fBlack> grid_points_;
			int num = clip_list_[i][j].size();
			Vec3f a, b, vector_;
			for (int k=0;k<clip_list_[i][j].size();k++)
			{
				clip_list_[i][j][k];
				clip_list_[i][j][(k + 1) % num];
				float x_min_ = a.x() <= b.x() ? a.x() : b.x();
				float x_max_ = a.x() >= b.x() ? a.x() : b.x();
				int min_wid_id_ = round((x_min_ + spot_size_ * 10000) / spot_size_ - 0.5) - 10000;
				int max_wid_id_ = round((x_max_ + spot_size_ * 10000) / spot_size_ - 0.5) - 10000;
				vector_ = b - a;
				for (int x_id_ = min_wid_id_; x_id_ <= max_wid_id_; x_id_++)
				{
					if (vector_.x() == 0) { continue; }
					if (x_id_*spot_size_ == x_max_) { continue; }
					else
					{
						Vec3f point_p_ = a + (x_id_*spot_size_ - a.x()) / vector_.x()*vector_;
						grid_points_.insert(point_p_);
					}
				}
			}
			auto iterB = grid_points_.begin();
			if (iterB == grid_points_.end())continue;;
			float x_cur_, y_cur_, x_last_, y_last_;
			x_cur_ = (*iterB).x();//used in small grid
			x_last_ = (*iterB).x();
			bool atHeadPoint_ = true;
			for (; iterB != grid_points_.end(); )
			{
				y_last_ = (*iterB).y();
				y_cur_ = (*iterB).y();
				x_cur_ = (*iterB).x();

				if (abs(x_cur_ - x_last_) > 1e-3)// means that we are visiting another line;
				{
					x_last_ = x_cur_;
					atHeadPoint_ = true;
				}

				if (iterB == grid_points_.end()) { break; }// ignore the small grid which has no crosspoint

				else if (atHeadPoint_)
				{
					Vec3f boundaryPD_ = *iterB;
					iterB++;//point to next grid
					if (iterB == grid_points_.end())continue;
					atHeadPoint_ = false;
					y_cur_ = (*iterB).y();
					float y_max_, y_min_;//modify at 2017/3/2
					if (y_cur_ - y_last_ > 1e-3)
					{
						y_max_ = y_cur_;
						y_min_ = y_last_;
					}
					else
					{
						y_max_ = y_last_;
						y_min_ = y_cur_;
					}
					int last_id_ = round((y_min_ + spot_size_ * 10000) / spot_size_ - 0.5) - 10000;
					int cur_id_ = round((y_max_ + spot_size_ * 10000) / spot_size_ - 0.5) - 10000;
					int x_id_ = round((x_cur_ + spot_size_ * 10000) / spot_size_ - 0.5) - 10000;
					if (last_id_*spot_size_ == y_max_)
					{
						last_id_--;
					}
					if (cur_id_*spot_size_ == y_max_)
					{
						cur_id_--;
					}

					if ((FieldType)(abs(x_id_ + last_id_) % 2) == BLACK)
					{
						BField * big_tempDw_ = new BField(x_id_, last_id_, k*thickness_);//new a big grid only has x and y information

						std::set<BField*, compareBField>::iterator iter1 = big_fields_.find(big_tempDw_);
						if (iter1 == big_fields_.end())//if has not found a big grid
						{

							if (boundaryPD_.y() - big_tempDw_->bottomcoor_ > 1e-6&&boundaryPD_.y() - big_tempDw_->topcoor_ < -1e-6)
							{
								big_tempDw_->Black_hatch_point_.insert(boundaryPD_);
							}
							big_fields_.insert(big_tempDw_);

						}
						else
						{

							if (boundaryPD_.y() - big_tempDw_->bottomcoor_ > 1e-6&&boundaryPD_.y() - big_tempDw_->topcoor_ < -1e-6)
							{
								(*iter1)->Black_hatch_point_.insert(boundaryPD_);
							}
							delete big_tempDw_;

						}
					}
					boundaryPD_ = *iterB;
					if ((FieldType)(abs(x_id_ + cur_id_) % 2) == BLACK)
					{
						BField * big_tempUp_ = new BField(x_id_, cur_id_, k*thickness_);//new a big grid only has x and y information
						std::set<BField*, compareBField>::iterator iter2 = big_fields_.find(big_tempUp_);
						if (iter2 == big_fields_.end())//if has found  a big grid has exist
						{
							if (boundaryPD_.y() - big_tempUp_->bottomcoor_ > 1e-6&&boundaryPD_.y() - big_tempUp_->topcoor_ < -1e-6)
							{
								big_tempUp_->Black_hatch_point_.insert(boundaryPD_);
							}
							big_fields_.insert(big_tempUp_);

						}
						else/*(iter1 == big_fields_[k].end())*/
						{
							if (boundaryPD_.y() - big_tempUp_->bottomcoor_ > 1e-6&&boundaryPD_.y() - big_tempUp_->topcoor_ < -1e-6)
							{
								(*iter2)->Black_hatch_point_.insert(boundaryPD_);
							}
							delete big_tempUp_;
						}
					}

					for (int y_id_ = last_id_; y_id_ <= cur_id_; y_id_++)
					{
						if ((FieldType)(abs(x_id_ + y_id_) % 2) == BLACK)
						{
							Vec3f low_point_(x_cur_, y_id_*field_height_ + field_overlap_, k*thickness_);
							Vec3f high_point_(x_cur_, (y_id_ + 1)*field_height_ - field_overlap_, k*thickness_);
							BField * big_temp_ = new BField(x_id_, y_id_, k*thickness_);//new a big grid only has x and y information

							std::set<BField*, compareBField>::iterator iter3 = big_fields_.find(big_temp_);
							if (iter3 == big_fields_.end())//if has found a big grid has exist
							{
								if (low_point_.y() - y_last_ > 1e-6 &&low_point_.y() - y_cur_ < -1e-6)
								{
									big_temp_->Black_hatch_point_.insert(low_point_);
								}
								if (high_point_.y() - y_last_ > 1e-6 &&high_point_.y() - y_cur_ < -1e-6)
								{
									big_temp_->Black_hatch_point_.insert(high_point_);
								}
								big_fields_.insert(big_temp_);

							}
							else/*(iter1 == big_fields_[k].end())*/
							{
								if (low_point_.y() - y_last_ > 1e-6 &&low_point_.y() - y_cur_ < -1e-6)
								{
									(*iter3)->Black_hatch_point_.insert(low_point_);
								}
								if (high_point_.y() - y_last_ > 1e-6 &&high_point_.y() - y_cur_ < -1e-6)
								{
									(*iter3)->Black_hatch_point_.insert(high_point_);
								}
								delete big_temp_;

							}
						}
					}
				}
				else if (!atHeadPoint_)// ignore the small grid which has no crosspoint
				{
					iterB++;
					if (iterB != hatch_point_black_.end()) { atHeadPoint_ = true; }
				}
			}






		}	
	}
}