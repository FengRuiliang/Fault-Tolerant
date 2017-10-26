#include "Hatch.h"
#include "QDebug"
#include <fstream>
#include <QMatrix4x4>
#include <omp.h>
//#include <vld.h>
Hatch::~Hatch()
{
	
	delete[]offset_vert_;
	for (int i=0;i<num_pieces_;i++)
	{
		for (int j=0;j<hatch_[i].size();j++)
		{
			delete hatch_[i][j];
		}
		
	}
	delete[] hatch_;
	offset_vert_ = NULL;
	hatch_ = NULL;

}
Hatch::Hatch() {


};



Hatch::Hatch(SliceCut*parent)
{
	boudary_edge_ = parent->GetPieces();
	offset_vert_ = new std::vector<std::vector<Vec3f>>[parent->GetNumPieces()];
	hatch_ = new std::vector<Vec3f*>[parent->GetNumPieces()];
}

//void doHatch() {};

HatchChessboard::HatchChessboard(SliceCut*parent)
{
	boudary_edge_ = parent->GetPieces();
	offset_vert_ = new std::vector<std::vector<Vec3f>>[parent->GetNumPieces()];
	hatch_ = new std::vector<Vec3f *>[parent->GetNumPieces()];
	num_pieces_ = parent->GetNumPieces();
	minimal_field_size_ = 10;
	white_board_angle_ = 10;
	black_board_angle = 10;
	contour_ = 10;
}
HatchChessboard::HatchChessboard(SliceCut*slice_model,SliceCut*slice_support)
{
	boudary_edge_ = slice_model->GetPieces();
	auto temp = slice_support->GetPieces();
	for (int i=0;i<slice_support->GetNumPieces();i++)
	{
		boudary_edge_[i].insert(boudary_edge_[i].end(), temp[i].begin(), temp[i].end());
	}
	offset_vert_ = new std::vector<std::vector<Vec3f>>[slice_model->GetNumPieces()];
	hatch_ = new std::vector<Vec3f *>[slice_model->GetNumPieces()];
	//hatch_type_ = NONE;
	num_pieces_ = slice_model->GetNumPieces();
	//offset_dis_ = 0;
	//increment_angle_ = 0.5;

	minimal_field_size_ = 10;
	white_board_angle_ = 10;
	black_board_angle = 10;
	contour_ = 10;

	//	offset_vert_rotate_ = NULL;
}

HatchChessboard::~HatchChessboard()
{
	boudary_edge_ = NULL;
}


void Hatch::clearHatch()
{
	delete[]offset_vert_;
	for (int i = 0; i < num_pieces_; i++)
	{
		for (int j = 0; j < hatch_[i].size(); j++)
		{
			delete hatch_[i][j];
		}

	}
	delete[] hatch_;
}

void Hatch::setLaserPower(float power)
{
	laser_power_hatch_ = power;
}

void Hatch::setLaserSpeed(float speed)
{
	laser_speed_hatch_ = speed;
}
void HatchChessboard::clearHatch()
{
	boudary_edge_ = NULL;
	delete[]offset_vert_;
	delete[]hatch_;


}
void HatchMeander::clearHatch()
{
	delete[]offset_vert_;
	delete[] hatch_;
	delete[]num_hatch;
	delete[]hatch_point_black_;
	delete[]hatch_point_white_;
}

void HatchChessboard::doHatch()
{

#pragma  omp parallel for
	for (int k = 0; k < num_pieces_; k++)
	{
		int angle_ = 0;
		QMatrix4x4 matrix_;
		QVector3D rotationAxis(0.0, 0.0, 1.0);
		//offset function
		//////////////////////////////////////////////////////////////////////////
		int i = k;
		//qDebug() << "k==" << k;
		for (int j = 0; j < boudary_edge_[i].size(); j++)
		{
			std::vector<Vec3f> Bpoints;
			std::vector<std::vector<Vec3f>> OFFPoints;
			for (int k = 0; k < (boudary_edge_[i])[j]->size(); k++)
			{
				Bpoints.push_back((boudary_edge_[i])[j]->at(k).position_vert[0]);
			}
			if (Bpoints.size()<3)
			{
				continue;
			}
			Offset(Bpoints, OFFPoints, offset_dis_,i);
			offset_vert_[i].insert(offset_vert_[i].end(), OFFPoints.begin(), OFFPoints.end());
		}

		//co.AddPaths(subj, jtMiter, etClosedPolygon);
		//co.Execute(solution, -offset_dis_*1e6);
// 		std::vector<Vec3f>* com_offset_vert_ = new std::vector<Vec3f>[solution.size()];
// 		for (int l = 0; l < solution.size(); l++)
// 		{
// 			for (int m = 0; m < solution[l].size(); m++)
// 			{
// 				Vec3f p(solution[l][m].X*1e-6, solution[l][m].Y*1e-6, i*thickness_);
// 				(com_offset_vert_[l]).push_back(p);
// 			}
// 			offset_vert_[i].push_back(&(com_offset_vert_[l]));//& operator
// 		}
		
		//////////////////////////////////////////////////////////////////////////
		//store cross point function
		//////////////////////////////////////////////////////////////////////////

		angle_ = i*increment_angle_;

		angle_ = angle_ % 360;

		matrix_.setToIdentity();

		matrix_.rotate((float)angle_, rotationAxis);

		std::multiset<Vec3f, comVec3fBlack> hatch_point_black_;
		std::multiset<Vec3f, comVec3fWhite> hatch_point_white_;
		for (int j = 0; j < offset_vert_[i].size(); j++)
		{
			for (int k = 0; k < (offset_vert_[i])[j].size(); k++)
			{

				Vec3f a, b, vector_;
				if (k == (offset_vert_[i])[j].size() - 1)
				{
					a = (((offset_vert_[i])[j]))[k];
					b = (((offset_vert_[i])[j]))[0];
				}
				else
				{
					a = (((offset_vert_[i])[j]))[k];
					b = (((offset_vert_[i])[j]))[k + 1];
				}

				QVector4D position_(a.x(), a.y(), a.z(), 1.0);
				position_ = matrix_*position_;
				a.x() = position_.x();
				a.y() = position_.y();
				a.z() = position_.z();
				position_[0] = b.x();
				position_[1] = b.y();
				position_[2] = b.z();
				position_[3] = 1.0;
				position_ = matrix_*position_;
				b.x() = position_.x();
				b.y() = position_.y();
				b.z() = position_.z();

				float x_min_ = a.x() <= b.x() ? a.x() : b.x();
				float x_max_ = a.x() >= b.x() ? a.x() : b.x();
				float y_min_ = a.y() <= b.y() ? a.y() : b.y();
				float y_max_ = a.y() >= b.y() ? a.y() : b.y();

				int min_wid_id_ = round((x_min_ + field_width_ * 10000) / field_width_ - 0.5) - 10000;
				int max_wid_id_ = round((x_max_ + field_width_ * 10000) / field_width_ - 0.5) - 10000;
				int min_hei_id_ = round((y_min_ + field_height_ * 10000) / field_height_ - 0.5) - 10000;
				int max_hei_id_ = round((y_max_ + field_height_ * 10000) / field_height_ - 0.5) - 10000;
				vector_ = b - a;
				for (int x_id_ = min_wid_id_; x_id_ <= max_wid_id_; x_id_++)
				{
					if (vector_.x() == 0) { continue; }
					if (x_id_*field_width_ == x_max_) { continue; }
					else
					{

						//for every field, compute hatch line coordinate
						float mid_hat_coor_ = (x_id_ + 0.5)*field_width_;
						for (int id_hatch_ = 0; id_hatch_*line_width_ < field_width_ / 2; id_hatch_++)
						{
							float x_coor_ = id_hatch_*line_width_ + mid_hat_coor_;

							if (x_coor_ < x_max_&&x_coor_ >= x_min_)
							{
								Vec3f point_p_ = a + (id_hatch_*line_width_ + mid_hat_coor_ - a.x()) / vector_.x()*vector_;
								hatch_point_black_.insert(point_p_);

							}
							if (id_hatch_ == 0)
							{
								continue;
							}
							x_coor_ = -id_hatch_*line_width_ + mid_hat_coor_;

							if (x_coor_ < x_max_&&x_coor_ >= x_min_)//pay attention at the condition that point at the endpoint,may has different y coordinate at the same location;
							{
								Vec3f point_n_ = a + (-id_hatch_*line_width_ + mid_hat_coor_ - a.x()) / vector_.x()*vector_;
								hatch_point_black_.insert(point_n_);
							}
						}
					}
				}
				for (int y_id_ = min_hei_id_; y_id_ <= max_hei_id_; y_id_++)
				{
					if (vector_.y() == 0) { continue; }
					if (y_id_*field_height_ == y_max_) { continue; }
					else
					{
						//for every field, compute hatch line coordinate
						float mid_hat_coor_ = (y_id_ + 0.5)*field_height_;
						for (int id_hatch_ = 0; id_hatch_*line_width_ < field_height_ / 2; id_hatch_++)
						{
							float y_coor_ = id_hatch_*line_width_ + mid_hat_coor_;
							if (y_coor_ < y_max_&&y_coor_ >= y_min_)
							{
								Vec3f point_p_ = a + (id_hatch_*line_width_ + mid_hat_coor_ - a.y()) / vector_.y()*vector_;
								hatch_point_white_.insert(point_p_);

							}
							if (id_hatch_ == 0)
							{
								continue;
							}
							y_coor_ = -id_hatch_*line_width_ + mid_hat_coor_;
							if (y_coor_ < y_max_&&y_coor_ >= y_min_)
							{
								Vec3f point_n_ = a + (-id_hatch_*line_width_ + mid_hat_coor_ - a.y()) / vector_.y()*vector_;
								hatch_point_white_.insert(point_n_);
							}
						}
					}
				}
			}
		}
		//////////////////////////////////////////////////////////////////////////
		//store Black field function
		//////////////////////////////////////////////////////////////////////////
		std::set<BField*, compareBField>  big_fields_;
		auto iterB = hatch_point_black_.begin();
		if (iterB == hatch_point_black_.end())continue;;
		float x_cur_, y_cur_, x_last_, y_last_, y_last_field_, y_cur_field_;
		x_cur_ = (*iterB).x();//used in small grid
		x_last_ = (*iterB).x();
		bool atHeadPoint_ = true;
		for (; iterB != hatch_point_black_.end(); )
		{
			y_last_ = (*iterB).y();
			y_cur_ = (*iterB).y();
			x_cur_ = (*iterB).x();

			if (abs(x_cur_ - x_last_) > 1e-3)// means that we are visiting another line;
			{
				x_last_ = x_cur_;
				atHeadPoint_ = true;
			}

			if (iterB == hatch_point_black_.end()) { break; }// ignore the small grid which has no crosspoint

			else if (atHeadPoint_)
			{
				Vec3f boundaryPD_ = *iterB;
				iterB++;//point to next grid
				if (iterB == hatch_point_black_.end())continue;
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
				int last_id_ = round((y_min_ + field_height_ * 10000) / field_height_ - 0.5) - 10000;
				int cur_id_ = round((y_max_ + field_height_ * 10000) / field_height_ - 0.5) - 10000;
				int x_id_ = round((x_cur_ + field_width_ * 10000) / field_width_ - 0.5) - 10000;
				if (last_id_*field_height_ == y_max_)
				{
					last_id_--;
				}
				if (cur_id_*field_height_ == y_max_)
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
		//////////////////////////////////////////////////////////////////////////
		//store white field function
		//////////////////////////////////////////////////////////////////////////
		auto iterW = hatch_point_white_.begin();
		if (iterW == hatch_point_white_.end())continue;
		y_cur_ = (*iterW).y();
		y_last_ = (*iterW).y();
		atHeadPoint_ = true;
		for (; iterW != hatch_point_white_.end(); )
		{
			y_cur_ = (*iterW).y();
			x_cur_ = (*iterW).x();
			x_last_ = (*iterW).x();

			if (abs(y_cur_ - y_last_) > 1e-3)// means that we are visiting another line;
			{
				y_last_ = y_cur_;
				atHeadPoint_ = true;
			}

			if (iterW == hatch_point_white_.end()) { break; }// ignore the small grid which has no crosspoint

			else if (atHeadPoint_)
			{
				Vec3f boundaryPD_ = *iterW;

				iterW++;//point to next grid
				if (iterW == hatch_point_white_.end())continue;
				atHeadPoint_ = false;
				x_cur_ = (*iterW).x();
				float x_max_, x_min_;//modify at 2017/3/2
				if (x_cur_ - x_last_ > 1e-3)
				{
					x_max_ = x_cur_;
					x_min_ = x_last_;
				}
				else
				{
					x_max_ = x_last_;
					x_min_ = x_cur_;
				}
				int last_id_ = round((x_min_ + field_width_ * 10000) / field_width_ - 0.5) - 10000;
				int cur_id_ = round((x_max_ + field_width_ * 10000) / field_width_ - 0.5) - 10000;
				if (last_id_*field_width_ == x_max_)
				{
					last_id_--;
				}
				if (cur_id_*field_width_ == x_max_)
				{
					cur_id_--;
				}
				int y_id_ = round((y_cur_ + field_height_ * 10000) / field_height_ - 0.5) - 10000;

				if ((FieldType)(abs(last_id_ + y_id_) % 2) == WHITE)
				{
					BField * big_tempLf_ = new BField(last_id_, y_id_, k*thickness_);
					std::set<BField*, compareBField>::iterator iter1 = big_fields_.find(big_tempLf_);
					if (iter1 == big_fields_.end())//if has found a big grid has exist
					{
						if (boundaryPD_.x() - big_tempLf_->leftcoor_ > 1e-6&&boundaryPD_.x() - big_tempLf_->rightcoot_ < -1e-6)
						{
							big_tempLf_->white_hatch_point_.insert(boundaryPD_);
						}

						big_fields_.insert(big_tempLf_);

					}
					else/*(iter1 == big_fields_[k].end())*/
					{
						if (boundaryPD_.x() - big_tempLf_->leftcoor_ > 1e-6&&boundaryPD_.x() - big_tempLf_->rightcoot_ < -1e-6)
						{
							(*iter1)->white_hatch_point_.insert(boundaryPD_);
						}
						delete big_tempLf_;
					}
				}
				boundaryPD_ = *iterW;
				if ((FieldType)(abs(cur_id_ + y_id_) % 2) == WHITE)
				{	
					BField * big_tempRt_ = new BField(cur_id_, y_id_, k*thickness_);//new a big grid only has x and y information
					std::set<BField*, compareBField>::iterator iter2 = big_fields_.find(big_tempRt_);
					if (iter2 == big_fields_.end())//if has found a big grid has exist
					{
						if (boundaryPD_.x() - big_tempRt_->leftcoor_ > 1e-6 &&boundaryPD_.x() - big_tempRt_->rightcoot_ < -1e-6)
						{
							big_tempRt_->white_hatch_point_.insert(boundaryPD_);
						}
						big_fields_.insert(big_tempRt_);
					}
					else/*(iter1 == big_fields_[k].end())*/
					{
						 if (boundaryPD_.x() - big_tempRt_->leftcoor_ > 1e-6 &&boundaryPD_.x() - big_tempRt_->rightcoot_ < -1e-6)
						{
							(*iter2)->white_hatch_point_.insert(boundaryPD_);
						}
						delete big_tempRt_;
					}
				}

				for (int x_id_ = last_id_; x_id_ <= cur_id_; x_id_++)
				{
					if ((FieldType)(abs(x_id_ + y_id_) % 2) == WHITE)
					{
						Vec3f left_point_(x_id_*field_width_ + field_overlap_, y_cur_, k*thickness_);
						Vec3f right_point_((x_id_ + 1)*field_width_ - field_overlap_, y_cur_, k*thickness_);
						BField * big_temp_ = new BField(x_id_, y_id_, k*thickness_);//new a big grid only has x and y information
						std::set<BField*, compareBField>::iterator iter3 = big_fields_.find(big_temp_);
						if (iter3 == big_fields_.end())
						{

							if (left_point_.x() - x_last_ > 1e-6 &&left_point_.x() - x_cur_ < -1e-6)
							{
								big_temp_->white_hatch_point_.insert(left_point_);
							}
							if (right_point_.x() - x_last_ > 1e-6 &&right_point_.x() - x_cur_ < -1e-6)
							{
								big_temp_->white_hatch_point_.insert(right_point_);
							}
							big_fields_.insert(big_temp_);
						}
						else
						{

							 if (left_point_.x() - x_last_ > 1e-6&&left_point_.x() - x_cur_ < -1e-6)
							{
								(*iter3)->white_hatch_point_.insert(left_point_);
							}
							if (right_point_.x() - x_last_ > 1e-6 &&right_point_.x() - x_cur_ < -1e-6)
							{
								(*iter3)->white_hatch_point_.insert(right_point_);
							}
							delete big_temp_;
						}
					}
				}
			}
			else if (!atHeadPoint_)// ignore the small grid which has no crosspoint
			{
				iterW++;
				if (iterW != hatch_point_white_.end()) { atHeadPoint_ = true; }
			}
		}
		//////////////////////////////////////////////////////////////////////////
		//sort hatch line function
		//////////////////////////////////////////////////////////////////////////
		for (auto iterField_ = big_fields_.begin(); iterField_ != big_fields_.end(); iterField_++)
		{
			for (auto iterPoint_ = (*iterField_)->Black_hatch_point_.begin(); iterPoint_ != (*iterField_)->Black_hatch_point_.end();)
			{

				Vec3f* temp = new Vec3f[2];
				temp[0] = *iterPoint_;
				iterPoint_++;
				if (iterPoint_ == (*iterField_)->Black_hatch_point_.end())
				{
					continue;
				}
				temp[1] = *iterPoint_;
				if (abs(temp[0].x() - temp[1].x()) < 1e-5)
				{
					(*iterField_)->hatch_line_.push_back(temp);
					iterPoint_++;
				}
			}
			for (auto iterPoint_ = (*iterField_)->white_hatch_point_.begin(); iterPoint_ != (*iterField_)->white_hatch_point_.end(); )
			{
				Vec3f* temp = new Vec3f[2];

				temp[0] = *iterPoint_;
				iterPoint_++;
				if (iterPoint_ == (*iterField_)->white_hatch_point_.end())
				{
					continue;
				}
				temp[1] = *iterPoint_;
				if (abs(temp[0].y() - temp[1].y()) < 1e-5)
				{
					(*iterField_)->hatch_line_.push_back(temp);
					iterPoint_++;
				}
			}
		}
		//////////////////////////////////////////////////////////////////////////
		//store hatch funtion
		//////////////////////////////////////////////////////////////////////////

		if (big_fields_.size() == 0)continue;
		int x_min_ = (*(big_fields_.begin()))->x_min_field_;
		int x_max_ = (*(big_fields_.rbegin()))->x_min_field_;

		auto iter_black_ = big_fields_.begin();
		auto iter_white_ = big_fields_.begin();
		if (iter_black_ == big_fields_.end())continue;
		if (iter_white_ == big_fields_.end())continue;

		for (int x = x_min_; x <= x_max_; x++)
		{
			for (; iter_black_ != big_fields_.end(); iter_black_++)
			{
				if ((*iter_black_)->x_min_field_ != x)
				{
					break;
				}
				else if ((*iter_black_)->getFieldType() == BLACK)
				{
					for (auto iter_line_black_ = (*iter_black_)->hatch_line_.begin(); iter_line_black_ != (*iter_black_)->hatch_line_.end(); iter_line_black_++)
					{
						hatch_[k].push_back(*iter_line_black_);
					}
				}
			}
			for (; iter_white_ != big_fields_.end(); iter_white_++)
			{
				if ((*iter_white_)->x_min_field_ != x)
				{
					break;
				}
				else if ((*iter_white_)->getFieldType() == WHITE)
				{
					for (auto iter_line_white_ = (*iter_white_)->hatch_line_.begin(); iter_line_white_ != (*iter_white_)->hatch_line_.end(); iter_line_white_++)
					{
						hatch_[k].push_back(*iter_line_white_);
					}
				}
			}
		}
		//////////////////////////////////////////////////////////////////////////
		//rotate backe
		//////////////////////////////////////////////////////////////////////////
		//angle_ = i*increment_angle_;
		//angle_ = angle_ % 360;
		matrix_.setToIdentity();
		matrix_.rotate((float)-angle_, rotationAxis);
		for (auto iterline = hatch_[i].begin(); iterline != hatch_[i].end(); iterline++)
		{
			//qDebug() << (*iterline)[0].x()<<(*iterline)[0].y() <<(*iterline)[0].z();
			QVector4D position_((*iterline)[0].x(), (*iterline)[0].y(), (*iterline)[0].z(), 1.0);
			position_ = matrix_*position_;
			(*iterline)[0].x() = position_.x();
			(*iterline)[0].y() = position_.y();
			(*iterline)[0].z() = position_.z();
			//	qDebug() << (*iterline)[0].x() << (*iterline)[0].y() << (*iterline)[0].z();
			//	qDebug() << (*iterline)[1].x() << (*iterline)[1].y() << (*iterline)[1].z();
			position_[0] = (*iterline)[1].x();
			position_[1] = (*iterline)[1].y();
			position_[2] = (*iterline)[1].z();
			position_[3] = 1.0;
			position_ = matrix_*position_;
			(*iterline)[1].x() = position_.x();
			(*iterline)[1].y() = position_.y();
			(*iterline)[1].z() = position_.z();
			//	qDebug() << (*iterline)[1].x() << (*iterline)[1].y() << (*iterline)[1].z();
		}
		//////////////////////////////////////////////////////////////////////////
		//clear this layer data
		//////////////////////////////////////////////////////////////////////////
		for (auto iterF = big_fields_.begin(); iterF != big_fields_.end(); iterF++)
		{
			delete (*iterF);
		}
		//////////////////////////////////////////////////////////////////////////
	}
}

bool Hatch::Offset(std::vector<Vec3f>& outer_, std::vector <std::vector<Vec3f>>& inner_, float offset_dist,int thick_id_)
{




	Path subj;
	ClipperOffset co;
	Paths solution;
	std::vector<HE_edge*> sweep_he_;
	std::vector<HE_vert*> sweep_hv_;
	std::multiset <HE_vert*, comVertex> order_point_;
	std::vector<Vec3f> com_vert_;
	for (size_t j = 0; j < outer_.size(); j++)
	{

		int X_ = outer_[j].x()*1e6;
		int Y_ = outer_[j].y()*1e6;

		subj << IntPoint(X_, Y_);

		HE_vert* v = new HE_vert(outer_[j]);
		HE_edge* e = new HE_edge();
		e->pvert_ = v;
		if (sweep_hv_.size() != 0)
		{
			e->start_ = sweep_hv_.back();
			sweep_hv_.back()->pedge_ = e;
			sweep_hv_.back()->pnext = v;
			v->pprev = sweep_hv_.back();
		}
		order_point_.insert(v);
		sweep_hv_.push_back(v);
		sweep_he_.push_back(e);
	}
	sweep_he_.front()->start_ = sweep_hv_.back();
	sweep_hv_.back()->pedge_ = sweep_he_.front();
	sweep_hv_.back()->pnext = sweep_hv_.front();
	sweep_hv_.front()->pprev = sweep_hv_.back();

	Vec3f p1, p2, p3, l1, l2;
	p1 = (*order_point_.begin())->position();
	p2 = (*order_point_.begin())->pprev->position();
	p3 = (*order_point_.begin())->pnext->position();

	int D_ = 0;
	l1 = p3 - p2;
	l2 = p1 - p2;
	if (l1.cross(l2).z() < 0)
	{
		D_ = -1;// ccw
	}
	else
	{
		D_ = 1;//cw
	}
	for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
	{
		delete *iterE;
	}
	for (auto iterV = sweep_hv_.begin(); iterV != sweep_hv_.end(); iterV++)
	{
		delete *iterV;
	}
	co.AddPath(subj, jtMiter, etClosedPolygon);
	co.Execute(solution, D_*(offset_dist)*1e6);
	order_point_.clear();
	for (int m = 0; m < solution.size(); m++)
	{
		std::vector<Vec3f> com_offset_vert_;
		for (int n = 0; n < solution[m].size(); n++)
		{
			Vec3f p(solution[m][n].X*1e-6, solution[m][n].Y*1e-6, thick_id_*thickness_);
			com_offset_vert_.push_back(p);
		}
		inner_.push_back(com_offset_vert_);//& operator
	}
	sweep_he_.clear();
	sweep_hv_.clear();
	for (int m = 0; m < inner_.size(); m++)
	{
		for (std::vector<Vec3f>::iterator iterInner = inner_[m].begin(); iterInner != inner_[m].end(); iterInner++)
		{

			HE_vert* v = new HE_vert((*iterInner));
			HE_edge* e = new HE_edge();
			e->pvert_ = v;
			if (sweep_hv_.size() != 0)
			{
				e->start_ = sweep_hv_.back();
				sweep_hv_.back()->pnext = v;
				v->pprev = sweep_hv_.back();
			}
			order_point_.insert(v);
			sweep_hv_.push_back(v);
			sweep_he_.push_back(e);
		}
		sweep_he_.front()->start_ = sweep_hv_.back();
		sweep_hv_.back()->pnext = sweep_hv_.front();
		sweep_hv_.front()->pprev = sweep_hv_.back();

		Vec3f p1, p2, p3, l1, l2;
		p1 = (*order_point_.begin())->position();
		p2 = (*order_point_.begin())->pprev->position();
		p3 = (*order_point_.begin())->pnext->position();
		l1 = p3 - p2;
		l2 = p1 - p2;

		if (l1.cross(l2).z()*D_ < 0)//同向
		{
			inner_[m].clear();
			for (int n = solution[m].size() - 1; n >= 0; n--)
			{
				Vec3f p(solution[m][n].X*1e-6, solution[m][n].Y*1e-6, thick_id_*thickness_);
				inner_[m].push_back(p);
			}
		}
	}
	for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
	{
		delete *iterE;
	}
	for (auto iterV = sweep_hv_.begin(); iterV != sweep_hv_.end(); iterV++)
	{
		delete *iterV;
	}
	return true;
}

void Hatch::rotateBack(size_t k)
{
	QMatrix4x4 matrix_;
	QVector3D rotationAxis(0.0, 0.0, 1.0);

	size_t i = k;
	int angle_ = 0;
	std::vector<Vec3f*>* tc_hatch_ = getHatch();
	angle_ = i*increment_angle_;
	//qDebug() << increment_angle_;
	angle_ = angle_ % 360;
	matrix_.setToIdentity();
	//qDebug() << (float)-angle_;
	matrix_.rotate((float)-angle_, rotationAxis);
	for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
	{
		//qDebug() << (*iterline)[0].x()<<(*iterline)[0].y() <<(*iterline)[0].z();
		QVector4D position_((*iterline)[0].x(), (*iterline)[0].y(), (*iterline)[0].z(), 1.0);
		position_ = matrix_*position_;
		(*iterline)[0].x() = position_.x();
		(*iterline)[0].y() = position_.y();
		(*iterline)[0].z() = position_.z();
		//	qDebug() << (*iterline)[0].x() << (*iterline)[0].y() << (*iterline)[0].z();
		//	qDebug() << (*iterline)[1].x() << (*iterline)[1].y() << (*iterline)[1].z();
		position_[0] = (*iterline)[1].x();
		position_[1] = (*iterline)[1].y();
		position_[2] = (*iterline)[1].z();
		position_[3] = 1.0;
		position_ = matrix_*position_;
		(*iterline)[1].x() = position_.x();
		(*iterline)[1].y() = position_.y();
		(*iterline)[1].z() = position_.z();
		//	qDebug() << (*iterline)[1].x() << (*iterline)[1].y() << (*iterline)[1].z();
	}
}
void Hatch::doHatch()
{
}

HatchMeander::HatchMeander(SliceCut*parent)
{
	boudary_edge_ = parent->GetPieces();
	offset_vert_ = new std::vector<std::vector<Vec3f>>[parent->GetNumPieces()];
	hatch_ = new std::vector<Vec3f *>[parent->GetNumPieces()];
	thickness_ = parent->getThickness();
	num_pieces_ = parent->GetNumPieces();
	contour_ = 10;
	hatch_point_black_ = new std::multiset<Vec3f, comVec3fBlack>[parent->GetNumPieces()];
	hatch_point_white_ = new std::multiset<Vec3f, comVec3fWhite>[parent->GetNumPieces()];
}

inline HatchMeander::~HatchMeander() 
{
	delete []hatch_point_black_;
	hatch_point_black_ = NULL;
	delete[]hatch_point_white_;
	hatch_point_white_ = NULL;
	
}

void HatchMeander::storeCrossPoint()
{
	std::vector < std::vector<Vec3f>>* offset_boundary_ = getOffsetVertex();
#pragma  omp parallel for
	for (int i = 0; i < num_pieces_; i++)
	{
		for (int j = 0; j < offset_boundary_[i].size(); j++)
		{
			for (int k = 0; k < (offset_boundary_[i])[j].size(); k++)
			{

				Vec3f a, b, vector_;
				if (k == (offset_boundary_[i])[j].size() - 1)
				{
					a = (((offset_boundary_[i])[j]))[k];
					b = (((offset_boundary_[i])[j]))[0];
				}
				else
				{
					a = (((offset_boundary_[i])[j]))[k];
					b = (((offset_boundary_[i])[j]))[k + 1];
				}
				float x_min_ = a.x() <= b.x() ? a.x() : b.x();
				float x_max_ = a.x() >= b.x() ? a.x() : b.x();
				float y_min_ = a.y() <= b.y() ? a.y() : b.y();
				float y_max_ = a.y() >= b.y() ? a.y() : b.y();

				int min_wid_id_ = round((x_min_ + line_width_ * 10000) / line_width_ - 0.5) - 10000;
				int max_wid_id_ = round((x_max_ + line_width_ * 10000) / line_width_ - 0.5) - 10000;
				int min_hei_id_ = round((y_min_ + line_width_ * 10000) / line_width_ - 0.5) - 10000;
				int max_hei_id_ = round((y_max_ + line_width_ * 10000) / line_width_ - 0.5) - 10000;
				vector_ = b - a;
				if (vector_.x() != 0)
				{
					for (int x_id_ = min_wid_id_ + 1; x_id_ <= max_wid_id_; x_id_++)
					{
						Vec3f point_p_ = a + (x_id_*line_width_ - a.x()) / vector_.x()*vector_;
						hatch_point_black_[i].insert(point_p_);
					}
				}
			}
		}
	}
}

void HatchMeander::storeHatchLine()
{
#pragma  omp parallel for
	for (int k = 0; k < num_pieces_; k++)
	{
		if (hatch_point_black_[k].size() == 0)continue;
		auto iter = hatch_point_black_[k].begin();
		if (iter == hatch_point_black_[k].end())continue;
		float x_cur_, y_cur_, x_last_, y_last_, y_last_field_, y_cur_field_;
		x_cur_ = (*iter).x();//used in small grid
		x_last_ = (*iter).x();
		bool atHeadPoint_ = true;
		for (; iter != hatch_point_black_[k].end(); )
		{
			y_last_ = (*iter).y();
			y_cur_ = (*iter).y();
			x_cur_ = (*iter).x();

			if (abs(x_cur_ - x_last_) > 1e-3)// means that we are visiting another line;
			{
				x_last_ = x_cur_;
				atHeadPoint_ = true;
			}
			if (iter == hatch_point_black_[k].end()) { break; }// ignore the small grid which has no crosspoint

			else if (atHeadPoint_)
			{
				atHeadPoint_ = false;
				Vec3f* temp = new Vec3f[2];
				temp[0] = *iter;
				iter++;
				temp[1] = *iter;
				hatch_[k].push_back(temp);

			}
			else if (!atHeadPoint_)// ignore the small grid which has no crosspoint
			{
				iter++;
				if (iter != hatch_point_black_[k].end()) { atHeadPoint_ = true; }
			}
		}
	}

}

void HatchMeander::doHatch()
{
	std::vector < std::vector<Vec3f>>* offset_boundary_ = getOffsetVertex();
#pragma  omp parallel for
	for (int i = 0; i < num_pieces_; i++)
	{
		//offset
		for (int j = 0; j < boudary_edge_[i].size(); j++)
		{
			std::vector<Vec3f> Bpoints;
			std::vector<std::vector<Vec3f>> OFFPoints;
			for (int k = 0; k < (boudary_edge_[i])[j]->size(); k++)
			{
				Bpoints.push_back((boudary_edge_[i])[j]->at(k).position_vert[0]);
			}
			if (Bpoints.size() < 3)
			{
				continue;
			}
			Offset(Bpoints, OFFPoints, offset_dis_, i);
			offset_vert_[i].insert(offset_vert_[i].end(), OFFPoints.begin(), OFFPoints.end());
		}
	}
	storeCrossPoint();
	storeHatchLine();
}

void HatchOffsetfilling::doHatch()
{
	//std::vector < std::vector<Vec3f>>* offset_boundary_ = getOffsetVertex();
#pragma  omp parallel for
	for (int i = 0; i < num_pieces_; i++)
	{
		//offset
		for (int j = 0; j < boudary_edge_[i].size(); j++)
		{
			std::vector<Vec3f> Bpoints;
			std::vector<std::vector<Vec3f>> OFFPoints;
			for (int k = 0; k < (boudary_edge_[i])[j]->size(); k++)
			{
				Bpoints.push_back((boudary_edge_[i])[j]->at(k).position_vert[0]);
			}
			if (Bpoints.size() < 3)
			{
				continue;
			}
			Offset(Bpoints, OFFPoints, offset_dis_, i);
			offset_vert_[i].insert(offset_vert_[i].end(), OFFPoints.begin(), OFFPoints.end());
		}
	}
#pragma  omp parallel for
	for (int i=0;i<num_pieces_;i++)
	{
		ClipperOffset co;
			Paths paths;
			Paths solution;
		for (int j = 0; j < offset_vert_[i].size(); j++)
		{
				Path subj;
			std::vector<std::vector<Vec3f>> out_;
			for (int k = 0; k < (offset_vert_[i])[j].size(); k++)
			{
				subj<<IntPoint(offset_vert_[i][j][k].x()*1e6,
					offset_vert_[i][j][k].y()*1e6);
			}
			paths.push_back(subj);
		}
		int j = 0;
		do 
		{
			solution.clear();
			co.Clear();
			co.AddPaths(paths, jtMiter, etClosedPolygon);
			co.Execute(solution, line_width_*-1e6);
			for (int m = 0; m < solution.size(); m++)
			{
				for (int n = 0; n < solution[m].size(); n++)
				{
					Vec3f* points = new Vec3f[2];
					points[0] = Vec3f(solution[m][n].X*1e-6, solution[m][n].Y*1e-6, i*thickness_);
					points[1] = Vec3f(solution[m][(n + 1) % solution[m].size()].X*1e-6,
						solution[m][(n + 1) % solution[m].size()].Y*1e-6, i*thickness_);
					hatch_[i].push_back(points);
				}
			}	
			paths = solution;
		} while (paths.size()!=0);
	}
}

HatchOffsetfilling::HatchOffsetfilling(SliceCut * inSlice_)
{

	boudary_edge_ = inSlice_->GetPieces();
	offset_vert_ = new std::vector<std::vector<Vec3f>>[inSlice_->GetNumPieces()];
	hatch_ = new std::vector<Vec3f *>[inSlice_->GetNumPieces()];
	num_pieces_ = inSlice_->GetNumPieces();
}

void HatchOffsetfilling::clearHatch()
{
	boudary_edge_ = NULL;
	delete[]offset_vert_;
	delete[]hatch_;
}
