#include "Sweepline.h"
#include <qdebug.h>
SweepLine::SweepLine(std::vector<cutLine> P)
{
	for (int i=0;i<P.size();i++)
	{
		Event* v1 = new Event(P[i].position_vert[0]);
		Event* v2 = new Event(P[i].position_vert[1]);
		v1->epair_ = v2;
		v2->epair_ = v1;
		Segment* seg = new Segment(v1,v2);
		v1->segment_ = seg;
		v2->segment_ = seg;
		event_list_.push_back(v1);
		event_list_.push_back(v2);
		que_event_.push_back(v1);
		que_event_.push_back(v2);
		segment_list_.push_back(seg);
	}
	std::sort(que_event_.begin(),que_event_.end(), compareEvent);
}

SweepLine::~SweepLine()
{
	for (int i=0;i<event_list_.size();i++)
	{
		delete event_list_.at(i);
	}
	for (int i = 0; i < segment_list_.size(); i++)
	{
		delete segment_list_.at(i);
	}
	for (auto iter=points_.begin();iter!=points_.end();iter++)
	{
		delete *iter;
	}
	segment_list_.clear();
	event_list_.clear();
	points_.clear();
}

void SweepLine::getContuor(std::map<float, std::vector<std::vector<cutLine>>>& layer_edge_)
{
	for (int i=0;i<segment_list_.size();i++)
	{
		if (!segment_list_[i]->visited_)
		{
			std::vector<cutLine> polygon_;
			Segment* sta_ = segment_list_[i];
			Segment* cur_ = sta_;
			//qDebug() << i;
			int n = 0;
			do 
			{
				cur_->visited_ = true;
				polygon_.push_back(cutLine(cur_->first_->position_, cur_->second_->position_));
				cur_ = cur_->next_seg_;
			} while (cur_ != NULL&&cur_ != sta_&&n++<1000);
			layer_edge_[sta_->first_->position_.z()].push_back(polygon_);
		}
	}
}

// void SweepLine::polygonization()
// {
// 	//////////////////////////////////////////////////////////////////////////
// 	while (!que_event_.empty())
// 	{
// 		std::vector<Segment*> segs_;
// 		Vec3f pos_ = que_event_.back()->position_;
// 		do
// 		{
// 			Event* cur_event_ = que_event_.back();
// 			Segment* cur_seg_ = cur_event_->segment_;
// 	
// 			for (auto iter = segs_.begin(); iter != segs_.end(); iter++)
// 			{
// 				if (((*iter)->second_->position_ - cur_event_->position_).length() < 1e-3)
// 				{
// 					(*iter)->next_seg_ = cur_seg_;
// 				}
// 				else if (((*iter)->first_->position_ - cur_event_->position_).length() < 1e-3)
// 				{
// 					cur_seg_->next_seg_ = (*iter);
// 				}
// 			}
// 			segs_.push_back(que_event_.back()->segment_);
// 			que_event_.pop_back();
// 		} while (!que_event_.empty() && (que_event_.back()->position_ - pos_).length() < 1e-3);
// 	}
// 	return;
// 	//////////////////////////////////////////////////////////////////////////
// 	while (!que_event_.empty())
// 	{
// 		Vec3f pos_ = que_event_.back()->position_;
// 		do 
// 		{
// 			Vec3f	angle_ = que_event_.back()->epair_->position_ - pos_;
// 			angle_.normalize();
// 			if (angle_.y()>=0)
// 				que_event_.back()->segment_->angle_=acos(angle_.dot(Vec3f(-1.0, 0.0, 0.0)));
// 			else
// 				que_event_.back()->segment_->angle_ = 6.28-acos(angle_.dot(Vec3f(1.0, 0.0, 0.0)));
// 			qsegment_.push_back(que_event_.back()->segment_);
// 			que_event_.pop_back();
// 		} while (!que_event_.empty()&&(que_event_.back()->position_-pos_).length()<1e-3);
// 		//qDebug() << que_event_.size()<<qsegment_.size();
// 		std::sort(qsegment_.begin(), qsegment_.end(), compSegment);
// 		int n = qsegment_.size();
// 		qDebug() << n;
// 		for (int i=0;i<qsegment_.size();i++)
// 		{
// 			if ((qsegment_[i]->first_->position_-qsegment_[(i+1)%n]->second_->position_).length()<1e-3)
// 			{
// 				qsegment_[i]->prev_seg_ = qsegment_[(i + 1) % n];
// 				qsegment_[(i + 1) % n]->next_seg_=qsegment_[i];
// 			}
// 			else
// 			{
// 				qsegment_[i]->next_seg_ = qsegment_[(i + 1) % n];
// 				qsegment_[(i + 1) % n]->prev_seg_ = qsegment_[i];
// 			}
// 		}
// 		qsegment_.clear();
// 	}
// }
bool compareEvent(const Event*  a, const Event*  b)
{
	if (a->position_.y()-b->position_.y()<-1e-3)
		return true;
	else if (a->position_.y()-b->position_.y()<1e-3)
	{
		return a->position_.x() - b->position_.x()<-1e-3;
	}
	return false;
}

bool compSegment(const Segment* a, const Segment* b)
{
	return a->angle_ < b->angle_;
}

std::vector<std::vector<std::pair<Vec3f, Vec3f>>> SweepLine::polygonization()
{

	sort(event_list_.begin(), event_list_.end(), compareEvent);
	Vec3f sta = event_list_.front();
	Vec3f cur = sta;


	std::vector<std::vector<std::pair<Vec3f, Vec3f>>> contours_;
	int k = 0;
	for (auto iter = points_.begin(); iter != points_.end(); iter++,k++)
	{
		if ((*iter)->segment_==NULL)
		{
			qDebug() << k;
		}
		if ((*iter)->selected_)
		{
			continue;
		}
			
		std::vector<std::pair<Vec3f, Vec3f>> contour_;

		Event* sta_ = *iter;
		Event* cur_ = sta_;
		do
		{
			if (cur_->segment_ == NULL)
			{
				cur_->selected_ = true;
				break;
			}
			std::pair<Vec3f, Vec3f> p2(cur_->segment_->first_->position_, cur_->segment_->second_->position_);
			contour_.push_back(p2);
			cur_ =cur_->segment_->second_;
			cur_->selected_ = true;
		} while (cur_ != sta_);
		contours_.push_back(contour_);
	}

	return contours_;
}

void SweepLine::insertSegment(std::pair<Vec3f, Vec3f> pair_points_)
{
	
	Event* a = new Event(pair_points_.first);
	Event* b = new Event(pair_points_.second);
	event_list_.push_back(a);
	event_list_.push_back(b);
	Segment* seg = new Segment(a, b);
	segment_list_.push_back(seg);
	a->segment_ = seg;
	a->is_fir_ = true;
	b->segment_ = seg;
	b->is_fir_ = false;
}