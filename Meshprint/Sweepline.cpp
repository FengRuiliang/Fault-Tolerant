#include "Sweepline.h"
#include <qdebug.h>
SweepLine::SweepLine(std::vector<std::pair<Vec3f, Vec3f>> P)
{
	for (int i=0;i<P.size();i++)
	{
		Event* v1 = new Event(P[i].first);
		Event* v2 = new Event(P[i].second);
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
	segment_list_.clear();
	event_list_.clear();
}

void SweepLine::getContuor(std::vector<std::vector<std::pair<Vec3f,Vec3f>>>& layer_edge_)
{
	for (int i=0;i<segment_list_.size();i++)
	{
		if (!segment_list_[i]->visited_)
		{
			std::vector<std::pair<Vec3f,Vec3f>> polygon_;
			Segment* sta_ = segment_list_[i];
			Segment* cur_ = sta_;
			//qDebug() << i;
			do 
			{
				cur_->visited_ = true;
				polygon_.push_back(std::pair<Vec3f, Vec3f>(cur_->first_->position_, cur_->second_->position_));
				cur_ = cur_->next_seg_;
				
			} while (cur_ != NULL&&cur_ != sta_);
			layer_edge_.push_back(polygon_);
		}
	}
}

void SweepLine::polygonization()
{
	while (!que_event_.empty())
	{
		Vec3f pos_ = que_event_.back()->position_;
		do 
		{
			Vec3f	angle_ = que_event_.back()->epair_->position_ - pos_;
			angle_.normalize();
			if (angle_.y()>=0)
				que_event_.back()->segment_->angle_=acos(angle_.dot(Vec3f(-1.0, 0.0, 0.0)));
			else
				que_event_.back()->segment_->angle_ = 6.28-acos(angle_.dot(Vec3f(1.0, 0.0, 0.0)));
			qsegment_.push_back(que_event_.back()->segment_);
			que_event_.pop_back();
		} while (!que_event_.empty()&&(que_event_.back()->position_-pos_).length()<1e-3);
		//qDebug() << que_event_.size()<<qsegment_.size();
		std::sort(qsegment_.begin(), qsegment_.end(), compSegment);
		int n = qsegment_.size();
		qDebug() << n;
		for (int i=0;i<qsegment_.size();i++)
		{
			if ((qsegment_[i]->first_->position_-qsegment_[(i+1)%n]->second_->position_).length()<1e-3)
			{
				qsegment_[i]->prev_seg_ = qsegment_[(i + 1) % n];
				qsegment_[(i + 1) % n]->next_seg_=qsegment_[i];
			}
			else
			{
				qsegment_[i]->next_seg_ = qsegment_[(i + 1) % n];
				qsegment_[(i + 1) % n]->prev_seg_ = qsegment_[i];
			}
		}
		qsegment_.clear();
	}
}
bool compareEvent(const Event*  a, const Event*  b)
{
	if (a->position_.y() < b->position_.y())
		return true;
	else if (a->position_.y()==b->position_.y())
	{
		return a->position_.x() > b->position_.x();
	}
	return false;
}

bool compSegment(const Segment* a, const Segment* b)
{
	return a->angle_ < b->angle_;
}
