#include "Sweepline.h"

SweepLine::SweepLine(std::vector<std::pair<Vec3f, Vec3f>> P)
{
	for (int i=0;i<P.size();i++)
	{
		Event* v1 = new Event(P[i].first);
		Event* v2 = new Event(P[i].second);
		Segment* seg = new Segment(v1,v2);
		v1->segment_ = seg;
		v2->segment_ = seg;
		event_list_->push_back(v1);
		event_list_->push_back(v2);
		qevent_.push_back(v1);
		qevent_.push_back(v2);
		segment_list_->push_back(seg);
	}
	std::sort(qevent_.begin(),qevent_.end(), compareEvent());
}

SweepLine::~SweepLine()
{
	for (int i=0;i<event_list_->size();i++)
	{
		delete event_list_->at(i);
	}
	for (int i = 0; i < segment_list_->size(); i++)
	{
		delete segment_list_->at(i);
	}
}

bool SweepLine::compareEvent(Event * a, Event * b)
{
	if (a->position_.y()<b->position_.y())
	{
		return true;
	}
	else if (a->position_.y()==b->position_.y())
	{
		return a->position_.x() < b->position_.y();
	}
	else
	{
		return false;
	}
}
std::vector<std::vector<Vec3f>> SweepLine::sweep()
{
	while (!qevent_.empty())
	{
		qsegment_.push_back(qevent_.back()->segment_);
	}
}
