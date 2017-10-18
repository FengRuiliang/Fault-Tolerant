#pragma once
#include "HE_mesh/Mesh3D.h"
#include"SliceCut.h"
#include <queue>
#include <set>
#include "globalFunctions.h"
class Event;
class Segment;
class cutLine;
struct Event
{
	int			is_fir_;          // event type: BEGIN or END vertex
	Vec3f		position_;		 // event vertex
	Segment*	segment_ = ((Segment*)NULL);
	std::vector<Segment*> in_seg_;
	std::vector<Segment*> out_seg_;
	Event*		epair_;
	bool		selected_=false;
	Event(Vec3f v) {
		position_ = v;
	};
	~Event() {};
};
// SweepLine segment data struct
struct Segment
{
	Event*		first_;			 // leftmost vertex point
	Event*		second_;			 // rightmost vertex point
	Event*		up_vert_;
	Event*		down_vert_;
	float		angle_{0};
	bool		visited_{ false };
	Segment*	prev_seg_=(Segment*)NULL;			  // segment above this one
	Segment*	next_seg_=(Segment*)NULL;			 // segment below this one
public:
	Segment(Event* e1, Event* e2) {
		first_ = e1;
		second_ = e2;
		if (e1->position_.y()<e2->position_.y())
		{
			up_vert_ = e2;
			down_vert_ = e1;
		}
		else
		{
			up_vert_ = e1;
			down_vert_ = e2;
		}
	}
	Segment(Event e1, Event e2) {

	}
	~Segment() {};
};


struct CompareEvent
{
	bool operator ()(Event* a, Event* b)const
	{
		if (a->position_.x()-b->position_.x()<-1e-3)
		{
			return true;
		}
		else if (a->position_.x()-b->position_.x()<1e-3)
		{
			if (a->position_.y()-b->position_.y()<-1e-3)
			{
				return true;
			}
			else if (a->position_.y()-b->position_.y()<1e-3)
			{
				if (a->position_.z() - b->position_.z() < -1e-3)
				{
					return true;
				}
				else
					return false;
			}
			else
			{
				return false;
			}
		}
		else 
		{
			return false;
		}
	}
};
struct CompareSegment
{
};

bool compSegment(const Segment* a, const Segment* b);
bool compareEvent(const Event*  a, const Event*  b);
class SweepLine
{
public:
	SweepLine() {};
	SweepLine(std::vector<cutLine> P);
	~SweepLine() ;
	void getContuor(std::map<float, std::vector<std::vector<cutLine>>>& layer_edge_);
private:
	std::vector<Event*>		que_event_;
	std::vector<Segment*>	qsegment_;
	std::vector<Event*>	event_list_;
	std::vector<Segment*>	segment_list_;
	std::set<Event*, CompareEvent> points_;
public:
	//void polygonization();
	std::vector<std::vector<std::pair<Vec3f, Vec3f>>> polygonization();
	void insertSegment(std::pair<Vec3f, Vec3f> pair_points_);
};
