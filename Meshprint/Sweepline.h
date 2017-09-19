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
	int			type;          // event type: BEGIN or END vertex
	Vec3f		position_;		 // event vertex
	Segment*	segment_ = ((Segment*)NULL);
	Event*		epair_;
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
	~Segment() {};
};

bool compSegment(const Segment* a, const Segment* b);
bool compareEvent(const Event*  a, const Event*  b);





class SweepLine
{
public:
	SweepLine() {};
	SweepLine(std::vector<std::pair<Vec3f, Vec3f>> P);
	~SweepLine() ;
	void getContuor(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>& layer_edge_);
private:
	std::vector<Event*>		que_event_;
	std::vector<Segment*>	qsegment_;
	std::vector<Event*>	event_list_;
	std::vector<Segment*>	segment_list_;
public:
	void polygonization();
};	
