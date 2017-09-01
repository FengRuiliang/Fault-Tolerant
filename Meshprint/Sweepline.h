#pragma once
#include "HE_mesh/Mesh3D.h"
#include"SliceCut.h"
#include <queue>
#include <set>
#include "globalFunctions.h"
class Event;
class Segment;
struct Event
{
	int			type;          // event type: BEGIN or END vertex
	Vec3f		position_;		 // event vertex
	Segment*	segment_ = ((Segment*)NULL);
	Event(Vec3f v) {
		position_ = v;
	};
	~Event() {};
};
// SweepLine segment data struct
struct Segment
{
private:
	Event*		first_;			 // leftmost vertex point
	Event*		second_;			 // rightmost vertex point
	Segment*	prev_seg_=(Segment*)NULL;			  // segment above this one
	Segment*	next_seg_=(Segment*)NULL;			 // segment below this one
public:
	Segment(Event* e1, Event* e2) {
		first_ = e1;
		second_ = e2;
	}
	~Segment() {};
};

class SweepLine
{
public:
	SweepLine() {};
	SweepLine(std::vector<std::pair<Vec3f, Vec3f>> P);
	~SweepLine() ;
private:
	std::vector<Event*>		qevent_;
	std::vector<Segment*>	qsegment_;
	std::vector<Event*>*	event_list_;
	std::vector<Segment*>*	segment_list_;
public:
	bool compareEvent(Event* a, Event* b);
	std::vector<std::vector<Vec3f>> sweep();
};	
