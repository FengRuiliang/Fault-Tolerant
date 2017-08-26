#pragma once
#include "SliceCut.h"
#include <set>
#include<QDebug>
#include <math.h>
#include "globalFunctions.h"
#include "clipper.hpp"
using namespace ClipperLib;
class QLine;
class cutLine;
// extern float offset_dis_;
struct comVec3fBlack
{
	bool operator ()(Vec3f a, Vec3f b)const
	{


		if (a.x() - b.x() < -5e-5) return true;//a.x < b.x
		if (abs(a.x() - b.x()) < 5e-5) return (a.y() - b.y() < -5e-5);
		return false;
	}
};
struct comVec3fWhite
{
	bool operator ()(Vec3f a, Vec3f b)const
	{
		if (a.y() - b.y() < -5e-5) return true;//a.x < b.x
		if (abs(a.y() - b.y()) < 5e-5) return (a.x() - b.x() < -5e-5);
		return false;
	}
};
//extern enum hatchType;
//static float getthickness() { return HatchChessboard::getThickness(); }
enum FieldType
{
	WHITE = 0,
	BLACK = 1,
	BLUE = 2,
};
enum CrossType
{
	X = 0,
	Y = 1,
	No = 2,
};

class BField
{
public:
	BField(int x, int y, float z) {
		
		x_min_field_ = x;
		y_min_field_ = y;
		x_max_field_ = x_min_field_ + 1;
		y_max_field_ = y_min_field_ + 1;
		z_height_ = z;
		type_ = (FieldType)(abs(x_min_field_ + y_min_field_) % 2);
		leftcoor_ = x*field_width_ + field_overlap_;
		rightcoot_ = (x + 1)*field_width_ - field_overlap_;
		topcoor_ = (y + 1)*field_height_ - field_overlap_;
		bottomcoor_ = y*field_height_ + field_overlap_;
	};

	~BField()
	{
		
		hatch_line_.clear();
		Black_hatch_point_.clear();
		white_hatch_point_.clear();
	}

	std::vector<Vec3f*>	hatch_line_;
	std::set<Vec3f, comVec3fBlack> Black_hatch_point_;
	std::set<Vec3f, comVec3fWhite> white_hatch_point_;
	float leftcoor_, rightcoot_, topcoor_, bottomcoor_;
private:
	float topcoordinate_;
public:
	int x_min_;
	int x_max_;
	int y_min_;
	int y_max_;
	int x_min_field_;
	int y_min_field_;
	int x_max_field_;
	int y_max_field_;
	float	z_height_;
	FieldType type_;
	FieldType getFieldType() { return type_; };
};

struct compareBField
{
	bool operator ()(BField* a, BField* b)const
	{
		if (a->x_min_field_ < b->x_min_field_) return true;
		if (a->x_min_field_ == b->x_min_field_) return (a->y_min_field_ < b->y_min_field_);
		return false;
	}
};
struct compareBField_y_
{
	bool operator ()(BField* a, BField* b)const
	{
		if (a->y_min_field_ < b->y_min_field_) return true;
		if (a->y_min_field_ == b->y_min_field_) return (a->x_min_field_ < b->x_min_field_);
		return false;
	}
};

struct compare_CUTLINE
{
	bool operator ()(cutLine* a, cutLine* b)
	{
		if (a->x_field_ < b->x_field_) return true;
		if (a->x_field_ == b->x_field_) return (a->y_field_ < b->y_field_);
		return false;
	}
};

class Hatch 
{
public:
	void setLaserPower(float power);
	void setLaserSpeed(float speed);
	std::vector<Vec3f*>* getHatch() { return hatch_; };
public:
	std::vector<Vec3f*>* hatch_;
	std::vector <std::vector<cutLine>*>*boudary_edge_;
	std::vector < std::vector<Vec3f>>*offset_vert_;
	//std::vector < std::vector<Vec3f>*>*offset_vert_rotate_;
public:
	Hatch(SliceCut*parent) {
		boudary_edge_ = parent->GetPieces();
		num_pieces_ = parent->GetNumPieces();
		offset_vert_ = new std::vector<std::vector<Vec3f>>[parent->GetNumPieces()];
		hatch_ = new std::vector<Vec3f *>[parent->GetNumPieces()];
	};
	virtual ~Hatch();
	Hatch();
	//void doHatch() {};
	void setHatch() {};
	//void rotateOffset();
	float getLaserPower() { return laser_power_hatch_; };
	float getLaserSpeed() { return laser_speed_hatch_; };
	int* getNumhatch() { return num_hatch; }
	std::vector < std::vector<Vec3f>>* getOffsetVertex() { return offset_vert_; }
	bool Offset(std::vector<Vec3f>& outer_, std::vector <std::vector<Vec3f>>& inner_, float offset_dist, int num_pieces);
	//std::vector < std::vector<Vec3f>*>* getOffsetVertexRotate() { return offset_vert_rotate_; }
	void rotateBack(size_t k);
	virtual void doHatch();
	virtual void clearHatch() {};
	public:
	int num_pieces_;
};
class HatchChessboard :public Hatch
{

public:
	void clearHatch()override;
	void doHatch()override;
	HatchChessboard(SliceCut*parent);
	HatchChessboard(SliceCut*slice_model, SliceCut*slice_support);
	~HatchChessboard() { clearHatch(); }
protected:
private:
	float space_;
	float angle_;
	float minimal_field_size_;
	float white_board_angle_;
	float black_board_angle;
	float contour_;
};
class HatchOffsetfilling :public Hatch
{
public:
protected:
private:
};
class HatchStrip :public Hatch
{
public:
	void clearHatch()override;
	HatchStrip(SliceCut*parent);
	~HatchStrip() {}
	void storeCrossPoint();
	void storeHatchLine();
	void doHatch() override;
	//virtual	void setHatch();
protected:
private:
	float space_;
	float angle_;
	//float increment_angle_;
	float contour_;
	std::multiset<Vec3f, comVec3fBlack>* hatch_point_black_;
	std::multiset<Vec3f, comVec3fWhite>* hatch_point_white_;
};
class HatchMeander :public Hatch
{
public:
protected:
private:
};