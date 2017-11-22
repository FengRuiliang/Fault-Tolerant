#pragma once
#include "SliceCut.h"
#include <set>
#include<QDebug>
#include <math.h>
#include "globalFunctions.h"
#include "clipper.hpp"
#include "Cutline.h"
using namespace ClipperLib;
class QLine;

// extern float offset_dis_;

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


class Hatch
{

public:
	virtual void clearHatch();
	void setLaserPower(float power);
	void setLaserSpeed(float speed);
	std::vector<Vec3f*>* getHatch() { return hatch_; };
	int GetNumPieces() { return num_pieces_; }
	float getThickness() { return thickness_; }
public:
	std::vector<Vec3f*>* hatch_;
	std::vector <std::vector<CutLine>*>*boudary_edge_;
	std::vector < std::vector<Vec3f>>*offset_vert_;
	int	num_pieces_;
public:
	Hatch(SliceCut*parent);
	Hatch();
	virtual ~Hatch();

	float getLaserPower() { return laser_power_hatch_; };
	float getLaserSpeed() { return laser_speed_hatch_; };
	int* getNumhatch() { return num_hatch; }
	std::vector < std::vector<Vec3f>>* getOffsetVertex() { return offset_vert_; }
	bool Offset(std::vector<Vec3f>& outer_, std::vector <std::vector<Vec3f>>& inner_, float offset_dist, int num_pieces);
	void rotateBack(size_t k);
	virtual void doHatch();

};
class HatchChessboard :public Hatch
{

public:
	void clearHatch()override;
	void doHatch()override;

	HatchChessboard(SliceCut*parent);
	HatchChessboard(SliceCut*slice_model, SliceCut*slice_support);
	~HatchChessboard();
protected:
private:
	float space_;
	float minimal_field_size_;
	float white_board_angle_;
	float black_board_angle;
	float contour_;
};
class HatchOffsetfilling:public Hatch
{
public:
	void doHatch()override;

	HatchOffsetfilling(SliceCut* inSlice_);
	void clearHatch()override;
	~HatchOffsetfilling() { 
		boudary_edge_ = NULL;

	}
protected:
private:
};
class HatchMeander :public Hatch
{
public:
	void clearHatch()override;
	HatchMeander(SliceCut*parent);
	~HatchMeander();
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
