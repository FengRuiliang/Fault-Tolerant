#ifndef GLOBALFUNCTIONS_H
#define GLOBALFUNCTIONS_H
#include "HE_mesh/Vec.h"
using trimesh::point;
typedef trimesh::point point;
typedef trimesh::vec2 Vec2f;
typedef trimesh::vec3  Vec3f;
typedef trimesh::vec4  Vec4f;
extern float field_width_;
extern float field_height_ ;
extern float line_width_ ;
extern float field_overlap_ ;
extern float unit ;
extern int units_y_;
extern int units_x_;
extern float thickness_;
extern float offset_dis_;
extern int * num_hatch;
//////////////////////////////////////////////////////////////////////////
extern float DEFAULT_L ;
extern float THRESHOLD;
extern float THRESHOLD1;
extern float GAP;
extern float SEGLENGTH ;
extern float RESO ;
extern float VERTICALGAP;
//////////////////////////////////////////////////////////////////////////
//extern float laser_power_ ;
//extern float laser_speed_;
extern float laser_power_hatch_ ;
extern float laser_speed_hatch_ ;
extern float laser_power_polygon_ ;
extern float laser_speed_polygon_ ;
extern int increment_angle_;
extern float scaleV;
extern float scaleT;
extern int sss;
extern int fildID;
extern float spot_size_;
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

template<class object>
void SafeDelete(object *ptr)
{
	if (ptr != NULL)
	{
		delete ptr;
		ptr = NULL;
	}
}
template<class object>
void SafeDeletes(object *ptr)
{
	if (ptr != NULL)
	{
		delete[] ptr;
		ptr = NULL;
	}
}
template <typename T>
void inline MySwap(T &t1, T &t2)
{
	T tmp = t1;
	t1 = t2;
	t2 = tmp;
};
#endif // GLOBALFUNCTIONS_H
