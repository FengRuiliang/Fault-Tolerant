#ifndef GLOBALFUNCTIONS_H
#define GLOBALFUNCTIONS_H
#include "HE_mesh/Mesh3D.h"
#define SWAP(a,b,T) {T tmp=(a); (a)=(b); (b)=tmp;}
#define min(a,b) a<b?a:b
#define max(a,b) a>b?a:b
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
extern bool PointinTriangle(HE_face* face, Vec3f point_in);
extern void CalPlaneLineIntersectPoint(Vec3f planeVector, Vec3f planePoint, Vec3f lineVector, Vec3f linePoint, Vec3f& point);


enum hatchType
{
	NONE = 0,
	CHESSBOARD,
	OFFSETFILLING,
	STRIP,
	MEANDER
};
struct compare_He_vert
{
	bool operator ()(HE_vert* a, HE_vert* b)
	{
		if (a->position()<b->position())
		{
			return true;
		}
		return false;
	}
};
#endif // GLOBALFUNCTIONS_H
