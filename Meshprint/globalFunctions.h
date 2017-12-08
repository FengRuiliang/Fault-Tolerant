#ifndef GLOBALFUNCTIONS_H
#define GLOBALFUNCTIONS_H
#include "HE_mesh/Vec.h"
#include <vector>
#include <utility>
#include "HE_mesh/Mesh3D.h"
#define ScaleNumber 1e6
#define SmoothRatio 0.05
#define MIN_DIS 0.8
#define eps 1e-9
#define MaxNumContours 1000

typedef std::pair<double, double> pa;
struct LayerGrid
{
	std::vector<std::pair<pa, pa> > grids[MaxNumContours];
};
struct LayerOffDis
{
	std::vector<double> dis[MaxNumContours];
};
struct New2Origin
{
	std::map<int, int>  layermap[MaxNumContours];
};
#define SWAP(a,b,T) {T tmp=(a); (a)=(b); (b)=tmp;}
#define min(a,b) a<b?a:b
#define max(a,b) a>b?a:b
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
extern bool PointinTriangle(std::vector<Vec3f> verts, Vec3f point_in);
extern bool CalPlaneLineIntersectPoint(Vec3f planeVector, Vec3f planePoint, Vec3f lineVector, Vec3f linePoint, Vec3f& point);


bool sortByZS(HE_vert* a, HE_vert* b);
bool sortByZB(HE_vert* a, HE_vert* b);
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
		if (a->position() < b->position())
		{
			return true;
		}
		return false;
	}
};


#endif // GLOBALFUNCTIONS_H
