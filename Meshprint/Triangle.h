#ifndef TRIANGLE_HAN
#define TRIANGLE_HAN
#include "TriangleglobalFunctions.h"
#include <QDebug>
#include "MSAABB.h"

/*
检测两个三角面的相交时思路如下：
考虑两个三角面的关系是以它们各自所处的平面为依托，
1.首先一个三角面在另一个三角面所在平面的一侧，这时两个三角面一定不相交，
实现上，用一个三角面上的一顶点分别以另一个三角面的三顶点组合的四点，判断四顶点的位置关系
2.三角面上的一顶点在另一个三角面所在的平面上，进行点是否在三角形内的判断
3.三角面上一点与其它两顶点分别位于另一个三角面所处平面的两侧（包含两顶点在平面上的情况）
实现上求线段与面的交点是否在三角面内
*/


enum TopologicalStructure
{
	INTERSECT, NONINTERSECT
};
enum pointInTriangleStatus
{
	INTHEFACE, ONTHETOP, NOINTHETRIANGLE
};

struct InsFaceIdAndEdge
{
//	int ID = -1;				//记录相交面的ID
	int insEdgeStartPoint = 0;	//相交边的start端点
	int insEdgeEndPoint = 0;	//相交边的End端点
	int insRedundantPoint = 0;  //剩余的端点
	Vec3f insPoint;				//相交点
	bool operator < (const InsFaceIdAndEdge &a) const // 重载“<”操作符，自定义排序规则  
	{
		//按ID由大到小排序。如果要由小到大排序，使用“>”即可。  
		return a.insPoint > insPoint;
	}
};

class Triangle
{
private:
	int triId;					//按照读取顺序，给三角面分配的ID号

public:
	Vec3f normal;
	Vec3f Vertex_1, Vertex_2, Vertex_3;
	AABB _aabb;					//AABB包围盒
	InsFaceIdAndEdge INS;

	Vec3f centerpoint;			//三角面的外点 -
	double radius;				//三角面的外接圆半径 -
	int selected = 0;			//标记三角面片是否相交的标志位
	int partitionNumber = 0;	//分区编号，记录三家门面处在哪个区 -
	std::vector<Triangle> triVar;		//记录与其相交的三角面ID号
	std::set<InsFaceIdAndEdge> triPoint;	//记录三角面自身所有的交点

	//std::vector<int> insEdgePoint; //相交边的一个端点


public:
	int getTriId() {
		return triId;
	}
	void setTriId(int num) {
		triId = num;
	}
	AABB getAABB() {
		return _aabb;
	}

	Vec3f mappingValue(int value) {
		if (value == 1)
		{
			return Vertex_1;
		}
		else if (value == 2)
		{
			return Vertex_2;
		}
		else if (value == 3)
		{
			return Vertex_3;
		}
	}

	inline bool operator==(Triangle &v)
	{
		if (this->Vertex_1 == v.Vertex_1&&this->Vertex_2 == v.Vertex_2&&this->Vertex_3 == v.Vertex_3)
			return true;
		else
			return false;
	}
};

struct pointTri
{
	double x = 0, y = 0;
};

//inline void expandBoundary() {
//
//#define MAX_FLOAT_VALUE (static_cast<float>(10e10))
//#define MIN_FLOAT_VALUE	(static_cast<float>(-10e10))
//
//	_max.x() = _max.y() = _max.z() = MIN_FLOAT_VALUE;
//	_min.x() = _min.y() = _min.z() = MAX_FLOAT_VALUE;
//
//	_min.x() = std::min(_min.x(), (*viter)->position_.x());
//	_min.y() = min(_min.y(), (*viter)->position_.y());
//	_min.z() = min(_min.z(), (*viter)->position_.z());
//	_max.x() = max(_max.x(), (*viter)->position_.x());
//	_max.y() = max(_max.y(), (*viter)->position_.y());
//	_max.z() = max(_max.z(), (*viter)->position_.z());
//
//
//	//扩大包围盒外檐，不至于只是一个面
//	_min.x() = _min.x() - 0.01;
//	_min.y() = _min.y() - 0.01;
//	_min.z() = _min.z() - 0.01;
//
//	_max.x() = _max.x() + 0.01;
//	_max.y() = _max.y() + 0.01;
//	_max.z() = _max.z() + 0.01;
//}

inline void copy_pointXY(pointTri &a, Vec3f b) {
	a.x = b[0];
	a.y = b[1];
}

inline void copy_pointXZ(pointTri &a, Vec3f b) {
	a.x = b[0];
	a.y = b[2];
}

inline void copy_pointYZ(pointTri &a, Vec3f b) {
	a.x = b[1];
	a.y = b[2];
}

inline double twoPointDistance(Vec3f a, Vec3f b) {
	return sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2) + pow((a[2] - b[2]), 2));
}

//判断两点是否相等
inline bool is_equal_vertex(Vec3f pointTri1, Vec3f pointTri2) {

	if (pointTri1==pointTri2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//四点行列式  
inline float get_vector4_det(Vec3f v1, Vec3f v2, Vec3f v3, Vec3f v4)
{
	float a[3][3];
	for (int i = 0; i != 3; ++i)
	{
		a[0][i] = v1[i] - v4[i];
		a[1][i] = v2[i] - v4[i];
		a[2][i] = v3[i] - v4[i];
	}

	return a[0][0] * a[1][1] * a[2][2]
		+ a[0][1] * a[1][2] * a[2][0]
		+ a[0][2] * a[1][0] * a[2][1]
		- a[0][2] * a[1][1] * a[2][0]
		- a[0][1] * a[1][0] * a[2][2]
		- a[0][0] * a[1][2] * a[2][1];

	///平面方程A*x+B*y+C*z+D=0;
	//float A = ((v2.y() - v1.y())*(v3.z() - v1.z()) - (v2.z() - v1.z())*(v3.y() - v1.y()));
	//float B = ((v2.z() - v1.z())*(v3.x() - v1.x()) - (v2.x() - v1.x())*(v3.z() - v1.z()));
	//float C = ((v2.x() - v1.x())*(v3.y() - v1.y()) - (v2.y() - v1.y())*(v3.x() - v1.x()));
	//float D = -(A * v1.x() + B * v1.y() + C * v1.z());

	//return (A*v4.x() + B*v4.y() + v4.z()*C + D);
}

//利用叉积计算点p相对线段p1p2的方位  
inline double direction(pointTri p1, pointTri p2, pointTri p) {
	return  (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

//确定与线段p1p2共线的点p是否在线段p1p2上  
inline int on_segment(pointTri p1, pointTri p2, pointTri p) {
	double max = p1.x > p2.x ? p1.x : p2.x;
	double min = p1.x < p2.x ? p1.x : p2.x;
	double max1 = p1.y > p2.y ? p1.y : p2.y;
	double min1 = p1.y < p2.y ? p1.y : p2.y;

	if (p.x >= min && p.x <= max && p.y >= min1 && p.y <= max1)//修改 去除“=”,除去共线点，与顶点相同
	{
		//在二维空间中去控制 边，点的关系，是糟糕的想法（就是上面注释的想法）
		return 1;
	}
	else
	{
		return 0;
	}
}

inline void get_center_point_of_circle(Triangle* tri)
{
	Vec3f centerpoint_var;

	double a1, b1, c1, d1;
	double a2, b2, c2, d2;
	double a3, b3, c3, d3;

	double x1 = tri->Vertex_1[0], y1 = tri->Vertex_1[1], z1 = tri->Vertex_1[2];
	double x2 = tri->Vertex_2[0], y2 = tri->Vertex_2[1], z2 = tri->Vertex_2[2];
	double x3 = tri->Vertex_3[0], y3 = tri->Vertex_3[1], z3 = tri->Vertex_3[2];

	a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
	b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
	c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
	d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

	a2 = 2 * (x2 - x1);
	b2 = 2 * (y2 - y1);
	c2 = 2 * (z2 - z1);
	d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

	a3 = 2 * (x3 - x1);
	b3 = 2 * (y3 - y1);
	c3 = 2 * (z3 - z1);
	d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

	centerpoint_var[0] = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
		/ (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	centerpoint_var[1] = (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
		/ (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	centerpoint_var[2] = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
		/ (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	tri->centerpoint = centerpoint_var;
	tri->radius = sqrt(pow((x1 - centerpoint_var[0]), 2) + pow((x1 - centerpoint_var[1]), 2) + pow((x1 - centerpoint_var[2]), 2));
}

inline void get_central_point(Vec3f centralPoint, Triangle* tri)
{
	centralPoint[0] = (tri->Vertex_1[0] + tri->Vertex_2[0] + tri->Vertex_3[0]) / 3;

	centralPoint[1] = (tri->Vertex_1[1] + tri->Vertex_2[1] + tri->Vertex_3[1]) / 3;

	centralPoint[2] = (tri->Vertex_1[2] + tri->Vertex_2[2] + tri->Vertex_3[2]) / 3;
}

//向量之差  
inline void get_vector_diff(Vec3f& aimV, const Vec3f a, const Vec3f b)
{
	aimV[0] = b[0] - a[0];

	aimV[1] = b[1] - a[1];

	aimV[2] = b[2] - a[2];
}

//向量内积  或 点积
inline double Dot(const Vec3f& v1, const Vec3f& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

//判断点与线段p3p4是否共线的主函数  
inline int segments_collinear(Vec3f p1, Vec3f p2) {
	Vec3f p;
	if (p1.cross(p2) == p)
	{
		return 1;
	}
	return 0;
}

//判断线段p1p2与线段p3p4是否相交的主函数  
inline int segments_intersert(pointTri p1, pointTri p2, pointTri p3, pointTri p4) {
	double d1, d2, d3, d4;
	d1 = direction(p3, p4, p1);
	d2 = direction(p3, p4, p2);
	d3 = direction(p1, p2, p3);
	d4 = direction(p1, p2, p4);
	if (d1 * d2 < 0 && d3 * d4 < 0)
	{
		return 1;
	}//共线但是点与顶点重合，判定不相交(共面且顶点重合的两个面，判定为不相交)
	else if (d1 == 0 && on_segment(p3, p4, p1) == 1)
	{
		return 1;
	}
	else if (d2 == 0 && on_segment(p3, p4, p2) == 1)
	{
		return 1;
	}
	else if (d3 == 0 && on_segment(p1, p2, p3) == 1)
	{
		return 1;
	}
	else if (d4 == 0 && on_segment(p1, p2, p4) == 1)
	{
		return 1;
	}
	return 0;
}

//重心法判断点是否在三角形内部  排除边上的情况 点和三角形位于同一个平面内 -_-...
//inline pointInTriangleStatus is_pointTri_within_triangle(Triangle* tri, Vec3f pointTri)
//{
//	Vec3f v0;
//	get_vector_diff(v0, tri->Vertex_1, tri->Vertex_3);
//	Vec3f v1;
//	get_vector_diff(v1, tri->Vertex_1, tri->Vertex_2);
//	Vec3f v2;
//	get_vector_diff(v2, tri->Vertex_1, pointTri);
//	float dot00 = Dot(v0, v0);
//	float dot01 = Dot(v0, v1);
//	float dot02 = Dot(v0, v2);
//	float dot11 = Dot(v1, v1);
//	float dot12 = Dot(v1, v2);
//	float inverDeno = 1 / (dot00* dot11 - dot01* dot01);
//	float u = (dot11* dot02 - dot01* dot12) * inverDeno;
//	if (u < 0 || u > 1) // if u out of range, return directly  添加了"="，去除在顶点的情况
//	{
//		return NOINTHETRIANGLE;
//	}
//	float v = (dot00* dot12 - dot01* dot02) * inverDeno;
//	if (v < 0 || v > 1) // if v out of range, return directly  
//	{
//		return NOINTHETRIANGLE;
//	}
//	if ((v == 0 && u == 1) || (u == 0 && v == 1) || (u == 0 && v == 0))
//	{
//		//qDebug() << "U+V==1";
//		return ONTHETOP;
//	}
//	else if (u + v <= 1)//等于1的情况应该是点在三边上
//	{
//		return INTHEFACE;
//	}
//	return NOINTHETRIANGLE;
//
//}

// Determine whether two vectors v1 and v2 point to the same direction
// v1 = Cross(AB, AC)
// v2 = Cross(AB, AP)
inline bool SameSide(Vec3f A, Vec3f B, Vec3f C, Vec3f P)
{
    Vec3f AB = B - A ;
    Vec3f AC = C - A ;
    Vec3f AP = P - A ;

    Vec3f v1 = AB.cross(AC) ;
    Vec3f v2 = AB.cross(AP) ;

    // v1 and v2 should point to the same direction
    return v1.dot(v2) >= 0 ;
}

// Same side method
// Determine whether point P in triangle ABC
inline bool PointinTriangle1(Triangle *tri, Vec3f P)
{
    return SameSide(tri->Vertex_1, tri->Vertex_2, tri->Vertex_3, P) &&
        SameSide(tri->Vertex_2, tri->Vertex_3, tri->Vertex_1, P) &&
        SameSide(tri->Vertex_3, tri->Vertex_1, tri->Vertex_2, P) ;
}

//边框相交检测,只是二维平面中的检测
inline bool borderCrossingDetection(Triangle* Tri1, Triangle* Tri2) {

	return (Tri1->_aabb._max.x() >= Tri2->_aabb._min.x())&&(Tri2->_aabb._max.x() >= Tri1->_aabb._min.x())&&(Tri1->_aabb._max.y() >= Tri2->_aabb._min.y())&&(Tri2->_aabb._max.y() >= Tri1->_aabb._min.y());

}

inline bool is_pointTri_within_triangle(Triangle* tri, Vec3f pointTri)
{
	//return SameSide(tri->Vertex_1, tri->Vertex_2, tri->Vertex_3, pointTri) &&
	//	SameSide(tri->Vertex_2, tri->Vertex_3, tri->Vertex_1, pointTri) &&
	//	SameSide(tri->Vertex_3, tri->Vertex_1, tri->Vertex_2, pointTri);
	
	Vec3f v0;
	get_vector_diff(v0, tri->Vertex_1, tri->Vertex_3);
	Vec3f v1;
	get_vector_diff(v1, tri->Vertex_1, tri->Vertex_2);
	Vec3f v2;
	get_vector_diff(v2, tri->Vertex_1, pointTri);
	double dot00 = Dot(v0, v0);
	double dot01 = Dot(v0, v1);
	double dot02 = Dot(v0, v2);
	double dot11 = Dot(v1, v1);
	double dot12 = Dot(v1, v2);
	double inverDeno = 1 / (dot00* dot11 - dot01* dot01);
	double u = (dot11* dot02 - dot01* dot12) * inverDeno;
	if (u < 0 || u > 1) // if u out of range, return directly  添加了"="，去除在顶点的情况
	{
		return false;
	}
	double v = (dot00* dot12 - dot01* dot02) * inverDeno;
	if (v < 0 || v > 1) // if v out of range, return directly  
	{
		return false;
	}
	else if (u + v <= 1)//等于1的情况应该是点在三边上
	{
		return true;
	}
	return false;
}
inline bool is_pointTri_within_Line(Vec3f f1,Vec3f f2,Vec3f point) {
		if (((f1.x() - point.x())*(point.x() - f2.x()))>=0&& ((f1.y() - point.y())*(point.y() - f2.y())) >= 0&& ((f1.z() - point.z())*(point.z() - f2.z())) >= 0)
		{
			return true;
		}
		return false;
}

//判断点是否是三角形的一个顶点
inline bool is_pointTri_within_triangle_vectex(Triangle *tri, Vec3f pointTri) {
	if (is_equal_vertex(tri->Vertex_1, pointTri) || is_equal_vertex(tri->Vertex_2, pointTri) || is_equal_vertex(tri->Vertex_3, pointTri))
	{
		return true;
	}
	else
	{
		return false;
	}
}
//判断空间中三点共线
inline bool is_three_point_within_SameLine(Vec3f f1, Vec3f f2, Vec3f f3) {
	Vec3f m1, m2;
	m1 = f2 - f1;
	m2 = f3 - f2;

	if (m1.x() / m2.x() == m1.y() / m2.y() && m1.y() / m2.y() == m1.z() / m2.z())
	{
		return true;
	}
	return false;
}


//判断点是否和三角形的一边共线(空间中的共线问题转化成2维的共线处理，有失公允)
inline bool is_pointTri_within_triangle_inSameEdgeLine(Triangle *tri, Vec3f f1, Vec3f f2) {

	if (is_three_point_within_SameLine(tri->Vertex_2, f1, f2) && is_three_point_within_SameLine(tri->Vertex_3, f1, f2))
	{
		return true;
	}
	else if (is_three_point_within_SameLine(tri->Vertex_1, f1, f2) && is_three_point_within_SameLine(tri->Vertex_3, f1, f2))
	{
		return true;
	}
	else if (is_three_point_within_SameLine(tri->Vertex_2, f1, f2) && is_three_point_within_SameLine(tri->Vertex_1, f1, f2))
	{
		return true;
	}
	return false;


	Vec3f pa1, pb1, pa2, pb2, pa3, pb3;

	pa1 = tri->Vertex_1 - f1;
	pb1 = tri->Vertex_2 - f2;

	pa2 = tri->Vertex_1 - f1;
	pb2 = tri->Vertex_3 - f2;

	pa3 = tri->Vertex_2 - f1;
	pb3 = tri->Vertex_3 - f2;

	//共线
	//if ((segments_collinear(pa1, pb1) && segments_collinear(pc1,pd1))||(segments_collinear(pa2, pb2)&& segments_collinear(pc2, pd2))||(segments_collinear(pa3, pb3)&&segments_collinear(pc3, pd3)))
	if (segments_collinear(pa1, pb1) || segments_collinear(pa2, pb2) || segments_collinear(pa3, pa3))
	{
		pa1 = tri->Vertex_1 - f2;
		pb1 = tri->Vertex_2 - f1;

		pa2 = tri->Vertex_1 - f2;
		pb2 = tri->Vertex_3 - f1;

		pa3 = tri->Vertex_2 - f2;
		pb3 = tri->Vertex_3 - f1;
		if (segments_collinear(pa1, pb1) || segments_collinear(pa2, pb2) || segments_collinear(pa3, pa3)) {
			return true;
		}

	}
	return false;
}

//针对model紧凑的三角面片的特点进行排除处理,对于没有满足要求的点，我不能说他不合适，还要继续下一步
inline bool need_triangle_rule_out(Triangle *tri, Vec3f f1, Vec3f f2) {

	//if 线段与三角面上的一条边共线

	//if 两点都在三角面内（这个时候已经在都在同一条边上了）
	//return NOINTERSECT
	//else if 只有一点在三角面上
	//if 是顶点
	//return NONINTERSECT

	if (is_pointTri_within_triangle(tri, f1) && is_pointTri_within_triangle(tri, f2))
	{
		return true;//NONINTERSECT
	}
	else if (is_pointTri_within_triangle(tri, f1))
	{
		if (is_pointTri_within_triangle_vectex(tri, f1))
		{
			return true;
		}
	}
	else if (is_pointTri_within_triangle(tri, f2))
	{
		if (is_pointTri_within_triangle_vectex(tri, f2))
		{
			return true;
		}
	}

	return false;
}

//判断同一平面的直线和三角形是否相交  
inline bool line_triangle_intersert_inSamePlane(Triangle* tri, Vec3f f1, Vec3f f2)
{

	//a:XY面 b:XZ面 c:YZ面
	pointTri pa1, pa2, pb1, pb2, pc1, pc2, pa3, pa4, pb3, pb4, pc3, pc4;

	copy_pointXY(pa1, f1);

	copy_pointXY(pa2, f2);


	copy_pointXZ(pb1, f1);

	copy_pointXZ(pb2, f2);


	copy_pointYZ(pc1, f1);

	copy_pointYZ(pc2, f2);



	copy_pointXY(pa3, tri->Vertex_1);

	copy_pointXY(pa4, tri->Vertex_2);


	copy_pointXZ(pb3, tri->Vertex_1);

	copy_pointXZ(pb4, tri->Vertex_2);


	copy_pointYZ(pc3, tri->Vertex_1);

	copy_pointYZ(pc4, tri->Vertex_2);

	if (segments_intersert(pa1, pa2, pa3, pa4) && segments_intersert(pb1, pb2, pb3, pb4) && segments_intersert(pc1, pc2, pc3, pc4))
	{
		return true;
	}

	copy_pointXY(pa3, tri->Vertex_2);

	copy_pointXY(pa4, tri->Vertex_3);


	copy_pointXZ(pb3, tri->Vertex_2);

	copy_pointXZ(pb4, tri->Vertex_3);


	copy_pointYZ(pc3, tri->Vertex_2);

	copy_pointYZ(pc4, tri->Vertex_3);

	if (segments_intersert(pa1, pa2, pa3, pa4) && segments_intersert(pb1, pb2, pb3, pb4) && segments_intersert(pc1, pc2, pc3, pc4))
	{
		return true;
	}

	copy_pointXY(pa3, tri->Vertex_1);

	copy_pointXY(pa4, tri->Vertex_3);


	copy_pointXZ(pb3, tri->Vertex_1);

	copy_pointXZ(pb4, tri->Vertex_3);


	copy_pointYZ(pc3, tri->Vertex_1);

	copy_pointYZ(pc4, tri->Vertex_3);

	if (segments_intersert(pa1, pa2, pa3, pa4) && segments_intersert(pb1, pb2, pb3, pb4) && segments_intersert(pc1, pc2, pc3, pc4))
	{
		return true;
	}

	return false;
}

//判断同一平面内的三角形是否相交  
inline bool triangle_intersert_inSamePlane(Triangle* tri1, Triangle* tri2)
{
	//同一平面中，共顶点的面（唯一一点）排除是顶点和在边上的点

	if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_1) && (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2)  || is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3)))
	{
		return false;
	}
	else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2) && is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3))
	{
		return false;
	}

	if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_1) && !is_pointTri_within_triangle(tri1, tri2->Vertex_2)&& !is_pointTri_within_triangle(tri1, tri2->Vertex_3))
	{
		return false;
	}
	else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2) && !is_pointTri_within_triangle(tri1, tri2->Vertex_1)&& !is_pointTri_within_triangle(tri1, tri2->Vertex_3))
	{
		return false;
	}
	else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3) && !is_pointTri_within_triangle(tri1, tri2->Vertex_2)&& !is_pointTri_within_triangle(tri1, tri2->Vertex_1))
	{
		return false;
	}



	if (line_triangle_intersert_inSamePlane(tri2, tri1->Vertex_1, tri1->Vertex_2))
	{
		return true;
	}
	else if (line_triangle_intersert_inSamePlane(tri2, tri1->Vertex_2, tri1->Vertex_3))
	{
		return true;
	}
	else if (line_triangle_intersert_inSamePlane(tri2, tri1->Vertex_1, tri1->Vertex_3))
	{
		return true;
	}
	else
	{
		//重叠面的检测并不准确，考虑每个三角面片建立包围盒，判定是否覆盖（此处的重叠延伸为相交不至于一点，即两面存在公共区域）

		Vec3f centralPoint1, centralPoint2;
		get_central_point(centralPoint1, tri1);
		get_central_point(centralPoint2, tri2);
		if (is_pointTri_within_triangle(tri2, centralPoint1) || is_pointTri_within_triangle(tri1, centralPoint2))
		{
			//return true;  //重叠即相交，应返回true. 再此认为它为重叠的情况，先简单的记录到overlap_num
			overlap_num++;
		}

		return false;
	}
}

//计算线与平面的交点 return point
inline bool Comp_Plane_Line_IntersectPoint(Triangle *tri,Vec3f f1,Vec3f f2,Vec3f& point)
{

	float p1_f1 = get_vector4_det(tri->Vertex_1, tri->Vertex_2, tri->Vertex_3, f1);

	float p1_f2 = get_vector4_det(tri->Vertex_1, tri->Vertex_2, tri->Vertex_3, f2);

	//qDebug() << p1_f1 << "    " << p1_f2 << "     " << "........";

	if (p1_f1*p1_f2>0)
	{
		return false;
	}

	Vec3f var0;
	get_vector_diff(var0, tri->Vertex_1, tri->Vertex_3);
	Vec3f var1;
	get_vector_diff(var1, tri->Vertex_1, tri->Vertex_2);

	Vec3f planeVector= var0.cross(var1);
	Vec3f planePoint = tri->Vertex_1;

	Vec3f lineVector;
	get_vector_diff(lineVector, f1, f2);

	Vec3f linePoint=f1;

	float vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t, vpt;
	vp1 = planeVector.x();
	vp2 = planeVector.y();
	vp3 = planeVector.z();
	n1 = planePoint.x();
	n2 = planePoint.y();
	n3 = planePoint.z();
	v1 = lineVector.x();
	v2 = lineVector.y();
	v3 = lineVector.z();
	m1 = linePoint.x();
	m2 = linePoint.y();
	m3 = linePoint.z();
	vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;

	/*if (abs(vpt) < 1e-3)
	{
		qDebug() << "..................";
		point = linePoint + lineVector;
		if (point < linePoint)
		{

			point = linePoint;
		}
		point.y() -= 0.01;
	}*/
	if(vpt!=0)
	{
		t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
		point.x() = m1 + v1 * t;
		point.y() = m2 + v2 * t;
		point.z() = m3 + v3 * t;
		return true;
	}
	return false;
}

//两三角面不在一个平面内，相交检测记录相交面ID,边界信息
inline bool is_TriangleIntersect_within_NoSamePlane(Triangle *tri1,Triangle *tri2) {
	
	insStartPoint = 0;	//相交边的start端点，用做记录
	insEndPoint = 0;	//相交边的End端点
	insRedunPoint = 0;	//相交边的冗余点

	INSPOINT_X = 0;
	INSPOINT_Y = 0;
	INSPOINT_Z = 0;

	Vec3f point;

	if (Comp_Plane_Line_IntersectPoint(tri2, tri1->Vertex_1, tri1->Vertex_2, point) && is_pointTri_within_triangle(tri2, point))
	{
		//qDebug() << ".........";
		if (is_pointTri_within_triangle_vectex(tri2, point))
		{
			return false;
		}
		//判断点是否在线段上，时我应该做出标记
		if (is_pointTri_within_Line(tri1->Vertex_1, tri1->Vertex_2, point))
		{
			insStartPoint = 1;
			insEndPoint = 2;
			insRedunPoint = 3;
			INSPOINT_X = point.x();
			INSPOINT_Y = point.y();
			INSPOINT_Z = point.z();
		}
		//如果点没在线段上，又因为他已经在三角面内，执行此已经可以判定，不再是简单的相交，转为陷入
		return true;
	}

	if (Comp_Plane_Line_IntersectPoint(tri2, tri1->Vertex_1, tri1->Vertex_3, point) && is_pointTri_within_triangle(tri2, point))
	{
		//qDebug() << "++++++++++++++++";
		if (is_pointTri_within_triangle_vectex(tri2, point))
		{
			return false;
		}
		//判断点是否在线段上，时我应该做出标记
		if (is_pointTri_within_Line(tri1->Vertex_1, tri1->Vertex_3, point))
		{
			insStartPoint = 1;
			insEndPoint = 3;
			insRedunPoint = 2;
			INSPOINT_X = point.x();
			INSPOINT_Y = point.y();
			INSPOINT_Z = point.z();
		}
		return true;
	}

	if (Comp_Plane_Line_IntersectPoint(tri2, tri1->Vertex_2, tri1->Vertex_3, point) && is_pointTri_within_triangle(tri2, point))
	{
		//qDebug() << "-----------------";
		if (is_pointTri_within_triangle_vectex(tri2, point))
		{
			return false;
		}
		//判断点是否在线段上，是我应该做出标记
		if (is_pointTri_within_Line(tri1->Vertex_2, tri1->Vertex_3, point))
		{
			insStartPoint = 2;
			insEndPoint = 3;
			insRedunPoint = 1;
			INSPOINT_X = point.x();
			INSPOINT_Y = point.y();
			INSPOINT_Z = point.z();
		}
		return true;
	}

	if (Comp_Plane_Line_IntersectPoint(tri1, tri2->Vertex_1, tri2->Vertex_2, point)&&is_pointTri_within_triangle(tri1,point))
	{
		if (is_pointTri_within_triangle_vectex(tri1, point))
		{
			return false;
		}
		//判断点是否在线段上，是我应该做出标记
		if (is_pointTri_within_Line(tri2->Vertex_1, tri2->Vertex_2, point))
		{
			insStartPoint = 1;
			insEndPoint = 2;
			insRedunPoint = 3; 
			INSPOINT_X = point.x();
			INSPOINT_Y = point.y();
			INSPOINT_Z = point.z();
		}
		return true;
	}
	
	if (Comp_Plane_Line_IntersectPoint(tri1, tri2->Vertex_1, tri2->Vertex_3, point)&&is_pointTri_within_triangle(tri1, point))
	{
		if (is_pointTri_within_triangle_vectex(tri1, point))
		{
			return false;
		}
		if (is_pointTri_within_Line(tri2->Vertex_1, tri2->Vertex_3, point))
		{
			insStartPoint = 1;
			insEndPoint = 3;
			insRedunPoint = 2;
			INSPOINT_X = point.x();
			INSPOINT_Y = point.y();
			INSPOINT_Z = point.z();
		}
		return true;
	}
	
	if (Comp_Plane_Line_IntersectPoint(tri1, tri2->Vertex_2, tri2->Vertex_3, point)&&is_pointTri_within_triangle(tri1, point))
	{

		if (is_pointTri_within_triangle_vectex(tri1, point))
		{
			return false;
		}
		if (is_pointTri_within_Line(tri2->Vertex_2, tri2->Vertex_3, point))
		{
			insStartPoint = 2;
			insEndPoint = 3;
			insRedunPoint = 1;
			INSPOINT_X = point.x();
			INSPOINT_Y = point.y();
			INSPOINT_Z = point.z();
		}
		return true;
	}
	
	return false;
}


//算法主函数  
inline TopologicalStructure judge_triangle_topologicalStructure(Triangle* tri1, Triangle* tri2)
{
	//设tri1所在的平面为p1,tri2所在的平面为p2
	float p1_tri2_vertex1 = get_vector4_det(tri1->Vertex_1, tri1->Vertex_2, tri1->Vertex_3, tri2->Vertex_1);

	float p1_tri2_vertex2 = get_vector4_det(tri1->Vertex_1, tri1->Vertex_2, tri1->Vertex_3, tri2->Vertex_2);

	float p1_tri2_vertex3 = get_vector4_det(tri1->Vertex_1, tri1->Vertex_2, tri1->Vertex_3, tri2->Vertex_3);


	if (p1_tri2_vertex1 > 0 && p1_tri2_vertex2 > 0 && p1_tri2_vertex3 > 0)
	{
		return NONINTERSECT;
	}

	if (p1_tri2_vertex1 < 0 && p1_tri2_vertex2 < 0 && p1_tri2_vertex3 < 0)
	{
		return NONINTERSECT;
	}


	if (p1_tri2_vertex1 == 0 && p1_tri2_vertex2 == 0 && p1_tri2_vertex3 == 0)
	{
		if (triangle_intersert_inSamePlane(tri1, tri2))
		{
			return INTERSECT;/////////////////////////
		}
		else
		{
			return NONINTERSECT;
		}
	}

	////两点在三角面内所在的平面中
	//if (p1_tri2_vertex1 == 0 && p1_tri2_vertex2 == 0 && p1_tri2_vertex3 != 0)
	//{
	//	if (is_pointTri_within_triangle(tri1, tri2->Vertex_1) == ONTHETOP&&is_pointTri_within_triangle(tri1, tri2->Vertex_2) == ONTHETOP)
	//	{
	//		return NONINTERSECT;
	//	}
	//}
	//else if (p1_tri2_vertex1 == 0 && p1_tri2_vertex3 == 0 && p1_tri2_vertex2 != 0)
	//{
	//	if (is_pointTri_within_triangle(tri1, tri2->Vertex_1) == ONTHETOP&&is_pointTri_within_triangle(tri1, tri2->Vertex_3) == ONTHETOP)
	//	{
	//		return NONINTERSECT;
	//	}
	//}
	//else if (p1_tri2_vertex2 == 0 && p1_tri2_vertex3 == 0 && p1_tri2_vertex1 != 0)
	//{
	//	if (is_pointTri_within_triangle(tri1, tri2->Vertex_2) == ONTHETOP&&is_pointTri_within_triangle(tri1, tri2->Vertex_3) == ONTHETOP)
	//	{
	//		return NONINTERSECT;
	//	}
	//}

	if (p1_tri2_vertex1 == 0 && p1_tri2_vertex2 == 0 && p1_tri2_vertex3 != 0)
	{
		if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_1)&& is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_1)&&!is_pointTri_within_triangle(tri1, tri2->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2) && !is_pointTri_within_triangle(tri1, tri2->Vertex_1))
		{
			return NONINTERSECT;
		}
		else if (line_triangle_intersert_inSamePlane(tri1, tri2->Vertex_1, tri2->Vertex_2))
		{
			return INTERSECT;
		}
	}
	else if (p1_tri2_vertex1 == 0 && p1_tri2_vertex3 == 0 && p1_tri2_vertex2 != 0)
	{
		if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_1)&&is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_1) && !is_pointTri_within_triangle(tri1, tri2->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3) && !is_pointTri_within_triangle(tri1, tri2->Vertex_1))
		{
			return NONINTERSECT;
		}
		else if (line_triangle_intersert_inSamePlane(tri1, tri2->Vertex_1, tri2->Vertex_3))
		{
			return INTERSECT;
		}
	}
	else if (p1_tri2_vertex2 == 0 && p1_tri2_vertex3 == 0 && p1_tri2_vertex1 != 0)
	{
		if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2)&&is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2) && !is_pointTri_within_triangle(tri1, tri2->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3) && !is_pointTri_within_triangle(tri1, tri2->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (line_triangle_intersert_inSamePlane(tri1, tri2->Vertex_2, tri2->Vertex_3))
		{
			return INTERSECT;
		}
	}
	

	////一点在三角面所在的平面中
	if (p1_tri2_vertex1 == 0 && p1_tri2_vertex2 * p1_tri2_vertex3 != 0)
	{
		//共顶点且只有一个，排除
		if (is_pointTri_within_triangle(tri1, tri2->Vertex_1))
		{
			if (is_pointTri_within_triangle_vectex(tri1,tri2->Vertex_1))
			{
				return NONINTERSECT;
			}
			return INTERSECT; /////////////////////////////
		}
		else
		{
			return NONINTERSECT;
		}
	}
	else if (p1_tri2_vertex2 == 0 && p1_tri2_vertex1 * p1_tri2_vertex3 != 0)
	{
		//共顶点且只有一个，排除
		if (is_pointTri_within_triangle(tri1, tri2->Vertex_2))
		{
			if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_2))
			{
				return NONINTERSECT;
			}
			return INTERSECT;//////////////////////////
		}
		else
		{
			return NONINTERSECT;
		}
	}
	else if (p1_tri2_vertex3 == 0 && p1_tri2_vertex1 * p1_tri2_vertex2 != 0)
	{
		//共顶点且只有一个，排除

		if (is_pointTri_within_triangle(tri1, tri2->Vertex_3))
		{
			if (is_pointTri_within_triangle_vectex(tri1, tri2->Vertex_3))
			{
				return NONINTERSECT;
			}
			return INTERSECT; ///////////////////////////
		}
		else
		{
			return NONINTERSECT;
		}
	}

	float p2_tri1_vertex1 = get_vector4_det(tri2->Vertex_1, tri2->Vertex_2, tri2->Vertex_3, tri1->Vertex_1);

	float p2_tri1_vertex2 = get_vector4_det(tri2->Vertex_1, tri2->Vertex_2, tri2->Vertex_3, tri1->Vertex_2);

	float p2_tri1_vertex3 = get_vector4_det(tri2->Vertex_1, tri2->Vertex_2, tri2->Vertex_3, tri1->Vertex_3);


	if (p2_tri1_vertex1 > 0 && p2_tri1_vertex2 > 0 && p2_tri1_vertex3 > 0)
	{
		return NONINTERSECT;
	}

	if (p2_tri1_vertex1 < 0 && p2_tri1_vertex2 < 0 && p2_tri1_vertex3 < 0)
	{
		return NONINTERSECT;
	}
	//两点在三角面所在的平面中
	if (p2_tri1_vertex1 == 0 && p2_tri1_vertex2 == 0 && p2_tri1_vertex3 != 0)
	{
		if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_1)&&is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_1) && !is_pointTri_within_triangle(tri2, tri1->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_2) && !is_pointTri_within_triangle(tri2, tri1->Vertex_1))
		{
			return NONINTERSECT;
		}
		else if (line_triangle_intersert_inSamePlane(tri2, tri1->Vertex_1, tri1->Vertex_2))
		{
			return INTERSECT;
		}
	}
	else if (p2_tri1_vertex1 == 0 && p2_tri1_vertex3 == 0 && p2_tri1_vertex2 != 0)
	{
		if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_1)&& is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_1) && !is_pointTri_within_triangle(tri2, tri1->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_3) && !is_pointTri_within_triangle(tri2, tri1->Vertex_1))
		{
			return NONINTERSECT;
		}
		else if (line_triangle_intersert_inSamePlane(tri2, tri1->Vertex_1, tri1->Vertex_3))
		{
			return INTERSECT;
		}
	}
	else if (p2_tri1_vertex3 == 0 && p2_tri1_vertex2 == 0 && p2_tri1_vertex1 != 0)
	{
		if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_3)&& is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_3) && !is_pointTri_within_triangle(tri2, tri1->Vertex_2))
		{
			return NONINTERSECT;
		}
		else if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_2) && !is_pointTri_within_triangle(tri2, tri1->Vertex_3))
		{
			return NONINTERSECT;
		}
		else if (line_triangle_intersert_inSamePlane(tri2, tri1->Vertex_3, tri1->Vertex_2))
		{
			return INTERSECT;
		}
	}

	if (p2_tri1_vertex1 == 0 && p2_tri1_vertex2 * p2_tri1_vertex3 != 0)
	{
		//共顶点且只有一个，排除
		if (is_pointTri_within_triangle(tri2, tri1->Vertex_1))
		{
			if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_1))
			{
				return NONINTERSECT;
			}
			return INTERSECT; ////////////////////////////
		}
		else
		{
			return NONINTERSECT;
		}
	}

	if (p2_tri1_vertex2 == 0 && p2_tri1_vertex1 * p2_tri1_vertex3 != 0)
	{
		//共顶点且只有一个，排除
		if (is_pointTri_within_triangle(tri2, tri1->Vertex_2))
		{
			if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_2))
			{
				return NONINTERSECT;
			}
			return INTERSECT;
		}
		else
		{
			return NONINTERSECT;
		}
	}

	if (p2_tri1_vertex3 == 0 && p2_tri1_vertex1 * p2_tri1_vertex2 != 0)
	{
		//共顶点且只有一个，排除
		if (is_pointTri_within_triangle(tri2, tri1->Vertex_3))
		{
			if (is_pointTri_within_triangle_vectex(tri2, tri1->Vertex_3))
			{
				return NONINTERSECT;
			}
			return INTERSECT;
		}
		else
		{
			return NONINTERSECT;
		}
	}

	//最后一种情况，分居两侧
	if (is_TriangleIntersect_within_NoSamePlane(tri1,tri2))
	{
		return INTERSECT;
	}
	else
	{
		return NONINTERSECT;
	}




	//float *tri1_a = tri1->Vertex_1, *tri1_b = tri1->Vertex_2, *tri1_c = tri1->Vertex_3
	//	, *tri2_a = tri2->Vertex_1, *tri2_b = tri2->Vertex_2, *tri2_c = tri2->Vertex_3;
	//float* m;
	//float im;
	//if (p2_tri1_vertex2 * p2_tri1_vertex3 >= 0 && p2_tri1_vertex1 != 0)
	//{
	//	if (p2_tri1_vertex1 < 0)
	//	{
	//		m = tri2_b;
	//		tri2_b = tri2_c;
	//		tri2_c = m;
	//		im = p1_tri2_vertex2;
	//		p1_tri2_vertex2 = p1_tri2_vertex3;
	//		p1_tri2_vertex3 = im;
	//	}
	//}
	//else if (p2_tri1_vertex1 * p2_tri1_vertex3 >= 0 && p2_tri1_vertex2 != 0)
	//{
	//	m = tri1_a;
	//	tri1_a = tri1_b;
	//	tri1_b = tri1_c;
	//	tri1_c = m;
	//	if (p2_tri1_vertex2 < 0)
	//	{
	//		m = tri2_b;
	//		tri2_b = tri2_c;
	//		tri2_c = m;
	//		im = p1_tri2_vertex2;
	//		p1_tri2_vertex2 = p1_tri2_vertex3;
	//		p1_tri2_vertex3 = im;
	//	}
	//}
	//else if (p2_tri1_vertex1 * p2_tri1_vertex2 >= 0 && p2_tri1_vertex3 != 0)
	//{
	//	m = tri1_a;
	//	tri1_a = tri1_c;
	//	tri1_c = tri1_b;
	//	tri1_b = m;
	//	if (p2_tri1_vertex3 < 0)
	//	{
	//		m = tri2_b;
	//		tri2_b = tri2_c;
	//		tri2_c = m;
	//		im = p1_tri2_vertex2;
	//		p1_tri2_vertex2 = p1_tri2_vertex3;
	//		p1_tri2_vertex3 = im;
	//	}
	//}
	//if (p1_tri2_vertex2 * p1_tri2_vertex3 >= 0 && p1_tri2_vertex1 != 0)
	//{
	//	if (p1_tri2_vertex1 < 0)
	//	{
	//		m = tri1_b;
	//		tri1_b = tri1_c;
	//		tri1_c = m;
	//	}
	//}
	//else if (p1_tri2_vertex1 * p1_tri2_vertex3 >= 0 && p1_tri2_vertex2 != 0)
	//{
	//	m = tri2_a;
	//	tri2_a = tri2_b;
	//	tri2_b = tri2_c;
	//	tri2_c = m;
	//	if (p1_tri2_vertex2 < 0)
	//	{
	//		m = tri1_b;
	//		tri1_b = tri1_c;
	//		tri1_c = m;
	//	}
	//}
	//else if (p1_tri2_vertex1 * p1_tri2_vertex2 >= 0 && p1_tri2_vertex3 != 0)
	//{
	//	m = tri2_a;
	//	tri2_a = tri2_c;
	//	tri2_c = tri2_b;
	//	tri2_b = m;
	//	if (p1_tri2_vertex3 < 0)
	//	{
	//		m = tri1_b;
	//		tri1_b = tri1_c;
	//		tri1_c = m;
	//	}
	//}
	////..有问题
	//if (get_vector4_det(tri1_a, tri1_b, tri2_a, tri2_b) <= 0 && get_vector4_det(tri1_a, tri1_c, tri2_c, tri2_a) <= 0)
	//{
	//	return INTERSECT;
	//}
	//else
	//{
	//	return NONINTERSECT;
	//}

}

//判断两个三角形是否是同一个三角形

#endif //TRIANGLE_HAN
