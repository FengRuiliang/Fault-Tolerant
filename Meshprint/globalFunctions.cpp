#include "qmath.h"
#include "globalFunctions.h"
float field_width_ = 5.0;
float field_height_ = 5.0;
float line_width_ = 0.150;
float field_overlap_ = 0.09;
float unit = 0.01;
int units_y_ = field_height_ / line_width_;
int units_x_ = field_width_ / line_width_;
float thickness_ = 0.3;
float offset_dis_ = 0.051;
int * num_hatch;
float laser_power_hatch_ = 200;
float laser_speed_hatch_ = 500;
float laser_power_polygon_ = 100;
float laser_speed_polygon_ = 700;
int increment_angle_ = 67;
float DEFAULT_L = 0.5f;
float THRESHOLD = cos(3.1415926 * 60 / 180);
float THRESHOLD1 = cos(3.1415926 * 70 / 180);
float GAP = 0.8f;
float SEGLENGTH = 1.5f;
float RESO = 0.5f;
float VERTICALGAP = 2.f;
float scaleV = 1.0;
float scaleT = 1.0;
int sss = 0;
int fildID = 0;
//////////////////////////////////////////////////////////////////////////
bool PointinTriangle(HE_face* face, Vec3f point_in)
{

	std::vector<HE_vert*> verts;
	verts = face->vertices_;
	Vec3f A(verts[0]->position().x(), verts[0]->position().y(), 0.0);
	Vec3f B(verts[1]->position().x(), verts[1]->position().y(), 0.0);
	Vec3f C(verts[2]->position().x(), verts[2]->position().y(), 0.0);
	Vec3f P = point_in;
	Vec3f v0 = C - A;
	Vec3f v1 = B - A;
	Vec3f v2 = P - A;

	float dot00 = v0.dot(v0);
	float dot01 = v0.dot(v1);
	float dot02 = v0.dot(v2);
	float dot11 = v1.dot(v1);
	float dot12 = v1.dot(v2);

	float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

	float u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
	if (u < 0 || u > 1) // if u out of range, return directly
	{
		return false;
	}

	float v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
	if (v < 0 || v > 1) // if v out of range, return directly
	{
		return false;
	}

	return u + v <= 1;
}

void CalPlaneLineIntersectPoint(Vec3f planeVector, Vec3f planePoint, Vec3f lineVector, Vec3f linePoint, Vec3f& point)
{
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

	if (abs(vpt) < 1e-3)
	{
		point = linePoint + lineVector;
		if (point < linePoint)
		{

			point = linePoint;
		}
		point.y() -= 0.01;
	}
	else
	{
		t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
		point.x() = m1 + v1 * t;
		point.y() = m2 + v2 * t;
		point.z() = m3 + v3 * t;
	}
}

//////////////////////////////////////////////////////////////////////////
