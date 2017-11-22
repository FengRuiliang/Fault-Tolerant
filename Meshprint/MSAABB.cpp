#include "MSAABB.h"
#include <assert.h>

AABB::AABB()
{
	reset();
}

AABB::AABB(const Vec3f& max, const Vec3f& min)
{
	set(max, min);
}

AABB::AABB(const AABB& box)
{
	set(box._min, box._max);
}

Vec3f AABB::getCenter()
{
	Vec3f center;
	center.x() = 0.5f*(_min.x() + _max.x());
	center.y() = 0.5f*(_min.y() + _max.y());
	center.z() = 0.5f*(_min.z() + _max.z());
	return center;
}

void AABB::getCorners(Vec3f *dst) const
{
	assert(dst);

	// Near face, specified counter-clockwise looking towards the origin from the positive z-axis.
	// Left-top-front.
	dst[0].set(_min.x(), _max.y(), _max.z());
	// Left-bottom-front.
	dst[1].set(_min.x(), _min.y(), _max.z());
	// Right-bottom-front.
	dst[2].set(_max.x(), _min.y(), _max.z());
	// Right-top-front.
	dst[3].set(_max.x(), _max.y(), _max.z());

	// Far face, specified counter-clockwise looking towards the origin from the negative z-axis.
	// Right-top-back.
	dst[4].set(_max.x(), _max.y(), _min.z());
	// Right-bottom-back.
	dst[5].set(_max.x(), _min.y(), _min.z());
	// Left-bottom-back.
	dst[6].set(_min.x(), _min.y(), _min.z());
	// Left-top-back.
	dst[7].set(_min.x(), _max.y(), _min.z());
}

bool AABB::intersects(const AABB& aabb) const
{
	return ((_min.x() >= aabb._min.x() && _min.x() <= aabb._max.x()) || (aabb._min.x() >= _min.x() && aabb._min.x() <= _max.x())) &&
		((_min.y() >= aabb._min.y() && _min.y() <= aabb._max.y()) || (aabb._min.y() >= _min.y() && aabb._min.y() <= _max.y())) &&
		((_min.z() >= aabb._min.z() && _min.z() <= aabb._max.z()) || (aabb._min.z() >= _min.z() && aabb._min.z() <= _max.z()));
}

bool AABB::containPoint(const Vec3f& point) const
{
	if (point.x() < _min.x()) return false;
	if (point.y() < _min.y()) return false;
	if (point.z() < _min.z()) return false;
	if (point.x() > _max.x()) return false;
	if (point.y() > _max.y()) return false;
	if (point.z() > _max.z()) return false;
	return true;
}

void AABB::merge(const AABB& box)
{
	// Calculate the new minimum point.
	_min.x() = std::min(_min.x(), box._min.x());
	_min.y() = std::min(_min.y(), box._min.y());
	_min.z() = std::min(_min.z(), box._min.z());

	// Calculate the new maximum point.
	_max.x() = std::max(_max.x(), box._max.x());
	_max.y() = std::max(_max.y(), box._max.y());
	_max.z() = std::max(_max.z(), box._max.z());
}

void AABB::set(const Vec3f& max, const Vec3f& min)
{
	this->_min = min;
	this->_max = max;
}

void AABB::reset()
{
	_min.set(99999.0f, 99999.0f, 99999.0f);
	_max.set(-99999.0f, -99999.0f, -99999.0f);
}

bool AABB::isEmpty() const
{
	return _min.x() > _max.x() || _min.y() > _max.y() || _min.z() > _max.z();
}

void AABB::updateMinMax(const Vec3f point)
{
		// Leftmost point.
		if (point.x() < _min.x())
			_min.x() = point.x();
		// Lowest point.
		if (point.y() < _min.y())
			_min.y() = point.y();
		// Farthest point.
		if (point.z() < _min.z())
			_min.z() = point.z();
		// Rightmost point.
		if (point.x() > _max.x())
			_max.x() = point.x();
		// Highest point.
		if (point.y() > _max.y())
			_max.y() = point.y();
		// Nearest point.
		if (point.z() > _max.z())
			_max.z() = point.z();

		//if (point.x() < _min.x()&&point.y()<_min.y()&&point.z()<_min.y())
		//{
		//	_min = point;
		//}
		//if (point.x()>_max.x()&&point.y()>_max.y()&&point.z()>_max.z())
		//{
		//	_max = point;
		//}
}

void AABB::expandBoundary() {

//#define MAX_FLOAT_VALUE (static_cast<float>(10e10))
//#define MIN_FLOAT_VALUE	(static_cast<float>(-10e10))
//
//	_max.x() = _max.y() = _max.z() = MIN_FLOAT_VALUE;
//	_min.x() = _min.y() = _min.z() = MAX_FLOAT_VALUE;
//
//	for (int i=0;i<3;i++)
//	{
//		_min.x() =std::min(_min.x(), (*viter)->position_.x());
//		_min.y() = min(_min.y(), (*viter)->position_.y());
//		_min.z() = min(_min.z(), (*viter)->position_.z());
//		_max.x() = max(_max.x(), (*viter)->position_.x());
//		_max.y() = max(_max.y(), (*viter)->position_.y());
//		_max.z() = max(_max.z(), (*viter)->position_.z());
//	}

	//扩大包围盒外檐，不至于只是一个面
	_min.x() = _min.x() - 0.01;
	_min.y() = _min.y() - 0.01;
	_min.z() = _min.z() - 0.01;

	_max.x() = _max.x() + 0.01;
	_max.y() = _max.y() + 0.01;
	_max.z() = _max.z() + 0.01;
}

//void AABB::transform(const Mat4& mat)
//{
//	Vec3f corners[8];
//	// Near face, specified counter-clockwise
//	// Left-top-front.
//	corners[0].set(_min.x(), _max.y(), _max.z());
//	// Left-bottom-front.
//	corners[1].set(_min.x(), _min.y(), _max.z());
//	// Right-bottom-front.
//	corners[2].set(_max.x(), _min.y(), _max.z());
//	// Right-top-front.
//	corners[3].set(_max.x(), _max.y(), _max.z());
//
//	// Far face, specified clockwise
//	// Right-top-back.
//	corners[4].set(_max.x(), _max.y(), _min.z());
//	// Right-bottom-back.
//	corners[5].set(_max.x(), _min.y(), _min.z());
//	// Left-bottom-back.
//	corners[6].set(_min.x(), _min.y(), _min.z());
//	// Left-top-back.
//	corners[7].set(_min.x(), _max.y(), _min.z());
//
//	// Transform the corners, recalculate the min and max points along the way.
//	for (int i = 0; i < 8; i++)
//		mat.transformPoint(&corners[i]);
//
//	reset();
//
//	updateMinMax(corners, 8);
//}