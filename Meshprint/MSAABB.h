#ifndef _AABB_H__
#define _AABB_H__

#include "HE_mesh\Vec.h"

typedef trimesh::vec3  Vec3f;

/**
* Axis Aligned Bounding Box (AABB), usually calculate some rough but fast collision detection.
*/
class  AABB
{

public:
	AABB();

	AABB(const Vec3f& min, const Vec3f& max);

	

	/**
	* Constructor.
	*/
	AABB(const AABB& box);

	/**
	* Gets the center point of the bounding box.
	*/
	Vec3f getCenter();

	/* Near face, specified counter-clockwise looking towards the origin from the positive z-axis.
	* verts[0] : left top front
	* verts[1] : left bottom front
	* verts[2] : right bottom front
	* verts[3] : right top front
	*
	* Far face, specified counter-clockwise looking towards the origin from the negative z-axis.
	* verts[4] : right top back
	* verts[5] : right bottom back
	* verts[6] : left bottom back
	* verts[7] : left top back
	*/
	void getCorners(Vec3f *dst) const;

	/**
	* Tests whether this bounding box intersects the specified bounding object.
	*/
	bool intersects(const AABB& aabb) const;

	/**
	* check whether the point is in.
	*/
	bool containPoint(const Vec3f& point) const;

	/**
	* Sets this bounding box to the smallest bounding box
	* that contains both this bounding object and the specified bounding box.
	*/
	void merge(const AABB& box);

	/**
	* Sets this bounding box to the specified values.
	*/
	void set(const Vec3f& min, const Vec3f& max);

	/**
	* Reset min and max value.If you invoke this method, isEmpty() shall return true.
	*/
	void reset();

	/* 
	*expand Boundary  add 0.01
	*/
	void expandBoundary();

	/**
	* check the AABB object is empty(reset).
	*/
	bool isEmpty() const;

	/**
	* update the _min and _max from the given point.
	*/
	void updateMinMax(const Vec3f point);

	/**
	* Transforms the bounding box by the given transformation matrix.
	*/
	//void transform(const Mat4& mat);

public:
	Vec3f _min;
	Vec3f _max;
};

// end of 3d group

#endif