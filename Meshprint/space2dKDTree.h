#pragma once

#include <vector>
#include "HE_mesh\Mesh3D.h"
#include "globalFunctions.h"

#ifndef EPSILON00001
#define EPSILON00001 1e-4f
#endif // !EPSILON00001

class LineSegment
{
public:
	Vec3f AA, BB; // bounding box
	Vec3f A, B, center;

	LineSegment(Vec3f A, Vec3f B){
		AA[0] = std::min(A[0], B[0]);
		AA[1] = std::min(A[1], B[1]);
		AA[2] = std::min(A[2], B[2]);

		BB[0] = std::max(A[0], B[0]);
		BB[1] = std::max(A[1], B[1]);
		BB[2] = std::max(A[2], B[2]);

		this->A = A;
		this->B = B;
		center = (A + B) / 2.0f;
	}
	~LineSegment(){}

	void RayIntersection2d(Vec3f sPoint, std::vector<Vec3f> &hitPointList){
		Vec3f dir = Vec3f(1, 0, 0);
		if ((sPoint.y() - AA.y() > EPSILON00001 && BB.y() - sPoint.y() > EPSILON00001)){
			hitPointList.push_back(A + (B - A) * (sPoint.y() - A.y()) / (B.y() - A.y()));
		}
	}

	void GetBoundingBox2d(Vec3f &AA, Vec3f &BB){
		AA = this->AA;
		BB = this->BB;
	}

	inline static bool SortByY(const LineSegment *t1, const LineSegment *t2)
	{
		return t1->center[1] < t2->center[1];
	}
};

inline static bool SortByX(const Vec3f t1, const Vec3f t2)
{
	return t1[0] < t2[0];
}

static void MergeBoundingBox(Vec3f &A, Vec3f &B, Vec3f A1, Vec3f B1, Vec3f A2, Vec3f B2)
{
	A[0] = std::min(A1[0], A2[0]);
	A[1] = std::min(A1[1], A2[1]);
	A[2] = std::min(A1[2], A2[2]);

	B[0] = std::max(B1[0], B2[0]);
	B[1] = std::max(B1[1], B2[1]);
	B[2] = std::max(B1[2], B2[2]);
}

static bool XRayHitAABB2d(Vec3f sPoint, Vec3f A, Vec3f B)
{
	Vec3f dir = Vec3f(1, 0, 0);
	if ((sPoint.y() - A.y() > EPSILON00001 && B.y() - sPoint.y() > EPSILON00001))
		return true;
	return false;
}

class Space2dKDTree
{
public:
	struct Tree2dNode
	{
		Tree2dNode()
		{
			lChild = NULL;
			rChild = NULL;
			segmentIdx.clear();
		}
		~Tree2dNode()
		{
			segmentIdx.clear();
		}
		// bounding box
		Vec3f AA; // min corner, the third dimension is never used
		Vec3f BB; // max corner, the third dimension is never used

		Tree2dNode *lChild, *rChild;
		std::vector<int> segmentIdx;
	};

	Tree2dNode* rootNode; // don't forget to set it to NULL

	Space2dKDTree(std::vector<LineSegment*> &segments);
	~Space2dKDTree();

	void RayIntersection2d(Vec3f sPoint, Tree2dNode *node, std::vector<LineSegment*> segments, std::vector<Vec3f> &hitPointList)
	{
		if (XRayHitAABB2d(sPoint, node->AA, node->BB))
		{
			if (node->segmentIdx.size() > 0)
			{
				for (std::vector<int>::iterator i = node->segmentIdx.begin(); i != node->segmentIdx.end(); i++)
				{
					segments[*i]->RayIntersection2d(sPoint, hitPointList);
				}
			}
			else
			{
				RayIntersection2d(sPoint, node->lChild, segments, hitPointList);
				RayIntersection2d(sPoint, node->rChild, segments, hitPointList);
			}
		}
	}

private:
	void BuildKDTree(std::vector<LineSegment*> &segments, int head, int tail, Tree2dNode *&node);
	void DeleteKDTree(Tree2dNode *&node);
};
