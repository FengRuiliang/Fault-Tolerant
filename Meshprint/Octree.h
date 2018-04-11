#pragma once
#include "HE_mesh/Mesh3D.h"
typedef struct AlignAxisBoundingBox
{
public:
	Vec3f	max_point;
	Vec3f	min_point;
	AlignAxisBoundingBox(Vec3f max_p, Vec3f min_p)
	{
		max_point = max_p;
		min_point = min_p;
	}
	AlignAxisBoundingBox(void) {}
}AABB;

class MeshOcNode
{
public:
	MeshOcNode()
	{
		for (int i = 0; i < 8; i++)
			child_node_[i] = NULL;
		ocn_face_list_ = NULL;
		is_leaf_ = false;
	};
	~MeshOcNode()
	{
		for (int i = 0; i < 8; i++)
		{
			delete child_node_[i];
			child_node_[i] = NULL;
		}
		delete ocn_face_list_;
		ocn_face_list_ = NULL;
	};

	//private:
	MeshOcNode*				child_node_[8]; // safe delete in the MeshOctree deconstruct function
	AABB					ocn_aabb_;
	std::vector<int>*		ocn_face_list_;
	bool					is_leaf_;
};

class Octree
{
public:
	MeshOcNode*				oc_root_;
	std::vector<HE_face*>*	oc_face_list_;
	std::vector<Vec3f>		bary_center;

	Octree(Mesh3D* mesh);
	~Octree() ;

	void DeleteMeshOcTree();

	void BuildOctree(Mesh3D* ptr_in);

	Vec3f InteractPoint(Vec3f point_in, Vec3f d, bool hitSelf = false, const std::vector<int>& faceRegionLabel = std::vector<int>(), int curRegion = 0, int & hitID_out = *(new int(-1)));

	void hitOctreeNode(MeshOcNode* node_in, Vec3f point_in, Vec3f& point_out, Vec3f d, int &hitID, float &t, bool hitSelf = false, const std::vector<int>& faceRegionLabel = std::vector<int>(), int curRegion = 0);

private:
	// if child faces num are equal to it parent's, stop recursion
	void create_octree(MeshOcNode*& node_in, std::vector<int>* face_list_idx, AABB aabb, bool stopFlag);

	void MergeBoundingBox(AABB &A, AABB A1, AABB A2)
	{
		A.max_point.x() = std::max(A1.max_point.x(), A2.max_point.x());
		A.max_point.y() = std::max(A1.max_point.y(), A2.max_point.y());
		A.max_point.z() = std::max(A1.max_point.z(), A2.max_point.z());

		A.min_point.x() = std::min(A1.min_point.x(), A2.min_point.x());
		A.min_point.y() = std::min(A1.min_point.y(), A2.min_point.y());
		A.min_point.z() = std::min(A1.min_point.z(), A2.min_point.z());
	}

	bool RayHitAABB(Vec3f raySPoint, Vec3f rayDirection, Vec3f A, Vec3f B);

	float isHitTriangle(HE_face* face_in, Vec3f point_in, Vec3f& point_out, Vec3f d, bool hitSelf);

	bool isCollidAABB(AABB ab_1, AABB ab_2)
	{
		bool iscoll = true;
		Vec3f coll_p1 = ab_2.max_point - ab_1.min_point;
		Vec3f coll_p2 = ab_2.min_point - ab_1.max_point;
		if (coll_p1.x() < 0 || coll_p1.y() < 0 || coll_p1.z() < 0)
			iscoll = false;
		else if (coll_p2.x() > 0 || coll_p2.y() > 0 || coll_p2.z() > 0)
			iscoll = false;
		return iscoll;
	}
};
