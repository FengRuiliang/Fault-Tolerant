#pragma once
#include "HE_mesh/Mesh3D.h"
#include "MSAABB.h"
#include <algorithm>
class AABB;
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
		A._max.x() = std::max(A1._max.x(), A2._max.x());
		A._max.y() = std::max(A1._max.y(), A2._max.y());
		A._max.z() = std::max(A1._max.z(), A2._max.z());

		A._min.x() = std::min(A1._min.x(), A2._min.x());
		A._min.y() = std::min(A1._min.y(), A2._min.y());
		A._min.z() = std::min(A1._min.z(), A2._min.z());
	}

	bool RayHitAABB(Vec3f raySPoint, Vec3f rayDirection, Vec3f A, Vec3f B);

	float isHitTriangle(HE_face* face_in, Vec3f point_in, Vec3f& point_out, Vec3f d, bool hitSelf);

	bool isCollidAABB(AABB ab_1, AABB ab_2)
	{
		bool iscoll = true;
		Vec3f coll_p1 = ab_2._max - ab_1._min;
		Vec3f coll_p2 = ab_2._min - ab_1._max;
		if (coll_p1.x() < 0 || coll_p1.y() < 0 || coll_p1.z() < 0)
			iscoll = false;
		else if (coll_p2.x() > 0 || coll_p2.y() > 0 || coll_p2.z() > 0)
			iscoll = false;
		return iscoll;
	}
};
