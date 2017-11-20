#pragma once
#include "SliceCut.h"
#include "HE_mesh\Mesh3D.h"
#include "globalFunctions.h"
#include <algorithm>
#include "Cubes.h"
class Cubes;
class Cubes::Box;
class Cubes::sortVec;
class CutLine;
#define ABS_FLOAT_0 1e-3

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
			SafeDelete(child_node_[i]);
		SafeDelete(ocn_face_list_);
	};

	//private:
	MeshOcNode*				child_node_[8]; // safe delete in the MeshOctree deconstruct function
	AABB					ocn_aabb_;
	std::vector<int>*		ocn_face_list_;
	bool					is_leaf_;
};

class MeshOctree
{
public:
	MeshOcNode*				oc_root_;
	std::vector<HE_face*>*	oc_face_list_;
	std::vector<Vec3f>		bary_center;

	MeshOctree()
		: oc_root_(NULL)
		, oc_face_list_(NULL)
	{
	}
	~MeshOctree() { DeleteMeshOcTree(); };

	void DeleteMeshOcTree();

	void BuildOctree(Mesh3D* ptr_in);

	Vec3f InteractPoint(Vec3f point_in, Vec3f d, bool hitSelf = false, const std::vector<int>& faceRegionLabel = std::vector<int>(), int curRegion = 0, int & hitID_out = *(new int(-1)));
	
	void hitOctreeNode(MeshOcNode* node_in, Vec3f point_in, Vec3f& point_out, Vec3f d, int &hitID, float &t, bool hitSelf = false, const std::vector<int>& faceRegionLabel = std::vector<int>(), int curRegion = 0);

private:
	// if child faces num are equal to it parent's, stop recursion
	void createOctree(MeshOcNode*& node_in, std::vector<int>* face_list_idx, AABB aabb, bool stopFlag);

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


class Support
{
public:
	Support(Mesh3D* ptr_in);
	~Support();

	// called by rendering-widget
	void AddPointSupport(Vec3f add_point, float p_width = -1);
	std::vector<Vec3f> getOffsetWallPoints(std::vector<Vec3f> points_in, std::vector<int> regions, int curRegion, bool isOuter,
		float small_offset = 0, std::vector<int>* segmentIdx = NULL);
	std::vector<Vec3f> AddLineSupport(std::vector<Vec3f> add_points, bool isOuter = true, const std::vector<int>& faceRegionLabel = std::vector<int>(), int curRegion = 0);
	void DeleteSupport(std::vector<int> &mousePointList);

	// set width and height
	void SetPoint(float w, float h)
	{
		if (w != 0)	point_width_ = w;
		if (h != 0)	point_height_ = h;
	}
	void SetLine(float w, float h)
	{
		if (w != 0)	line_width_ = w;
		if (h != 0)	line_height_ = h;
	}
	void SetLineWallWidth(float w)
	{
		if (w != 0)	line_wall_width_ = w;
	}

	inline void updateSupportMesh() { mesh_support_->UpdateMesh(); }

	MeshOctree* GetMeshInOctree() { return mesh_octree_; }
	MeshOctree* GetSupportOctree() { return mesh_support_octree_; }
	Mesh3D* GetMeshSupport() { return mesh_support_; }

	void BuildSupportOctree()
	{
		mesh_support_octree_->BuildOctree(mesh_support_);
	}

	void ClearData();





private:
	MeshOctree* mesh_octree_;
	Mesh3D* mesh_in_;
	MeshOctree* mesh_support_octree_;
	Mesh3D* mesh_support_;
	float point_height_;
	float point_width_;
	float line_width_;
	float line_height_;
	float line_wall_width_;

	void appendToMesh(Mesh3D* input, const std::vector<Vec3f>& verts, const std::vector<int>& triIdx);

	void pushCuboidtoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list,
		Vec3f(&center_point)[2], float w, float(&h)[2]);
	void pushLinetoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list,
		std::vector<Vec3f> center_points, std::vector<bool>& low_flag,
		float w, std::vector<float> hs = std::vector<float>());

	// tools for offset line-support
	Vec3f getDirectionInVector(Vec3f p_0, Vec3f p_1, Vec3f p_2);
	void pushLineCuboidToVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list,
		std::vector<Vec3f> center_points, std::vector<bool>& low_flag,
		float w, bool isClosed = false, bool isOuter = true, bool isTop = true, std::vector<float> hs = std::vector<float>());
	void pushLinetoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list, std::vector<Vec3f> center_points,
		bool isClosed = false, bool isOuter = true);

	// using triangle
	std::vector<int> getWallEdges(std::vector<Vec3f> points_outer, std::vector<Vec3f> points_inter, std::vector<int> segmentIdx, bool isOuter);
	void pushLineWalltoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list, std::vector<Vec3f> points_outer, std::vector<Vec3f> points_inter, std::vector<int> edges_up, std::vector<bool>& low_flag, std::vector<float> hs = std::vector<float>());
	// get support height
	double getSupportHeight(Vec3f point_in);

	Vec3f getSamplePoint3D(Vec3f point_in, const std::vector<int>& regions, int curRegion);
	//////////////////////////////////////////////////////////////////////////

public:
	void FindSupportRegion();
	std::vector<HE_vert*> InsertVertex(Vec3f pos1, Vec3f pos2, Vec3f pos3);
	std::vector<HE_vert*> InsertVertexSup(Vec3f pos1, Vec3f pos2, Vec3f pos3);
	void MarkFace();
	Mesh3D* GetRegionSup() { return mesh_region_; }
	std::vector < std::vector<Vec3f>> GetInnerLoop() { return inner_loop_; }
	std::vector<std::vector<Vec3f>> GetOutLoop() { return outer_loop_; }
	void CutByInnerLine(std::vector<Vec3f>& com_offset_vert_, int no);
	bool PointinTriangle(HE_face* face, Vec3f point_in);
	void DFSWall(HE_face* face);
	void Triangulation(std::vector<Vec3f>& in_, bool isout_, std::vector<std::vector<Vec3f>>& facelist, std::vector<std::vector<Vec3f>>&outer_, float offset_dist = DEFAULT_L);
	void CalPlaneLineIntersectPoint(Vec3f planeVector, Vec3f planePoint, Vec3f lineVector, Vec3f linePoint, Vec3f & point);



	std::vector<HE_vert *> face_verts;
	struct comHE
	{
		bool operator ()(HE_edge* a, HE_edge* b)const
		{
			if (a->insert_point_.y() < b->insert_point_.y())
			{
				return true;
			}
			return false;
		}
	};
	std::multiset<HE_vert *, comVertex>& GetSweepPoints() { return sweep_point_; };
	std::multiset<HE_edge*, comHE>& GetSegments() { return segments; };


	void Trapezoidal(std::set<HE_edge*, comHE>::iterator &iter, std::multiset <HE_vert*, comVertex>::iterator& iterP);
	bool Offset(std::vector<Vec3f>& outer_, bool isouter_, std::vector <std::vector<Vec3f>>& inner_, float offset_dist = DEFAULT_L);
	void AddNewSegments(std::multiset <HE_vert*, comVertex>::iterator iterP);
private:


	Mesh3D* mesh_region_;
	//Cubes* region_grid_;
	//Cubes* model_grid_;
	std::set<HE_vert*, comVertex> input_vertex_list_;
	std::set<HE_vert*, comVertex> input_vertex_list_sup;
	std::vector < std::vector<Vec3f>> inner_loop_;
	std::vector < std::vector<Vec3f>> outer_loop_;
	std::vector<HE_edge*> sweep_edge_;
	std::multiset<HE_edge*, comHE> segments;
	std::multiset <HE_vert*, comVertex> sweep_point_;
	void updateLineWall(std::vector<std::vector<Vec3f>> facelist, std::vector<std::vector<Vec3f>> outer_, std::vector<int> regions, int curRegion, bool isUpWall = false);
};