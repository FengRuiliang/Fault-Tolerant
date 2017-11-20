#include "Support.h"
#include "openGLProjector.h"
#include <set>
#include <queue>
#include "clipper.hpp"
#include "QDebug"
#include "SliceCut.h"
// Triangulation
#include "triangle/triangle.h"
using namespace ClipperLib;
class CutLine;

void MeshOctree::DeleteMeshOcTree()
{
	SafeDelete(this->oc_root_);
}

void MeshOctree::BuildOctree(Mesh3D* ptr_in)
{
	oc_face_list_ = ptr_in->get_faces_list();
	std::vector<int>* face_list_idx = new std::vector < int >;
	bary_center.clear();
	face_list_idx->reserve(oc_face_list_->size());
	bary_center.reserve(oc_face_list_->size());

	for (int i = 0; i < oc_face_list_->size(); i++)
	{
		face_list_idx->push_back(i);

		std::vector<HE_vert *> tmp_face_verts;
		oc_face_list_->at(i)->face_verts(tmp_face_verts);
		Vec3f tmp_bary_center;
		for (int j = 0; j < tmp_face_verts.size(); j++)
			tmp_bary_center += tmp_face_verts[j]->position_;
		tmp_bary_center /= tmp_face_verts.size();
		bary_center.push_back(tmp_bary_center);
	}
	AABB aabb = AABB(ptr_in->getBoundingBox()[0], ptr_in->getBoundingBox()[1]);
	DeleteMeshOcTree();
	createOctree(oc_root_, face_list_idx, aabb, false);

	SafeDelete(face_list_idx);
}

void MeshOctree::createOctree(MeshOcNode*& node_in, std::vector<int>* face_list_idx, AABB aabb, bool stopFlag)
{
	node_in = new MeshOcNode();
	if (face_list_idx->size() == 0)
		return;

	if (face_list_idx->size() <= 8 || stopFlag)
	{
		node_in->is_leaf_ = true;
		node_in->ocn_face_list_ = new std::vector < int >;
		for (std::vector<int>::iterator i = face_list_idx->begin(); i != face_list_idx->end(); i++)
			node_in->ocn_face_list_->push_back(*i);

		// compute the bounding box
		AABB cur_aabb;
		oc_face_list_->at(face_list_idx->at(0))->getBoundingBox(cur_aabb.max_point, cur_aabb.min_point);
		for (std::vector<int>::iterator i = face_list_idx->begin(); i != face_list_idx->end(); i++)
		{
			AABB tmp_aabb;
			oc_face_list_->at(*i)->getBoundingBox(tmp_aabb.max_point, tmp_aabb.min_point);
			MergeBoundingBox(cur_aabb, cur_aabb, tmp_aabb);
		}
		node_in->ocn_aabb_ = cur_aabb;

		return;
	}

	std::vector<int>* tmp_face_list_idx[8];
	for (int i = 0; i < 8; i++)
		tmp_face_list_idx[i] = new std::vector < int >;

	Vec3f split_center = (aabb.max_point + aabb.min_point) / 2.0f;
	for (std::vector<int>::iterator i = face_list_idx->begin(); i != face_list_idx->end(); i++)
	{
		float bcx = bary_center[*i].x();
		float bcy = bary_center[*i].y();
		float bcz = bary_center[*i].z();
		if (bcy < split_center.y())
		{
			if (bcx < split_center.x() && bcz >= split_center.z())
				tmp_face_list_idx[0]->push_back(*i);
			else if (bcx >= split_center.x() && bcz >= split_center.z())
				tmp_face_list_idx[1]->push_back(*i);
			else if (bcx >= split_center.x() && bcz < split_center.z())
				tmp_face_list_idx[2]->push_back(*i);
			else
				tmp_face_list_idx[3]->push_back(*i);
		}
		else
		{
			if (bcx < split_center.x() && bcz >= split_center.z())
				tmp_face_list_idx[4]->push_back(*i);
			else if (bcx >= split_center.x() && bcz >= split_center.z())
				tmp_face_list_idx[5]->push_back(*i);
			else if (bcx >= split_center.x() && bcz < split_center.z())
				tmp_face_list_idx[6]->push_back(*i);
			else
				tmp_face_list_idx[7]->push_back(*i);
		}
	}

	for (int i = 0; i < 8; i++)
	{
		if (tmp_face_list_idx[i]->size() == face_list_idx->size())
		{
			stopFlag = true;
			break;
		}
	}

	Vec3f hab = (aabb.max_point - aabb.min_point) / 2.0f;

	createOctree(node_in->child_node_[0], tmp_face_list_idx[0], AABB(split_center, aabb.min_point), stopFlag);
	createOctree(node_in->child_node_[1], tmp_face_list_idx[1], AABB(split_center + Vec3f(hab.x(), 0, 0), aabb.min_point + Vec3f(hab.x(), 0, 0)), stopFlag);
	createOctree(node_in->child_node_[2], tmp_face_list_idx[2], AABB(aabb.max_point - Vec3f(0, hab.y(), 0), split_center - Vec3f(0, hab.y(), 0)), stopFlag);
	createOctree(node_in->child_node_[3], tmp_face_list_idx[3], AABB(split_center - Vec3f(0, 0, hab.z()), aabb.min_point - Vec3f(0, 0, hab.z())), stopFlag);

	createOctree(node_in->child_node_[4], tmp_face_list_idx[4], AABB(split_center + Vec3f(0, hab.y(), 0), aabb.min_point + Vec3f(0, hab.y(), 0)), stopFlag);
	createOctree(node_in->child_node_[5], tmp_face_list_idx[5], AABB(aabb.max_point + Vec3f(0, 0, hab.z()), split_center + Vec3f(0, 0, hab.z())), stopFlag);
	createOctree(node_in->child_node_[6], tmp_face_list_idx[6], AABB(aabb.max_point, split_center), stopFlag);
	createOctree(node_in->child_node_[7], tmp_face_list_idx[7], AABB(aabb.max_point - Vec3f(hab.x(), 0, 0), split_center - Vec3f(hab.x(), 0, 0)), stopFlag);

	for (int i = 0; i < 8; i++)
		SafeDelete(tmp_face_list_idx[i]);

	int i = 0;
	for (; i < 8; i++) {
		if (node_in->child_node_[i] != NULL) {
			node_in->ocn_aabb_ = node_in->child_node_[i]->ocn_aabb_;
			break;
		}
	}
	for (; i < 8; i++) {
		if (node_in->child_node_[i] != NULL) {
			MergeBoundingBox(node_in->ocn_aabb_, node_in->ocn_aabb_, node_in->child_node_[i]->ocn_aabb_);
		}
	}
}

Vec3f MeshOctree::InteractPoint(Vec3f point_in, Vec3f d, bool hitSelf, const std::vector<int>& faceRegionLabel, int curRegion, int &hitID_out)
{
	{
		Vec3f point_out(point_in.x(), point_in.y(), 0.0f);
		if (d[2] > 0)		
			point_out.z() = 999999.0f;

		int hitID = -1;
		float temp_t = -1.0f;		
		hitOctreeNode(oc_root_, point_in, point_out, d, hitID, temp_t, hitSelf, faceRegionLabel, curRegion);
		hitID_out = hitID;
		return point_out;
	}
}
void MeshOctree::hitOctreeNode(MeshOcNode* node_in, Vec3f point_in, Vec3f& point_out, Vec3f d, int &hitID, float &t, bool hitSelf, const std::vector<int>& faceRegionLabel, int curRegion)
{
	if (!node_in)
		return;

	if (RayHitAABB(point_in, d, node_in->ocn_aabb_.min_point, node_in->ocn_aabb_.max_point))
	{
		if (!node_in->is_leaf_)
		{
			for (int child_ind = 0; child_ind < 8; child_ind++)
				hitOctreeNode(node_in->child_node_[child_ind], point_in, point_out, d, hitID, t, hitSelf, faceRegionLabel, curRegion);
		}
		else
		{
			std::vector<int>* faces = node_in->ocn_face_list_;
			for (std::vector<int>::iterator fidx = faces->begin(); fidx != faces->end(); ++fidx)
			{
				if (curRegion != 0 && faceRegionLabel[*fidx] != curRegion)
					continue;

				Vec3f temp_point;
				float temp_t = isHitTriangle(oc_face_list_->at(*fidx), point_in, temp_point, d, hitSelf);

				if (temp_t > -ABS_FLOAT_0 && (temp_t < t || t < -0.5f))
				{
					hitID = *fidx;
					t = temp_t;
					point_out = temp_point;
				}
			}
		}
	}
}

bool MeshOctree::RayHitAABB(Vec3f raySPoint, Vec3f rayDirection, Vec3f A, Vec3f B)
{
	float dx = rayDirection[0];
	float dy = rayDirection[1];
	float dz = rayDirection[2];

	Vec3f A_RS = A - raySPoint;
	Vec3f B_RS = B - raySPoint;

	if (abs(dx) < ABS_FLOAT_0 && (A_RS.x() > ABS_FLOAT_0 || B_RS.x() < -ABS_FLOAT_0))
		return false;
	if (abs(dy) < ABS_FLOAT_0 && (A_RS.y() > ABS_FLOAT_0 || B_RS.y() < -ABS_FLOAT_0))
		return false;
	if (abs(dz) < ABS_FLOAT_0 && (A_RS.z() > ABS_FLOAT_0 || B_RS.z() < -ABS_FLOAT_0))
		return false;

	float maxTMin = -99999, minTMax = 99999, maxTMax = -99999;

	if (abs(dx) > ABS_FLOAT_0)
	{
		float t1 = A_RS.x() / rayDirection[0];
		float t2 = B_RS.x() / rayDirection[0];
		if (t1 > t2)
			MySwap(t1, t2);

		if (maxTMin < t1) maxTMin = t1;
		if (minTMax > t2) minTMax = t2;
		if (maxTMax < t2) maxTMax = t2;
	}
	if (abs(dy) > ABS_FLOAT_0)
	{
		float t1 = A_RS.y() / rayDirection[1];
		float t2 = B_RS.y() / rayDirection[1];
		if (t1 > t2)
			MySwap(t1, t2);

		if (maxTMin < t1) maxTMin = t1;
		if (minTMax > t2) minTMax = t2;
		if (maxTMax < t2) maxTMax = t2;
	}
	if (abs(dz) > ABS_FLOAT_0)
	{
		float t1 = A_RS.z() / rayDirection[2];
		float t2 = B_RS.z() / rayDirection[2];
		if (t1 > t2)
			MySwap(t1, t2);

		if (maxTMin < t1) maxTMin = t1;
		if (minTMax > t2) minTMax = t2;
		if (maxTMax < t2) maxTMax = t2;
	}

	if (maxTMin < minTMax + ABS_FLOAT_0 && maxTMax > -ABS_FLOAT_0)
		return true;
	else
		return false;
}

float MeshOctree::isHitTriangle(HE_face* face_in, Vec3f point_in, Vec3f& point_out, Vec3f d, bool hitSelf)
{
	std::vector<HE_vert *> face_verts;
	face_in->face_verts(face_verts);

	Vec3f s = point_in - face_verts[0]->position_;

	Vec3f eAC = face_verts[2]->position_ - face_verts[0]->position_;
	Vec3f eAB = face_verts[1]->position_ - face_verts[0]->position_;

	Vec3f faceNormal = eAB.cross(eAC);
	faceNormal.normalize();
	float denominator = d.cross(eAC).dot(eAB);

	float b1 = d.cross(eAC).dot(s) / denominator;
	float b2 = eAB.cross(d).dot(s) / denominator;
	float t = eAB.cross(eAC).dot(s) / denominator;

	if (t >= ABS_FLOAT_0 && b1 > -ABS_FLOAT_0 && b2 > -ABS_FLOAT_0 && b1 + b2 < 1 + ABS_FLOAT_0)
	{
		if (faceNormal.dot(d) < 0 || hitSelf)
		{
			point_out = point_in + d * t;
			return t;
		}
	}
	// if hit point(s) are very close
	if (t > -ABS_FLOAT_0 && t < ABS_FLOAT_0 && b1 > -ABS_FLOAT_0 && b2 > -ABS_FLOAT_0 && b1 + b2 < 1 + ABS_FLOAT_0)
	{
		// hit self
		if (faceNormal.dot(d) >= 0 && hitSelf)
		{
			point_out = point_in;
			return 0;
		}
		// this is not hit self, we need to count this
		if (faceNormal.dot(d) < 0)
		{
			point_out = point_in;
			return 0;
		}
	}

	// no hit
	return -1.0f;
}


Support::Support(Mesh3D* ptr_in)
	: mesh_in_(ptr_in)
{
	mesh_octree_ = new MeshOctree();
	mesh_support_ = new Mesh3D();
	mesh_support_octree_ = new MeshOctree();
	point_height_ = DEFAULT_L;
	point_width_ = DEFAULT_L;
	line_width_ = DEFAULT_L;
	line_height_ = DEFAULT_L;
	line_wall_width_ = DEFAULT_L;
	//////////////////////////////////////////////////////////////////////////
	mesh_region_ = new Mesh3D;
};

Support::~Support() {
	ClearData();
	SafeDelete(mesh_octree_);
	SafeDelete(mesh_support_octree_);
	SafeDelete(mesh_support_);
	SafeDelete(mesh_region_);
}

double Support::getSupportHeight(Vec3f point_in) {
	Vec3f point_out = mesh_octree_->InteractPoint(point_in, Vec3f(0, 0, -1));
	return (point_out - point_in).length();
}




void Support::AddPointSupport(Vec3f add_point, float p_width) {
	if (p_width == -1)
	{
		p_width = point_width_;
	}
	std::vector<Vec3f> add_points_list;
	std::vector<int> add_idx_list;

	// add top-cuboid
	Vec3f center_points[2] = { add_point - Vec3f(0, p_width / 2, 0), add_point + Vec3f(0, p_width / 2, 0) };
	float support_heights[2] = { point_height_, point_height_ };
	pushCuboidtoVector(add_points_list, add_idx_list, center_points, p_width, support_heights);
	// update mesh
	appendToMesh(mesh_support_, add_points_list, add_idx_list);

	// add the cross-support
	center_points[0] += Vec3f(0, 0, -point_height_);
	center_points[1] += Vec3f(0, 0, -point_height_);
	support_heights[0] = support_heights[1] = getSupportHeight(add_point) - point_height_;
	if (support_heights[0] < 0.5)
	{
		//std::cout << "point's position is too low" << std::endl;
		return;
	}
	pushCuboidtoVector(add_points_list, add_idx_list, center_points, p_width * 3, support_heights);
	// update mesh
	appendToMesh(mesh_support_, add_points_list, add_idx_list);

	center_points[0] -= Vec3f(0, p_width, 0);
	center_points[1] += Vec3f(0, p_width, 0);
	pushCuboidtoVector(add_points_list, add_idx_list, center_points, p_width, support_heights);

	appendToMesh(mesh_support_, add_points_list, add_idx_list);
}

// tools for offset points
Vec3f verticalDir2D(Vec3f p_1, Vec3f p_2) {
	if (p_1 == p_2)
	{
		return Vec3f(0, 0, 0);
	}
	Vec3f length_dir = p_1 - p_2;
	Vec3f width_dir(length_dir[1], -length_dir[0], 0);
	width_dir.normalize();
	return width_dir;
}
Vec3f Support::getDirectionInVector(Vec3f p_0, Vec3f p_1, Vec3f p_2)
{
	// p_0: current point; p_1: point before; p_2: point after
	// get the direction in 2D
	int inter_flag = -1;
	Vec3f width_dir = verticalDir2D(p_1, p_0) + verticalDir2D(p_0, p_2);
	width_dir /= 2;
	width_dir.normalize();
	return width_dir;
}
std::vector<Vec3f> Support::getOffsetWallPoints(std::vector<Vec3f> points_in, std::vector<int> regions, int curRegion, bool isOuter, float small_offset, std::vector<int>* segmentIdx)
{
	int outer_flag = isOuter ? -1 : 1;
	Path subj;
	ClipperOffset co;
	Paths solution;
	// prepare points to offset
	int p_num = points_in.size();
	for (int i = 0; i < p_num; i++)
	{
		int X_ = points_in[i].x()*1e6;
		int Y_ = points_in[i].y()*1e6;
		subj << IntPoint(X_, Y_);
	}
	// do offset operation
	co.AddPath(subj, jtMiter, etClosedPolygon);
	co.Execute(solution, outer_flag*(line_wall_width_ + small_offset)*1e6);
	std::vector<Vec3f> points_out;
	if (solution.size() > 1)
	{
		int t = solution.size();
		qDebug() << "offset solution size" << t << "\n input points num:" << p_num;
		//return points_out;
	}
	for (int l = 0; l < solution.size(); l++)
	{
		for (int m = 0; m < solution[l].size(); m++)
		{
			Vec3f cur_p(solution[l][m].X*1e-6, solution[l][m].Y*1e-6, 0);
			// hit within current region to get 3D coordinate
			cur_p = mesh_octree_->InteractPoint(cur_p, Vec3f(0, 0, 1), false, regions, curRegion);
			points_out.push_back(cur_p);
		}
	}
	//if (points_out.size() < p_num / 2)
	//{
	//	points_out = points_in;
	//	if (segmentIdx != NULL)
	//	{
	//		segmentIdx->push_back(0);
	//		segmentIdx->push_back(points_out.size());
	//	}
	//}
	if (segmentIdx != NULL)
	{
		segmentIdx->clear();
		int idx = 0;
		for (int l = 0; l < solution.size(); l++)
		{
			segmentIdx->push_back(idx);
			idx += solution[l].size();
		}
		segmentIdx->push_back(idx);
	}

	return points_out;
}

// tools for triangulation: using CMUtriangle
std::vector<int> Support::getWallEdges(std::vector<Vec3f> points_outer, std::vector<Vec3f> points_inter, std::vector<int> segmentIdx, bool isOuter)
{
	int outer_num = points_outer.size();
	int inter_num = points_inter.size();
	struct triangulateio t_in, t_out, vorout;
	// initialize in parameters
	t_in.numberofpointattributes = 0;
	t_in.numberofpoints = outer_num + inter_num;
	t_in.pointlist = (REAL *)malloc(t_in.numberofpoints * 2 * sizeof(REAL));
	// points
	for (int i = 0; i < outer_num; i++)
	{
		t_in.pointlist[2 * i] = points_outer[i].x();
		t_in.pointlist[2 * i + 1] = points_outer[i].y();
	}
	for (int i = 0; i < inter_num; i++)
	{
		t_in.pointlist[2 * (outer_num + i)] = points_inter[i].x();
		t_in.pointlist[2 * (outer_num + i) + 1] = points_inter[i].y();
	}
	t_in.pointmarkerlist = NULL;
	// segments
	t_in.numberofsegments = inter_num + outer_num;
	t_in.segmentlist = (int *)malloc(t_in.numberofsegments * 2 * sizeof(int));
	for (int i = 0; i < outer_num; i++)
	{
		t_in.segmentlist[2 * i] = i;
		t_in.segmentlist[2 * i + 1] = (i + 1) % outer_num;
	}
	// holes and related segments
	if (segmentIdx.size() < 2)
	{
		qDebug() << "segments less than one";
	}
	t_in.numberofholes = segmentIdx.size() - 1;
	t_in.holelist = (REAL *)malloc(t_in.numberofholes * 2 * sizeof(REAL));
	//t_in.numberofholes = 0;
	//t_in.holelist = NULL;
	int seg_start = outer_num;
	for (int i = 0; i < segmentIdx.size() - 1; i++)
	{
		int curSegNum = segmentIdx[i + 1] - segmentIdx[i];
		Vec3f center_t(0, 0, 0);
		for (int j = segmentIdx[i]; j < segmentIdx[i + 1]; j++)
		{
			t_in.segmentlist[2 * (seg_start + j)] = j + seg_start;
			t_in.segmentlist[2 * (seg_start + j) + 1] = segmentIdx[i] + seg_start + (j + 1 - segmentIdx[i]) % curSegNum;
			center_t += points_inter[j];
		}
		seg_start += curSegNum;
		// hole points (one point inside one hole !! cannot be on the segment)
		center_t /= curSegNum;
		Vec3f cur_offp = getDirectionInVector(points_inter[segmentIdx[i] + 1], points_inter[segmentIdx[i]], points_inter[segmentIdx[i] + 2]);
		center_t = points_inter[segmentIdx[i] + 1] + 0.001f*cur_offp;
		t_in.holelist[2 * i] = center_t.x();
		t_in.holelist[2 * i + 1] = center_t.y();
	}
	t_in.segmentmarkerlist = NULL;
	t_in.numberofregions = 0;
	t_in.regionlist = NULL;

	t_out.numberoftriangleattributes = 0;
	t_out.trianglelist = NULL;
	t_out.segmentlist = NULL;
	t_out.segmentmarkerlist = NULL;
	triangulate("pqzYCNBQ", &t_in, &t_out, &vorout);
	int* edges_sol = t_out.trianglelist;
	int edge_num = t_out.numberoftriangles * 3;
	std::vector<int> edges_out;
	for (int i = 0; i < edge_num; i++)
	{
		edges_out.push_back(edges_sol[i]);
	}

	// NO-NEED: check triangles outside and delete them
	// for OuterWall: outer-clockwise, inter-anticlockwise
	// for InterWall: outer-anticlockwise, inter-clockwise
	// check: if e.dot(dir) < 0 for OuterWall / e.dot(dir) > 0 for InterWall, delete them
	//int face_num = t_out.numberoftriangles;
	//std::vector<Vec3f> points_list;
	//points_list.insert(points_list.end(), points_outer.begin(), points_outer.end());
	//points_list.insert(points_list.end(), points_inter.begin(), points_inter.end());
	//for (int i = 0; i < face_num; i++)
	//{
	//	std::vector<int> cur_face;
	//	cur_face.push_back(edges_sol[i * 3]);
	//	// sort points in current face
	//	for (int ii = 1; ii < 3; ii++)
	//	{
	//		int cur_p = edges_sol[i*3 + ii];
	//		int cur_size = cur_face.size();
	//		int tti = 0;
	//		for (tti = 0; tti < cur_size;tti++)
	//		{
	//			if (cur_p < cur_face[tti])
	//			{
	//				cur_face.insert(cur_face.begin() + tti, cur_p);
	//				break;
	//			}
	//		}
	//		if (tti == cur_size)
	//		{
	//			cur_face.push_back(cur_p);
	//		}
	//	}
	//	// check current face: 3 points continuous in point-list
	//	if (cur_face[1] - cur_face[0] == 1 && cur_face[2] - cur_face[1] == 1)
	//	{
	//		// check face-points orientation
	//		Vec3f cur_dir = getDirectionInVector(points_list[cur_face[1]], points_list[cur_face[0]], points_list[cur_face[2]]);
	//		Vec3f face_side = (points_list[cur_face[0]] + points_list[cur_face[2]]);
	//		face_side /= 2;
	//		face_side -= points_list[cur_face[1]];
	//		if ((face_side.dot(cur_dir) < 0 && isOuter) || (face_side.dot(cur_dir) > 0 && !isOuter))
	//		{
	//			// delete current face
	//			continue;
	//		}
	//		else
	//		{
	//			edges_out.insert(edges_out.end(), cur_face.begin(), cur_face.end());
	//		}
	//	}
	//}

	return edges_out;
}
void Support::pushLineWalltoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list, std::vector<Vec3f> points_outer, std::vector<Vec3f> points_inter, std::vector<int> edges_up, std::vector<bool>& low_flag, std::vector<float> hs)
{
	points_list.clear();
	idx_list.clear();
	int num = points_outer.size() + points_inter.size();
	if (num < 4)
	{
		return;
	}
	if (hs.empty())
	{
		// top-line
		for (int i = 0; i < num; i++)
		{
			hs.push_back(line_height_);
		}
	}
	for (int i = 0; i < num; i++)
	{
		if (low_flag[i])
			hs[i] = 0.0f;
	}
	// append points-up
	points_list.insert(points_list.end(), points_outer.begin(), points_outer.end());
	points_list.insert(points_list.end(), points_inter.begin(), points_inter.end());
	// edges: up
	idx_list.insert(idx_list.end(), edges_up.begin(), edges_up.end());

	int p_num = points_list.size();
	// points-down & edges-down
	for (int i = 0; i < p_num; i++)
	{
		points_list.push_back(points_list[i] + Vec3f(0, 0, -hs[i]));
	}
	for (int i = 0; i < edges_up.size(); i++)
	{
		idx_list.push_back(edges_up[i] + p_num);
	}

	// edges_aside
	int out_num = points_outer.size();
	for (int i = 0; i < out_num; i++)
	{
		int next_idx = (i + 1) % out_num;
		// add indexes
		int idx_arr[6];
		// side faces
		idx_arr[0] = i;		idx_arr[1] = i + p_num;			idx_arr[2] = next_idx + p_num;
		idx_arr[3] = i;		idx_arr[4] = next_idx + p_num;	idx_arr[5] = next_idx;
		// append edges
		idx_list.insert(idx_list.end(), idx_arr, idx_arr + 6);
	}
	int in_num = points_inter.size();
	for (int i = 0; i < in_num; i++)
	{
		int next_idx = (i + 1) % in_num;
		// add indexes
		int idx_arr[6];
		// side faces
		idx_arr[0] = i + out_num;		idx_arr[1] = next_idx + out_num + p_num;	idx_arr[2] = i + out_num + p_num;
		idx_arr[3] = i + out_num;		idx_arr[4] = next_idx + out_num;			idx_arr[5] = next_idx + out_num + p_num;
		// append edges
		idx_list.insert(idx_list.end(), idx_arr, idx_arr + 6);
	}


}

void Support::updateLineWall(std::vector<std::vector<Vec3f>> facelist, std::vector<std::vector<Vec3f>> outer_, std::vector<int> regions, int curRegion, bool isUpWall)
{
	// up and down faces
	for (int fi = 0; fi < facelist.size(); fi++)
	{
		std::vector<Vec3f> cur_f = facelist[fi];
		if (cur_f.size() < 3)
		{
			continue;
		}
		std::vector<Vec3f> cur_up_f;
		std::vector<Vec3f> cur_down_f;
		for (int fii = 0; fii < cur_f.size(); fii++)
		{
			Vec3f cur_pi = mesh_octree_->InteractPoint(cur_f[fii], Vec3f(0, 0, 1), true, regions, curRegion);
			float cur_height = getSupportHeight(cur_pi);
			//if (cur_pi.z() > 99999)
			//	continue;
			if (!isUpWall)
			{
				if (cur_height < line_height_)
				{
					cur_up_f.push_back(cur_pi);
					cur_down_f.push_back(cur_pi + Vec3f(0, 0, -cur_height));
					continue;
				}
				cur_up_f.push_back(cur_pi + Vec3f(0, 0, -line_height_));
				cur_down_f.push_back(cur_pi + Vec3f(0, 0, -cur_height));
			}
			else
			{
				if (cur_height < line_height_)
				{
					cur_up_f.push_back(cur_pi);
					cur_down_f.push_back(cur_pi);
					continue;
				}
				cur_up_f.push_back(cur_pi);
				cur_down_f.push_back(cur_pi + Vec3f(0, 0, -line_height_));
			}
		}
		if (cur_down_f.size() < 3)
		{
			continue;
		}
		std::vector<HE_vert*> tri;
		tri = InsertVertexSup(cur_up_f[0], cur_up_f[1], cur_up_f[2]);
		mesh_support_->InsertFaceSup(tri);
		tri = InsertVertexSup(cur_down_f[0], cur_down_f[2], cur_down_f[1]);
		mesh_support_->InsertFaceSup(tri);
	}
	// side faces
	std::vector<std::vector<Vec3f>> outer_up(outer_.size());
	std::vector<std::vector<Vec3f>> outer_down(outer_.size());
	// get all points
	for (int ci = 0; ci < outer_.size(); ci++)
	{
		std::vector<Vec3f> cur_c = outer_[ci];
		int cur_len = cur_c.size();
		for (int cii = 0; cii < cur_len; cii++)
		{
			Vec3f cur_pi = mesh_octree_->InteractPoint(cur_c[cii], Vec3f(0, 0, 1), false, regions, curRegion);
			float cur_height = getSupportHeight(cur_pi);
			if (cur_pi.z() > 99999)
				continue;
			if (!isUpWall)
			{
				if (cur_height < line_height_)
				{
					outer_up[ci].push_back(cur_pi);
					outer_down[ci].push_back(cur_pi + Vec3f(0, 0, -cur_height));
					continue;
				}
				outer_up[ci].push_back(cur_pi + Vec3f(0, 0, -line_height_));
				outer_down[ci].push_back(cur_pi + Vec3f(0, 0, -cur_height));
			}
			else
			{
				if (cur_height < line_height_)
				{
					outer_up[ci].push_back(cur_pi);
					outer_down[ci].push_back(cur_pi);
					continue;
				}
				outer_up[ci].push_back(cur_pi);
				outer_down[ci].push_back(cur_pi + Vec3f(0, 0, -line_height_));
			}
		}
	}
	// append all side faces
	for (int ci = 0; ci < outer_up.size(); ci++)
	{
		int cur_len = outer_up[ci].size();
		for (int cii = 0; cii < cur_len; cii++)
		{
			// side faces
			std::vector<HE_vert*> tri;
			tri = InsertVertexSup(outer_up[ci][cii], outer_down[ci][cii], outer_down[ci][(cii + 1) % cur_len]);
			mesh_support_->InsertFaceSup(tri);
			tri = InsertVertexSup(outer_up[ci][cii], outer_down[ci][(cii + 1) % cur_len], outer_up[ci][(cii + 1) % cur_len]);
			mesh_support_->InsertFaceSup(tri);
		}
	}
}
std::vector<std::vector<Vec3f>> ReSamplePoints(std::vector<std::vector<Vec3f>> points_in, int limNum, float span_w)
{
	std::vector<std::vector<Vec3f>> toAdd;

	int curPNum = 0;
	for (int ui = 0; ui < points_in.size(); ui++)
	{
		curPNum += points_in[ui].size();
	}
	if (limNum > curPNum)
	{
		// sample points
		for (int ui = 0; ui < points_in.size(); ui++)
		{
			toAdd.push_back(std::vector<Vec3f>());
			int pNum = points_in[ui].size();
			for (int i = 0; i < pNum; i++)
			{
				Vec3f point1 = points_in[ui][i];
				Vec3f point2 = points_in[ui][(i + 1) % pNum];
				float lengthTmp = (point2 - point1).length();
				Vec3f direc = (point2 - point1) / lengthTmp;
				float t = 0.0f;
				while (t < lengthTmp)
				{
					toAdd[ui].push_back(point1 + t * direc);
					t += span_w;
				}
			}

		}
	}
	else
	{
		toAdd = points_in;
	}

	return toAdd;
}
Vec3f Support::getSamplePoint3D(Vec3f point_in, const std::vector<int>& regions, int curRegion)
{
	Vec3f point1T = point_in;
	Vec3f point1TUp = GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, 1), true, regions, curRegion);
	Vec3f point1TDo = GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, -1), true, regions, curRegion);

	point1T = point1TUp[2] - point1T[2] > point1T[2] - point1TDo[2] ? point1TDo : point1TUp;
	return point1T;
}
std::vector<Vec3f> Support::AddLineSupport(std::vector<Vec3f> add_points, bool isOuter, const std::vector<int>& regions, int curRegion) {
	// add continuous line-support
	if (add_points.size() < 2)
		return add_points;

	// record new points and faces
	std::vector<Vec3f> add_points_list;
	std::vector<int> add_idx_list;

	// check if line is a circle
	bool isClosed = false;
	if (add_points.front() == add_points.back())
		isClosed = true;

	// get offset points of support if the line is wall
	std::vector<Vec3f> offset_points;
	if (isClosed)
	{
		// offset points
		//offset_points = getOffsetWallPoints(add_points, regions, curRegion, isOuter);
		// delete last point in add-points
		add_points.pop_back();
		if (!isOuter)
		{
			qDebug() << "current wall is inter.";
		}

		// using Ryan-sup triangulation

		// down-lineWall
		std::vector<std::vector<Vec3f>> upper_points;
		Offset(add_points, isOuter, upper_points, line_wall_width_ / 2 + 0.11);
		int temp_up_num = upper_points.size();
		for (int ui = 0; ui < temp_up_num; ui++)
		{
			std::vector<std::vector<Vec3f>> upper_temp;
			Offset(upper_points[ui], isOuter, upper_temp, -0.1);
			upper_points.insert(upper_points.end(), upper_temp.begin(), upper_temp.end());
		}
		upper_points.erase(upper_points.begin(), upper_points.begin() + temp_up_num);
		// sample points on the boundary and add support structure to the mesh
		upper_points = ReSamplePoints(upper_points, add_points.size() / 2, GAP+RESO+line_width_/2);
		for (int ui = 0; ui < upper_points.size(); ui++)
		{
			int cur_num = upper_points[ui].size();
			for (int uii = 0; uii < cur_num; uii++)
			{
				Vec3f add_start, add_end;
				add_start = upper_points[ui][uii];
				add_end = upper_points[ui][(uii + 1) % cur_num];
				Vec3f cur_dir = add_end - add_start;
				float dis_sum = cur_dir.length();
				while (dis_sum < RESO)
				{
					uii++;
					add_end = upper_points[ui][(uii) % cur_num];
					cur_dir = add_end - add_start;
					dis_sum = cur_dir.length();
				}
				cur_dir.normalize();
				add_start = mesh_octree_->InteractPoint(add_start, Vec3f(0, 0, 1), true, regions, curRegion);
				do
				{
					add_start = getSamplePoint3D(add_start, regions, curRegion);
					add_end = add_start + RESO*cur_dir;
					//add_end = mesh_octree_->InteractPoint(add_end, Vec3f(0, 0, 1), true, regions, curRegion);

					Vec3f center_points[2] = { add_start, add_end };
					float support_heights[2];
					// add bottom-cuboid
					center_points[0] += Vec3f(0, 0, -line_height_);
					center_points[1] += Vec3f(0, 0, -line_height_);
					support_heights[0] = getSupportHeight(add_start);
					support_heights[1] = getSupportHeight(add_end);
					if (support_heights[0] < line_height_ || support_heights[1] < line_height_)
					{
						//std::cout << "point's position is too low" << std::endl;
						break;
					}
					support_heights[0] = getSupportHeight(add_start) - line_height_;
					support_heights[1] = getSupportHeight(add_end) - line_height_;
					pushCuboidtoVector(add_points_list, add_idx_list, center_points, line_width_, support_heights);
					appendToMesh(mesh_support_, add_points_list, add_idx_list);
					// add top-cuboid
					center_points[0] = add_start;
					center_points[1] = add_end;
					support_heights[0] = support_heights[1] = line_height_;
					pushCuboidtoVector(add_points_list, add_idx_list, center_points, line_width_ / 2, support_heights);
					appendToMesh(mesh_support_, add_points_list, add_idx_list);

					dis_sum -= line_width_+RESO+GAP;
					add_start = add_end + (GAP+line_width_)*cur_dir;
				} while (dis_sum > 0);
			}
		}
	}
	else
	{
		// push points and faces
		pushLinetoVector(add_points_list, add_idx_list, add_points, isClosed);
		// update mesh
		appendToMesh(mesh_support_, add_points_list, add_idx_list);
	}

	return offset_points;
}


void Support::DeleteSupport(std::vector<int> &mousePointList)
{
	OpenGLProjector myProjector = OpenGLProjector();

	std::vector<HE_face*>* faceList = this->mesh_support_->get_faces_list();

	//all faces that are visited will be removed
	std::vector<bool> visited(faceList->size());

	std::set<int> faceToDeleteSet;
	std::set<int> edgeToDeleteSet;
	std::set<int> vertToDeleteSet;
	for (int i = 0; i < mousePointList.size(); i += 2)
	{
		// detect selected face
		Vec3f myPointN(mousePointList[i], mousePointList[i + 1], -1.0f);
		Vec3f myPointF(mousePointList[i], mousePointList[i + 1], 1.0f);
		Vec3f add_pointN = myProjector.UnProject(myPointN);
		Vec3f add_pointF = myProjector.UnProject(myPointF);
		Vec3f direc = add_pointF - add_pointN;
		add_pointN -= direc * 10.0f;
		direc.normalize();

		Vec3f hitPointT;
		int hitID = -1;
		float temp_t = -1.0f;
		this->mesh_support_octree_->hitOctreeNode(this->mesh_support_octree_->oc_root_, add_pointN, hitPointT, direc, hitID, temp_t);

		if (hitID >= 0)
		{
			std::queue<int> residualFaceList;
			if (!visited[hitID])
			{
				residualFaceList.push(hitID);
				visited[hitID] = true;
				faceToDeleteSet.insert(hitID);
				while (!residualFaceList.empty())
				{
					int curF = residualFaceList.front();
					residualFaceList.pop();

					HE_edge* edgeTmp = faceList->at(curF)->pedge_;
					do
					{
						if (!edgeTmp->isBoundary())
						{
							int pairF = edgeTmp->ppair_->pface_->id_;
							if (!visited[pairF])
							{
								residualFaceList.push(pairF);
								visited[pairF] = true;
								faceToDeleteSet.insert(pairF);
							}
						}
						edgeTmp = edgeTmp->pnext_;
					} while (edgeTmp != faceList->at(curF)->pedge_);
				}
			}
		}
	}

	std::vector<int> faceToDelete(faceToDeleteSet.begin(), faceToDeleteSet.end());
	sort(faceToDelete.begin(), faceToDelete.end());
	for (std::vector<int>::iterator i = faceToDelete.begin(); i != faceToDelete.end(); i++)
	{
		HE_edge* pedge = faceList->at(*i)->pedge_;
		do
		{
			edgeToDeleteSet.insert(pedge->id_);
			vertToDeleteSet.insert(pedge->pvert_->id_);
			pedge = pedge->pnext_;
		} while (pedge != faceList->at(*i)->pedge_);
	}

	std::vector<int> edgeToDelete(edgeToDeleteSet.begin(), edgeToDeleteSet.end());
	std::vector<int> vertToDelete(vertToDeleteSet.begin(), vertToDeleteSet.end());
	sort(edgeToDelete.begin(), edgeToDelete.end());
	sort(vertToDelete.begin(), vertToDelete.end());

	// apply the delete procedure
	int deleteIdx = 0;
	int originSize = faceList->size();
	for (int i = 0; i < originSize; i++)
	{
		if (deleteIdx == faceToDelete.size())
			faceList->at(i - deleteIdx)->id_ -= deleteIdx;
		else
		{
			for (; i != faceToDelete[deleteIdx]; i++)
				faceList->at(i - deleteIdx)->id_ -= deleteIdx;

			std::vector<HE_face*>::iterator ii = faceList->begin() + (i - deleteIdx);
			SafeDelete(*ii);
			faceList->erase(ii);
			deleteIdx++;
		}
	}

	deleteIdx = 0;
	std::vector<HE_edge*>* edgeList = mesh_support_->get_edges_list();
	originSize = edgeList->size();
	for (int i = 0; i < originSize; i++)
	{
		if (deleteIdx == edgeToDelete.size())
			edgeList->at(i - deleteIdx)->id_ -= deleteIdx;
		else
		{
			for (; i != edgeToDelete[deleteIdx]; i++)
				edgeList->at(i - deleteIdx)->id_ -= deleteIdx;

			std::vector<HE_edge*>::iterator ii = edgeList->begin() + (i - deleteIdx);

			// this should also be updated
			HE_vert* v1 = (*ii)->pvert_;
			HE_vert* v2 = (*ii)->ppair_->pvert_;
			mesh_support_->edgemap_.erase(std::pair<HE_vert*, HE_vert* >(v1, v2));
			mesh_support_->edgemap_.erase(std::pair<HE_vert*, HE_vert* >(v2, v1));

			SafeDelete(*ii);
			edgeList->erase(ii);
			deleteIdx++;
		}
	}

	deleteIdx = 0;
	std::vector<HE_vert*>* vertList = mesh_support_->get_vertex_list();
	originSize = vertList->size();
	for (int i = 0; i < originSize; i++)
	{
		if (deleteIdx == vertToDelete.size())
			vertList->at(i - deleteIdx)->id_ -= deleteIdx;
		else
		{
			for (; i != vertToDelete[deleteIdx]; i++)
				vertList->at(i - deleteIdx)->id_ -= deleteIdx;

			std::vector<HE_vert*>::iterator ii = vertList->begin() + (i - deleteIdx);
			SafeDelete(*ii);
			vertList->erase(ii);
			deleteIdx++;
		}
	}

	this->updateSupportMesh();
	this->BuildSupportOctree();
}

void Support::ClearData()
{
	outer_loop_.clear();
	inner_loop_.clear();
	input_vertex_list_.clear();
	input_vertex_list_sup.clear();
	mesh_support_->ClearData();
	mesh_region_->ClearData();
	SafeDelete(mesh_octree_);
	SafeDelete(mesh_support_octree_);
	mesh_octree_ = new MeshOctree;
	mesh_support_octree_ = new MeshOctree;
}

void Support::appendToMesh(Mesh3D* input, const std::vector<Vec3f>& verts, const std::vector<int>& triIdx) {
	int start_idx = 0;
	if (input->get_vertex_list() != NULL)
	{
		start_idx = input->get_vertex_list()->size();
	}
	for (unsigned int i = 0; i < verts.size(); i++)
	{
		input->InsertVertex(verts[i]);
	}
	for (unsigned int i = 0; i < triIdx.size(); i = i + 3)
	{
		std::vector<HE_vert*> tri;
		HE_vert *v0 = input->get_vertex(start_idx + triIdx[i]);
		HE_vert *v1 = input->get_vertex(start_idx + triIdx[i + 1]);
		HE_vert *v2 = input->get_vertex(start_idx + triIdx[i + 2]);
		if (!v0 || !v1 || !v2) continue;
		tri.push_back(v0);
		tri.push_back(v1);
		tri.push_back(v2);
		input->InsertFace(tri);
	}
}

void Support::pushCuboidtoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list, Vec3f(&center_point)[2], float w, float(&h)[2]) {
	points_list.clear();
	idx_list.clear();
	int start_idx = points_list.size();
	// calculate the directions
	Vec3f length_dir = center_point[0] - center_point[1];
	Vec3f width_dir(length_dir[1], -length_dir[0], 0);
	width_dir.normalize();
	width_dir *= w / 2;
	// add points
	// top four points
	points_list.push_back(center_point[0] - width_dir);
	points_list.push_back(center_point[1] + width_dir);
	points_list.push_back(center_point[0] + width_dir);
	points_list.push_back(center_point[1] - width_dir);
	// down four points
	points_list.push_back(center_point[0] - width_dir + Vec3f(0, 0, -h[0]));
	points_list.push_back(center_point[1] + width_dir + Vec3f(0, 0, -h[1]));
	points_list.push_back(center_point[0] + width_dir + Vec3f(0, 0, -h[0]));
	points_list.push_back(center_point[1] - width_dir + Vec3f(0, 0, -h[1]));
	// add indexes
	int idx_arr[36];
	idx_arr[0] = start_idx; idx_arr[1] = start_idx + 1; idx_arr[2] = start_idx + 2;
	idx_arr[3] = start_idx; idx_arr[4] = start_idx + 3; idx_arr[5] = start_idx + 1;
	idx_arr[6] = start_idx; idx_arr[7] = start_idx + 7; idx_arr[8] = start_idx + 3;
	idx_arr[9] = start_idx; idx_arr[10] = start_idx + 4; idx_arr[11] = start_idx + 7;
	// left
	idx_arr[12] = start_idx; idx_arr[13] = start_idx + 2; idx_arr[14] = start_idx + 6;
	idx_arr[15] = start_idx; idx_arr[16] = start_idx + 6; idx_arr[17] = start_idx + 4;

	idx_arr[18] = start_idx + 2; idx_arr[19] = start_idx + 1; idx_arr[20] = start_idx + 5;
	idx_arr[21] = start_idx + 2; idx_arr[22] = start_idx + 5; idx_arr[23] = start_idx + 6;
	// right
	idx_arr[24] = start_idx + 1; idx_arr[25] = start_idx + 3; idx_arr[26] = start_idx + 7;
	idx_arr[27] = start_idx + 1; idx_arr[28] = start_idx + 7; idx_arr[29] = start_idx + 5;

	idx_arr[30] = start_idx + 5; idx_arr[31] = start_idx + 7; idx_arr[32] = start_idx + 4;
	idx_arr[33] = start_idx + 5; idx_arr[34] = start_idx + 4; idx_arr[35] = start_idx + 6;
	idx_list.insert(idx_list.end(), idx_arr, idx_arr + 36);
}

void pushLinePoint(std::vector<Vec3f>& points_list, Vec3f outer_p, Vec3f inter_p, float hs)
{
	// top points
	points_list.push_back(outer_p);
	points_list.push_back(inter_p);
	// down points
	points_list.push_back(outer_p + Vec3f(0, 0, -hs));
	points_list.push_back(inter_p + Vec3f(0, 0, -hs));
}
void Support::pushLineCuboidToVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list,
	std::vector<Vec3f> center_points, std::vector<bool>& low_flag,
	float w, bool isClosed, bool isOuter, bool isTop, std::vector<float> hs)
{
	// add continuous line-faces
	int num = center_points.size();
	if (num < 2)
	{
		return;
	}
	if (hs.empty())
	{
		// top-line
		for (int i = 0; i < num; i++)
		{
			hs.push_back(line_height_);
		}
	}
	for (int i = 0; i < num; i++)
	{
		if (low_flag[i])
			hs[i] = 0.0f;
	}
	float outer_flag = isOuter ? 1 : 1;
	// add first point	
	int start_idx = points_list.size(); // record first position
										// calculate the directions
	Vec3f width_dir = isClosed ?
		getDirectionInVector(center_points[0], center_points[num - 1], center_points[1]) :
		getDirectionInVector(center_points[0], center_points[0], center_points[1]);
	width_dir *= w;
	// add points
	Vec3f outer_point;
	Vec3f inter_point;
	//if (isOuter)
	//{
	//	outer_point = isTop ? center_points[0] : center_points[0]-outer_flag/4*width_dir;
	//	inter_point = center_points[0] + outer_flag*width_dir;
	//}
	//else{
	inter_point = isTop ? center_points[0] + outer_flag / 2 * width_dir : center_points[0];
	outer_point = center_points[0] + outer_flag*width_dir;
	//}
	pushLinePoint(points_list, outer_point, inter_point, hs[0]);
	// add points and faces
	Vec3f fake_p = isClosed ? center_points[0] : center_points[num - 1];
	center_points.push_back(fake_p);
	for (int i = 1; i < num; i++)
	{
		// add only top-down-front-back faces
		int start_idx = points_list.size();
		// calculate the directions
		width_dir = w * getDirectionInVector(center_points[i], center_points[i - 1], center_points[i + 1]);
		// add points
		//if (isOuter)
		//{
		//	outer_point = isTop ? center_points[i] : center_points[i] - outer_flag / 4 * width_dir;
		//	inter_point = outer_point + outer_flag*width_dir;
		//}
		//else{
		inter_point = isTop ? center_points[i] + outer_flag / 2 * width_dir : center_points[i];
		outer_point = inter_point + outer_flag*width_dir;
		//}
		pushLinePoint(points_list, outer_point, inter_point, hs[i]);
		// add indexes
		int idx_arr[24];
		// top
		idx_arr[0] = start_idx - 3; idx_arr[1] = start_idx + 1; idx_arr[2] = start_idx - 4;
		idx_arr[3] = start_idx + 1; idx_arr[4] = start_idx; idx_arr[5] = start_idx - 4;
		// back
		idx_arr[6] = start_idx; idx_arr[7] = start_idx + 2; idx_arr[8] = start_idx - 4;
		idx_arr[9] = start_idx + 2; idx_arr[10] = start_idx - 2; idx_arr[11] = start_idx - 4;
		// front
		idx_arr[12] = start_idx + 3; idx_arr[13] = start_idx + 1; idx_arr[14] = start_idx - 3;
		idx_arr[15] = start_idx - 1; idx_arr[16] = start_idx + 3; idx_arr[17] = start_idx - 3;
		// bottom
		idx_arr[18] = start_idx - 2; idx_arr[19] = start_idx + 2; idx_arr[20] = start_idx + 3;
		idx_arr[21] = start_idx - 1; idx_arr[22] = start_idx - 2; idx_arr[23] = start_idx + 3;
		idx_list.insert(idx_list.end(), idx_arr, idx_arr + 24);
	}
	int end_idx = points_list.size(); // record last position
	if (isClosed)
	{
		// closed line-support
		int idx_arr[24];
		// top
		idx_arr[0] = end_idx - 3; idx_arr[1] = start_idx + 1; idx_arr[2] = end_idx - 4;
		idx_arr[3] = start_idx + 1; idx_arr[4] = start_idx; idx_arr[5] = end_idx - 4;
		// back
		idx_arr[6] = start_idx; idx_arr[7] = start_idx + 2; idx_arr[8] = end_idx - 4;
		idx_arr[9] = start_idx + 2; idx_arr[10] = end_idx - 2; idx_arr[11] = end_idx - 4;
		// front
		idx_arr[12] = start_idx + 3; idx_arr[13] = start_idx + 1; idx_arr[14] = end_idx - 3;
		idx_arr[15] = end_idx - 1; idx_arr[16] = start_idx + 3; idx_arr[17] = end_idx - 3;
		// bottom
		idx_arr[18] = end_idx - 2; idx_arr[19] = start_idx + 2; idx_arr[20] = start_idx + 3;
		idx_arr[21] = end_idx - 1; idx_arr[22] = end_idx - 2; idx_arr[23] = start_idx + 3;
		idx_list.insert(idx_list.end(), idx_arr, idx_arr + 24);
	}
	else
	{
		// add side-faces
		int idx_arr[12];
		// right
		idx_arr[0] = start_idx + 1; idx_arr[1] = start_idx + 3; idx_arr[2] = start_idx;
		idx_arr[3] = start_idx + 3; idx_arr[4] = start_idx + 2; idx_arr[5] = start_idx;
		// left
		idx_arr[6] = end_idx - 4; idx_arr[7] = end_idx - 2; idx_arr[8] = end_idx - 3;
		idx_arr[9] = end_idx - 2; idx_arr[10] = end_idx - 1; idx_arr[11] = end_idx - 3;
		idx_list.insert(idx_list.end(), idx_arr, idx_arr + 12);
	}
}
void Support::pushLinetoVector(std::vector<Vec3f>& points_list, std::vector<int>& idx_list, std::vector<Vec3f> center_points, bool isClosed, bool isOuter)
{
	points_list.clear();
	idx_list.clear();
	// calculate support heights
	std::vector<bool> pt_low_flag;
	std::vector<float> hs;
	for (std::vector<Vec3f>::iterator pt = center_points.begin(); pt != center_points.end(); ++pt)
	{
		float pt_height_ = getSupportHeight(*pt) - line_height_;
		// checking points too low
		if (pt_height_ < 0.1f)
			pt_low_flag.push_back(true);
		else
			pt_low_flag.push_back(false);
		hs.push_back(pt_height_);
	}
	float add_width = isClosed ? line_wall_width_ : line_width_;
	// add top-line
	pushLineCuboidToVector(points_list, idx_list, center_points, pt_low_flag, add_width / 2, isClosed, isOuter);
	// add bottom-line
	// update points to add
	int i = 0;
	for (std::vector<Vec3f>::iterator pt = center_points.begin(); pt != center_points.end(); ++pt)
	{
		if (!pt_low_flag[i])
			*pt += Vec3f(0, 0, -line_height_);
		i++;
	}
	pushLineCuboidToVector(points_list, idx_list, center_points, pt_low_flag, add_width, isClosed, isOuter, false, hs);
}
//////////////////////////////////////////////////////////////////////////
void Support::FindSupportRegion()
{

	std::vector<HE_face*>* faceList = mesh_in_->get_faces_list();
	mesh_region_->ClearData();
	input_vertex_list_.clear();
	input_vertex_list_sup.clear();

	const Vec3f _z(0, 0, -1);
	int iii = 0;
	for (std::vector<HE_face*>::iterator Fiter = faceList->begin(); Fiter != faceList->end(); Fiter++)
	{

		HE_edge* cur_edge_ = (*Fiter)->pedge_;
		Vec3f point_A_ = cur_edge_->pvert_->position();
		Vec3f point_B_ = cur_edge_->pnext_->pvert_->position();
		Vec3f point_C_ = cur_edge_->pprev_->pvert_->position();
		Vec3f eAB = point_B_ - point_A_;
		Vec3f eAC = point_C_ - point_A_;
		Vec3f vNormal = eAB.cross(eAC);
		vNormal.normalize();
		if (vNormal.dot(_z) > THRESHOLD)
		{
			std::vector<HE_vert*> vec_hv = InsertVertex(point_A_, point_B_, point_C_);
			mesh_region_->InsertFaceSup(vec_hv);
		}
	}
	mesh_region_->UpdateMeshSup();
}
std::vector<HE_vert*> Support::InsertVertex(Vec3f pos1, Vec3f pos2, Vec3f pos3)
{

	std::vector<HE_vert*> hv;
	hv.push_back(new HE_vert(pos1));
	hv.push_back(new HE_vert(pos2));
	hv.push_back(new HE_vert(pos3));
	for (int i = 0; i < 3; i++)
	{
		hv[i]->set_id(mesh_region_->num_of_vertex_list());
		std::set<HE_vert*, comVertex>::iterator  iterVert = input_vertex_list_.insert(hv[i]).first;
		if ((*iterVert)->id() == mesh_region_->num_of_vertex_list())
		{
			mesh_region_->InsertVertex(hv[i]);
		}
		else
		{
			delete hv[i];
			hv[i] = *iterVert;
		}
	}
	return hv;
}
std::vector<HE_vert*> Support::InsertVertexSup(Vec3f pos1, Vec3f pos2, Vec3f pos3)
{

	std::vector<HE_vert*> hv;
	hv.push_back(new HE_vert(pos1));
	hv.push_back(new HE_vert(pos2));
	hv.push_back(new HE_vert(pos3));
	for (int i = 0; i < 3; i++)
	{
		hv[i]->set_id(mesh_support_->num_of_vertex_list());
		std::set<HE_vert*, comVertex>::iterator  iterVert = input_vertex_list_sup.insert(hv[i]).first;
		if ((*iterVert)->id() == mesh_support_->num_of_vertex_list())
		{
			mesh_support_->InsertVertex(hv[i]);
		}
		else
		{
			delete hv[i];
			hv[i] = *iterVert;
		}
	}
	return hv;
}

void Support::MarkFace()
{
	std::vector<std::vector<HE_edge*>> bloop = mesh_region_->GetBLoop();

	std::vector<Vec3f> in;

	for (int j = 0; j < bloop[0].size(); j++)
	{
		in.push_back(bloop[0][j]->pvert_->position());
	}
	std::vector<std::vector<Vec3f>> facelist;


	Triangulation(in, true, facelist, outer_loop_);
	return;

	for (size_t i = 1; i < 2; i++)
		//for (size_t i = 0; i < bloop.size(); i++)
	{
		if (bloop[i].size() < 4)
		{
			continue;
		}
		Path subj;
		ClipperOffset co;
		Paths solution;
		std::vector<Vec3f> com_vert_;
		int direction_ = -1;
		for (size_t j = 0; j < bloop[i].size(); j++)
		{
			int X_ = (bloop[i])[j]->pvert_->position().x()*1e6;
			int Y_ = (bloop[i])[j]->pvert_->position().y()*1e6;
			subj << IntPoint(X_, Y_);
			Vec3f p(X_*1e-6, Y_*1e-6, 0.0);
			com_vert_.push_back(p);
		}
		outer_loop_.push_back(com_vert_);
		co.AddPath(subj, jtMiter, etClosedPolygon);
		co.Execute(solution, direction_*0.5*1e6);
		for (int m = 0; m < solution.size(); m++)
		{
			std::vector<Vec3f> com_offset_vert_;
			for (int n = 0; n < solution[m].size(); n++)
			{
				Vec3f p(solution[m][n].X*1e-6, solution[m][n].Y*1e-6, 0.0);
				com_offset_vert_.push_back(p);
			}
			inner_loop_.push_back(com_offset_vert_);//& operator
													//CutByInnerLine(com_offset_vert_, i);
		}
		continue;
		auto outLoop = mesh_region_->GetBLoop()[i];
		for (int m = 0; m < outLoop.size(); m++)
		{
			outLoop[m]->is_selected_ = true;
			DFSWall(outLoop[m]->ppair_->pface_);
		}

	}
}
void Support::CutByInnerLine(std::vector<Vec3f>& com_offset_vert_, int no)
{
	Vec3f pos(com_offset_vert_[0].x(), com_offset_vert_[0].y(), com_offset_vert_[0].z());
	Vec3f posT = pos;
	std::vector<HE_face* >* faces = mesh_region_->get_faces_list();
	HE_face* first_face_ = NULL;
	HE_face* cur_face_ = NULL;
	for (auto iterF = faces->begin(); iterF != faces->end(); iterF++)
	{
		(*iterF)->set_selected(UNSELECTED);
	}

	for (auto iterF = faces->begin(); iterF != faces->end(); iterF++)
	{
		if ((*iterF)->com_flag != no)
		{
			continue;
		}
		else
		{
			if (PointinTriangle((*iterF), pos))
			{
				cur_face_ = first_face_ = *iterF;
				//cur_face_->set_selected(SELECTED);
				break;
			}
		}
	}

	for (int i = 0; i < com_offset_vert_.size(); i++)
		//for (int i = 0; i <7; i++)
	{


		Vec3f a, b, dir;
		if (i == com_offset_vert_.size() - 1)
		{
			a = com_offset_vert_[i];
			b = com_offset_vert_[0];
		}
		else
		{
			a = com_offset_vert_[i];
			b = com_offset_vert_[i + 1];
		}
		dir = b - a;
		// 		if (i == 6)
		// 		{
		// 			dir.x() =-1.0;
		// 			dir.y() = 0.0;
		// 		}

		//dir = a - b;
		if (!PointinTriangle(cur_face_, a))
		{

			qDebug() << "something is wrong";
		}
		for (int j = 0; !PointinTriangle(cur_face_, b); j++)
		{

			// 			if (i == 67 && j == 1)
			// 			qDebug() <<i<<j;

			HE_edge* sta = cur_face_->pedge_;
			HE_edge* cur = sta;
			do
			{

				Vec3f A(cur->start_->position().x(), cur->start_->position().y(), 0.0);
				Vec3f B(cur->pvert_->position().x(), cur->pvert_->position().y(), 0.0);
				bool ab_cross_AB = (a - A).cross(B - A).z() > 0 && (b - A).cross(B - A).z() < 0;
				bool AB_cross_ab = (A - a).cross(b - a).z() < 0 && (B - a).cross(b - a).z() > 0;
				if (ab_cross_AB&&AB_cross_ab)//a ��AB���Ҳ� b��AB���?
				{
					/*if (i == 67)*/
					{
						cur_face_->set_selected(SELECTED);
						//append to mesh support
						mesh_support_->get_faces_list()->push_back(cur_face_);
						//cur->is_selected_ = true;
						//cur->ppair_->is_selected_ = true;
					}

					float a1 = (A.y() - B.y()) / (A.x() - B.x());
					float b1 = A.y() - a1 * (A.x());
					float a2 = (a.y() - b.y()) / (a.x() - b.x());
					float b2 = a.y() - a1 * (a.x());
					Vec3f CrossP(0.0, 0.0, 0.0);
					CrossP.x() = (b1 - b2) / (a2 - a1);
					CrossP.y() = a1 * CrossP.x() + b1;
					CrossP = A + (B - A)*((CrossP - A).length() / (B - A).dot(CrossP - A));


					cur_face_ = cur->ppair_->pface_;
					cur_face_->pedge_ = cur->ppair_;
					break;
				}
				cur = cur->pnext_;
			} while (cur != sta);
		}
	}

}
// Determine whether point P in triangle ABC

bool Support::PointinTriangle(HE_face* face, Vec3f point_in)
{

	std::vector<HE_vert*> verts;
	face->face_verts(verts);
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

void Support::DFSWall(HE_face* face)
{
	if (face->selected() == UNSELECTED)
	{
		face->set_selected(SELECTED);
		//mesh_support_->get_faces_list()->push_back(face);
		HE_edge*sta = face->pedge_;
		HE_edge*cur = sta;
		do
		{
			if (cur->ppair_->pface_ != NULL)
			{
				DFSWall(cur->ppair_->pface_);
			}
			cur = cur->pnext_;
		} while (cur != sta);

	}
}

void Support::Triangulation(std::vector<Vec3f>& in_, bool isout_, std::vector<std::vector<Vec3f>>& facelist, std::vector<std::vector<Vec3f>>&outer_, float offset_dist)
{

	input_vertex_list_sup.clear();
	input_vertex_list_.clear();
	mesh_region_->ClearData();
	segments.clear();
	sweep_point_.clear();
	//outer_.push_back(in_);
	float min_x = 1e6;
	float min_y = 1e6;
	float max_x = -1e6;
	float max_y = -1e6;
	std::vector<std::vector<Vec3f>> inner_;
	Offset(in_, isout_, inner_, offset_dist);
	//////////////////////////////////////////////////////////////////////////
	std::vector<HE_edge*> sweep_he_;
	std::vector<HE_vert*> sweep_hv_;
	std::vector<Vec3f> temp;
	for (size_t j = 0; j < in_.size(); j++)
	{

		int X_ = in_[j].x()*1e6;
		int Y_ = in_[j].y()*1e6;
		Vec3f p = Vec3f(X_*1e-6, Y_*1e-6, 0.0);
		temp.push_back(p);
		/*subj << IntPoint(X_, Y_);*/
		min_x = min_x < p.x() ? min_x : p.x();
		min_y = min_y < p.y() ? min_y : p.y();
		max_x = max_x > p.x() ? max_x : p.x();
		max_y = max_y > p.y() ? max_y : p.y();
		HE_vert* v = new HE_vert(p);
		HE_edge* e = new HE_edge();
		e->set_boundary_flag(BOUNDARY);
		e->pvert_ = v;
		if (sweep_hv_.size() != 0)
		{
			e->start_ = sweep_hv_.back();
			e->left_point_ = sweep_hv_.back()->position();
			sweep_hv_.back()->pedge_ = e;
			sweep_hv_.back()->pnext = v;
			v->pprev = sweep_hv_.back();
		}
		sweep_point_.insert(v);
		sweep_hv_.push_back(v);
		sweep_he_.push_back(e);
		sweep_edge_.push_back(e);
	}
	sweep_he_.front()->start_ = sweep_hv_.back();
	sweep_he_.front()->left_point_ = sweep_hv_.back()->position();
	sweep_hv_.back()->pedge_ = sweep_he_.front();
	sweep_hv_.back()->pnext = sweep_hv_.front();
	sweep_hv_.front()->pprev = sweep_hv_.back();
	for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
	{
		(*iterE)->left_point_ = (*iterE)->start_->position() < (*iterE)->pvert_->position() ? (*iterE)->start_->position() : (*iterE)->pvert_->position();
		(*iterE)->start_->set_boundary_flag(BOUNDARY);
		(*iterE)->pvert_->set_boundary_flag(BOUNDARY);
	}
	for (int m = 0; m < inner_.size(); m++)
	{
		sweep_he_.clear();
		sweep_hv_.clear();
		std::multiset <HE_vert*, comVertex> inner_point_;
		for (std::vector<Vec3f>::iterator iterInner = inner_[m].begin(); iterInner != inner_[m].end(); iterInner++)
		{
			min_x = min_x < (*iterInner).x() ? min_x : (*iterInner).x();
			min_y = min_y < (*iterInner).y() ? min_y : (*iterInner).y();
			max_x = max_x > (*iterInner).x() ? max_x : (*iterInner).x();
			max_y = max_y > (*iterInner).y() ? max_y : (*iterInner).y();
			HE_vert* v = new HE_vert((*iterInner));
			HE_edge* e = new HE_edge();
			e->set_boundary_flag(BOUNDARY);
			e->pvert_ = v;
			if (sweep_hv_.size() != 0)
			{
				e->start_ = sweep_hv_.back();
				sweep_hv_.back()->pnext = v;
				v->pprev = sweep_hv_.back();
			}
			sweep_point_.insert(v);
			//////////////////////////////////////////////////////////////////////////
			inner_point_.insert(v);
			//////////////////////////////////////////////////////////////////////////
			sweep_hv_.push_back(v);
			sweep_he_.push_back(e);
			sweep_edge_.push_back(e);
		}
		sweep_he_.front()->start_ = sweep_hv_.back();
		sweep_hv_.back()->pnext = sweep_hv_.front();
		sweep_hv_.front()->pprev = sweep_hv_.back();
		for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
		{
			(*iterE)->left_point_ = (*iterE)->start_->position() < (*iterE)->pvert_->position() ? (*iterE)->start_->position() : (*iterE)->pvert_->position();
			(*iterE)->start_->pedge_ = (*iterE);
			(*iterE)->start_->set_boundary_flag(INNER);
			(*iterE)->pvert_->set_boundary_flag(INNER);
		}
		// 		Vec3f p1, p2, p3, l1, l2;
		// 		p1 = (*inner_point_.begin())->position();
		// 		p2 = (*inner_point_.begin())->pprev->position();
		// 		p3 = (*inner_point_.begin())->pnext->position();
		// 		l1 = p3 - p2;
		// 		l2 = p1 - p2;
		// 		if (l1.cross(l2).z()*D_ > 0)//同向
		// 		{
		// 			for (auto iterIN = inner_point_.begin(); iterIN != inner_point_.end(); iterIN++)
		// 			{
		// 				HE_vert *temp = (*iterIN)->pprev;
		// 				(*iterIN)->pprev = (*iterIN)->pnext;
		// 				(*iterIN)->pnext = temp;
		// 			}
		// 			for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
		// 			{
		// 				HE_vert *temp = (*iterE)->pvert_;
		// 				(*iterE)->pvert_ = (*iterE)->start_;
		// 				(*iterE)->start_ = temp;
		// 				(*iterE)->start_->pedge_ = (*iterE);
		// 			}
		// 		}
	}
	outer_.push_back(in_);
	//return;
	//////////////////////////////////////////////////////////////////////////
	HE_edge* low = new HE_edge();
	HE_edge* high = new HE_edge();
	min_y -= 0.5;
	max_y += 0.5;
	min_x -= 0.5;
	max_x += 0.5;
	HE_vert* ll = new HE_vert(Vec3f(min_x, min_y, 0.0));
	HE_vert* lr = new HE_vert(Vec3f(max_x, min_y, 0.0));
	HE_vert* ul = new HE_vert(Vec3f(min_x, max_y, 0.0));
	HE_vert* ur = new HE_vert(Vec3f(max_x, max_y, 0.0));
	low->pvert_ = ll;
	low->start_ = lr;
	high->pvert_ = ur;
	high->start_ = ul;
	low->insert_point_ = low->pvert_->position();
	high->insert_point_ = high->start_->position();
	low->helperL.clear();
	segments.insert(low);
	segments.insert(high);

	for (auto iterP = sweep_point_.begin(); iterP != sweep_point_.end();)
	{
		std::set<HE_edge*, comHE>::iterator iter = segments.begin();
		for (auto iterSEG = segments.begin(); iterSEG != segments.end();)
		{

			(*iterSEG)->is_selected_ = false;
			Vec3f& rP_ = (*iterSEG)->pvert_->position().x() > (*iterSEG)->start_->position().x() ? (*iterSEG)->pvert_->position() : (*iterSEG)->start_->position();
			Vec3f& lP_ = (*iterSEG)->pvert_->position().x() < (*iterSEG)->start_->position().x() ? (*iterSEG)->pvert_->position() : (*iterSEG)->start_->position();
			if ((*iterP)->position().x() - rP_.x() > -1e-5)//删除�?
			{
				iterSEG = segments.erase(iterSEG);
			}
			else
			{
				if ((rP_ - lP_).cross((*iterP)->position() - lP_).z() > 0)// is up the segment
				{
					iter = iterSEG;
				}
				iterSEG++;
			}
		}
		(*iter)->is_selected_ = 1;
		(*++iter)->is_selected_ = 1;
		--iter;

		Trapezoidal(iter, iterP);
		if (sss == 110)
		{
			//break;
		}
	}
	delete low;
	delete high;
	delete ll;
	delete lr;
	delete ul;
	delete ur;
	segments.clear();
	mesh_region_->UpdateMeshSup();
	outer_.clear();
	inner_.clear();

	std::vector<std::vector<HE_edge*>>  bloop = mesh_region_->GetBLoop();
	for (int j = 0; j < bloop.size(); j++)
	{
		std::vector<Vec3f> temp;
		for (int k = 0; k < bloop[j].size(); k++)
		{

			temp.push_back(bloop[j][k]->pvert_->position());
		}
		outer_.push_back(temp);
	}
	std::vector<HE_face*>* faces = mesh_region_->get_faces_list();
	for (int j = 0; j < mesh_region_->num_of_face_list(); j++)
	{
		HE_edge *sta = faces->at(j)->pedge_;
		HE_edge*cur = sta;
		std::vector<Vec3f> temp;
		do
		{
			temp.push_back(cur->pvert_->position());
			cur = cur->pnext_;

		} while (cur != sta);
		facelist.push_back(temp);
	}

	for (auto iterE = sweep_edge_.begin(); iterE != sweep_edge_.end(); iterE++)
	{
		delete *iterE;
	}
	for (auto iterV = sweep_point_.begin(); iterV != sweep_point_.end(); iterV++)
	{
		delete *iterV;
	}
	sweep_edge_.clear();
	sweep_point_.clear();
}

void Support::CalPlaneLineIntersectPoint(Vec3f planeVector, Vec3f planePoint, Vec3f lineVector, Vec3f linePoint, Vec3f& point)
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
	//首先判断直线是否与平面平�? 
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

void Support::Trapezoidal(std::set<HE_edge*, comHE>::iterator &iter, std::multiset <HE_vert*, comVertex>::iterator& iterP)
{


	std::vector<Vec3f> helperToTriangle, helperOld, helperNew;
	Vec3f p1, p2;
	HE_edge* lowEdge = (*iter);
	HE_edge* HighEdge = (*++iter);
	std::vector<HE_vert*> points_array_;
	Vec3f curposition = (*iterP)->position();
	CalPlaneLineIntersectPoint(Vec3f(-1.0, 0.0, 0.0), curposition, lowEdge->pvert_->position() - lowEdge->start_->position(), lowEdge->start_->position(), p1);
	CalPlaneLineIntersectPoint(Vec3f(-1.0, 0.0, 0.0), curposition, HighEdge->pvert_->position() - HighEdge->start_->position(), HighEdge->start_->position(), p2);
	HE_vert* V1 = new HE_vert(p1);
	HE_vert* V2 = new HE_vert(p2);
	V1->pprev = lowEdge->start_;
	V1->pnext = lowEdge->pvert_;
	V2->pprev = HighEdge->start_;
	V2->pnext = HighEdge->pvert_;

	//////////////////////////////////////////////////////////////////////////
	points_array_.push_back(V1);
	do
	{
		sss++;
		AddNewSegments(iterP);
		points_array_.push_back((*iterP));
		++iterP;
	} while (iterP != sweep_point_.end() && (*iterP)->position().x() - curposition.x() < 1e-5 && (*iterP)->position().y() - p2.y() < -1e-5);
	points_array_.push_back(V2);
	//////////////////////////////////////////////////////////////////////////

	HE_edge*  edge_to_triangle_;
	HE_edge* edge_to_update;
	edge_to_triangle_ = HighEdge;
	edge_to_update = HighEdge;
	//////////////////////////////////////////////////////////////////////////
	int i = points_array_.size() - 1;

	do
	{
		helperToTriangle.push_back(points_array_[i]->position());
		i--;
		for (; points_array_[i]->pprev->position().x() - curposition.x() > -1e-5
			&& points_array_[i]->pnext->position().x() - curposition.x() > -1e-5; i--)
		{
			helperToTriangle.push_back(points_array_[i]->position());
		}
		helperToTriangle.push_back(points_array_[i]->position());

		helperOld = edge_to_triangle_->helperL;
		if (helperOld.size() != 0)
		{
			for (int i = 0; i < helperOld.size() - 1; i++)
			{

				std::vector<HE_vert*>he = InsertVertex(helperOld[i], helperOld[i + 1], helperToTriangle[0]);//ccw

				if (he[0] != he[1] && he[0] != he[2] && he[1] != he[2])
				{
					mesh_region_->InsertFaceSup(he);
				}
			}
			for (int i = 0; i < helperToTriangle.size() - 1; i++)
			{
				std::vector<HE_vert*>he = InsertVertex(helperOld.back(), helperToTriangle[i + 1], helperToTriangle[i]);//ccw
				if (he[0] != he[1] && he[0] != he[2] && he[1] != he[2])
				{
					mesh_region_->InsertFaceSup(he);
				}
			}
		}
		helperToTriangle.clear();
		// merge vertex
		if (points_array_[i]->pprev->position().x() - points_array_[i]->position().x() < -1e-5
			&& points_array_[i]->pnext->position().x() - points_array_[i]->position().x() < -1e-5)
		{
			helperToTriangle.push_back(points_array_[i]->position());
			helperToTriangle.push_back(points_array_[i]->position());
			Vec3f pp, pn, l1, l2;
			pp = points_array_[i]->pprev->position();
			pn = points_array_[i]->pnext->position();
			l1 = points_array_[i]->position() - points_array_[i]->pprev->position();
			l2 = points_array_[i]->pnext->position() - points_array_[i]->pprev->position();
			if (l2.cross(l1).z() > 0)
			{

				// 				helperOld = points_array_[i]->pprev->pedge_->helperL;
				// 
				// 				if (helperOld.size() != 0)
				// 				{
				// 					for (int i = 0; i < helperOld.size() - 1; i++)
				// 					{
				// 
				// 						std::vector<HE_vert*>he = InsertVertex(helperOld[i], helperOld[i + 1], helperToTriangle[0]);//ccw
				// 
				// 						if (he[0] != he[1] && he[0] != he[2] && he[1] != he[2])
				// 						{
				// 							mesh_region_->InsertFaceSup(he);
				// 						}
				// 					}
				// 					for (int i = 0; i < helperToTriangle.size() - 1; i++)
				// 					{
				// 						std::vector<HE_vert*>he = InsertVertex(helperOld.back(), helperToTriangle[i + 1], helperToTriangle[i]);//ccw
				// 						if (he[0] != he[1] && he[0] != he[2] && he[1] != he[2])
				// 						{
				// 							mesh_region_->InsertFaceSup(he);
				// 						}
				// 					}
				// 				}

				edge_to_triangle_ = points_array_[i]->pedge_;
			}
			else
			{
				helperOld = points_array_[i]->pedge_->helperL;

				if (helperOld.size() != 0)
				{
					for (int i = 0; i < helperOld.size() - 1; i++)
					{

						std::vector<HE_vert*>he = InsertVertex(helperOld[i], helperOld[i + 1], helperToTriangle[0]);//ccw

						if (he[0] != he[1] && he[0] != he[2] && he[1] != he[2])
						{
							mesh_region_->InsertFaceSup(he);
						}
					}
					for (int i = 0; i < helperToTriangle.size() - 1; i++)
					{
						std::vector<HE_vert*>he = InsertVertex(helperOld.back(), helperToTriangle[i + 1], helperToTriangle[i]);//ccw
						if (he[0] != he[1] && he[0] != he[2] && he[1] != he[2])
						{
							mesh_region_->InsertFaceSup(he);
						}
					}
				}

				edge_to_triangle_ = points_array_[i]->pprev->pedge_;

			}

		}
		else
		{
			if (points_array_[i]->pprev->position().x() - curposition.x() < -1e-5)
			{
				edge_to_triangle_ = points_array_[i]->pprev->pedge_;
			}
			else
			{
				edge_to_triangle_ = points_array_[i]->pedge_;
			}
		}

	} while (i > 0);
	//////////////////////////////////////////////////////////////////////////
	i = points_array_.size() - 1;
	do
	{


		helperNew.clear();
		Vec3f lastposition = points_array_[i]->position();
		helperNew.push_back(points_array_[i]->position());
		i--;

		for (; points_array_[i]->pprev->position().x() - curposition.x() < 1e-5
			&& points_array_[i]->pnext->position().x() - curposition.x() < 1e-5; i--)
		{
			helperNew.push_back(points_array_[i]->position());
		}
		if (points_array_[i]->position() == lastposition)
		{
			int j = i - 1;
			for (; points_array_[j]->pprev->position().x() - curposition.x() < 1e-5
				&& points_array_[j]->pnext->position().x() - curposition.x() < 1e-5; j--)
			{
				helperNew.push_back(points_array_[j]->position());
			}
			helperNew.push_back(points_array_[j]->position());
		}
		else
		{
			helperNew.push_back(points_array_[i]->position());
		}

		if (edge_to_update->pvert_->position() < edge_to_update->start_->position())
		{
			edge_to_update->helperL = helperNew;
		}
		//slipt vertex
		if (points_array_[i]->pprev->position().x() - points_array_[i]->position().x() > 1e-5
			&& points_array_[i]->pnext->position().x() - points_array_[i]->position().x() > 1e-5)
		{
			helperNew.clear();
			helperNew.push_back(points_array_[i]->position());
			helperNew.push_back(points_array_[i]->position());
			Vec3f pp, pn, l1, l2;
			pp = points_array_[i]->pprev->position();
			pn = points_array_[i]->pnext->position();
			l1 = points_array_[i]->position() - points_array_[i]->pprev->position();
			l2 = points_array_[i]->pnext->position() - points_array_[i]->pprev->position();
			if (l2.cross(l1).z() > 0)
			{
				edge_to_update = points_array_[i]->pprev->pedge_;
			}
			else
			{
				points_array_[i]->pprev->pedge_->helperL = helperNew;
				edge_to_update = points_array_[i]->pedge_;
			}
		}
		else
		{
			if (points_array_[i]->pprev->position().x() - curposition.x() > 1e-5)
			{
				edge_to_update = points_array_[i]->pprev->pedge_;
			}
			else
			{
				edge_to_update = points_array_[i]->pedge_;
			}
		}



	} while (i > 0);

}

bool Support::Offset(std::vector<Vec3f>& outer_, bool isout_, std::vector <std::vector<Vec3f>>& inner_, float offset_dist)
{

	Path subj;
	ClipperOffset co;
	Paths solution;
	std::vector<HE_edge*> sweep_he_;
	std::vector<HE_vert*> sweep_hv_;
	std::multiset <HE_vert*, comVertex> order_point_;
	std::vector<Vec3f> com_vert_;
	for (size_t j = 0; j < outer_.size(); j++)
	{

		int X_ = outer_[j].x()*1e6;
		int Y_ = outer_[j].y()*1e6;

		subj << IntPoint(X_, Y_);

		HE_vert* v = new HE_vert(outer_[j]);
		HE_edge* e = new HE_edge();
		e->pvert_ = v;
		if (sweep_hv_.size() != 0)
		{
			e->start_ = sweep_hv_.back();
			sweep_hv_.back()->pedge_ = e;
			sweep_hv_.back()->pnext = v;
			v->pprev = sweep_hv_.back();
		}
		order_point_.insert(v);
		sweep_hv_.push_back(v);
		sweep_he_.push_back(e);
	}
	sweep_he_.front()->start_ = sweep_hv_.back();
	sweep_hv_.back()->pedge_ = sweep_he_.front();
	sweep_hv_.back()->pnext = sweep_hv_.front();
	sweep_hv_.front()->pprev = sweep_hv_.back();

	Vec3f p1, p2, p3, l1, l2;
	p1 = (*order_point_.begin())->position();
	p2 = (*order_point_.begin())->pprev->position();
	p3 = (*order_point_.begin())->pnext->position();

	int D_ = 0;
	l1 = p3 - p2;
	l2 = p1 - p2;
	if (l1.cross(l2).z() < 0)
	{
		D_ = -1;// ccw
	}
	else
	{
		D_ = 1;//cw
	}
	if (D_ == -1 && !isout_)
	{
		std::vector<Vec3f> temp;
		for (int i = outer_.size() - 1; i >= 0; i--)
		{
			temp.push_back(outer_[i]);
		}
		outer_ = temp;
		D_ = 1;
	}
	if (D_ == 1 && isout_)
	{
		std::vector<Vec3f> temp;
		for (int i = outer_.size() - 1; i >= 0; i--)
		{
			temp.push_back(outer_[i]);
		}
		outer_ = temp;
		D_ = -1;
	}
	for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
	{
		delete *iterE;
	}
	for (auto iterV = sweep_hv_.begin(); iterV != sweep_hv_.end(); iterV++)
	{
		delete *iterV;
	}
	co.AddPath(subj, jtMiter, etClosedPolygon);
	co.Execute(solution, D_*(offset_dist)*1e6);
	order_point_.clear();
	for (int m = 0; m < solution.size(); m++)
	{
		std::vector<Vec3f> com_offset_vert_;
		for (int n = 0; n < solution[m].size(); n++)
		{
			Vec3f p(solution[m][n].X*1e-6, solution[m][n].Y*1e-6, 0.0);
			com_offset_vert_.push_back(p);
		}
		inner_.push_back(com_offset_vert_);//& operator
	}
	sweep_he_.clear();
	sweep_hv_.clear();
	for (int m = 0; m < inner_.size(); m++)
	{
		for (std::vector<Vec3f>::iterator iterInner = inner_[m].begin(); iterInner != inner_[m].end(); iterInner++)
		{

			HE_vert* v = new HE_vert((*iterInner));
			HE_edge* e = new HE_edge();
			e->pvert_ = v;
			if (sweep_hv_.size() != 0)
			{
				e->start_ = sweep_hv_.back();
				sweep_hv_.back()->pnext = v;
				v->pprev = sweep_hv_.back();
			}
			order_point_.insert(v);
			sweep_hv_.push_back(v);
			sweep_he_.push_back(e);
		}
		sweep_he_.front()->start_ = sweep_hv_.back();
		sweep_hv_.back()->pnext = sweep_hv_.front();
		sweep_hv_.front()->pprev = sweep_hv_.back();

		Vec3f p1, p2, p3, l1, l2;
		p1 = (*order_point_.begin())->position();
		p2 = (*order_point_.begin())->pprev->position();
		p3 = (*order_point_.begin())->pnext->position();
		l1 = p3 - p2;
		l2 = p1 - p2;

		if (l1.cross(l2).z()*D_ > 0)//同向
		{
			inner_[m].clear();
			for (int n = solution[m].size() - 1; n >= 0; n--)
			{
				Vec3f p(solution[m][n].X*1e-6, solution[m][n].Y*1e-6, 0.0);
				inner_[m].push_back(p);
			}
		}
	}
	for (auto iterE = sweep_he_.begin(); iterE != sweep_he_.end(); iterE++)
	{
		delete *iterE;
	}
	for (auto iterV = sweep_hv_.begin(); iterV != sweep_hv_.end(); iterV++)
	{
		delete *iterV;
	}
	return true;
}

void Support::AddNewSegments(std::multiset <HE_vert*, comVertex>::iterator iterP)
{
	//////////////////////////////////////////////////////////////////////////
	//添加新边进去
	Vec3f curPoint;
	do
	{

		curPoint = (*iterP)->position();
		for (auto m = segments.begin(); m != segments.end();)
		{
			HE_edge* edge_ = *m;
			Vec3f& rP_ = (*m)->pvert_->position() > (*m)->start_->position() ? (*m)->pvert_->position() : (*m)->start_->position();
			Vec3f& lP_ = (*m)->pvert_->position() < (*m)->start_->position() ? (*m)->pvert_->position() : (*m)->start_->position();
			if (curPoint == rP_)
			{
				m = segments.erase(m);
			}
			else
			{
				m++;
				CalPlaneLineIntersectPoint(Vec3f(-1.0, 0.0, 0.0), curPoint + Vec3f(0.001, 0.0, 0.0),
					edge_->pvert_->position() - edge_->start_->position(), edge_->start_->position(), edge_->insert_point_);
			}
		}

		HE_edge* next_edge_ = (*iterP)->pedge_;
		HE_edge* prev_edge_ = (*iterP)->pprev->pedge_;
		CalPlaneLineIntersectPoint(Vec3f(-1.0, 0.0, 0.0), curPoint + Vec3f(0.001, 0.0, 0.0),
			next_edge_->pvert_->position() - next_edge_->start_->position(), next_edge_->start_->position(), next_edge_->insert_point_);
		CalPlaneLineIntersectPoint(Vec3f(-1.0, 0.0, 0.0), curPoint + Vec3f(0.001, 0.0, 0.0),
			prev_edge_->pvert_->position() - prev_edge_->start_->position(), prev_edge_->start_->position(), prev_edge_->insert_point_);
		if (next_edge_->pvert_->position().x() - curPoint.x() > -1e-4)
		{
			segments.insert(next_edge_);
		}
		if (prev_edge_->start_->position().x() - curPoint.x() > -1e-4)
		{
			segments.insert(prev_edge_);
		}
		++iterP;
	} while (iterP != sweep_point_.end() && curPoint == (*iterP)->position());
	//////////////////////////////////////////////////////////////////////////
}