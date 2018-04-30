#include "Octree.h"
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
