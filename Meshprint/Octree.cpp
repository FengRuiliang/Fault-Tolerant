#include "Octree.h"


Octree::~Octree()
{
	delete this->oc_root_;
	oc_root_ = NULL;
}

Octree::Octree(Mesh3D * mesh)
{

	oc_root_ = NULL;
	oc_face_list_ = NULL;
	oc_face_list_ = mesh->get_faces_list();
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
	mesh->ComputeBoundingBox();
	AABB aabb = AABB(mesh->getBoundingBox()[0], mesh->getBoundingBox()[1]);
	delete this->oc_root_;
	oc_root_ = NULL;
	create_octree(oc_root_, face_list_idx, aabb, false);
	delete face_list_idx;
	face_list_idx = NULL;
}
void Octree::create_octree(MeshOcNode*& node_in, std::vector<int>* face_list_idx, AABB aabb, bool stopFlag)
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
		oc_face_list_->at(face_list_idx->at(0))->getBoundingBox(cur_aabb._max, cur_aabb._min);
		for (std::vector<int>::iterator i = face_list_idx->begin(); i != face_list_idx->end(); i++)
		{
			AABB tmp_aabb;
			oc_face_list_->at(*i)->getBoundingBox(tmp_aabb._max, tmp_aabb._min);
			MergeBoundingBox(cur_aabb, cur_aabb, tmp_aabb);
		}
		node_in->ocn_aabb_ = cur_aabb;

		return;
	}

	std::vector<int>* tmp_face_list_idx[8];
	for (int i = 0; i < 8; i++)
		tmp_face_list_idx[i] = new std::vector < int >;

	Vec3f split_center = (aabb._max + aabb._min) / 2.0f;
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

	Vec3f hab = (aabb._max - aabb._min) / 2.0f;

	create_octree(node_in->child_node_[0], tmp_face_list_idx[0], AABB(split_center, aabb._min), stopFlag);
	create_octree(node_in->child_node_[1], tmp_face_list_idx[1], AABB(split_center + Vec3f(hab.x(), 0, 0), aabb._min + Vec3f(hab.x(), 0, 0)), stopFlag);
	create_octree(node_in->child_node_[2], tmp_face_list_idx[2], AABB(aabb._max - Vec3f(0, hab.y(), 0), split_center - Vec3f(0, hab.y(), 0)), stopFlag);
	create_octree(node_in->child_node_[3], tmp_face_list_idx[3], AABB(split_center - Vec3f(0, 0, hab.z()), aabb._min - Vec3f(0, 0, hab.z())), stopFlag);

	create_octree(node_in->child_node_[4], tmp_face_list_idx[4], AABB(split_center + Vec3f(0, hab.y(), 0), aabb._min + Vec3f(0, hab.y(), 0)), stopFlag);
	create_octree(node_in->child_node_[5], tmp_face_list_idx[5], AABB(aabb._max + Vec3f(0, 0, hab.z()), split_center + Vec3f(0, 0, hab.z())), stopFlag);
	create_octree(node_in->child_node_[6], tmp_face_list_idx[6], AABB(aabb._max, split_center), stopFlag);
	create_octree(node_in->child_node_[7], tmp_face_list_idx[7], AABB(aabb._max - Vec3f(hab.x(), 0, 0), split_center - Vec3f(hab.x(), 0, 0)), stopFlag);

	for (int i = 0; i < 8; i++)
	{
		delete tmp_face_list_idx[i];
		tmp_face_list_idx[i] = NULL;
	}

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
