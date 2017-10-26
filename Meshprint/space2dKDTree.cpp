#include "space2dKDTree.h"

Space2dKDTree::Space2dKDTree(std::vector<LineSegment*> &segments)
	: rootNode(NULL)
{
	BuildKDTree(segments, 0, segments.size(), rootNode);
}

Space2dKDTree::~Space2dKDTree()
{
	DeleteKDTree(this->rootNode);
}

void Space2dKDTree::BuildKDTree(std::vector<LineSegment*> &segments, int head, int tail, Tree2dNode *&node)
{
	node = new Tree2dNode();

	if (tail - head <= 2)
	{
		for (int i = head; i < tail; i++)
			node->segmentIdx.push_back(i);
		// compute the bounding box
		segments[head]->GetBoundingBox2d(node->AA, node->BB);
		for (std::vector<LineSegment*>::iterator i = segments.begin() + head + 1; i < segments.begin() + tail; i++)
		{
			Vec3f AT, BT;
			(*i)->GetBoundingBox2d(AT, BT);
			MergeBoundingBox(node->AA, node->BB, node->AA, node->BB, AT, BT);
		}

		return;
	}

	sort(segments.begin() + head, segments.begin() + tail, LineSegment::SortByY);

	int middle = (head + tail) / 2;
	BuildKDTree(segments, head, middle, node->lChild);
	BuildKDTree(segments, middle, tail, node->rChild);

	MergeBoundingBox(node->AA, node->BB, node->lChild->AA, node->lChild->BB, node->rChild->AA, node->rChild->BB);
}

void Space2dKDTree::DeleteKDTree(Tree2dNode *&node)
{
	if (node)
	{
		DeleteKDTree(node->lChild);
		DeleteKDTree(node->rChild);
		SafeDelete(node);
	}
}