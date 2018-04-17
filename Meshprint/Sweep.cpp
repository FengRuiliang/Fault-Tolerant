#include "Sweep.h"
#include <algorithm>
Sweep::Sweep()
{
}


Sweep::~Sweep()
{
}

Sweep::Sweep(std::vector<Vec3f> input)
{
	for (size_t i=0;i<input.size();i++)
	{
		SweepPoint* p=new SweepPoint;
		p->position_ = input[i];
		points_.push_back(p);
	}
	for (size_t j=0;j<points_.size();j++)
	{
		SweepLine* l_ = new SweepLine;
		SweepPoint* p1_ = points_[j];
		SweepPoint* p2_ = points_[(j + 1) % points_.size()];
		l_->left_ = p1_->position_< p2_->position_ ? p1_ : p2_;
		l_->right_ = p1_->position_ < p2_->position_ ? p2_ : p1_;
		lines_.push_back(l_);
	}
	sort(points_.begin(), points_.end());
}
