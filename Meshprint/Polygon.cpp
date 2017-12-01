#include "Polygon.h"
#include "Cutline.h"
#include "HE_mesh/Vec.h"
#include <algorithm>
#include "clipper.hpp"
#include "QDebug"
typedef trimesh::vec3  Vec3f;

static bool sortByAngle(const CutLine* a, const CutLine* b)
{

	if (a->angle_ - b->angle_ < -1e-2)
	{
		return true;
	}
	else if (a->angle_ - b->angle_ < 1e-2)
	{
		if (a->isoutedge && !b->isoutedge)
		{
			return false;
		}
		return true;
	}
	else
		return false;
}
static bool sortByCrossPoint(const CutLine* a, const CutLine* b)
{

	if (a->cross_point_.y() - b->cross_point_.y() < -1e-2)
	{
		return true;
	}
	return false;
}
static bool sortByXYCOR(const CutPoint* a, const CutPoint* b)
{
	Vec3f pos1 = a->getPosition();
	Vec3f pos2 = b->getPosition();
	if (pos1.x()-pos2.x()<-1e-2)
	{
		return true;
	}
	else if (pos1.x()-pos2.x()<1e-2)
	{
		return pos1.y() - pos2.y() < -1e-2;
	}
	return false;
}

Polygon::Polygon()
{
}

Polygon::~Polygon()
{
	for (auto iter = edges.begin(); iter != edges.end(); iter++)
	{
		delete *iter;
	}
	for (auto iter = points.begin(); iter != points.end(); iter++)
	{
		delete *iter;
	}

}

CutLine* Polygon::insertEdge(CutLine* e)
{
	edges.push_back(e);
	return e;
}

CutLine* Polygon::insertEdge(Vec3f a, Vec3f b)
{

	CutPoint*   p1 = new CutPoint(a);
	CutPoint*	p2 = new CutPoint(b);
	auto pair1_ = points.insert(p1);
	auto pair2_ = points.insert(p2);
	if (pair1_.first == pair2_.first)
	{
		return NULL;
	}

	CutLine* e = new CutLine(*pair1_.first, *pair2_.first);
	(*pair1_.first)->setOutEdge(e);
	(*pair2_.first)->setInEdge(e);
	if (!pair1_.second)
		delete p1;
	if (!pair2_.second)
		delete p2;
	edges.push_back(e);
	return e;
}

int Polygon::sweepPolygon()
{
	std::set<CutPoint*, comPointsLarge> tempPoints;
	std::vector<CutLine*> str_line_;
	std::vector<CutPoint*> queue_;
	for (auto iter=points.rbegin();iter!=points.rend();iter++)
	{
		queue_.push_back(*iter);
	}
	while (!queue_.empty())
	{
		CutPoint* ptr_point_ = queue_.back();
		queue_.pop_back();
		std::vector<CutLine*> left_line_, righ_line_, cros_line_;
		UpdateStructure(ptr_point_, str_line_, left_line_, righ_line_, cros_line_);
		Vec3f cur_pos_ = ptr_point_->getPosition();
		if (left_line_.size() + cros_line_.size() == 0)
		{
			CutLine* down_{ NULL }, *up_{ NULL };
			Vec3f criterion, line_a_;
			for (int i = 0; i < str_line_.size(); i++)
			{
				criterion = str_line_[i]->order_pos_[1]->getPosition() - str_line_[i]->order_pos_[0]->getPosition();
				line_a_ = cur_pos_ - str_line_[i]->order_pos_[0]->getPosition();
				if ((line_a_^criterion).z() > 0)
				{
					up_ = str_line_[i];
					if (i--)
						down_ = str_line_[i];
					break;
				}
			}
			FindNewEvent(down_, up_, ptr_point_, queue_);
		}
	}
	for (auto iter = points.begin(); iter != points.end(); iter++)
	{
		if ((*iter)->getInEdges().size() == 1 && (*iter)->getOutEdges().size() == 1)
		{
			(*iter)->getInEdges()[0]->pnext_ = (*iter)->getOutEdges()[0];
			(*iter)->getOutEdges()[0]->pprev_ = (*iter)->getInEdges()[0];
		}
		else if ((*iter)->getInEdges().size() > 0 && (*iter)->getOutEdges().size() > 0)
		{
			std::vector<CutLine*> lines;
			for (auto iterIn = (*iter)->getInEdges().begin(); iterIn != (*iter)->getInEdges().end(); iterIn++)
			{
				(*iterIn)->angle_ = angleWithXAxis((*iterIn)->position_vert[0] - (*iterIn)->position_vert[1]);
				(*iterIn)->isoutedge = false;
				lines.push_back(*iterIn);
			}
			for (auto iterOut = (*iter)->getOutEdges().begin(); iterOut != (*iter)->getOutEdges().end(); iterOut++)
			{
				(*iterOut)->angle_ = angleWithXAxis((*iterOut)->position_vert[1] - (*iterOut)->position_vert[0]);
				(*iterOut)->isoutedge = true;
				lines.push_back(*iterOut);
			}
			std::sort(lines.begin(), lines.end(), sortByAngle);//ccw  allow the two line at the same angle
			int count = lines.size();
			while (count)
			{
				count = lines.size() == 1 ? 0 : lines.size();
				while (count)
				{
					if (!lines.back()->isoutedge&&lines[lines.size() - 2]->isoutedge)
					{
						break;
					}
					else
					{
						lines.insert(lines.begin(), lines.back());
						lines.pop_back();
						count--;
					}
				}
				if (count != 0)
				{
					CutLine* in_ = lines.back();
					lines.pop_back();
					in_->pnext_ = lines.back();
					lines.back()->pprev_ = in_;
					lines.pop_back();
				}
				else
				{
					(*iter)->getOutEdges().clear();
					(*iter)->getInEdges().clear();
					for (int i = 0; i < lines.size(); i++)
					{
						if (lines[i]->isoutedge)
						{
							(*iter)->getOutEdges().push_back(lines[i]);
						}
						else {
							(*iter)->getInEdges().push_back(lines[i]);
						}
					}
					auto iterTem = tempPoints.insert(*iter);
					if (!iterTem.second)//if  insert failed
					{
						(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
							(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
						(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
							(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
					}
				}
			}
		}
		else if ((*iter)->getInEdges().size() == 1)
		{
			CutPoint* p0 = (*iter)->getInEdges()[0]->cut_point_[0];
			CutPoint*p1 = (*iter)->getInEdges()[0]->cut_point_[1];
			Vec3f length = p0->getPosition() - p1->getPosition();
			if (p0->getEdgeSize() == 1 && abs(length.x()) < 75 * LIMIT&&abs(length.y()) < 75 * LIMIT)
			{
				(*iter)->getInEdges()[0]->visit = true;//this edge become no use;
				continue;
			}				
			auto iterTem = tempPoints.insert(*iter);
			if (!iterTem.second)//if  insert failed
			{
				(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
					(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
				(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
					(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
			}
		}
		else if ((*iter)->getOutEdges().size() == 1)
		{
			CutPoint* p0 = (*iter)->getOutEdges()[0]->cut_point_[0];
			CutPoint*p1 = (*iter)->getOutEdges()[0]->cut_point_[1];
			Vec3f length = p0->getPosition() - p1->getPosition();
			if (p1->getEdgeSize() == 1 && abs(length.x()) < 3 * LIMIT&&abs(length.y()) < 3 * LIMIT)
			{
				(*iter)->getOutEdges()[0]->visit = true;//this edge become no use;
				continue;
			}
			auto iterTem = tempPoints.insert(*iter);
			if (!iterTem.second)//if  insert failed
			{
				(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
					(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
				(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
					(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
			}
		}
	}
	std::set<CutPoint*, comPointsHuge> LargePoints;
	for (auto iter = tempPoints.begin(); iter != tempPoints.end(); iter++)
	{
		if ((*iter)->getInEdges().size() == 1 && (*iter)->getOutEdges().size() == 1)
		{
			(*iter)->getInEdges()[0]->pnext_ = (*iter)->getOutEdges()[0];
			(*iter)->getOutEdges()[0]->pprev_ = (*iter)->getInEdges()[0];
		}
		else if ((*iter)->getInEdges().size() > 0 && (*iter)->getOutEdges().size() > 0)
		{
			std::vector<CutLine*> lines;
			for (auto iterIn = (*iter)->getInEdges().begin(); iterIn != (*iter)->getInEdges().end(); iterIn++)
			{
				(*iterIn)->angle_ = angleWithXAxis((*iterIn)->position_vert[0] - (*iterIn)->position_vert[1]);
				(*iterIn)->isoutedge = false;
				lines.push_back(*iterIn);
			}
			for (auto iterOut = (*iter)->getOutEdges().begin(); iterOut != (*iter)->getOutEdges().end(); iterOut++)
			{
				(*iterOut)->angle_ = angleWithXAxis((*iterOut)->position_vert[1] - (*iterOut)->position_vert[0]);
				(*iterOut)->isoutedge = true;
				lines.push_back(*iterOut);
			}
			std::sort(lines.begin(), lines.end(), sortByAngle);//ccw  allow the two line at the same angle
			int count = lines.size();
			while (count)
			{
				count = lines.size() == 1 ? 0 : lines.size();
				while (count)
				{
					if (!lines.back()->isoutedge&&lines[lines.size() - 2]->isoutedge)
					{
						break;
					}
					else
					{
						lines.insert(lines.begin(), lines.back());
						lines.pop_back();
						count--;
					}
				}
				if (count != 0)
				{
					CutLine* in_ = lines.back();
					lines.pop_back();
					in_->pnext_ = lines.back();
					lines.back()->pprev_ = in_;
					lines.pop_back();
				}
				else
				{
					(*iter)->getOutEdges().clear();
					(*iter)->getInEdges().clear();
					for (int i = 0; i < lines.size(); i++)
					{
						if (lines[i]->isoutedge)
						{
							(*iter)->getOutEdges().push_back(lines[i]);
						}
						else {
							(*iter)->getInEdges().push_back(lines[i]);
						}
					}
					auto iterTem = LargePoints.insert(*iter);
					if (!iterTem.second)//if  insert failed
					{
						(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
							(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
						(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
							(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
					}
				}
			}
		}
		else if ((*iter)->getInEdges().size() == 1)
		{
			CutPoint* p0 = (*iter)->getInEdges()[0]->cut_point_[0];
			CutPoint*p1 = (*iter)->getInEdges()[0]->cut_point_[1];
			Vec3f length = p0->getPosition() - p1->getPosition();
			if (p0->getEdgeSize() == 1 && abs(length.x()) < 17 * LIMIT&&abs(length.y()) < 17 * LIMIT)
			{
				(*iter)->getInEdges()[0]->visit = true;//this edge become no use;
				continue;
			}
			auto iterTem = LargePoints.insert(*iter);
			if (!iterTem.second)//if  insert failed
			{
				(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
					(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
				(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
					(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
			}
		}
		else if ((*iter)->getOutEdges().size() == 1)
		{
			CutPoint* p0 = (*iter)->getOutEdges()[0]->cut_point_[0];
			CutPoint* p1 = (*iter)->getOutEdges()[0]->cut_point_[1];
			Vec3f length = p0->getPosition() - p1->getPosition();
			if (p1->getEdgeSize() == 1 && abs(length.x()) < 17 * LIMIT&&abs(length.y()) < 17 * LIMIT)
			{
				(*iter)->getOutEdges()[0]->visit = true;//this edge become no use;
				continue;
			}
			auto iterTem = LargePoints.insert(*iter);
			if (!iterTem.second)//if  insert failed
			{
				(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
					(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
				(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
					(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
			}
		}

	}
	
	for (auto iter = LargePoints.begin(); iter != LargePoints.end(); iter++)
	{

		if ((*iter)->getInEdges().size() == 1 && (*iter)->getOutEdges().size() == 1)
		{
			(*iter)->getInEdges()[0]->pnext_ = (*iter)->getOutEdges()[0];
			(*iter)->getOutEdges()[0]->pprev_ = (*iter)->getInEdges()[0];
		}
		else if ((*iter)->getInEdges().size() > 0 && (*iter)->getOutEdges().size() > 0)
		{

			std::vector<CutLine*> lines;
			for (auto iterIn = (*iter)->getInEdges().begin(); iterIn != (*iter)->getInEdges().end(); iterIn++)
			{
				(*iterIn)->angle_ = angleWithXAxis((*iterIn)->position_vert[0] - (*iterIn)->position_vert[1]);
				(*iterIn)->isoutedge = false;
				lines.push_back(*iterIn);
			}
			for (auto iterOut = (*iter)->getOutEdges().begin(); iterOut != (*iter)->getOutEdges().end(); iterOut++)
			{
				(*iterOut)->angle_ = angleWithXAxis((*iterOut)->position_vert[1] - (*iterOut)->position_vert[0]);
				(*iterOut)->isoutedge = true;
				lines.push_back(*iterOut);
			}
			std::sort(lines.begin(), lines.end(), sortByAngle);//ccw  allow the two line at the same angle
			int count = lines.size();
			while (count)
			{
				count = lines.size() == 1 ? 0 : lines.size();
				while (count)
				{
					if (!lines.back()->isoutedge&&lines[lines.size() - 2]->isoutedge)
					{
						break;
					}
					else
					{
						lines.insert(lines.begin(), lines.back());
						lines.pop_back();
						count--;
					}
				}
				if (count != 0)
				{
					CutLine* in_ = lines.back();
					lines.pop_back();
					in_->pnext_ = lines.back();
					lines.back()->pprev_ = in_;
					lines.pop_back();
				}
				else
					qDebug() << "error at line 300 in polygon cpp";
			}
		}

	}
return 0;
	
}

inline float Polygon::angleWithXAxis(Vec3f dir)
{
	if (dir.x() == 0.0)
	{
		if (dir.y() > 0)
		{
			return 90;
		}
		else
		{
			return 270;
		}
	}
	else if (dir.x() < 0)
	{
		return atan(dir.y() / dir.x()) + 180;
	}
	else if (dir.y() < 0)
	{
		return atan(dir.y() / dir.x()) + 360;
				}
	else
		return atan(dir.y() / dir.x());
			}

void Polygon::UpdateStructure(CutPoint* ptr_point_, std::vector<CutLine *>& str_line_, std::vector<CutLine *>& left_line_, std::vector<CutLine *>& righ_line_, std::vector<CutLine *>& cros_line_)
{
	auto lines_in_ = ptr_point_->getInEdges();
	auto line_ou_ = ptr_point_->getOutEdges();
	for (int i = 0; i < lines_in_.size(); i++)
	{	
		if (lines_in_[i]->order_pos_[1] == ptr_point_)
		{
			Vec3f p1 = lines_in_[i]->order_pos_[0];
			Vec3f p2 = lines_in_ [i]->order_pos_[1];
			Vec3f Q = (ptr_point_)->getPosition();
			if ((p2 - p1).x() < LIMIT)
			{
				Q.y() = 10000;
				lines_in_[i]->cross_point_ = Q;
			}
			else
			{
				lines_in_[i]->cross_point_ = p1 + (Q.x() + (float)LIMIT - p2.x()) / (p2 - p1).x()*(p2 - p1);
			}
			righ_line_.push_back(lines_in_[i]);
		}
	}
	for (int i = 0; i < line_ou_.size(); i++)
	{
		if (line_ou_[i]->order_pos_[0] == ptr_point_)
		{
			Vec3f p1 = str_line_[i]->order_pos_[0];
			Vec3f p2 = str_line_[i]->order_pos_[1];
			Vec3f Q = (ptr_point_)->getPosition();
			if ((p2 - p1).x() < LIMIT)
			{
				Q.y() = 10000;
				str_line_[i]->cross_point_ = Q;
			}
			else
			{
				str_line_[i]->cross_point_ = p1 + (Q.x() + (float)LIMIT - p2.x()) / (p2 - p1).x()*(p2 - p1);
			}
			righ_line_.push_back(line_ou_[i]);
		}
	}
	for (auto iter=str_line_.begin();iter!=str_line_.end();)
	{
		Vec3f p1 = (*iter)->position_vert[0];
		Vec3f p2 = (*iter)->position_vert[1];
		Vec3f Q = (ptr_point_)->getPosition();
		if ((*iter)->order_pos_[1] == ptr_point_)
		{
			left_line_.push_back(*iter);
			str_line_.erase(iter);
		}
		else if (((Q - p1) ^ (p2 - p1)).length() < 1e-2)
		{
				cros_line_.push_back((*iter));
				CutLine* l1_ = new CutLine((*iter)->cut_point_[0], ptr_point_);
				CutLine* l2_ = new CutLine(ptr_point_, (*iter)->cut_point_[1]);
				edges.push_back(l1_);
				edges.push_back(l2_);
				l1_->pprev_ = (*iter)->pprev_;
				l1_->pnext_ = l2_;
				l2_->pnext_ = (*iter)->pnext_;
				l2_->pprev_ = l1_;
				CutPoint* cur = (*iter)->cut_point_[0];
				auto to_split_ = find(cur->getOutEdges().begin(), cur->getOutEdges().end(), (*iter));
				cur->getOutEdges().erase(to_split_);
				cur->getOutEdges().push_back(l1_);
				cur = (*iter)->cut_point_[1];
				to_split_ = find(cur->getInEdges().begin(), cur->getInEdges().end(), (*iter));
				cur->getInEdges().erase(to_split_);
				cur->getInEdges().push_back(l2_);
				str_line_.erase(iter);
				if (l1_->order_pos_[1] == ptr_point_)
				{
					if ((p2 - p1).x() < LIMIT)
					{
						Q.y() = 10000;
						l1_->cross_point_ = Q;
					}
					else
					{
						l1_->cross_point_ = p1 + (Q.x() + (float)LIMIT - p2.x()) / (p2 - p1).x()*(p2 - p1);
					}
					str_line_.push_back(l1_);
				}
				else
				{
					if ((p2 - p1).x() < LIMIT)
					{
						Q.y() = 10000;
						l2_->cross_point_ = Q;
					}
					else
					{
						l2_->cross_point_ = p1 + (Q.x() + (float)LIMIT - p2.x()) / (p2 - p1).x()*(p2 - p1);
					}
					str_line_.push_back(l2_);
				}

		}
		else
		{
			if ((p2 - p1).x() < LIMIT)
			{
				Q.y() = 10000;
				(*iter)->cross_point_ = Q;
			}
			else
			{
				(*iter)->cross_point_ = p1 + (Q.x() + (float)LIMIT - p2.x()) / (p2 - p1).x()*(p2 - p1);
			}
			iter++;
		}
	}
	str_line_.insert(str_line_.end(), righ_line_.begin(), righ_line_.end());
	std::sort(str_line_.begin(), str_line_.end(), sortByCrossPoint);
}

void Polygon::storePathToPieces(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_list_, int id)
{
//#define  MORETHANTWOTEST
#ifdef MORETHANTWOTEST
//to test more than two line cross one point 
	std::vector<std::pair<Vec3f, Vec3f>> circle;
	int mark = -1;
	for (auto iter = points.begin(); iter != points.end(); iter++)
	{
		if ((*iter)->getEdgeSize() > 2 && mark++ > 0)
		{

			qDebug() << (*iter)->getEdgeSize() << (*iter)->getPosition().x() << (*iter)->getPosition().y() << (*iter)->getPosition().z();
			auto in = (*iter)->getInEdges();
			auto out = (*iter)->getOutEdges();
			for (int i = 0; i < in.size(); i++)
			{
				std::pair<Vec3f, Vec3f> linein(in[i]->position_vert[0], in[i]->position_vert[1]);
				circle.push_back(linein);
				qDebug() << "in" << linein.first.x() << linein.first.y() << linein.first.z();
				qDebug() << "in" << linein.second.x() << linein.second.y() << linein.second.z();
			}
			for (int j = 0; j < out.size(); j++)
			{
				std::pair<Vec3f, Vec3f> linein(out[j]->position_vert[0], out[j]->position_vert[1]);
				circle.push_back(linein);
				qDebug() << "out" << linein.first.x() << linein.first.y() << linein.first.z();
				qDebug() << "out" << linein.second.x() << linein.second.y() << linein.second.z();
			}
		}
	}
	pieces_list_[id].push_back(circle);
	return;
	//end test
#endif

#ifndef MORETHANTWOTEST
	for (int i = 0; i < edges.size(); i++)
	{
		bool can_be_regarded_as_one{ true };
		Vec3f length;
		if (!edges[i]->visit)
		{
			std::vector<std::pair<Vec3f, Vec3f>> circle_;
			CutLine* sta = edges[i];
			CutLine* cur = sta;
			do
			{
				if (cur->pprev_ == NULL)
				{
					break;
				}
				cur = cur->pprev_;

			} while (cur != sta);
			do
			{
				cur->visit = true;
				circle_.push_back(std::pair<Vec3f, Vec3f>(cur->position_vert[0], cur->position_vert[1]));
				length = cur->position_vert[0] - cur->position_vert[1];
				if (abs(length.x()) > 5 * LIMIT || abs(length.y()) > 5 * LIMIT)
					can_be_regarded_as_one = false;
				cur = cur->pnext_;
			} while (cur != NULL && !cur->visit);
			if (cur == NULL)//means this path is one open path
			{
				qDebug() << "this layer has one open path";
				Vec3f p0 = circle_.back().second;
				Vec3f p1 = circle_.front().first;
				if ((p0 - p1).length() < 75 * LIMIT)
					circle_.push_back(std::pair<Vec3f, Vec3f>(p0, p1));
				else
					qDebug() << "this layer has one open path";
			}
			if (!can_be_regarded_as_one)
			{
				pieces_list_[id].push_back(circle_);

			}
		}
	}
	qDebug() << id << pieces_list_[id].size();
	
#endif
}

void Polygon::FindNewEvent(CutLine* down,CutLine* up,CutPoint* point, std::vector<CutPoint*> queue)
{
	if (down->order_pos_[1]!=up->order_pos_[1])
	{

		Vec3f A = down->order_pos_[0]->getPosition();
		Vec3f B = down->order_pos_[1]->getPosition();
		Vec3f C = down->order_pos_[0]->getPosition();
		Vec3f D = down->order_pos_[1]->getPosition();
		float aeraDAB = ((D - A) ^ (B - A)).z();
		float aeraCAB = ((C - A) ^ (B - A)).z();
		float aeraACD = ((A - C) ^ (D - C)).z();
		float aeraBCD = ((B - C) ^ (D - C)).z();
		if (aeraDAB*aeraCAB < 0 && aeraACD*aeraBCD < 0)
		{
			Vec3f intersect_p_ = A + abs(aeraACD)/(abs(aeraACD)+abs(aeraBCD))*(B - A);
			CutPoint* new_event_=new CutPoint(intersect_p_);
			points.insert(new_event_);
			queue.push_back(new_event_);
			sort(queue.begin(), queue.end(), sortByCrossPoint);
		}

	}

	


}


