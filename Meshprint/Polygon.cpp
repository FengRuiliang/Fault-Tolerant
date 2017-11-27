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
		if (a->isoutedge&&!b->isoutedge)
		{
			return false;
		}
		return true;
	}
	else
		return false;
}

Polygon::Polygon()
{
}

Polygon::~Polygon()
{
	for (auto iter=edges.begin();iter!=edges.end();iter++)
	{
		delete *iter;
	}
	for (auto iter=points.begin();iter!=points.end();iter++)
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
	if (pair1_.first==pair2_.first)
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
			int count=lines.size();
			while (count)
			{
				count = lines.size() == 1 ? 0 : lines.size();
				while (count)
				{
					if (!lines.back()->isoutedge&&lines[lines.size()-2]->isoutedge)
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
		else if ((*iter)->getInEdges().size()== 1)
		{
			CutLine* cl = (*iter)->getInEdges()[0];
			if (cl->cut_point_[0]->getEdgeSize()==1)
			{
				cl = NULL;
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
	for (auto iter=tempPoints.begin();iter!=tempPoints.end();iter++)
	{
		if ((*iter)->getInEdges().size() == 1 && (*iter)->getOutEdges().size() == 1)
		{
			(*iter)->getInEdges()[0]->pnext_ = (*iter)->getOutEdges()[0];
			(*iter)->getOutEdges()[0]->pprev_ = (*iter)->getInEdges()[0];
		}
		else if ((*iter)->getEdgeSize() == 1)
		{
			auto iterTem = LargePoints.insert(*iter);
			if (!iterTem.second)//if  insert failed
			{
				(*iterTem.first)->getInEdges().insert((*iterTem.first)->getInEdges().end(),
					(*iter)->getInEdges().begin(), (*iter)->getInEdges().end());
				(*iterTem.first)->getOutEdges().insert((*iterTem.first)->getOutEdges().end(),
					(*iter)->getOutEdges().begin(), (*iter)->getOutEdges().end());
			}
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

void Polygon::storePathToPieces(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_list_, int id)
{
//#define  MORETHANTWOTEST
#ifdef MORETHANTWOTEST
//to test more than two line cross one point 
 	std::vector<std::pair<Vec3f, Vec3f>> circle;
	int mark =-1 ;
	for (auto iter = points.begin(); iter != points.end(); iter++)
	{
		if ((*iter)->getEdgeSize() > 2&&mark++>0)
		{

			qDebug() <<(*iter)->getEdgeSize()<< (*iter)->getPosition().x() << (*iter)->getPosition().y() << (*iter)->getPosition().z();
			auto in = (*iter)->getInEdges();
			auto out = (*iter)->getOutEdges();
			for (int i = 0; i < in.size(); i++)
			{
				std::pair<Vec3f, Vec3f> linein(in[i]->position_vert[0], in[i]->position_vert[1]);
				circle.push_back(linein);
				qDebug() << "in"<<linein.first.x() << linein.first.y() << linein.first.z();
				qDebug() << "in"<<linein.second.x() << linein.second.y() << linein.second.z();
			}
			for (int j = 0; j < out.size(); j++)
			{
				std::pair<Vec3f, Vec3f> linein(out[j]->position_vert[0], out[j]->position_vert[1]);
				circle.push_back(linein);
				qDebug() << "out"<<linein.first.x() << linein.first.y() << linein.first.z();
				qDebug() << "out"<<linein.second.x() << linein.second.y() << linein.second.z();
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
		if (!edges[i]->visit)
		{
			std::vector<std::pair<Vec3f, Vec3f>> circle_;
			CutLine* sta = edges[i];
			CutLine* cur = sta;
			do 
			{
				if (cur->pprev_==NULL)
				{
					break;
				}
				cur = cur->pprev_;

			} while (cur!=sta);
			do
			{
				cur->visit = true;
				circle_.push_back(std::pair<Vec3f, Vec3f>(cur->position_vert[0], cur->position_vert[1]));
				cur = cur->pnext_;
			} while (cur != NULL&&!cur->visit);
			if (cur==NULL)//means this path is one open path
			{
				Vec3f p0 = circle_.back().second;
				Vec3f p1 = circle_.front().first;
				if ((p0 - p1).length() < 75*LIMIT)
					circle_.push_back(std::pair<Vec3f, Vec3f>(p0, p1));
				else
					qDebug() << "this layer has one open path";
			}
			pieces_list_[id].push_back(circle_);
		}
	}
	qDebug() << id << pieces_list_[id].size();
#endif
}

