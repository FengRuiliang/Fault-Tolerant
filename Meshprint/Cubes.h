#pragma once
#include "globalFunctions.h"
#include <vector>
#include <set>
class CutLine;
class Cubes
{
public:
	class Box;
	Cubes() {};
	~Cubes() {};
public:
	void StoreBox();
	Cubes::Box * insertToBox(Vec3f pos, CutLine ab);
	void insertToBox(Box box);
	void setUnit(float param1, float p2, float p3) { unit_ = Vec3f(param1, p2, p3); }

public:	struct sortBox
{
	bool operator ()(Box a, Box b)const
	{

		std::vector<int> aId = a.getID();
		std::vector<int> bId = b.getID();
		if (aId[0] < bId[0]) {
			return true;
		}
		else if (aId[0] == bId[0]) {

			if (aId[1] < bId[1]) {
				return true;
			}
			else if (aId[1] == bId[1])
			{
				if (aId[2] < bId[2])
				{
					return true;
				}
				else
				{
					return false;
				}
			}
			else {
				return false;
			}
		}
		else
		{
			return false;
		}

	}
};
		struct sortVec
		{
			bool operator ()(Vec3f a, Vec3f b)const
			{
				if (a[0] - b[0] < -1e-5) {
					return true;
				}
				else if (a[0] - b[0] < 1e-5) {

					if (a[1] - b[1] < -1e-5) {
						return true;
					}
					else if (a[1] - b[1] < 1e-5)
					{
						if (a[2] - b[2] < -1e-5)
						{
							return true;
						}
						else
						{
							return false;
						}
					}
					else {
						return false;
					}
				}
				else
				{
					return false;
				}

			}
		};
	class Box
	{
	public:
		Box();
		Box(int x,int y, int z,Vec3f hei);
		Box(int x, int y, int z);
		void setCube(float x, float y, float z, float h);
		Vec4f GetCoordinate();
		std::vector<int> getID() const;
		void insertP(Vec3f point);
	public:
		~Box() {};
	private:
		float xmin_, ymin_, zmin_,xlength,ywidth,zheight_;
		int idX_, idY, idZ_;
	public:
	};
	std::set<Box, sortBox> boxes_;
	std::set<Box, sortBox> GetBoxes() { return boxes_; }
	Vec3f GetUnit() { return unit_; }
private:
	
	Vec3f unit_;
};
