#pragma once
#include "HE_mesh\Mesh3D.h"
#include "globalFunctions.h"
#include "clipper.hpp"
#include "Sweepline.h"
#include "Cutline.h"
class Mesh3D;
class vector;
class Field;
class SweepLine;
#define DEFAULT_T 0.02f




class SliceCut
{
public:
	SliceCut()
	{}
	SliceCut(Mesh3D* ptr_in, float tn = DEFAULT_T) :mesh_in_(ptr_in)
		{
		//thickness_=0.5;//层厚
		num_pieces_= mesh_in_->getBoundingBox().at(0).at(2) /thickness_+1;//层数
		
		storage_Face_list_ = NULL;// new #2/thickness_三角面片分层
		pieces_list_ = NULL;//层的链表
	};
	~SliceCut();

	void SetThickness(float tn = DEFAULT_T){
		if (tn != 0)
		{
			thickness_ = tn;
		}
	};

	void clearcut();
	std::vector<int> sortVertInFace(int faceid);
	void CutInPieces();

	void cutThrouthVertex();

	void sweepPline();

	std::pair<Vec3f, Vec3f> cutFacet(HE_face * facet, float cur_height_);

	void cutFacetInmiddlePoint();
	float getThickness() { return thickness_; };
	std::vector < std::vector<std::pair<Vec3f, Vec3f>>>* GetPieces();
	HE_edge * InsertEdgeBySweep(HE_vert * vstart, HE_vert * vend);
	std::map<float, std::vector<std::vector<CutLine>>> getMapPieces() { return cut_list_; }
	int GetNumPieces() { return num_pieces_; }
	std::vector<int> * StoreFaceIntoSlice();
	std::vector<int>  *storage_Face_list_;
	std::set<float> thickf_;
	int  num_pieces_;

private:
	//float thickness_;
	Mesh3D* mesh_in_;
	/* std::vector<cutLine>* circle_list_;*/
	std::vector < std::vector<std::pair<Vec3f,Vec3f>>>* pieces_list_;
	std::map<float, std::vector<std::vector<CutLine>>> cut_list_;
	int isEdgeInFace(HE_vert* pvert1, HE_vert* pvert2, HE_face* pface);
	HE_edge*  getLeftEdge(HE_face* face_, float height_);
};

