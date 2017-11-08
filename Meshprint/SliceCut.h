#pragma once
#include "HE_mesh\Mesh3D.h"
#include "globalFunctions.h"
#include "clipper.hpp"
#include "Cubes.h"
#include "Cutline.h"
using namespace ClipperLib;
class Mesh3D;
class vector;
class Field;
class cutLine;
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
	void CutInPieces();
	void clipPolygon();
	void storeClipIntoCube();
	void Exportslice();
	float getThickness() { return thickness_; };
	std::vector < std::vector<cutLine>* >* GetPieces(){ return pieces_list_; }
	std::vector < std::vector<Vec3f>>* GetClip() { return clip_list_; }
	int GetNumPieces() { return num_pieces_; }
	 std::vector<int> * storeMeshIntoSlice();
	 std::vector<int>  *storage_Face_list_;
	Cubes cubes_support_;
	int  num_pieces_;
	
private:
	//float thickness_;
	Mesh3D* mesh_in_;
	
	 std::vector < std::vector<cutLine>*>*pieces_list_;
	 std::vector < std::vector<Vec3f>>* clip_list_{NULL};
	int isEdgeInFace(HE_vert* pvert1, HE_vert* pvert2, HE_face* pface);
	std::vector<int> sortVertInFace(int faceid);
};

