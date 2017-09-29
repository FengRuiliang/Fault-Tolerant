#include "Mesh3D.h"
#include <string>
#include <fstream>
#include <iostream>
#include <strstream>
#include <xutility>
#include <QDebug>
#include <QFile>
#include <QDataStream>
#include <QString>
#include <QTextCodec>
#include <fstream>
#include <iostream>
#include "Mesh3D.h"
#include <winsock.h>
//#include "matrix.h"
#include <QMatrix4x4>
#include "QQuaternion"
#include "qvector.h"
#include <qmath.h>
#include <QTime>
#include "globalFunctions.h"
#pragma comment(lib,"ws2_32.lib")


Mesh3D::Mesh3D(void)
{
	// intialization
	pvertices_list_ = new std::vector<HE_vert*>;
	pfaces_list_ = new std::vector<HE_face*>;
	pedges_list_ = new std::vector<HE_edge*>;
	bheList = new std::vector<HE_edge*>;
	iheList = new std::vector<HE_edge*>;
	
	//input_vertex_list_ = NULL;
	xmax_ = ymax_ = zmax_ = 1.f;
	xmin_ = ymin_ = zmin_ = -1.f;
	no_loop = 0;
	num_components_ = 0;
	average_edge_length_ = 1.f;
}

void Mesh3D::ClearData(void)
{
	ClearVertex();
	ClearEdges();
	ClearFaces();
	input_vertex_list_.clear();
	edgemap_.clear();
	xmax_ = ymax_ = zmax_ = 1.f;
	xmin_ = ymin_ = zmin_ = -1.f;
	bLoop.clear();
	bheList->clear();
}

void Mesh3D::ClearVertex(void)
{

	if (pvertices_list_==NULL)
	{
		return;
	}
	else
	{
		for (VERTEX_ITER viter = pvertices_list_->begin(); viter != pvertices_list_->end(); viter++)
		{
			if (*viter != NULL)
			{
				delete *viter;
				*viter = NULL;
			}
			else
			{
				// ERROR
			}
		}
		delete pvertices_list_;
		pvertices_list_ = NULL;
	}
}

void Mesh3D::ClearEdges(void)
{
	if (pedges_list_ == NULL)
	{
		return;
	}
	else
	{
		for (EDGE_ITER eiter = pedges_list_->begin(); eiter!=pedges_list_->end(); eiter++)
		{
			if (*eiter != NULL)
			{
				delete *eiter;
				*eiter = NULL;
			}
			else
			{
				// ERROR
			}
		}
		delete pedges_list_;
		pedges_list_ = NULL;
	}
}

void Mesh3D::ClearFaces(void)
{
	if (pfaces_list_==NULL)
	{
		return;
	}
	else
	{
		for (FACE_ITER fiter = pfaces_list_->begin(); fiter!=pfaces_list_->end(); fiter++)
		{
			if (*fiter != NULL)
			{
				delete *fiter;
				*fiter = NULL;
			}
			else
			{
				// ERROR
			}
		}
		delete pfaces_list_;
		pfaces_list_ = NULL;
	}
}

HE_vert* Mesh3D::InsertVertex(const Vec3f& v)
{
	HE_vert* pvert = new HE_vert(v);
	if (pvertices_list_ == NULL)
	{
		pvertices_list_ = new std::vector<HE_vert*>;
	}
	pvert->id_ = static_cast<int>(pvertices_list_->size());
	pvertices_list_->push_back(pvert);
	return pvert;
}

HE_vert* Mesh3D::InsertVertex(HE_vert* & v)
{
	
	if (pvertices_list_ == NULL)
	{
		pvertices_list_ = new std::vector<HE_vert*>;
	}
	v->id_ = static_cast<int>(pvertices_list_->size());
	pvertices_list_->push_back(v);
	return v;
}

HE_edge* Mesh3D::InsertEdge(HE_vert* vstart, HE_vert* vend)
{
	if (vstart == NULL || vend == NULL)
	{
		return NULL;
	}

	if (pedges_list_ == NULL)
	{
		pedges_list_ = new std::vector<HE_edge*>;
	}

	if (edgemap_[PAIR_VERTEX(vstart, vend)] != NULL)
	{
		return edgemap_[PAIR_VERTEX(vstart, vend)];
	}
	//  if  has not find  an  exist edge
	
	HE_edge* pedge = new HE_edge;
	pedge->pvert_ = vend;
	pedge->start_ = vstart;
	pedge->pvert_->degree_++;
	vstart->pedge_ = pedge;
	edgemap_[PAIR_VERTEX(vstart, vend)] = pedge;
	pedge->id_ = static_cast<int>(pedges_list_->size());
	pedges_list_->push_back(pedge);
	return pedge;
}

HE_face* Mesh3D::InsertFace(std::vector<HE_vert* >& vec_hv)
{
	if (pfaces_list_ == NULL)
	{
		pfaces_list_ = new std::vector<HE_face*>;
	}
	HE_face *pface = new HE_face;
	int vsize = static_cast<int>(vec_hv.size());
	pface->valence_ = vsize;
	HE_edge *he1 = NULL, *he2 = NULL, *he3 = NULL, *he1_pair_ = NULL, *he2_pair_ = NULL, *he3_pair_ = NULL;
	he1 = InsertEdge(vec_hv[0], vec_hv[1]);
	he2 = InsertEdge(vec_hv[1], vec_hv[2]);
	he3 = InsertEdge(vec_hv[2], vec_hv[0]);
	he1_pair_ = InsertEdge(vec_hv[1], vec_hv[0]);
	he2_pair_ = InsertEdge(vec_hv[2], vec_hv[1]);
	he3_pair_ = InsertEdge(vec_hv[0], vec_hv[2]);
	he1->ppair_ = he1_pair_;
	he1_pair_->ppair_ = he1;
	he2->ppair_ = he2_pair_;
	he2_pair_->ppair_ = he2;
	he3->ppair_ = he3_pair_;
	he3_pair_->ppair_ = he3;
	pface->pedge_ = he1;
	he1->pnext_ = he2;
	he1->pprev_ = he3;
	he1->pface_ = pface;
	he2->pnext_ = he3;
	he2->pprev_ = he1;
	he2->pface_ = pface;
	he3->pnext_ = he1;
	he3->pprev_ = he2;
	he3->pface_ = pface;
	vec_hv[0]->adjHEdges.push_back(he3_pair_);
	vec_hv[1]->adjHEdges.push_back(he1_pair_);
	vec_hv[2]->adjHEdges.push_back(he2_pair_);

	pface->id_ = static_cast<int>(pfaces_list_->size());
	pfaces_list_->push_back(pface);
	return pface;

}

HE_face* Mesh3D::InsertFace(std::vector<HE_vert* >& vec_hv,Vec3f normal_read_)
{
	if (pfaces_list_==NULL)
	{
		pfaces_list_ = new std::vector<HE_face *>;
	}
	HE_face *pface = new HE_face;
	pface->id_ = static_cast<int>(pfaces_list_->size());
	pfaces_list_->push_back(pface);
	pface->normal() = normal_read_;
	//if the normal is wrong ,exchange the order of vec_hv
	Vec3f vector1 = vec_hv[1]->position() - vec_hv[0]->position();
	Vec3f vector2 = vec_hv[2]->position() - vec_hv[0]->position();
	Vec3f n_ = vector1^vector2;
	n_.normalize();
	if (n_.dot(normal_read_) < -0.98)
	{
		for (int j = 0; j < 3; j++)
			pface->vertices_[j] = vec_hv[2 - j];
	}
	else
		pface->vertices_ = vec_hv;
	if (abs(n_*Vec3f(0.0,0.0,1.0))<0.98)
	{
		sort(vec_hv.begin(), vec_hv.end(), sortByZB);
		vec_hv[0]->mergeFace.push_back(pface->id());
		vec_hv[2]->splitFace.push_back(pface->id());
	}
	return pface;
	HE_edge *he1 = NULL, *he2 = NULL, *he3 = NULL, *he1_pair_ = NULL, *he2_pair_ = NULL, *he3_pair_ = NULL;
	he1 = InsertEdge(vec_hv[0], vec_hv[1]);
	he2 = InsertEdge(vec_hv[1], vec_hv[2]);
	he3 = InsertEdge(vec_hv[2], vec_hv[0]);
	he1_pair_ = InsertEdge(vec_hv[1], vec_hv[0]);
	he2_pair_ = InsertEdge(vec_hv[2], vec_hv[1]);
	he3_pair_ = InsertEdge(vec_hv[0], vec_hv[2]);
	he1->ppair_ = he1_pair_;
	he2->ppair_ = he2_pair_;
	he3->ppair_ = he3_pair_;
	he1_pair_->ppair_ = he1; 
	he2_pair_->ppair_ = he2; 
	he3_pair_->ppair_ = he3;
	he1->pnext_ = he2;
	he2->pnext_ = he3;
	he3->pnext_ = he1;	
	he1->pprev_ = he3;
	he2->pprev_ = he1;
	he3->pprev_ = he2;
	vec_hv[0]->adjHEdges.push_back(he3_pair_);
	vec_hv[1]->adjHEdges.push_back(he1_pair_);
	vec_hv[2]->adjHEdges.push_back(he2_pair_);
	he1->pface_ = pface;
	he2->pface_ = pface;
	he3->pface_ = pface;
	pface->pedge_ = he1;
	return pface;
}

bool Mesh3D::LoadFromOBJFile(const char* fins)//读取obj文件
{
	FILE *pfile = fopen(fins, "r");

	char *tok;
	//char *tok_tok;
	char temp[128];

	try
	{
		ClearData();
		//read vertex
		fseek(pfile, 0, SEEK_SET);
		char pLine[512];

		while (fgets(pLine, 512, pfile))
		{
			if (pLine[0] == 'v' && pLine[1] == ' ')
			{
				Vec3f nvv;
				tok = strtok(pLine, " ");
				for (int i = 0; i < 3; i++)
				{
					tok = strtok(NULL, " ");
					strcpy(temp, tok);
					temp[strcspn(temp, " ")] = 0;
					nvv[i] = (float)atof(temp);
				}
				InsertVertex(nvv);
			}
		}

		//read facets
		fseek(pfile, 0, SEEK_SET);

		while (fgets(pLine, 512, pfile))
		{
			char *pTmp = pLine;
			if (pTmp[0] == 'f')
			{
				std::vector<HE_vert* > s_faceid;

				tok = strtok(pLine, " ");
				while ((tok = strtok(NULL, " ")) != NULL)
				{
					strcpy(temp, tok);
					temp[strcspn(temp, "/")] = 0;
					int id = (int)strtol(temp, NULL, 10) - 1;
					HE_vert* hv = get_vertex(id);
					bool findit = false;
					for (int i = 0; i < (int)s_faceid.size(); i++)
					{
						if (hv == s_faceid[i])	//remove redundant vertex id if it exists
						{
							//	cout << "remove redundant vertex" << endl;
							findit = true;
							break;
						}
					}
					if (findit == false && hv != NULL)
					{
						s_faceid.push_back(hv);
					}
				}
				if ((int)s_faceid.size() >= 3)
				{
					InsertFace(s_faceid)->vertices_=s_faceid;

				}
			}
		}
		// 		if (pedges_list_!=NULL)
		// 		{
		// 			for (int i = 0; i < pedges_list_->size(); i++)
		// 			{
		// 				qDebug() << pedges_list_->at(i)->id() << pedges_list_->at(i)->pface_;
		// 			}
		// 		}

		//read texture coords
		fseek(pfile, 0, SEEK_SET);
		std::vector<Vec3f> texCoordsTemp;
		while (fscanf(pfile, "%s", pLine) != EOF)
		{
			if (pLine[0] == 'v' && pLine[1] == 't')
			{
				Vec3f texTemp(0.f, 0.f, 0.f);
				fscanf(pfile, "%f %f", &texTemp[0], &texTemp[1]);
				texCoordsTemp.push_back(texTemp);
			}
		}
		//read texture index

		if (texCoordsTemp.size() > 0)
		{
			fseek(pfile, 0, SEEK_SET);

			int faceIndex = 0;
			while (fscanf(pfile, "%s", pLine) != EOF)
			{

				if (pLine[0] == 'f')
				{
					int v, t;
					fscanf(pfile, "%s", pLine);
					if (sscanf(pLine, "%d/%d", &v, &t) == 2)
					{
						std::map<int, int> v2tex;
						v2tex[v - 1] = t - 1;

						fscanf(pfile, "%s", pLine);
						sscanf(pLine, "%d/%d", &v, &t);
						v2tex[v - 1] = t - 1;

						fscanf(pfile, "%s", pLine);
						sscanf(pLine, "%d/%d", &v, &t);
						v2tex[v - 1] = t - 1;

						HE_edge* edgeTemp = pfaces_list_->at(faceIndex)->pedge_;
						edgeTemp->texCoord_ = texCoordsTemp.at(v2tex[edgeTemp->pvert_->id_]);
						edgeTemp->pvert_->texCoord_ = edgeTemp->texCoord_;
						edgeTemp = edgeTemp->pnext_;
						edgeTemp->texCoord_ = texCoordsTemp.at(v2tex[edgeTemp->pvert_->id_]);
						edgeTemp->pvert_->texCoord_ = edgeTemp->texCoord_;
						edgeTemp = edgeTemp->pnext_;
						edgeTemp->texCoord_ = texCoordsTemp.at(v2tex[edgeTemp->pvert_->id_]);
						edgeTemp->pvert_->texCoord_ = edgeTemp->texCoord_;
						faceIndex++;
					}
				}
			}
		}



		UpdateMesh();
		Unify(2.f);
	}
	catch (...)
	{
		ClearData();
		xmax_ = ymax_ = zmax_ = 1.f;
		xmin_ = ymin_ = zmin_ = -1.f;

		fclose(pfile);
		return false;
	}

	fclose(pfile);
	// 	for (EDGE_ITER iter = pedges_list_->begin(); iter != pedges_list_->end(); iter++)
	// 	{
	// 		qDebug() << (*iter)->id() << (*iter)->pface_;
	// 	}
	return isValid();
}

void Mesh3D::WriteToOBJFile(const char* fouts)
{
	std::ofstream fout(fouts);

	fout<<"g object\n";
	fout.precision(16);
	//output coordinates of each vertex
	VERTEX_ITER viter = pvertices_list_->begin();
	for (;viter!=pvertices_list_->end(); viter++) 
	{
		fout<<"v "<< std::scientific <<(*viter)->position_.x() 
			<<" "<<(*viter)->position_.y() <<" "<< (*viter)->position_.z() <<"\n";
	}

	// 		for (viter = pvertices_list_->begin();viter!=pvertices_list_->end(); viter++) 
	// 		{
	// 			fout<<"vn "<< std::scientific <<(*viter)->normal_.x() 
	// 				<<" "<<(*viter)->normal_.y() <<" "<<(*viter)->normal_.z() <<"\n";
	// 		}
	//output the valence of each face and its vertices_list' id

	FACE_ITER fiter = pfaces_list_->begin();

	for (;fiter!=pfaces_list_->end(); fiter++) 
	{
		fout<<"f";

		HE_edge* edge = (*fiter)->pedge_; 

		do {
			fout<<" "<<edge->ppair_->pvert_->id_+1;
			edge = edge->pnext_;

		} while (edge != (*fiter)->pedge_);
		fout<<"\n";
	}

	qDebug() << pvertices_list_->size();
	fout.close();
}

bool Mesh3D::LoadFromSTLFile(const char* fins)
{
	
	//支持中文
	QString filename = QString::fromLocal8Bit(fins);
	QFile file(filename);


	if (!file.open(QIODevice::ReadOnly))
	{

		std::cerr << "Cannot open file for reading:" << qPrintable(file.errorString()) << std::endl;
		return false;
	}
	ClearData();
	// read ASCII .stl file
	if (file.read(1) == "s")
	{
		QTextStream inASCII(file.readAll());
		ClearData();
		std::vector<HE_vert* > s_faceid;
		Vec3f normal;
		QString temp;
		while (!inASCII.atEnd())
		{
			inASCII >> temp;
			if (temp=="normal")
			{
				inASCII >> normal[0] >> normal[1] >> normal[2];
			}
			else if (temp == "vertex")
			{
				HE_vert* hv;
				Vec3f nvv;
				inASCII >> nvv[0] >> nvv[1] >> nvv[2];
				//now we have a new vertex
				hv = new HE_vert(nvv);
				if (pvertices_list_==NULL)
				{
					pvertices_list_ = new std::vector<HE_vert *>;
				}
				hv->set_id(static_cast<int>(pvertices_list_->size()));
				std::set<HE_vert*, comVertex>::iterator  iterVert = input_vertex_list_.insert(hv).first;//get the 
				if ((*iterVert)->id()== pvertices_list_->size())
				{
					pvertices_list_->push_back(hv);
				}
				else
				{
					hv = *iterVert;
				}
				s_faceid.push_back(hv);
			}
			else if (temp == "endfacet")
			{
				if (s_faceid.size() >= 3)
				{
					InsertFace(s_faceid,normal)->vertices_=s_faceid;
				}
				s_faceid.clear();
			}
		}
	}

	// read Binary .stl file
	else
	{
		quint32 num_of_face;
		file.seek(80);
		QDataStream inBinary(file.readAll());
		//qDebug() << file.pos();
		inBinary.setVersion(QDataStream::Qt_5_5);
		inBinary.setFloatingPointPrecision(QDataStream::SinglePrecision);
		inBinary >> num_of_face;
		//qDebug() << num_of_face;
		num_of_face = htonl(num_of_face);
		while (!inBinary.atEnd())
		{
			Vec3f nor_;
			quint32 normalvector[3];
			inBinary >> normalvector[0] >> normalvector[1] >> normalvector[2];//can confirm its correctness
			normalvector[0] = htonl(normalvector[0]);
			normalvector[1] = htonl(normalvector[1]);
			normalvector[2] = htonl(normalvector[2]);
			nor_[0] = *(float*)&normalvector[0];
			nor_[1] = *(float*)&normalvector[1];
			nor_[2] = *(float*)&normalvector[2];
			Vec3f nvv;
			//qDebug() << sizeof(nvv[0]);
			quint16 info;
			//qDebug() << sizeof(temp);
			std::vector<HE_vert* > s_faceid;
			for (int i = 0; i < 3; i++)
			{
				quint32 temp[3];
				inBinary >> temp[0] >> temp[1] >> temp[2];
				//qDebug() << nvv[0] << nvv[1] << nvv[2];
				temp[0] = htonl(temp[0]);
				temp[1] = htonl(temp[1]);
				temp[2] = htonl(temp[2]);
				nvv[0] = *(float*)&temp[0];
				nvv[1] = *(float*)&temp[1];
				nvv[2] = *(float*)&temp[2];
				//	qDebug() << nvv[0] << nvv[1] << nvv[2];
				//s_faceid.push_back(InsertVertex(nvv));
				HE_vert* hv = new HE_vert(nvv);
				if (pvertices_list_ == NULL)
				{
					pvertices_list_ = new std::vector<HE_vert*>;
					hv->set_id(0);
					//pvertices_list_->push_back(hv);
				}
				else
				{
					hv->set_id(static_cast<int>(pvertices_list_->size()));
				}

				std::set<HE_vert*, comVertex>::iterator  iterVert = input_vertex_list_.insert(hv).first;//get the 
				if ((*iterVert)->id() == pvertices_list_->size())
				{
					pvertices_list_->push_back(hv);
				}
				else
				{
					hv = *iterVert;
				}
				s_faceid.push_back(hv);
			}
			inBinary >> info;
			InsertFace(s_faceid,nor_)->vertices_=s_faceid;
			s_faceid.clear();
		}
	}
	file.close();
	Unify(2.f);
	return isValid();;
}

void Mesh3D::UpdateMesh(void)
{
	ComputeBoundingBox();
	SetBoundaryFlag();
	BoundaryCheck();
	countBoundaryComponat();
	UpdateNormal();
	ComputeAvarageEdgeLength();
	if (input_vertex_list_.size() == 0)
		SetNeighbors();
	//Unify(1);
}

void Mesh3D::SetBoundaryFlag(void)
{
	if (bheList==NULL)
	{
		bheList = new std::vector<HE_edge *>;
	}
	else
	{
		bheList->clear();
	}
	for (EDGE_ITER eiter = pedges_list_->begin(); eiter != pedges_list_->end(); eiter++)
	{
		if ((*eiter)->pface_ == NULL)
		{
			(*eiter)->set_boundary_flag(BOUNDARY);
			(*eiter)->ppair_->set_boundary_flag(BOUNDARY);
			(*eiter)->pvert_->set_boundary_flag(BOUNDARY);
			(*eiter)->ppair_->pvert_->set_boundary_flag(BOUNDARY);
			(*eiter)->ppair_->pface_->set_boundary_flag(BOUNDARY);
			bheList->push_back(*eiter);
		}
	}
}

void Mesh3D::BoundaryCheck()
{
	for (VERTEX_ITER viter=pvertices_list_->begin(); viter!=pvertices_list_->end(); viter++)
	{
		if ((*viter)->isOnBoundary())
		{
			HE_edge* edge = (*viter)->pedge_;
			int deg = 0;
			while (edge->pface_!=NULL && deg<(*viter)->degree())
			{
				edge = edge->pprev_->ppair_;
				deg ++;
			}
			(*viter)->pedge_ = edge;
		}
	}
}

void Mesh3D::countBoundaryComponat()
{
	if (bLoop.size()!=0)
	{
		return;
		bLoop.clear();
	}
	else
	{
		bLoop.resize(no_loop + 1);//initial the vectro bloop
	}
	//count the number of boundary loops
	size_t i;
	for (i = 0; i < bheList->size(); i++)
	{
		HE_edge *cur = bheList->at(i);
		HE_edge *nex = cur;
		while (nex->start_->selected() != SELECTED)
		{
			bLoop[no_loop].push_back(nex);
			nex->start_->set_seleted(SELECTED);
			nex = nex->pvert_->pedge_;
			if (nex == cur)
			{
				no_loop++;
				bLoop.resize(no_loop+1);
				break;
			}
		}
	}
	for (i = 0; i < bheList->size(); i++)
	{
		bheList->at(i)->start_->set_seleted(UNSELECTED);
	}
	bLoop.resize(no_loop);
	bLoop;
}

void Mesh3D::UpdateNormal(void)
{
	ComputeFaceslistNormal();
	ComputeVertexlistNormal();
}

void Mesh3D::ComputeFaceslistNormal(void)
{
	for (FACE_ITER fiter = pfaces_list_->begin(); fiter != pfaces_list_->end(); fiter++)
	{
		//if ((*fiter)->normal().x() == 0 && (*fiter)->normal().y() == 0 && (*fiter)->normal().z() == 0)
		{	
			ComputePerFaceNormal(*fiter);
		}
	
	}
}

void Mesh3D::ComputePerFaceNormal(HE_face* hf)
{
	HE_edge *pedge = hf->pedge_;
	HE_edge *nedge = hf->pedge_->pnext_;

	HE_vert *p = pedge->pvert_;
	HE_vert *c = pedge->pnext_->pvert_;
	HE_vert *n = nedge->pnext_->pvert_;

	Vec3f pc, nc;
	pc = p->position_ - c->position_;
	nc = n->position_ - c->position_;

	hf->normal_ = nc ^ pc;	// cross prodoct
	hf->normal_.normalize();
	//qDebug() << "normal compute";
}

void Mesh3D::ComputeVertexlistNormal(void)
{
	for (VERTEX_ITER viter = pvertices_list_->begin(); viter!=pvertices_list_->end(); viter++) 
	{
		ComputePerVertexNormal(*viter);
	}
}

void Mesh3D::ComputePerVertexNormal(HE_vert* hv)
{
	if (hv->degree_ < 2)
	{
		// ERROR: the degree of the vertex is less than 2
		hv->normal_ = Vec3f(1.f,0.f,0.f);
		return;
	}

	HE_edge *edge = hv->pedge_;
	if (edge == NULL)
	{
		// ERROR: the edge attached to the vertex is NULL
		hv->normal_ = Vec3f(1.f,0.f,0.f);
		return;
	}

	hv->normal_ = Vec3f(0.f,0.f,0.f);
	if (hv->boundary_flag_ == INNER)
	{
		int iterNum = 0;
		do 
		{
			iterNum++;
			if (iterNum > hv->degree())
			{
				/*hv->set_position(hv->position() * 1.1f);*/
				//std::cout << "    iterNum > hv->degree : " << hv->id() << "\n";
				break;
			}
			//hv->normal_ = hv->normal_ + edge->pface_->normal_;
			Vec3f  p = edge->pvert_->position(),
				q = edge->pnext_->pvert_->position(),
				r = edge->pprev_->pvert_->position();
			Vec3f  n = (q-p) ^ (r-p);
			hv->normal_ = hv->normal_ + n;
			edge = edge->ppair_->pnext_;
		} while (edge != hv->pedge_ && edge != NULL);
	}
	else
	{
		// NOTE: for the boundary vertices, this part may be something wrong
		//	     Up to now, define the normals all as unity
		hv->normal_ = Vec3f(1.f, 0.f, 0.f);

		//int degree_flag = 0;
		//for (int i=0; i<hv->degree_-1; i++)
		//{
		//	edge = edge->ppair_->pnext_;
		//	if (edge == NULL)
		//	{
		//		// ERROR: the algorithm of computing boundary vertices has errors!
		//		break;
		//	}
		//	if (edge->pface_ != NULL)
		//	{
		//		hv->normal_ = hv->normal_ + edge->pface_->normal_;
		//	}
		//}
	}
	hv->normal_.normalize();
}

void Mesh3D::ComputeBoundingBox(void)
{
	if (pvertices_list_->size() < 3)
	{
		return;
	}

#define MAX_FLOAT_VALUE (static_cast<float>(10e10))
#define MIN_FLOAT_VALUE	(static_cast<float>(-10e10))
	
	xmax_ = ymax_ = zmax_ = MIN_FLOAT_VALUE;
	xmin_ = ymin_ = zmin_ = MAX_FLOAT_VALUE;

	VERTEX_ITER viter = pvertices_list_->begin();
	for (; viter!=pvertices_list_->end(); viter++)
	{
		xmin_ = min(xmin_, (*viter)->position_.x());
		ymin_ = min(ymin_, (*viter)->position_.y());
		zmin_ = min(zmin_, (*viter)->position_.z());
		xmax_ = max(xmax_, (*viter)->position_.x());
		ymax_ = max(ymax_, (*viter)->position_.y());
		zmax_ = max(zmax_, (*viter)->position_.z());
	}
}

void Mesh3D::Unify(float size)
{
	ComputeBoundingBox();
	//qDebug() << "z position" << zmax_;
	float scaleX = xmax_ - xmin_;
	float scaleY = ymax_ - ymin_;
	float scaleZ = zmax_ - zmin_;
	float scaleMax;

	if (scaleX < scaleY)
	{
		scaleMax = scaleY;
	}
	else
	{
		scaleMax = scaleX;
	}
	if (scaleMax < scaleZ)
	{
		scaleMax = scaleZ;
	}
// 	scaleV = size / scaleMax;
// 	scaleT = scaleV;
// 	scaleV = 1;
	Vec3f centerPos((xmin_ +xmax_)/2.0, (ymin_+ymax_)/2.0, (zmin_));
	//Vec3f centerPos(xmin_ , ymin_, zmin_);
	for (size_t i = 0; i != pvertices_list_->size(); i++)
	{
		pvertices_list_->at(i)->position_ = (pvertices_list_->at(i)->position_ - centerPos)*scaleT;
	}
	//	update bounding box and average edge length
	ComputeBoundingBox();
	ComputeAvarageEdgeLength();
}

void Mesh3D::ComputeAvarageEdgeLength(void)
{
	if(!isValid())
	{
		average_edge_length_ = 0.f;
		return;
	}
	float aveEdgeLength = 0.f;
	for (int i=0; i<num_of_half_edges_list(); i++)
	{
		HE_edge* edge = get_edges_list()->at(i);
		HE_vert* v0 = edge->pvert_;
		HE_vert* v1 = edge->ppair_->pvert_;
		aveEdgeLength += (v0->position() - v1->position()).length();
	}
	average_edge_length_ = aveEdgeLength/num_of_half_edges_list();
	//std::cout << "Average_edge_length = " << average_edge_length_ << "\n";
}

HE_face* Mesh3D::get_face(int vId0, int vId1, int vId2)
{
	HE_vert *v0 = get_vertex(vId0);
	HE_vert *v1 = get_vertex(vId1);
	HE_vert *v2 = get_vertex(vId2);
	if (!v0 || !v1 || !v2)
	{
		return NULL;
	}

	HE_face* face=NULL;

	// 由于对边界点的邻域遍历有bug，所以找到非边界点进行邻域遍历
	if (v0->isOnBoundary())
	{
		if (!v1->isOnBoundary())
		{
			SWAP(v0, v1, HE_vert*);
		}
		else if (!v2->isOnBoundary())
		{
			SWAP(v0, v2, HE_vert*);
		}
		else
		{
			// v0, v1, v2 都是边界点
			// 暂时先不处理
			return NULL;
		}
	}

	if (!v0->isOnBoundary())	// 对边界点的遍历有bug
	{
		HE_edge* edge=v0->pedge_;
		bool inFace = true;
		do 
		{
			bool b1 = isFaceContainVertex(edge->pface_, v1);
			bool b2 = isFaceContainVertex(edge->pface_, v2);
			if (!b1 && !b1)
			{
				edge = edge->ppair_->pnext_;
			}
			else if(b1 && b2)
			{
				face = edge->pface_;
				break;
			}
			else
			{
				inFace = false;
				break;
			}
		} while (edge!=v0->pedge_ && edge!=NULL);
	}

	return face;
}

HE_face* Mesh3D::get_face(const std::vector<unsigned int>& ids)
{
	if (ids.size()<3)
	{
		std::cout << "查询点数过少，无法返回面\n";
		return NULL;
	}
	// 首先找到一个非边界点
	HE_vert* v = NULL;
	for (unsigned int i=0; i<ids.size(); i++)
	{
		if (!get_vertex(ids[i])->isOnBoundary())
		{
			v = get_vertex(ids[i]);
			break;
		}
	}
	if (!v)
	{
		// 所有点都是边界点
		// 暂不处理
		return NULL;
	}

	HE_edge *edge = v->pedge_;
	HE_face *face = NULL;
	do 
	{
		face = edge->pface_;
		edge = edge->ppair_->pnext_;
		bool bInFace = isFaceContainVertex(face, get_vertex(ids[0]));
		if (!bInFace)
		{
			continue;
		}
		for (unsigned int i=1; i<ids.size(); i++)
		{
			bool b = isFaceContainVertex(face, get_vertex(ids[i]));
			if (b!=bInFace)
			{
				bInFace = false;
				break;
			}
		}
		if (bInFace)
		{
			return face;
		}
	} while (edge!=v->pedge_ && edge!=NULL);
	return NULL;
}

bool Mesh3D::isFaceContainVertex(HE_face* face, HE_vert* vert)
{
	HE_edge* edge = face->pedge_;
	do 
	{
		if (edge->pvert_==vert)
		{
			return true;
		}
		edge = edge->pnext_;
	} while (edge!=face->pedge_ && edge!=NULL);
	return false;
}

void Mesh3D::Transformation(float * matrix)
{
	if (pvertices_list_ != NULL)
	{
		for (VERTEX_ITER iter = pvertices_list_->begin(); iter != pvertices_list_->end(); iter++)
		{
			Vec4f temp((*iter)->position().x(), (*iter)->position().y(), (*iter)->position().z(), 1);
			(*iter)->position_[0] = matrix[0] * temp[0] + matrix[1] * temp[1] + matrix[2] * temp[2] + matrix[3] * temp[3];
			(*iter)->position_[1] = matrix[4] * temp[0] + matrix[5] * temp[1] + matrix[6] * temp[2] + matrix[7] * temp[3];
			(*iter)->position_[2] = matrix[8] * temp[0] + matrix[9] * temp[1] + matrix[10] * temp[2] + matrix[11] * temp[3];
					
		}
	}
	ComputeBoundingBox();
	//Unify(2.0);

}


void Mesh3D::SetDirection(int faceid)
{
	if (faceid==-1)
	{
		return;
	}
	QVector3D normal_(pfaces_list_->at(faceid)->normal().x(), pfaces_list_->at(faceid)->normal().y(), pfaces_list_->at(faceid)->normal().z());
	QVector3D center_(pfaces_list_->at(faceid)->center().x(), pfaces_list_->at(faceid)->center().y(), pfaces_list_->at(faceid)->center().z());
	QVector3D print_dir_(0.0,0.0,-1.0);

	
	float cosTheta =QVector3D::dotProduct(normal_, print_dir_);
	if (fabs(cosTheta - 1.0) <= 1.0e-6)
		cosTheta =1;//0
	else if (fabs(cosTheta + 1.0) <= 1.0e-6)
		cosTheta = -1;//180
	
	//qDebug() << normal_ << print_dir_<<qAcos(cosTheta) * 180 / qAcos(-1) <<qAcos(-1) * 180 / qAcos(-1);
	QMatrix4x4  matrix_;
	QVector3D rotationAxis;
	if (cosTheta == -1)
	{
		rotationAxis = QVector3D(1.0, 0.0, 0.0);
	}
	else if (cosTheta==1)
	{
		rotationAxis = QVector3D(1.0, 0.0, 0.0);
	}
	else
	{
		rotationAxis = QVector3D::crossProduct(normal_, print_dir_);
	}
	matrix_.setToIdentity();
	matrix_.rotate(qAcos(cosTheta) * 180 / qAcos(-1), rotationAxis);
	// 		QVector3D temp = matrix_*normal_;
	// 		if ((temp-print_dir_).length()> 1.0e-6)
	// 		{
	// 			matrix_.setToIdentity();
	// 			matrix_.rotate((2*qAcos(-1) - qAcos(cosTheta)) * 180 / qAcos(-1), rotationAxis);
	// 		}
	// 		temp = matrix_*normal_;


	for (VERTEX_ITER iter=pvertices_list_->begin();iter!=pvertices_list_->end();iter++)
	{
		
		QVector4D position_((*iter)->position().x(), (*iter)->position().y(), (*iter)->position().z(), 1.0);
		position_ = matrix_*position_;
		(*iter)->position().x() = position_[0];
		(*iter)->position().y() = position_[1];
		(*iter)->position().z() = position_[2];
	}
}


void Mesh3D::ClearSlice()
{
	throw std::logic_error("The method or operation is not implemented.");
}


int Mesh3D::GetFaceId(HE_face* face)
{
	return !face ? -1 : face->id();
}

void Mesh3D::ResetFaceSelectedTags(int tag)
{
	for (int i=0; i<num_of_face_list(); i++)
	{
		get_face(i)->set_selected(tag);
	}
}

void Mesh3D::ResetVertexSelectedTags(int tag)
{
	for (int i=0; i<num_of_vertex_list(); i++)
	{
		get_vertex(i)->set_seleted(tag);
	}
}

bool Mesh3D::isNeighbors(HE_vert* v0, HE_vert* v1)
{
	if (!v0 || !v1)
	{
		return false;
	}

	HE_edge *edge = v0->pedge_;
	do 
	{
		if (edge->pvert_==v1)
		{
			return true;
		}
		edge = edge->ppair_->pnext_;
	} while (edge!=v0->pedge_ && edge);
	return false;
}

int Mesh3D::GetSelectedVrtId()
{
	if (!isValid())
	{
		return -1;
	}
	for (int i=0; i<num_of_vertex_list(); i++)
	{
		if (get_vertex(i)->selected()==SELECTED)
		{
			return i;
		}
	}
	return -1;
}

void Mesh3D::CreateMesh(const std::vector<Vec3f>& verts, const std::vector<int>& triIdx)
{
	ClearData();
	for (unsigned int i=0; i<verts.size(); i++)
	{
		InsertVertex(verts[i]);
	}
	for (unsigned int i=0; i<triIdx.size(); i=i+3)
	{
		std::vector<HE_vert*> tri;
		HE_vert *v0 = get_vertex(triIdx[i]);
		HE_vert *v1 = get_vertex(triIdx[i+1]);
		HE_vert *v2 = get_vertex(triIdx[i+2]);
		if (!v0 || !v1 || !v2) continue;
		tri.push_back(v0);
		tri.push_back(v1);
		tri.push_back(v2);
		InsertFace(tri);
	}
	UpdateMesh();
}

void Mesh3D::CreateMesh(const std::vector<double>& verts, const std::vector<unsigned>& triIdx)
{
	ClearData();
	for (unsigned int i=0; i<verts.size(); i=i+3)
	{
		InsertVertex(Vec3f(verts[i], verts[i+1], verts[i+2]));
	}
	for (unsigned int i=0; i<triIdx.size(); i=i+3)
	{
		std::vector<HE_vert*> tri;
		HE_vert *v0 = get_vertex(triIdx[i]);
		HE_vert *v1 = get_vertex(triIdx[i+1]);
		HE_vert *v2 = get_vertex(triIdx[i+2]);
		if (!v0 || !v1 || !v2) continue;
		tri.push_back(v0);
		tri.push_back(v1);
		tri.push_back(v2);
		InsertFace(tri);
	}
	UpdateMesh();
}

int Mesh3D::GetBoundaryVrtSize()
{
	int count = 0;
	for (int i=0; i<num_of_vertex_list(); i++)
	{
		if (get_vertex(i)->isOnBoundary())
		{
			count ++;
		}
	}
	return count;
}

void Mesh3D::meshTranslate(float param1, float param2)
{
	if (pvertices_list_ != NULL)
	{
		for (VERTEX_ITER iter = pvertices_list_->begin(); iter != pvertices_list_->end(); iter++)
		{
			

		}
	}
}

void Mesh3D::scalemesh(float size)
{
	Unify(size);
}

Mesh3D::~Mesh3D(void)
{
	ClearData();
}

void Mesh3D::exportNeighborId()
{
	//FILE outfile("C:\\Users\\Dell\\Desktop\\zhulin.txt");
	std::fstream out;
	out.open("C:\\Users\\Dell\\Desktop\\zhulin.txt");
	out.clear();
	if (!out.is_open())
	{
		return;
	}
	{
  		for (VERTEX_ITER iter = pvertices_list_->begin(); iter != pvertices_list_->end(); iter++)
  		{	
  			//out << "v "<<(*iter)->id()<<" ";
  			//out << (*iter)->position().x() << " " << (*iter)->position().y() << " " << (*iter)->position().z();
  			for (int i=0;i<(*iter)->neighborIdx.size();i++)
  			{
  				out<<(*iter)->neighborIdx[i]<<" ";
  			}
  			out << "\r\n";
  		}
		for (FACE_ITER iterface = pfaces_list_->begin(); iterface != pfaces_list_->end(); iterface++)
		{
			
			HE_edge * cur = (*iterface)->pedge_; 
			HE_edge *last = cur;
			do 
			{
				out << "f " << (*iterface)->id() << " ";
				out << cur->id() << " " <<cur->ppair_->id() << " " << cur->pprev_->pvert_->id() << " " << cur->pvert_->id()<<"\r\n";
				cur = cur->pnext_;
			} while (last!=cur);
		}
	}
	out.close();
}

//////////////////////////////////////////////////////////////////////////
HE_face* Mesh3D::InsertFaceSup(std::vector<HE_vert* >& vec_hv)
{
	if (pfaces_list_ == NULL)
	{
		pfaces_list_ = new std::vector<HE_face*>;
	}

	int i = 0;
	HE_edge *he[3], *bhe[3];
	HE_vert *v[3];
	HE_face *f;

	// obtain objects
	for (i = 0; i < 3; i++) he[i] = new HE_edge();
	for (i = 0; i < 3; i++) {
		bhe[i] = new HE_edge();
		bhe[i]->set_boundary_flag(BOUNDARY);
	}
	v[0] = vec_hv[0];
	v[1] = vec_hv[1];
	v[2] = vec_hv[2];
	f = new HE_face();

	// connect prev-next pointers
	he[0]->pnext_ = he[1];
	he[1]->pprev_ = he[0];
	he[1]->pnext_ = he[2];
	he[2]->pprev_ = he[1];
	he[2]->pnext_ = he[0];
	he[0]->pprev_ = he[2];
	bhe[0]->pnext_ = bhe[1];
	bhe[1]->pprev_ = bhe[0];
	bhe[1]->pnext_ = bhe[2];
	bhe[2]->pprev_ = bhe[1];
	bhe[2]->pnext_ = bhe[0];
	bhe[0]->pprev_ = bhe[2];
	// connect twin pointers
	he[0]->ppair_ = bhe[0];
	bhe[0]->ppair_ = he[0];
	he[1]->ppair_ = bhe[2];
	bhe[2]->ppair_ = he[1];
	he[2]->ppair_ = bhe[1];
	bhe[1]->ppair_ = he[2];

	// connect start pointers for bhe
	bhe[0]->start_ = v[1];
	bhe[0]->pvert_ = v[0];
	bhe[1]->start_ = v[0];
	bhe[1]->pvert_ = v[2];
	bhe[2]->start_ = v[2];
	bhe[2]->pvert_ = v[1];
	he[0]->start_ = v[0];
	he[0]->pvert_ = v[1];
	he[1]->start_ = v[1];
	he[1]->pvert_ = v[2];
	he[2]->start_ = v[2];
	he[2]->pvert_ = v[0];
	// connect start pointers
	// connect face-hedge pointers
	for (i = 0; i < 3; i++) {
		v[i]->pedge_ = (he[i]);
		he[i]->pface_ = f;
		f->pedge_ = he[i];
	}

	if (pedges_list_ == NULL)
	{
		pedges_list_ = new std::vector<HE_edge*>;
	}
	if (bheList == NULL)
	{
		bheList = new std::vector<HE_edge *>;
	}
	for (i = 0; i < 3; i++)
	{

		he[i]->id_ = static_cast <int>(pedges_list_->size());
		pedges_list_->push_back(he[i]);
	}
	for (i = 0; i < 3; i++) bheList->push_back(bhe[i]);
	// merge boundary if needed
	for (i = 0; i < 3; i++) {
		HE_vert *start = bhe[i]->start_;
		HE_vert *end = bhe[i]->pvert_;
		for (size_t j = 0; j < end->adjHEdges.size(); j++) {
			HE_edge *curr = end->adjHEdges[j];
			if (curr->boundary_flag() == BOUNDARY && curr->pvert_ == start) {

				bhe[i]->pprev_->pnext_ = curr->pnext_;
				curr->pnext_->pprev_ = bhe[i]->pprev_;
				curr->pprev_->pnext_ = bhe[i]->pnext_;
				bhe[i]->pnext_->pprev_ = curr->pprev_;
				bhe[i]->ppair_->ppair_ = curr->ppair_;
				curr->ppair_->ppair_ = bhe[i]->ppair_;
				bhe[i]->start_ = NULL;	// mark as unused
				curr->start_ = NULL;	// mark as unused
				break;
			}
		}
	}
	v[0]->adjHEdges.push_back(he[0]);
	v[0]->adjHEdges.push_back(he[2]);
	v[0]->adjHEdges.push_back(bhe[0]);
	v[0]->adjHEdges.push_back(bhe[1]);
	v[1]->adjHEdges.push_back(he[1]);
	v[1]->adjHEdges.push_back(he[0]);
	v[1]->adjHEdges.push_back(bhe[0]);
	v[1]->adjHEdges.push_back(bhe[2]);
	v[2]->adjHEdges.push_back(he[1]);
	v[2]->adjHEdges.push_back(he[2]);
	v[2]->adjHEdges.push_back(bhe[1]);
	v[2]->adjHEdges.push_back(bhe[2]);

	// finally add hedges and faces to list
	f->id_ = static_cast<int>(pfaces_list_->size());
	pfaces_list_->push_back(f);
	return f;
}
void Mesh3D::UpdateMeshSup(void)
{
	
	UpdateBList();
	ComputeBoundingBox();
	//Unify(1.0);
	SetNeighbors();
	SetBoundaryFlag();
	computeComponent();
	UpdateNormal();
}
void Mesh3D::UpdateBList(void)
{
	std::vector<HE_edge*>* list = new std::vector<HE_edge*>;
	for (int i = 0; i < bheList->size(); i++)
	{
		if ((*bheList)[i]->start_ != NULL)
		{
			list->push_back(bheList->at(i));
			bheList->at(i)->id_ = static_cast<int> (pedges_list_->size());
			pedges_list_->push_back(bheList->at(i));
		}
		else
		{
			delete bheList->at(i);
			bheList->at(i) = NULL;
		}
	}
	bheList = list;
}
void Mesh3D::computeComponent()
{
	if (bheList == NULL)
	{
		return;
	}
	bLoop.clear();
	num_components_ = 0;
	for (size_t i = 0; i < bheList->size(); i++)
	{
		bheList->at(i)->is_selected_=false;
	}
	for (size_t i = 0; i < bheList->size(); i++)
	{
		HE_edge *sta = bheList->at(i);
		HE_edge *cur = sta;
		if (cur->is_selected_)
		{
			continue;
		}

		bLoop.resize(++num_components_);

		do
		{
			cur->is_selected_=true;
			bLoop[num_components_ - 1].push_back(cur);
			cur = cur->pnext_;
		} while (cur != sta);
	}
	for (size_t i = 0; i < bheList->size(); i++)
	{
		bheList->at(i)->start_->set_seleted(UNSELECTED);
	}
	return;
	for (int i = 0; i < num_components_; i++)
	{
		HE_face* facet = bLoop[i].at(1)->ppair_->pface_;

		FaceDFS(facet, i);
	}
	for (int i = 0; i < num_of_face_list(); i++)
	{
		pfaces_list_->at(i)->set_selected(UNSELECTED);
	}
}
void Mesh3D::FaceDFS(HE_face* facet, int no)
{
	facet->set_selected(SELECTED);
	facet->com_flag= no;
	HE_edge* sta = facet->pedge_;
	HE_edge* cur = sta;
	do
	{
		facet = cur->ppair_->pface_;
		if (facet != NULL&&facet->selected() == UNSELECTED)
		{
			FaceDFS(cur->ppair_->pface_, no);
		}
		cur = cur->pnext_;
	} while (cur != sta&&cur != NULL);

}
void Mesh3D::MarkEdge()
{
	for (int i = 0; i < pedges_list_->size(); i++)
	{
		if (pedges_list_->at(i)->ppair_->pface_ == NULL || pedges_list_->at(i)->pface_==NULL)
		{
			pedges_list_->at(i)->set_boundary_flag(BOUNDARY);
		}
		else if (pedges_list_->at(i)->pface_->normal().dot(pedges_list_->at(i)->ppair_->pface_->normal())<0.1)
		{
			pedges_list_->at(i)->set_boundary_flag(BOUNDARY);
		}
	}
}




