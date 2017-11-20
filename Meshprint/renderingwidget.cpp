#pragma once
#include "renderingwidget.h"
#include <QKeyEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <iostream>
#include <QtWidgets/QMenu>
#include <QtWidgets/QAction>
#include <QTextCodec>
#include <gl/GLU.h>
#include <gl/glut.h>
#include <algorithm>
#include <queue>
#include "ArcBall.h"
#include "globalFunctions.h"
#include "Support.h"
#include "SliceCut.h"
#include "openGLProjector.h"
#include "QDebug"
#include "meshprint.h"
#include <fstream>
#include "Hatch.h"
#include "space2dKDTree.h"
#include <QTime>
#include "Grid.h"
#include <omp.h>
#include "MeshInfo.h"
#include <QMessageBox>
#include "Cubes.h"
class Support;
class Cubes;
class Cubes::Box;
static GLdouble winwidth, winHeight;
RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
	: QOpenGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(5.0),
	has_lighting_(true), is_draw_point_(false), is_draw_edge_(false), is_draw_face_(true)
{
	ptr_arcball_ = new CArcBall(width(), height());
	ptr_arcball_module_ = new CArcBall(width(), height());
	ptr_mesh_ = new Mesh3D();
	mycut = NULL;
	mycutsup = NULL;
	myhatch = NULL;
	myhatchsup = NULL;
	current_face_ = -1;
	ptr_support_ = new Support(ptr_mesh_);
	isAddPoint = false;
	isAddLine = false;
	is_select_face = false;
	is_draw_hatch_ = false;
	isDelete = false;
	is_load_texture_ = false;
	is_draw_axes_ = false;
	is_draw_texture_ = (false);
	is_draw_grid_ = (false);
	is_draw_cutpieces_ = (false);
	is_move_module_ = (false);
	is_draw_region_ = false;
	is_show_all = false;
	eye_goal_[0] = eye_goal_[1] = eye_goal_[2] = 0.0;
	eye_direction_[0] = eye_direction_[1] = 0.0;
	eye_direction_[2] = 1.0;
	slice_check_id_ = 1;
	hatch_type_ = NONE;
	is_draw_support_ = true;
}

RenderingWidget::~RenderingWidget()
{
	SafeDelete(ptr_arcball_);
	SafeDelete(ptr_arcball_module_);
	SafeDelete(ptr_mesh_);
	SafeDelete(ptr_support_);
}

void RenderingWidget::initializeGL()
{
	glClearColor(0.68, 0.68, 0.68, 0.0);
	glShadeModel(GL_FLAT);
	glEnable(GL_DOUBLEBUFFER);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1);

	SetLight();
}

void RenderingWidget::resizeGL(int w, int h)
{
	h = (h == 0) ? 1 : h;
	winwidth = w;
	winHeight = h;
	ptr_arcball_->reSetBound(w, h);
	ptr_arcball_module_->reSetBound(w, h);
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, GLdouble(w) / GLdouble(h), 0.1, 1000);
// 	glOrtho(GLdouble(-winwidth / 2), GLdouble(winwidth / 2),
// 		GLdouble(-winHeight / 2), GLdouble(winHeight / 2), -1000, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void RenderingWidget::paintGL()
{
	glShadeModel(GL_SMOOTH);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	register vec eyepos = eye_distance_*eye_direction_;
	gluLookAt(eyepos[0], eyepos[1], eyepos[2],
		eye_goal_[0], eye_goal_[1], eye_goal_[2],
		0.0, 1.0, 0.0);
// 	glOrtho(GLdouble(-winwidth / 2)*0.1, GLdouble(winwidth / 2)*0.1,
// 		GLdouble(-winHeight / 2)*0.1, GLdouble(winHeight / 2)*0.1, -1000, 1000);
	if (has_lighting_)
	{
		SetLight();
	}
	else
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_LIGHT0);
	}

	glPushMatrix();
	glMultMatrixf(ptr_arcball_->GetBallMatrix());
	Render();
	glPopMatrix();
}

void RenderingWidget::timerEvent(QTimerEvent * e)
{
	update();
}

void RenderingWidget::mousePressEvent(QMouseEvent *e)
{

	switch (e->button())
	{
	case Qt::LeftButton:
	{
		makeCurrent();
		OpenGLProjector myProjector = OpenGLProjector();

		Vec3f myPointN(e->x(), height() - e->y(), -1.0f);
		Vec3f myPointF(e->x(), height() - e->y(), 1.0f);
		Vec3f add_pointN = myProjector.UnProject(myPointN);
		Vec3f add_pointF = myProjector.UnProject(myPointF);


		Vec3f direc = add_pointF - add_pointN;
		add_pointN -= direc * 10.0f;
		direc.normalize();

		Vec3f hitPointT;
		int hitID = -1;
		float temp_t = -1.0f;
		this->ptr_support_->GetMeshInOctree()->hitOctreeNode(this->ptr_support_->GetMeshInOctree()->oc_root_, add_pointN, hitPointT, direc, hitID, temp_t);

		if (isAddPoint && hitID != -1)
		{

			const Vec3f _z(0, 0, -1);
			std::vector<HE_vert *> tmp_face_verts;
			this->ptr_mesh_->get_faces_list()->at(hitID)->face_verts(tmp_face_verts);

			Vec3f eAB = tmp_face_verts[1]->position_ - tmp_face_verts[0]->position_;
			Vec3f eAC = tmp_face_verts[2]->position_ - tmp_face_verts[0]->position_;
			Vec3f vNormal = eAB.cross(eAC);
			vNormal.normalize();

			if (vNormal.dot(_z) <= 0)
			{
				std::cout << "this point is not facing downward" << std::endl;
				break;
			}

			ptr_support_->AddPointSupport(hitPointT);
			ptr_support_->updateSupportMesh();
			ptr_support_->BuildSupportOctree();
			parent->ui.applyPointSupportButton->setChecked(true);
			update();
		}
		else if (isAddLine)
		{
			if (line_points_.size() < 2 && hitID != -1)
			{
				line_points_.push_back(hitPointT);
			}
			if (line_points_.size() == 2)
			{
				//sample points on (line_points_[0], line_points_[1]) and add support structure to the mesh
				float lengthTmp = (line_points_[1] - line_points_[0]).length();
				Vec3f direc = line_points_[1] - line_points_[0];
				direc.normalize();

				std::vector<Vec3f> toAdd;
				float t = 0;
				while (t < lengthTmp)
				{
					Vec3f point1T = line_points_[0] + t * direc;
					////////////////////////// hit UP and DOWN, choose interacted point outside the model //////////////////// 		
					int hitID_up = -1;
					int hitID_down = -1;
					Vec3f point1TUp = ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, 1), true, std::vector<int>(), 0, hitID_up);
					Vec3f point1TDo = ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, -1), true, std::vector<int>(), 0, hitID_down);
					if (hitID_up != -1)
					{
						if (ptr_mesh_->get_face(hitID_up)->normal() * Vec3f(0, 0, -1) > 0)
						{
							point1T = point1TUp;
						}
						else if (ptr_mesh_->get_face(hitID_down)->normal() * Vec3f(0, 0, -1) > 0)
						{
							point1T = point1TDo;
						}
					}
					else
					{
						// hit downside		
						point1T = ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, -1), true);
					}
					////////////////////////// hit UP and DOWN, choose interacted point outside the model ////////////////////

					toAdd.push_back(point1T);

					t += RESO;
				}
				toAdd.push_back(line_points_[1]);
				ptr_support_->AddLineSupport(toAdd);
				ptr_support_->updateSupportMesh();
				ptr_support_->BuildSupportOctree();
				line_points_.clear();
				update();
			}
		}
		else
		{
			ptr_arcball_->MouseDown(e->pos());
		}
		update();
	}
	break;
	case Qt::MidButton:
		current_position_ = e->pos();
		break;
	case  Qt::RightButton:
	{
		makeCurrent();
		OpenGLProjector myProjector = OpenGLProjector();
		Vec3f myPointN(e->x(), height() - e->y(), -1.0f);
		Vec3f myPointF(e->x(), height() - e->y(), 1.0f);
		Vec3f add_pointN = myProjector.UnProject(myPointN);
		Vec3f add_pointF = myProjector.UnProject(myPointF);
		Vec3f direc = add_pointF - add_pointN;
		add_pointN -= direc * 10.0f;
		direc.normalize();
		Vec3f hitPointT;
		int hitID = -1;
		float temp_t = -1.0f;
		this->ptr_support_->GetMeshInOctree()->hitOctreeNode(this->ptr_support_->GetMeshInOctree()->oc_root_, add_pointN, hitPointT, direc, hitID, temp_t);
		SetDirection(hitID);
		ptr_mesh_->UpdateMesh();
		ptr_mesh_->scalemesh(1.0);
		//ptr_support_->GetMeshInOctree()->DeleteMeshOcTree();
		ptr_support_->GetMeshInOctree()->BuildOctree(ptr_mesh_);
		current_position_ = e->pos();
		update();
		break;
	}

	default:
		break;
	}
}
void RenderingWidget::mouseMoveEvent(QMouseEvent *e)
{
	switch (e->buttons())
	{
		setCursor(Qt::ClosedHandCursor);

	case Qt::LeftButton:
		if (isDelete && delete_points_.size() < 1000)
		{
			delete_points_.push_back(e->x());
			delete_points_.push_back(height() - e->y());
		}
		else if (is_move_module_)
		{
			ptr_arcball_module_->MouseMove(e->pos());
		}
		else
		{
			ptr_arcball_->MouseMove(e->pos());
		}
		break;

	case Qt::MidButton:
		if (is_move_module_)
		{
			ptr_mesh_->meshTranslate(float(e->x() - current_position_.x()), float(e->y() - current_position_.y()));
			ptr_support_->GetMeshSupport()->meshTranslate(float(e->x() - current_position_.x()), float(e->y() - current_position_.y()));
			current_position_ = e->pos();
		}
		else
		{
			if (ptr_mesh_ != NULL)
			{
				eye_goal_[0] -= ptr_mesh_->getBoundingBox().at(0).at(2)*scaleV*GLfloat(e->x() - current_position_.x()) / GLfloat(width());
				eye_goal_[1] += ptr_mesh_->getBoundingBox().at(0).at(2)*scaleV*GLfloat(e->y() - current_position_.y()) / GLfloat(height());
			}
			current_position_ = e->pos();
		}
		break;
	default:
		break;
	}
	update();
}
void RenderingWidget::mouseReleaseEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		if (is_move_module_)
		{
			ptr_arcball_module_->MouseUp(e->pos());
		}
		else
		{
			ptr_arcball_->MouseUp(e->pos());
		}

		setCursor(Qt::ArrowCursor);
		if (isDelete)
		{
			if (!delete_points_.empty())
			{
				makeCurrent();
				ptr_support_->DeleteSupport(delete_points_);
			}
			delete_points_.clear();
			update();
		}
		else
		{
			ptr_arcball_->MouseUp(e->pos());
			setCursor(Qt::ArrowCursor);
		}
		break;
	case Qt::RightButton:
		break;
	default:
		break;
	}
}
void RenderingWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	default:
		break;
	}
	update();
}
void RenderingWidget::wheelEvent(QWheelEvent *e)
{
	if (ptr_mesh_ != NULL)
	{
		//eye_distance_ -= e->delta()*0.1;
		eye_distance_ -= e->delta()*ptr_mesh_->getBoundingBox().at(0).at(2)*scaleV / 1000;
	}
	eye_distance_ = eye_distance_ < 0 ? 0 : eye_distance_;
	update();
}

void RenderingWidget::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		break;
	default:
		break;
	}
}

void RenderingWidget::keyReleaseEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		break;
	default:
		break;
	}
}

void RenderingWidget::Render()
{
	DrawAxes(is_draw_axes_);
	DrawGrid(is_draw_grid_);
	DrawPoints(is_draw_point_);
	DrawEdge(is_draw_edge_);
	DrawFace(is_draw_face_);
	DrawTexture(is_draw_texture_);
	DrawCutPieces(is_draw_cutpieces_);
	DrawCutPiecesSup(is_draw_cutpieces_);
	DrawHatch(is_draw_hatch_);
	DrawHatchsup(is_draw_hatch_);
	DrawSupFace(is_draw_region_);
	DrawInnerLoop(is_draw_support_);
	DrawOuterLoop(is_draw_support_);
	DrawSupFaceZero(false);
	DrawSupport(is_draw_support_);
	DrawSegments(true);
	drawCub(true);
}

void RenderingWidget::SetLight()
{
	//return;
	static GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	static GLfloat mat_shininess[] = { 50.0f };
	static GLfloat light_position0[] = { 1.0f, 1.0f, 0.5f, 0.0f };
	static GLfloat light_position1[] = { -1.0f, -1.0f, 0.5f, 0.0f };
	static GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	static GLfloat dim_light[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	static GLfloat lmodel_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	//glMaterialfv(GL_FRONT, GL_AMBIENT, mat_specular);
	//glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT0, GL_SPECULAR, bright);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, dim_light);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, white_light);
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
}

bool booladsad = true;
void RenderingWidget::SetBackground()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("background color"));
	GLfloat r = (color.red()) / 255.0f;
	GLfloat g = (color.green()) / 255.0f;
	GLfloat b = (color.blue()) / 255.0f;
	GLfloat alpha = color.alpha() / 255.0f;
	makeCurrent();
	glClearColor(r, g, b, alpha);

	//updateGL();
	update();
}

void RenderingWidget::ReadMesh()
{
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	ptr_arcball_->reSetBound(width(), height());
	ptr_arcball_module_->reSetBound(width(), height());
	SafeDelete(ptr_mesh_);
	SafeDelete(ptr_support_);
	ptr_mesh_ = new Mesh3D;
	ptr_support_ = new Support(ptr_mesh_);
	delete mycut;
	delete mycutsup;
	delete  myhatch;
	delete myhatchsup;
	mycut = mycutsup = NULL;
	myhatch = myhatchsup=NULL;

	is_draw_grid_ = true;
	is_draw_face_ = true;
	is_draw_cutpieces_ = true;
	is_draw_hatch_ = true;
	has_lighting_ = true;
	//ClearSlice();
	//ClearHatch();
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Read Mesh"),
			"\models", tr("Meshes (*.obj *.stl)"));

	if (filename.isEmpty())
	{
		QMessageBox::information( this, "Warning Message!", "Please select a file");
		return;
	}
	//中文路径支持
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);

	//mycut->clearcut();

	QByteArray byfilename = filename.toLocal8Bit();
	QFileInfo fileinfo = QFileInfo(filename);
	//qDebug() << "read Mesh start time" << str;
	//qDebug() << byfilename.data();
	qDebug() << "load model time at " << time;

	if (fileinfo.suffix() == "obj")
	{
		ptr_mesh_->LoadFromOBJFile(byfilename.data());
	}
	else if (fileinfo.suffix() == "stl" || fileinfo.suffix() == "STL")
	{
		ptr_mesh_->LoadFromSTLFile(byfilename.data());
	}
	faceNeedSupport.resize(ptr_mesh_->num_of_face_list());
	ptr_mesh_->markEdges();
	float max_ = ptr_mesh_->getBoundingBox().at(0).at(0);
	max_ = max_ > ptr_mesh_->getBoundingBox().at(0).at(1) ? max_ : ptr_mesh_->getBoundingBox().at(0).at(1);
	max_ = max_ > ptr_mesh_->getBoundingBox().at(0).at(2) ? max_ : ptr_mesh_->getBoundingBox().at(0).at(2);

	update();
	ptr_support_->GetMeshInOctree()->BuildOctree(ptr_mesh_);
	scaleT = scaleV;
	eye_goal_[2] = ptr_mesh_->getBoundingBox().at(0)[2] / 2;
	eye_distance_ = 2 * max_;
	QString a = "this model has ";
	a.append(QString::number(ptr_mesh_->num_of_face_list(),10));
	a.append(" faces, and it's size is ");
	double v1 = ptr_mesh_->getBoundingBox()[0][0] * 2;
	double v2= ptr_mesh_->getBoundingBox()[0][1] * 2;
	double v3= ptr_mesh_->getBoundingBox()[0][2];
	QString s = QString("%1").arg(v1,0,'g',4);
	a.append(s);
	a.append(" x ");
	s = QString("%1").arg(v2, 0, 'g', 4);
	a.append(s);
	a.append(" x ");
	s = QString("%1").arg(v3, 0, 'g', 4);
	a.append(s);

	QMessageBox::information(this, "Read Successfully!", a);
	return;
	MeshInfo* winfo=new MeshInfo();
	winfo->setMeshinfo(ptr_mesh_->num_of_face_list(), ptr_mesh_->getBoundingBox());
	winfo->show();
}

void RenderingWidget::WriteMesh()
{
	if (ptr_mesh_->num_of_vertex_list() == 0)
	{
		emit(QString("The Mesh is Empty !"));
		return;
	}
	QString filename = QFileDialog::
		getSaveFileName(this, tr("Write Mesh"),
			"..", tr("Meshes (*.obj)"));

	if (filename.isEmpty())
		return;

	QByteArray byfilename = filename.toLocal8Bit();
	ptr_mesh_->WriteToOBJFile(byfilename);

}
void RenderingWidget::Export()
{
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
															//qDebug() << "export AFF file start time" << str;
	if (myhatch == NULL || ptr_mesh_ == NULL || mycut == NULL)
	{
		return;
	}
	QString filename = QFileDialog::
		getSaveFileName(this, tr("export hatch"),
			"..", tr("aff (*.aff)"));
	if (filename.isEmpty())
		return;

	QByteArray byfilename = filename.toLocal8Bit();
	QFile file(byfilename);
	file.open(QIODevice::WriteOnly);
	file.resize(0);
	char *ptr;


	int layers = mycut->GetNumPieces() - 1;
	float power = myhatch->getLaserPower();
	float xmin = ptr_mesh_->getBoundingBox().at(1).x();
	float xmax = ptr_mesh_->getBoundingBox().at(0).x();
	float ymin = ptr_mesh_->getBoundingBox().at(1).y();
	float ymax = ptr_mesh_->getBoundingBox().at(0).y();
	int zmin = 300;
	int zmax = (myhatch->GetNumPieces() - 1) * 300;
	float speed = myhatch->getLaserSpeed();
	thickness_ = mycut->getThickness();
	int  THICKNESS = thickness_ * 10000;
	QDataStream outBinary(&file);
	outBinary.setVersion(QDataStream::Qt_4_1);
	QByteArray* ts = new QByteArray[17];
	ts[0] = "<Header type=\"Autofab Buildfile\" version=\"1.0\">\n";
	ts[1] = "<Origin>HT 149</Origin>\n";
	ts[2] = "<Layers>";
	ts[2].append(QString("%1").arg(layers));
	ts[2].append("</Layers>\n");
	ts[3] = "<Bounds xmin=\"";
	ts[3].append(QString("%1").arg(xmin));
	ts[3].append("\" ymin=\"");
	ts[3].append(QString("%1").arg(ymin));
	ts[3].append("\" xmax=\"");
	ts[3].append(QString("%1").arg(xmax));
	ts[3].append("\" ymax=\"");
	ts[3].append(QString("%1").arg(ymax));
	ts[3].append("\"></Bounds>\n");
	ts[4] = "<Zunit>10000</Zunit>\n";
	ts[5] = "<Zmin>";
	ts[5].append(QString("%1").arg(zmin));
	ts[5].append("</Zmin>\n");
	ts[6] = "<Zmax>";
	ts[6].append(QString("%1").arg(zmax));
	ts[6].append("</Zmax>\n");
	ts[7] = "<LayerThickness>";
	ts[7].append(QString("%1").arg(THICKNESS));
	ts[7].append("</LayerThickness>\n");
	ts[8] = "<Machine>My Own Fabber</Machine>\n";
	ts[9] = "<Material>StainlessSteel_100</Material>\n";
	ts[10] = "<Part name=\"PAWN\" id=\"1\"></Part>\n";
	ts[11] = "<Part name=\"KING\" id=\"2\"></Part>\n";
	ts[12] = "<VectorAttribute name=\"LaserPower\" id=\"6\"></VectorAttribute>\n";
	ts[13] = "<VectorAttribute name=\"Speed\" id=\"7\"></VectorAttribute>\n";
	ts[14] = "<VectorAttribute name=\"Focus\" id=\"8\"></VectorAttribute>\n";
	ts[15] = "<VectorAttribute name=\"PartId\" id=\"11\"></VectorAttribute>\n";
	ts[16] = "</Header>\r\n";
	for (int i = 0; i < 17; i++)
	{
		outBinary.writeRawData(ts[i], sizeof(char) * (ts[i].size()));
	}
	std::vector<Vec3f *>* tc = myhatch->getHatch();
	std::vector < std::vector<CutLine>* >*tc2 = (mycut->GetPieces());
	std::vector < std::vector<Vec3f>>* tc3 = myhatch->getOffsetVertex(); 
	std::vector<Vec3f *>* tc_s_hatch_ = NULL;
	std::vector < std::vector<CutLine>* >* tc_s_pieces_=NULL;
	std::vector < std::vector<Vec3f>>* tc_s_offsetvertex_ = NULL;
	if (mycutsup!=NULL)
	{
		tc_s_pieces_ = (mycutsup->GetPieces());
	}
	if (myhatchsup!=NULL)
	{
		tc_s_hatch_ = myhatchsup->getHatch();
		tc_s_offsetvertex_ = myhatchsup->getOffsetVertex();
	}
	
	int Layer_Section = 1;
	int LayerZpos_Section = 12;
	int Polygon_Section = 2;
	int Hatch_Section = 4;
	int LaserPower_Section = 6;
	int LaserSpeed_Section = 7;
	int FocusShift_Section = 8;
	int PolygonCoordinates_Section = 3;
	int HatchCoordinates_Section = 5;
	int PartID_Section = 5;
	float laserpower = 0;
	float laserspeed = 0;
#define  POLYGONCOORDINATE//modify at 2017/3/7 to 
	for (int i = 1; i < myhatch->GetNumPieces(); i++)
	{
		int zposition = i*myhatch->getThickness() * 10000;
		int len_layer = 0;
		int len_Zpos = 4;
		int len_Polygon = 0;
		int len_Hatch = 0;
		int len_LaserPower = 4;
		int len_laserSpeed = 4;
		int len_Part = 2;
		int*len_PolygonCoor = NULL;
		int len_HatchCoor = 0;
		QByteArray newlayer = NULL;
		QByteArray zPosition = NULL;
		QByteArray hatch = NULL;
		QByteArray hatchCoor = NULL;
		QByteArray polygon = NULL;
		QByteArray* polygonCoor = NULL;
		QByteArray laserPower = NULL;
		QByteArray laserSpeed = NULL;
		QByteArray laserPowerPolygon = NULL;
		QByteArray laserSpeedPolygon = NULL;
		QByteArray partID = NULL;
		//write all hatch coordinate section
		len_HatchCoor = tc[i].size() * 2 * 8;//the length of hatch coordinate section 
		hatchCoor.append(reinterpret_cast<const char *>(&HatchCoordinates_Section), 2);

		hatchCoor.append(reinterpret_cast<const char *>(&len_HatchCoor), 4);
		for (auto iterhatch_ = tc[i].begin(); iterhatch_ != tc[i].end(); iterhatch_++)
		{
			hatchCoor.append(reinterpret_cast<const char *>(&(*iterhatch_)[0].x()), sizeof(float));
			hatchCoor.append(reinterpret_cast<const char *>(&(*iterhatch_)[0].y()), sizeof(float));
			hatchCoor.append(reinterpret_cast<const char*>(&(*iterhatch_)[1].x()), sizeof(float));
			hatchCoor.append(reinterpret_cast<const char*>(&(*iterhatch_)[1].y()), sizeof(float));
		}
		//write all hatch section
		laserPower.append(reinterpret_cast<const char *>(&LaserPower_Section), 2);
		laserPower.append(reinterpret_cast<const char *>(&len_LaserPower), 4);
		laserPower.append(reinterpret_cast<const char *>(&laser_power_hatch_), 4);
		laserSpeed.append(reinterpret_cast<const char *>(&LaserSpeed_Section), 2);
		laserSpeed.append(reinterpret_cast<const char *>(&len_laserSpeed), 4);
		laserSpeed.append(reinterpret_cast<const char *>(&laser_speed_hatch_), 4);
		//partID.append(reinterpret_cast<const char *>(&PartID_Section), 2);
		len_Hatch = len_HatchCoor + 6 + len_LaserPower + 6 + len_laserSpeed + 6;//the length of hatch section,include laser power length,laser speed length
		hatch.append(reinterpret_cast<const char *>(&Hatch_Section), 2);
		hatch.append(reinterpret_cast<const char *>(&len_Hatch), 4);
		hatch.append(laserPower);
		hatch.append(laserSpeed);
		//hatch.append(partID);
		hatch.append(hatchCoor);

		// write all polygon Coordinates
		len_PolygonCoor = new int[tc3[i].size()];
		int tempLenPolyCoor = 0;
		int tempAllLenPoly = 0;
		for (int j1 = 0; j1 < tc3[i].size(); j1++)
		{
			len_PolygonCoor[j1] = (tc3[i])[j1].size() * 8;
#ifdef POLYGONCOORDINATE
			tempLenPolyCoor += len_PolygonCoor[j1] + 6;
#endif		
		}
		polygonCoor = new QByteArray[tc3[i].size()];
		for (int j = 0; j < tc3[i].size(); j++)
		{
			polygonCoor[j].append(reinterpret_cast<const char *>(&PolygonCoordinates_Section), 2);//here only one polygon coordinate section
			polygonCoor[j].append(reinterpret_cast<const char *>(&len_PolygonCoor[j]), 4);
			for (int m = 0; m < (tc3[i])[j].size(); m++)
			{
				polygonCoor[j].append(reinterpret_cast<const char *>(&((tc3[i])[j]).at(m).x()), 4);
				polygonCoor[j].append(reinterpret_cast<const char *>(&((tc3[i])[j]).at(m).y()), 4);
			}
		}
		//write laser power and speed to polygon
		laserPowerPolygon.append(reinterpret_cast<const char *>(&LaserPower_Section), 2);
		laserPowerPolygon.append(reinterpret_cast<const char *>(&len_LaserPower), 4);
		laserPowerPolygon.append(reinterpret_cast<const char *>(&laser_power_polygon_), 4);
		laserSpeedPolygon.append(reinterpret_cast<const char *>(&LaserSpeed_Section), 2);
		laserSpeedPolygon.append(reinterpret_cast<const char *>(&len_laserSpeed), 4);
		laserSpeedPolygon.append(reinterpret_cast<const char *>(&laser_speed_polygon_), 4);
#ifdef POLYGONCOORDINATE
		len_Polygon = len_LaserPower + 6 + len_laserSpeed + 6 + tempLenPolyCoor;
		polygon.append(reinterpret_cast<const char *>(&Polygon_Section), 2);
		polygon.append(reinterpret_cast<const char *>(&len_Polygon), 4);
		polygon.append(laserPowerPolygon);
		polygon.append(laserSpeedPolygon);
		for (size_t j2 = 0; j2 < tc3[i].size(); j2++)
		{
			polygon.append(polygonCoor[j2]);
		}
#else

		for (size_t j2 = 0; j2 < tc3[i].size(); j2++)
		{
			len_Polygon = len_LaserPower + 6 + len_laserSpeed + 6 + len_PolygonCoor[j2] + 6;
			tempAllLenPoly += len_Polygon + 6;
			polygon.append(reinterpret_cast<const char *>(&Polygon_Section), 2);
			polygon.append(reinterpret_cast<const char *>(&len_Polygon), 4);
			polygon.append(laserPowerPolygon);
			polygon.append(laserSpeedPolygon);
			polygon.append(polygonCoor[j2]);
		}
#endif
		//here only one polygon coordinate section
		//write z position
		zPosition.append(reinterpret_cast<const char *>(&LayerZpos_Section), 2);
		zPosition.append(reinterpret_cast<const char *>(&len_Zpos), 4);
		zPosition.append(reinterpret_cast<const char *>(&zposition), 4);

		//////////////////////////////////////////////////////////////////////////
		// write support hatch 
		QByteArray hatch_coor_s;
		QByteArray hatch_s_;
		QByteArray* polygon_coor_s;
		QByteArray polygon_s_;

		int len_HatchCoor_s_ = 0;
		int len_Hatch_s_ = 0;
		int* len_PolygonCoor_s_ = NULL;
		int tempLenPolyCoor_s_ = 0;
		int tempAllLenPoly_s_ = 0;
		int	len_Polygon_s_ = 0;
		if (myhatchsup!=NULL&&i < myhatchsup->GetNumPieces())
		{
			len_HatchCoor_s_ = tc_s_hatch_[i].size() * 2 * 8;//the length of hatch coordinate section 
			hatch_coor_s.append(reinterpret_cast<const char *>(&HatchCoordinates_Section), 2);
			hatch_coor_s.append(reinterpret_cast<const char *>(&len_HatchCoor_s_), 4);
			for (auto iter_hatch_s_ = tc_s_hatch_[i].begin(); iter_hatch_s_ != tc_s_hatch_[i].end(); iter_hatch_s_++)
			{
				hatch_coor_s.append(reinterpret_cast<const char *>(&(*iter_hatch_s_)[0].x()), sizeof(float));
				hatch_coor_s.append(reinterpret_cast<const char *>(&(*iter_hatch_s_)[0].y()), sizeof(float));
				hatch_coor_s.append(reinterpret_cast<const char*>(&(*iter_hatch_s_)[1].x()), sizeof(float));
				hatch_coor_s.append(reinterpret_cast<const char*>(&(*iter_hatch_s_)[1].y()), sizeof(float));
			}
			len_Hatch_s_ = len_HatchCoor_s_ + 6 + len_LaserPower + 6 + len_laserSpeed + 6;//the length of hatch section,include laser power length,laser speed length

			hatch_s_.append(reinterpret_cast<const char *>(&Hatch_Section), 2);
			hatch_s_.append(reinterpret_cast<const char *>(&len_Hatch_s_), 4);
			hatch_s_.append(laserPower);
			hatch_s_.append(laserSpeed);
			hatch_s_.append(hatch_coor_s);
			//write support polygon
			len_PolygonCoor_s_ = new int[tc_s_offsetvertex_[i].size()];
			tempLenPolyCoor_s_ = 0;
			tempAllLenPoly_s_ = 0;
			for (int j1 = 0; j1 < tc_s_offsetvertex_[i].size(); j1++)
			{
				len_PolygonCoor_s_[j1] = (tc_s_offsetvertex_[i])[j1].size() * 8;
				tempLenPolyCoor_s_ += len_PolygonCoor_s_[j1] + 6;

			}
			polygon_coor_s = new QByteArray[tc_s_offsetvertex_[i].size()];
			for (int j = 0; j < tc_s_offsetvertex_[i].size(); j++)
			{
				polygon_coor_s[j].append(reinterpret_cast<const char *>(&PolygonCoordinates_Section), 2);//here only one polygon coordinate section
				polygon_coor_s[j].append(reinterpret_cast<const char *>(&len_PolygonCoor_s_[j]), 4);
				for (int m = 0; m < (tc_s_offsetvertex_[i])[j].size(); m++)
				{
					polygon_coor_s[j].append(reinterpret_cast<const char *>(&((tc_s_offsetvertex_[i])[j]).at(m).x()), 4);
					polygon_coor_s[j].append(reinterpret_cast<const char *>(&((tc_s_offsetvertex_[i])[j]).at(m).y()), 4);
				}
			}
			//write laser power and speed to polygon
			len_Polygon_s_ = len_LaserPower + 6 + len_laserSpeed + 6 + tempLenPolyCoor_s_;
			polygon_s_.append(reinterpret_cast<const char *>(&Polygon_Section), 2);
			polygon_s_.append(reinterpret_cast<const char *>(&len_Polygon_s_), 4);
			polygon_s_.append(laserPowerPolygon);
			polygon_s_.append(laserSpeedPolygon);
			for (size_t j2 = 0; j2 < tc_s_offsetvertex_[i].size(); j2++)
			{
				polygon_s_.append(polygon_coor_s[j2]);
			}

		}
		//////////////////////////////////////////////////////////////////////////
		if (myhatchsup!=NULL)
		{
			len_layer = len_Zpos + 6 + len_Hatch + 6 + len_Polygon + 6 + len_Hatch_s_ + 6 + len_Polygon_s_ + 6;
		}
		else
		{
			len_layer = len_Zpos + 6 + len_Hatch + 6 + len_Polygon + 6;

		}
		
		newlayer.append(reinterpret_cast<const char *>(&Layer_Section), 2);
		newlayer.append(reinterpret_cast<const char *>(&len_layer), 4);
		newlayer.append(zPosition);
		newlayer.append(hatch);
		newlayer.append(polygon);
		if (myhatchsup!=NULL)
		{
			newlayer.append(hatch_s_);
			newlayer.append(polygon_s_);
		}
		outBinary.writeRawData(newlayer, sizeof(char) * (newlayer.size()));
		delete[]polygonCoor;
		delete[]len_PolygonCoor;
	}
	file.flush();
	file.close();
	time = QDateTime::currentDateTime();//获取系统现在的时间
	str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
													//qDebug() << "export AFF file end time :" << str;
	SafeDeletes(ts);
}
void RenderingWidget::LoadTexture()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Load Texture"),
		"..", tr("Images(*.bmp *.jpg *.png *.jpeg)"));
	if (filename.isEmpty())
	{
		//emit(operatorInfo(QString("Load Texture Failed!")));
		return;
	}


	glGenTextures(1, &texture_[0]);
	QImage tex1, buf;
	if (!buf.load(filename))
	{
		//        QMessageBox::warning(this, tr("Load Fialed!"), tr("Cannot Load Image %1").arg(filenames.at(0)));
		//emit(operatorInfo(QString("Load Texture Failed!")));
		return;

		//QImage dummy(128, 128, QImage::Format_ARGB32);
		//dummy.fill(Qt::green);
		//buf = dummy;
		//
	}

	glBindTexture(GL_TEXTURE_2D, texture_[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, tex1.width(), tex1.height(),
		GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());

	is_load_texture_ = true;
	//emit(operatorInfo(QString("Load Texture from ") + filename + QString(" Done")));
}

void RenderingWidget::SetSliceCheckId(int id)
{
	if (mycut != NULL)
	{
		slice_check_id_ = round((mycut->num_pieces_*id) / 10000);
		if (slice_check_id_ >= mycut->num_pieces_)
		{
			slice_check_id_ = mycut->num_pieces_ - 1;
		}
		update();
	}
	//qDebug() <<id<< mycut->num_pieces_<<slice_check_id_;
}

void RenderingWidget::CheckDrawPoint()
{
	is_draw_point_ = !is_draw_point_;
	//updateGL();
	update();

}
void RenderingWidget::CheckDrawEdge()
{
	is_draw_edge_ = !is_draw_edge_;
	//updateGL();
	if (is_draw_edge_ == false)
	{
		qDebug() << "修复完成";
		qDebug() << "孔洞个数为：" << 0;
		qDebug() << "法向面片错误" << sss;
	}
	update();

}
void RenderingWidget::CheckDrawFace()
{
	is_draw_face_ = !is_draw_face_;

	//updateGL();
	update();

}
void RenderingWidget::CheckLight()
{
	has_lighting_ = !has_lighting_;
	//updateGL();
	update();

}
void RenderingWidget::CheckGrid()
{
	is_draw_grid_ = !is_draw_grid_;
	//qDebug() << is_draw_grid_;
	//updateGL();
	update();

}
void RenderingWidget::CheckDrawTexture()
{
	is_draw_texture_ = !is_draw_texture_;
	if (is_draw_texture_)
		glEnable(GL_TEXTURE_2D);
	else
		glDisable(GL_TEXTURE_2D);

	//updateGL();
	update();

}
void RenderingWidget::CheckDrawAxes()
{
	is_draw_axes_ = !is_draw_axes_;
	//updateGL();
	update();

}
void RenderingWidget::CheckDrawCutPieces()
{

	is_draw_cutpieces_ = true;
	//updateGL();
	update();


}
void RenderingWidget::Checkmoduletranslate()
{
	is_move_module_ = !is_move_module_;
	//updateGL();
	update();

}
void RenderingWidget::CheckSetFace()
{
	is_select_face = !is_select_face;
	//updateGL();
	update();


}
void RenderingWidget::CheckRegion(bool bv)
{
	is_draw_region_ = bv;
	//updateGL();
	update();
}
void RenderingWidget::CheckSupport(bool bv)
{
	is_draw_support_ = bv;
	qDebug() << is_draw_support_;
	//updateGL();
	update();
}
void RenderingWidget::CheckRotateModel(bool bv)
{
	is_move_module_ = bv;
}


std::vector<int> qwewqe1(500000);
std::vector<int> qwewqe(500000);

void RenderingWidget::DrawAxes(bool bv)
{
	if (!bv)
		return;
	//x axis
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.7, 0.0, 0.0);
	glEnd();
	glPushMatrix();
	glTranslatef(0.7, 0, 0);
	glRotatef(90, 0.0, 1.0, 0.0);
	//glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//y axis
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.7, 0.0);
	glEnd();

	glPushMatrix();
	glTranslatef(0.0, 0.7, 0);
	glRotatef(90, -1.0, 0.0, 0.0);
	//glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//z axis
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.0, 0.7);
	glEnd();
	glPushMatrix();
	glTranslatef(0.0, 0, 0.7);
	//glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	glColor3f(1.0, 1.0, 1.0);
}
void RenderingWidget::DrawPoints(bool bv)
{
	if (!bv || ptr_mesh_ == NULL)
		return;
	if (ptr_mesh_->num_of_vertex_list() == 0)
	{
		return;
	}

	const std::vector<HE_vert*>& verts = *(ptr_mesh_->get_vertex_list());
	//glColor3f(0, 0, 0);
	//glPointSize(1);
	glBegin(GL_POINTS);
	for (size_t i = 0; i != ptr_mesh_->num_of_vertex_list(); ++i)
	{
		glNormal3fv(verts[i]->normal().data());
		glVertex3fv((verts[i]->position()*scaleV).data());
	}
	glEnd();
	if (ptr_support_->GetMeshSupport()->num_of_face_list() == 0)
	{
		return;
	}
	const std::vector<HE_vert*>& verts1 = *(ptr_support_->GetMeshSupport()->get_vertex_list());
	//glColor3f(0, 0, 0);
	//glPointSize(1);
	glBegin(GL_POINTS);
	for (size_t i = 0; i != ptr_support_->GetMeshSupport()->num_of_vertex_list(); ++i)
	{
		glNormal3fv(verts1[i]->normal().data());
		glVertex3fv((verts1[i]->position()*scaleV).data());
	}
	glEnd();
}
void RenderingWidget::DrawEdge(bool bv)
{
	if (!bv || ptr_mesh_ == NULL)
		return;

	if (ptr_mesh_->num_of_face_list() == 0)
	{
		return;
	}

	const std::vector<HE_edge *>& edges = *(ptr_mesh_->get_edges_list());
	const std::vector<HE_edge *>& bedges = *(ptr_mesh_->get_bedges_list());

	for (size_t i = 0; i != edges.size(); ++i)
	{
		glBegin(GL_LINES);
		glColor4ub(255, 255, 255, 255);

		glNormal3fv(edges[i]->start_->normal().data());
		glVertex3fv((edges[i]->start_->position()*scaleV).data());
		glNormal3fv(edges[i]->pvert_->normal().data());
		glVertex3fv((edges[i]->pvert_->position()*scaleV).data());
		glEnd();
	}

	for (size_t i = 0; i != bedges.size(); ++i)
	{
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glNormal3fv(bedges[i]->start_->normal().data());
		glVertex3fv((bedges[i]->start_->position()*scaleV).data());
		glNormal3fv(bedges[i]->pvert_->normal().data());
		glVertex3fv((bedges[i]->pvert_->position()*scaleV).data());
		glEnd();
	}
	auto bl = ptr_mesh_->GetBLoop();
	for (size_t i = 0; i != bl.size(); i++)
	{
		glBegin(GL_LINE_LOOP);
		glColor3f(1.0, 0.0, 0.0);
		for (int j = 0; j < bl[i].size(); j++)
		{
			glNormal3fv(bl[i][j]->start_->normal().data());
			glVertex3fv((bl[i][j]->start_->position()*scaleV).data());
		}
		glEnd();
	}
	if (ptr_support_->GetMeshSupport()->num_of_face_list() == 0)
	{
		return;
	}
	const std::vector<HE_edge *>& edges1 = *(ptr_support_->GetMeshSupport()->get_edges_list());
	const std::vector<HE_edge *>& bedges1 = *(ptr_support_->GetMeshSupport()->get_bedges_list());
	for (size_t i = 0; i != edges1.size(); ++i)
	{
		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 1.0);
		glNormal3fv(edges1[i]->start_->normal().data());
		glVertex3fv((edges1[i]->start_->position()*scaleV).data());
		glNormal3fv(edges1[i]->pvert_->normal().data());
		glVertex3fv((edges1[i]->pvert_->position()*scaleV).data());
		glEnd();
	}
	return;
	for (size_t i = 0; i != bedges1.size(); ++i)
	{
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glNormal3fv(bedges1[i]->start_->normal().data());
		glVertex3fv((bedges1[i]->start_->position()*scaleV).data());
		glNormal3fv(bedges1[i]->pvert_->normal().data());
		glVertex3fv((bedges1[i]->pvert_->position()*scaleV).data());
		glEnd();
	}
}
void RenderingWidget::DrawFace(bool bv)
{
	if (!bv || ptr_mesh_ == NULL)
		return;

	if (ptr_mesh_->num_of_face_list() == 0)
	{
		return;
	}

	const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());
	const std::vector<HE_edge *>& edges = *(ptr_mesh_->get_edges_list());
	glColor4ub(0, 0, 0, 255);
	glBegin(GL_LINES);
	for (size_t i=0;i<edges.size();i++)
	{
		if (edges[i]->boundary_flag()==BOUNDARY)
		{
			glVertex3fv(edges[i]->start_->position());
			glVertex3fv(edges[i]->pvert_->position());
		}
	}
	glEnd();

	for (size_t i = 0; i < faceNeedSupport.size(); ++i)
	{
		if (faceNeedSupport[i]&&is_draw_region_)
		{
			glColor4ub(170, 170, 0, 255);
		}
		else
		{
			glColor4ub(0, 170, 0, 255);
		}
		glBegin(GL_TRIANGLES);
		HE_edge *pedge(faces[i]->pedge_);
		glNormal3fv(faces[i]->normal());
		do 
		{
			glVertex3fv(pedge->pvert_->position());
			pedge = pedge->pnext_;

		} while (pedge != faces[i]->pedge_);
		glEnd();
	}
	
}
void RenderingWidget::DrawSupport(bool bv)
{

	if (ptr_support_->GetMeshSupport()->num_of_face_list() == 0 || bv == false)
	{
		return;
	}

	const std::vector<HE_face *>& faces1 = *(ptr_support_->GetMeshSupport()->get_faces_list());

	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < faces1.size(); ++i)
	{
		glColor4f(1.0, 1.0, 1.0, 1.0);
		HE_edge *pedge(faces1.at(i)->pedge_);
		do
		{
			if (pedge == NULL)
			{
				break;
			}
			if (pedge == NULL || pedge->pface_->id() != faces1.at(i)->id())
			{
				faces1.at(i)->pedge_ = NULL;
				break;
			}
			glNormal3fv(pedge->pvert_->normal().data());
			glVertex3fv((pedge->pvert_->position()*scaleV).data());
			pedge = pedge->pnext_;
		} while (pedge != faces1.at(i)->pedge_);
	}
	glEnd();
}
void RenderingWidget::DrawSupFace(bool bv)
{
	if (!bv)
		return;
	const std::vector<HE_face *>* faces = (ptr_mesh_->get_faces_list());
	const std::vector<HE_edge *>* edges = (ptr_mesh_->get_edges_list());
	for (size_t i = 0; i < BoundaryEdge.size(); i++)
	{
		if (BoundaryEdge[i] > 0)
		{
			glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 1.0);
			glNormal3fv(edges->at(i)->start_->normal().data());
			glVertex3fv((edges->at(i)->start_->position()*scaleV).data());
			glNormal3fv(edges->at(i)->pvert_->normal().data());
			glVertex3fv((edges->at(i)->pvert_->position()*scaleV).data());
			glEnd();

		}
	}
	glColor4ub(170, 170, 0, 255);
	for (size_t i = 0; i < faceNeedSupport.size(); ++i)
	{
		if (faceNeedSupport[i])
		{
			HE_edge *pedge(faces->at(i)->pedge_);
			glBegin(GL_TRIANGLES);
			do
			{
				if (pedge == NULL)
				{
					break;
				}
				if (pedge == NULL || pedge->pface_->id() != faces->at(i)->id())
				{
					faces->at(i)->pedge_ = NULL;
					qDebug() << faces->at(i)->id() << "facet display wrong";
					break;
				}
				glNormal3fv(pedge->pvert_->normal().data());
				glVertex3fv((pedge->pvert_->position()*scaleV).data());
				pedge = pedge->pnext_;
			} while (pedge != faces->at(i)->pedge_);
			glEnd();
		}
	}


}
void RenderingWidget::DrawOctree(bool bv)
{
	ptr_support_->GetMeshInOctree();
}

void RenderingWidget::DrawSegments(bool bv)
{
	std::multiset<HE_edge*, Support::comHE>& segments = ptr_support_->GetSegments();
	for (auto iterS = segments.begin(); iterS != segments.end(); iterS++)
	{
		glBegin(GL_LINES);
		if ((*iterS)->is_selected_)
		{
			glColor3f(0.0, 1.0, 1.0);
		}
		else
		{
			glColor3f(1.0, 0.0, 0.0);
		}

		glVertex2f((*iterS)->start_->position().x(), (*iterS)->start_->position().y());
		glVertex2f((*iterS)->pvert_->position().x(), (*iterS)->pvert_->position().y());
		glEnd();
	}

	std::multiset <HE_vert*, comVertex>& points = ptr_support_->GetSweepPoints();
	int i = 0;
	for (auto iterP = points.begin(); iterP != points.end(); iterP++)
	{
		i++;

		if (i != fildID)
		{
			continue;
		}
		glBegin(GL_TRIANGLES);
		glColor3f(0.0, 1.0, 0.0);
		glVertex2f((*iterP)->position().x(), (*iterP)->position().y());
		glVertex2f((*iterP)->position().x() + 0.1, (*iterP)->position().y() + 0.1);
		glVertex2f((*iterP)->position().x() + 0.1, (*iterP)->position().y());
		glEnd();
	}
}


void RenderingWidget::DrawInnerLoop(bool bv)
{
	if (ptr_support_ == NULL || bv == false)
	{
		return;
	}
	std::vector < std::vector<Vec3f>> innerPoints = ptr_support_->GetInnerLoop();
	for (int i = 0; i < innerPoints.size(); i++)
	{

		glBegin(GL_LINE_LOOP);


		for (int j = 0; j < innerPoints.at(i).size(); j++)
		{
			if (j == 0)
			{
				glColor3f(1.0, 0.0, 0.0);

			}
			else if (j == 1)
			{
				glColor3f(0.0, 1.0, 0.0);

			}
			else
			{
				glColor3f(1.0, 1.0, 1.0);
			}
			glVertex3fv((innerPoints.at(i).at(j)*scaleV).data());
		}
		glEnd();

	}

}
void RenderingWidget::DrawOuterLoop(bool bv)
{
	if (ptr_support_ == NULL || bv == false)
	{
		return;
	}
	std::vector < std::vector<Vec3f>> outerPoints = ptr_support_->GetOutLoop();
	for (int i = 0; i < outerPoints.size(); i++)
	{
		if (outerPoints.at(i).size() < 4)
		{
			continue;
		}
		glBegin(GL_LINE_LOOP);

		for (int j = 0; j < outerPoints.at(i).size(); j++)
		{
			if (j == 0)
			{
				glColor3f(1.0, 0.0, 0.0);

			}
			else if (j == 1)
			{
				glColor3f(0.0, 1.0, 0.0);

			}
			else
			{
				glColor3f(1.0, 1.0, 1.0);
			}
			glVertex3fv((outerPoints[i][j] * scaleV).data());
		}
		glEnd();
	}

}
void RenderingWidget::DrawSupFaceZero(bool bv)
{
	if (!bv || ptr_support_->GetRegionSup() == NULL || ptr_support_->GetRegionSup()->num_of_face_list() == 0)
		return;
	const std::vector<HE_face *>& faces = *(ptr_support_->GetRegionSup()->get_faces_list());

	std::vector<std::vector<HE_edge*>> bl = ptr_support_->GetRegionSup()->GetBLoop();

	//for (int i=fildID;i<bl.size()&&i<fildID+1;i++)
	for (int i = 0; i < bl.size(); i++)
	{
		if (i != fildID)
		{
			continue;
		}
		if (bl[i].size() < 4)
		{
			continue;
		}
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < bl[i].size(); j++)
		{
			if (j == 0)
			{
				glColor4f(1.0, 0.0, 0.0, 1.0);
			}
			else if (j == 1)
			{
				glColor4f(0.0, 1.0, 0.0, 1.0);
			}
			else
			{
				glColor4f(1.0, 1.0, 1.0, 1.0);
			}


			glVertex3f(bl[i][j]->pvert_->position().x(), bl[i][j]->pvert_->position().y(), 0.0);
			//glVertex3f(bl[i][j+1]->pvert_->position().x(), bl[i][j+1]->pvert_->position().y(), 0.0);

		}
		glEnd();
	}


	std::vector < std::vector<Vec3f>> il = ptr_support_->GetInnerLoop();
	//for (int i = fildID; i<bl.size() && i<fildID + 1; i++)
	for (int i = 0; i < il.size(); i++)
	{
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < il[i].size(); j++)
		{
			if (j == 0)
			{
				glColor4f(1.0, 0.0, 0.0, 1.0);
			}
			else if (j == 1)
			{
				glColor4f(0.0, 1.0, 0.0, 1.0);
			}
			else
			{
				glColor4f(1.0, 1.0, 1.0, 1.0);
			}




			glVertex3f(il[i][j].x(), il[i][j].y(), 0.0);
			//glVertex3f(il[i][j+1].x(), il[i][j+1].y(), 0.0);

		}
		glEnd();
	}
	return;
	for (size_t i = 0; i < faces.size(); ++i)
	{
		if (faces.at(i)->com_flag != 4)
		{
			continue;
		}
		HE_edge *pedge(faces.at(i)->pedge_);

		do
		{
			glBegin(GL_LINES);
			if (pedge->is_selected_)
			{

				glColor4f(0.0, 0.0, 0.0, 1.0);
				glVertex3f(pedge->start_->position().x(), pedge->start_->position().y(), 0.0);
				glVertex3f(pedge->pvert_->position().x(), pedge->pvert_->position().y(), 0.0);
			}
			else
			{
				glColor4f(1.0, 1.0, 1.0, 0.5);
				glVertex3f(pedge->start_->position().x(), pedge->start_->position().y(), 0.0);
				glVertex3f(pedge->pvert_->position().x(), pedge->pvert_->position().y(), 0.0);
			}

			glEnd();
			pedge = pedge->pnext_;
		} while (pedge != faces.at(i)->pedge_);

		if (faces.at(i)->selected() == SELECTED)
		{
			glColor4f(1.0, 0.0, 0.0, 1.0);
		}
		else
		{
			glColor4f(0.5, 0.5, 0.5, 0.5);
			continue;
		}
		glBegin(GL_TRIANGLES);
		do
		{
			glVertex3f(pedge->pvert_->position().x(), pedge->pvert_->position().y(), 0.0);
			pedge = pedge->pnext_;
		} while (pedge != faces.at(i)->pedge_);
		glEnd();

	}
	const std::vector<HE_edge *>& edges = *(ptr_support_->GetRegionSup()->get_edges_list());
	for (size_t i = 0; i != edges.size(); ++i)
	{

	}

}

void RenderingWidget::DrawTexture(bool bv)
{
	if (!bv)
		return;
	if (ptr_mesh_->num_of_face_list() == 0 || !is_load_texture_)
		return;

	//默认使用球面纹理映射，效果不好
	ptr_mesh_->SphereTex();

	const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());

	glBindTexture(GL_TEXTURE_2D, texture_[0]);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i != faces.size(); ++i)
	{
		HE_edge *pedge(faces.at(i)->pedge_);
		do
		{
			/* 请在此处绘制纹理，添加纹理坐标即可 */
			glTexCoord2fv(pedge->pvert_->texCoord_.data());
			glNormal3fv(pedge->pvert_->normal().data());
			glVertex3fv((pedge->pvert_->position()*scaleV).data());

			pedge = pedge->pnext_;

		} while (pedge != faces.at(i)->pedge_);
	}

	glEnd();
}
void RenderingWidget::DrawGrid(bool bv)
{
	if (!bv)
		return;
	//x axis
	//glDisable(GL_LIGHTING);

	glColor3f(0.9, 0.9, 0.9);
	glBegin(GL_LINES);


	Vec3f box(ptr_mesh_->getBoundingBox().at(0)*scaleV - ptr_mesh_->getBoundingBox().at(1)*scaleV);
	for (int i = 1; i < 16; i++)
	{
		glVertex2f(-box[0], -box[1] + i*box[1] / 8);
		glVertex2f(box[0], -box[1] + i*box[1] / 8);

		glVertex2f(-box[0] + i*box[0] / 8, -box[1]);
		glVertex2f(-box[0] + i*box[0] / 8, box[1]);
	}

	glEnd();

	//glEnable(GL_LIGHTING);
}
void RenderingWidget::DrawCutPieces(bool bv)
{
	if (!bv)
	{
		return;
	}
	if (ptr_mesh_->num_of_vertex_list() == 0 || mycut == NULL)
	{
		return;
	}
	std::vector < std::vector<CutLine>* >*tc = (mycut->GetPieces());
	std::vector < std::vector<Vec3f>>* clip = mycut->GetClip();
 	glColor3f(0.0, 1.0, 0.0);
 	//for (int i = 0; i<mycut->num_pieces_; i++)
 	for (int i=slice_check_id_;i<slice_check_id_+1;i++)
 	{
 		
 		std::vector < std::vector<Vec3f>>& slice = clip[i];
 		for (int j=0;j<slice.size();j++)
 		{
 			glBegin(GL_LINE_LOOP);
 			for (int k=0;k<slice[j].size();k++)
 			{
 				glVertex3fv(slice[j][k]);
 			}
 			glEnd();
 		}
 
 	
 	}
	return;
	if (tc==NULL)
	{
		return;
	}
	glColor3f(1.0, 0.0, 0.0);
	for (int i = slice_check_id_; i < slice_check_id_+1; i++)
	{
		glBegin(GL_LINES);
		for (size_t j = 0; j < tc[i].size(); j++)
		{
			for (int k = 0; k < (tc[i])[j]->size(); k++)
			{
				glVertex3fv((((tc[i])[j])->at(k).position_vert[0] * scaleV).data());
				glVertex3fv((((tc[i])[j])->at(k).position_vert[1] * scaleV).data());
			}

		}
		glEnd();
		
	}
}
void RenderingWidget::DrawCutPiecesSup(bool bv)
{
	if (!bv)
	{
		return;
	}
	if (ptr_support_->GetMeshSupport()->num_of_vertex_list() == 0 || mycutsup == NULL)
	{
		return;
	}
	std::vector < std::vector<CutLine>* >*tc = (mycutsup->GetPieces());
	glColor3f(1.0, 0.0, 0.0);
	//for (int i = 0; i<mycut->num_pieces_; i++)
	if (slice_check_id_ >= mycutsup->GetNumPieces())
	{
		return;
	}
	for (int i = slice_check_id_; i < slice_check_id_ + 1; i++)
	{
		glBegin(GL_LINES);
		for (size_t j = 0; j < tc[i].size(); j++)
		{
			for (int k = 0; k < (tc[i])[j]->size(); k++)
			{
				glVertex3fv((((tc[i])[j])->at(k).position_vert[0] * scaleV).data());
				glVertex3fv((((tc[i])[j])->at(k).position_vert[1] * scaleV).data());
			}

		}
		glEnd();
	}
}


void RenderingWidget::drawCub(bool bv)
{
	if (mycut == NULL)
		return;
	std::set<Cubes::Box, Cubes::sortBox> box=mycut->cubes_support_.GetBoxes();





	glColor4f(1.0, 0.0, 0.0, 1.0);
	glBegin(GL_QUADS);
	for (int i=slice_check_id_;i<slice_check_id_+1;i++)
	{
		for (auto iter=box.begin();iter!=box.end();iter++)
		{
			
			if (iter->getID()[2]==i)
			{


				std::vector<int> id = iter->getID();
				glVertex3f(id[0] * spot_size_, id[1] * spot_size_, i*thickness_);
				glVertex3f((id[0]+1) * spot_size_, id[1] * spot_size_, i*thickness_);
				glVertex3f((id[0]+1) * spot_size_, (id[1]+1) * spot_size_, i*thickness_);
				glVertex3f(id[0] * spot_size_, (id[1]+1) * spot_size_, i*thickness_);
			}
			
		}
	}
	glEnd();
}
void RenderingWidget::DrawHatch(bool bv)
{
	if (!bv)
	{
		return;
	}
	if (ptr_mesh_->num_of_vertex_list() == 0 || mycut == NULL || myhatch == NULL)
	{
		return;
	}
	std::vector<Vec3f*>* tc_hatch_ = myhatch->getHatch();
	std::vector < std::vector<Vec3f>>* tc_offset_ = myhatch->getOffsetVertex();
	if (slice_check_id_ > myhatch->GetNumPieces() - 1)
	{
		return;
	}
	if (is_show_all)
	{
		for (int i = 0; i < myhatch->GetNumPieces(); i++)
		{
			for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
			{
				glColor3f(0.0, 1.0, 0.0);
				glBegin(GL_LINES);
				glVertex3fv(((*iterline)[0] * scaleV));
				glVertex3fv(((*iterline)[1] * scaleV));
				glEnd();
			}

			for (int j = 0; j < tc_offset_[i].size(); j++)
			{
				glColor3f(1.0, 0.0, 0.0);
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
				{
					glVertex3fv((((tc_offset_[i])[j]).at(k)*scaleV).data());
				}
				glEnd();
			}
		}
	}
	else
	{
		for (int i = slice_check_id_; i < slice_check_id_ + 1; i++)
		{
			for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
			{
				glColor3f(0.0, 1.0, 0.0);
				glBegin(GL_LINES);
				glVertex3fv(((*iterline)[0] * scaleV));
				glVertex3fv(((*iterline)[1] * scaleV));
				glEnd();
			}

			for (int j = 0; j < tc_offset_[i].size(); j++)
			{
				glColor3f(1.0, 0.0, 0.0);
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
				{
					//qDebug() << ((tc_offset_[i])[j])->at(k).x() << ((tc_offset_[i])[j])->at(k).y() << ((tc_offset_[i])[j])->at(k).z();
					glVertex3fv((((tc_offset_[i])[j]).at(k)*scaleV).data());
				}
				glEnd();
			}
			//  		for (int j = 0; j < tc_offset_rotate_[i].size(); j++)
			//  		{
			//  			glColor3f(0.0, 0.0, 1.0);
			//  			glBegin(GL_LINE_LOOP);
			//  			for (int k = 0; k < ((tc_offset_rotate_[i])[j])->size(); k++)
			//  			{
			//  				//qDebug() << ((tc_offset_[i])[j])->at(k).x() << ((tc_offset_[i])[j])->at(k).y() << ((tc_offset_[i])[j])->at(k).z();
			//  				glVertex3fv(((tc_offset_rotate_[i])[j])->at(k).data());
			//  			}
			//  			glEnd();
			//  		}

		}
	}
	//	std::vector < std::vector<Vec3f>*>* tc_offset_rotate_ = myhatch->getOffsetVertexRotate();

}
void RenderingWidget::DrawHatchsup(bool bv)
{
	if (!bv)
	{
		return;
	}
	if (ptr_support_->GetMeshSupport()->num_of_vertex_list() == 0 || mycutsup == NULL || myhatchsup == NULL)
	{
		return;
	}
	std::vector<Vec3f*>* tc_hatch_ = myhatchsup->getHatch();
	std::vector < std::vector<Vec3f>>* tc_offset_ = myhatchsup->getOffsetVertex();
	//	std::vector < std::vector<Vec3f>*>* tc_offset_rotate_ = myhatch->getOffsetVertexRotate();
	if (slice_check_id_ > myhatchsup->GetNumPieces() - 1)
	{
		return;
	}
	if (is_show_all)
	{
		for (int i = 0; i < myhatchsup->GetNumPieces(); i++)
		{
			for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
			{
				glColor3f(0.0, 1.0, 0.0);
				glBegin(GL_LINES);
				glVertex3fv(((*iterline)[0] * scaleV));
				glVertex3fv(((*iterline)[1] * scaleV));
				glEnd();
			}

			for (int j = 0; j < tc_offset_[i].size(); j++)
			{
				glColor3f(0.0, 0.0, 1.0);
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
				{
					//qDebug() << ((tc_offset_[i])[j])->at(k).x() << ((tc_offset_[i])[j])->at(k).y() << ((tc_offset_[i])[j])->at(k).z();
					glVertex3fv((((tc_offset_[i])[j]).at(k)*scaleV).data());
				}
				glEnd();
			}
		}
	}
	else
	{
		for (int i = slice_check_id_; i < slice_check_id_ + 1; i++)
		{
			for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
			{
				glColor3f(0.0, 1.0, 0.0);
				glBegin(GL_LINES);
				glVertex3fv(((*iterline)[0] * scaleV));
				glVertex3fv(((*iterline)[1] * scaleV));
				glEnd();
			}

			for (int j = 0; j < tc_offset_[i].size(); j++)
			{
				glColor3f(0.0, 0.0, 1.0);
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
				{
					//qDebug() << ((tc_offset_[i])[j])->at(k).x() << ((tc_offset_[i])[j])->at(k).y() << ((tc_offset_[i])[j])->at(k).z();
					glVertex3fv((((tc_offset_[i])[j]).at(k)*scaleV).data());
				}
				glEnd();
			}
		}
	}

}

void RenderingWidget::ClearSlice()
{
	if (mycut != NULL)
	{
		mycut->clearcut();
		mycut = NULL;
	}
	if (mycutsup != NULL)
	{
		mycutsup->clearcut();
		mycutsup = NULL;
	}
}
void RenderingWidget::ClearHatch()
{
	if (myhatch != NULL)
	{
		myhatch->clearHatch();
		myhatch = NULL;
	}
	if (myhatchsup != NULL)
	{
		myhatchsup->clearHatch();
		myhatchsup = NULL;
	}
}
void RenderingWidget::ClearSupport()
{

	ptr_support_->GetMeshSupport()->ClearData();

}


// support operators
void RenderingWidget::AddPointSupport() {
	isAddPoint = !isAddPoint;
}
void RenderingWidget::AddLineSupport() {
	isAddLine = !isAddLine;

}
void RenderingWidget::DeleteSupport() {
	isDelete = !isDelete;
}

void RenderingWidget::AutoSupport()
{
	QDateTime start = QDateTime::currentDateTime(); //获取系统现在的时间
	if (ptr_mesh_->get_faces_list() == NULL)
		return;
	std::vector<HE_face*>* faceList = ptr_mesh_->get_faces_list();
	int faceNum = faceList->size();
	std::vector<HE_edge*>* edgeList = ptr_mesh_->get_edges_list();
	int edgeNum = ptr_mesh_->get_edges_list()->size();

	// for each face, set a need-support flag

	std::vector<bool> faceNeedSupportFlag(faceNum);
	std::vector<int> neighorNeedSupportNum(faceNum);

	const Vec3f _z(0, 0, -1);
	for (int i = 0; i < faceNum; i++)
	{
		std::vector<HE_vert *> tmp_face_verts;
		faceList->at(i)->face_verts(tmp_face_verts);

		Vec3f eAB = tmp_face_verts[1]->position_ - tmp_face_verts[0]->position_;
		Vec3f eAC = tmp_face_verts[2]->position_ - tmp_face_verts[0]->position_;
		Vec3f vNormal = eAB.cross(eAC);
		vNormal.normalize();

		float angle = vNormal.dot(_z);
		faceNeedSupportFlag[i] = angle > THRESHOLD ? true : false;

		// if true, try to set its neighbor to true
		if (faceNeedSupportFlag[i])
		{
			HE_edge* faceEdge = faceList->at(i)->pedge_;
			do
			{
				if (faceEdge->pface_ && faceEdge->ppair_->pface_)
				{
					int pairFace = faceEdge->ppair_->pface_->id_;
					if (faceNeedSupportFlag[pairFace])
					{
						faceEdge = faceEdge->pnext_;
						continue;
					}
					neighorNeedSupportNum[pairFace]++;
					if (neighorNeedSupportNum[pairFace] >= 2)
					{
						std::vector<HE_vert *> tmp_face_verts1;
						faceList->at(pairFace)->face_verts(tmp_face_verts1);

						Vec3f eAB1 = tmp_face_verts1[1]->position_ - tmp_face_verts1[0]->position_;
						Vec3f eAC1 = tmp_face_verts1[2]->position_ - tmp_face_verts1[0]->position_;
						Vec3f vNormal1 = eAB1.cross(eAC1);
						vNormal1.normalize();

						float angle1 = vNormal1.dot(_z);
						faceNeedSupportFlag[pairFace] = angle1 > THRESHOLD1 ? true : false;
					}

				}
				faceEdge = faceEdge->pnext_;
			} while (faceEdge != faceList->at(i)->pedge_);
		}
	}
	faceNeedSupport = faceNeedSupportFlag;
	// find need-support regions
	int regionNum = 0;
	std::vector<int> visited(faceNum); // also records the region a face belongs to (region starts from 1)
	std::queue<int> facesInQueue;
	for (int i = 0; i < faceNum; i++)
	{
		if (visited[i] == 0 && faceNeedSupportFlag[i])
		{
			facesInQueue.push(i);
			visited[i] = regionNum + 1;
			while (!facesInQueue.empty())
			{
				int topFaceInQueue = facesInQueue.front();
				facesInQueue.pop();

				HE_edge* faceEdge = faceList->at(topFaceInQueue)->pedge_;
				do
				{
					if (!faceEdge->isBoundary())
					{
						int pairFace = faceEdge->ppair_->pface_->id_;
						if (visited[pairFace] == 0 && faceNeedSupportFlag[pairFace])
						{
							facesInQueue.push(pairFace);
							visited[pairFace] = regionNum + 1;
						}
					}
					faceEdge = faceEdge->pnext_;
				} while (faceEdge != faceList->at(topFaceInQueue)->pedge_);
			}
			regionNum++;
		}
	}
	for (int iiii = 0; iiii < visited.size(); iiii++)
		qwewqe[iiii] = visited[iiii];

	// find region-boundary edge
	// and which region it is in (region's idx starts from 1)
	std::vector<int> regionBoundaryEdgeFlag(edgeNum);
	for (int i = 0; i < edgeNum; i++)
	{
		HE_face* edgeFace = edgeList->at(i)->pface_;
		if (edgeList->at(i)->isBoundary())
		{
			if (edgeFace == NULL)
				regionBoundaryEdgeFlag[i] = 0;
			else if (faceNeedSupportFlag[edgeFace->id_])
				regionBoundaryEdgeFlag[i] = visited[edgeFace->id_];
			else
				regionBoundaryEdgeFlag[i] = 0;
		}
		else if (faceNeedSupportFlag[edgeFace->id_])
		{
			int idT = edgeFace->id_;
			edgeFace = edgeList->at(i)->ppair_->pface_;
			if (faceNeedSupportFlag[edgeFace->id_])
				regionBoundaryEdgeFlag[i] = 0;
			else
				regionBoundaryEdgeFlag[i] = visited[idT];
		}
		qwewqe1[i] = regionBoundaryEdgeFlag[i];
	}
	BoundaryEdge = regionBoundaryEdgeFlag;
	// make boundary edges in order
	std::vector<int>* boundaryEdges = new std::vector<int>[regionNum]; // boundaryEdges that belong to the region
	std::vector<int>* boundaryEdgesSep = new std::vector<int>[regionNum]; // separate different boundaries (when a region has more than one boundaries)
	std::vector<bool>* boundaryOuterLoopFlag = new std::vector<bool>[regionNum]; // inner-boundary or outer-boundary
	std::vector<float>* boundaryEdgesLeftMostX = new std::vector<float>[regionNum]; // sth similiar to AABB, help assign values to boundaryOuterLoopFlag
	for (int i = 0; i < edgeNum; i++)
	{
		if (regionBoundaryEdgeFlag[i] != 0)
		{
			int curFlag = regionBoundaryEdgeFlag[i];

			std::vector<int> boundaryEdgesT;
			Vec3f aBoundingBoxAA(999999.0f);
			Vec3f aBoundingBoxBB(-999999.0f);

			HE_edge* anEdge = edgeList->at(i);
			int anEdgePoint = anEdge->pvert_->id_;

			float leftMostX = 999999.0f;
			while (true)
			{
				HE_edge* pairEdge = anEdge->pnext_;
				HE_edge* lastValidBoundary = NULL;
				while (pairEdge != anEdge->ppair_ && !pairEdge->isBoundary())
				{
					if (regionBoundaryEdgeFlag[pairEdge->id_] == curFlag)
						lastValidBoundary = pairEdge;
					pairEdge = pairEdge->ppair_->pnext_;
				}
				if (pairEdge->isBoundary() && regionBoundaryEdgeFlag[pairEdge->id_] == curFlag)
					lastValidBoundary = pairEdge;
				if (lastValidBoundary == NULL)
					break;
				if (lastValidBoundary->pvert_->id_ != anEdgePoint)
				{
					if (leftMostX > edgeList->at(lastValidBoundary->id_)->pvert_->position_.x())
						leftMostX = edgeList->at(lastValidBoundary->id_)->pvert_->position_.x();
					boundaryEdgesT.push_back(lastValidBoundary->id_);
					MergeBoundingBox(aBoundingBoxAA, aBoundingBoxBB, aBoundingBoxAA,
						aBoundingBoxBB, lastValidBoundary->pvert_->position_, lastValidBoundary->pvert_->position_);
					regionBoundaryEdgeFlag[lastValidBoundary->id_] = 0;
					anEdge = lastValidBoundary;
				}
				else
					break;
			}

			if (leftMostX > edgeList->at(i)->pvert_->position_.x())
				leftMostX = edgeList->at(i)->pvert_->position_.x();
			boundaryEdgesT.push_back(i);
			MergeBoundingBox(aBoundingBoxAA, aBoundingBoxBB, aBoundingBoxAA,
				aBoundingBoxBB, edgeList->at(i)->pvert_->position_, edgeList->at(i)->pvert_->position_);

			// if the bounding box is too small, ignore this boundary
			// OR
			// if edges num is too small, ignore this boundary
			Vec3f B_A = aBoundingBoxBB - aBoundingBoxAA;
			if (B_A[0] * B_A[1] < 2.0f)
			{
				foreach(auto j, boundaryEdgesT)
					qwewqe1[j] = -1;
				continue;
			}

			int regionId = regionBoundaryEdgeFlag[i] - 1;
			boundaryEdges[regionId].reserve(boundaryEdges[regionId].size() + boundaryEdgesT.size());
			boundaryEdges[regionId].insert(boundaryEdges[regionId].end(), boundaryEdgesT.begin(), boundaryEdgesT.end());
			boundaryEdgesSep[regionId].push_back(boundaryEdges[regionId].size());
			boundaryEdgesLeftMostX[regionId].push_back(leftMostX);
			boundaryOuterLoopFlag[regionId].push_back(false); // set all boundary edges to be the inner one

			regionBoundaryEdgeFlag[i] = 0;
		}
	}
	// change the largest boundary edge to be the outer boundary
	for (int i = 0; i < regionNum; i++)
	{
		int outerIdx = -1;
		float leftMostX = 999999.0f;
		for (int j = 0; j < boundaryEdgesLeftMostX[i].size(); j++)
		{
			if (leftMostX > boundaryEdgesLeftMostX[i].at(j))
			{
				outerIdx = j;
				leftMostX = boundaryEdgesLeftMostX[i].at(j);
			}
		}
		if (outerIdx != -1)
			boundaryOuterLoopFlag[i].at(outerIdx) = true;
	}

	// add boundary support line to mesh
std::vector<LineSegment*> segments;
	for (int i = 0; i < regionNum; i++)
	{
		
		segments.clear();
		for (int i1 = 0; i1 < boundaryEdgesSep[i].size(); i1++)
		{
			std::vector<Vec3f> toOffset; // record boundary afterOffset
			std::vector<Vec3f> toAdd;
			int startJ = 0, lengthJ = boundaryEdgesSep[i][i1];
			if (i1 != 0)
			{
				startJ = boundaryEdgesSep[i][i1 - 1];
				lengthJ -= boundaryEdgesSep[i][i1 - 1];
			}
			for (int j = 0; j < lengthJ; j++)
			{
				Vec3f point1 = edgeList->at(boundaryEdges[i][j + startJ])->pvert_->position_;
				Vec3f point2 = edgeList->at(boundaryEdges[i][(j + 1) % lengthJ + startJ])->pvert_->position_;
				//segments.push_back(new LineSegment(point1, point2));

				// sample points on the boundary and add support structure to the mesh
				float lengthTmp = (point2 - point1).length();
				Vec3f direc = (point2 - point1) / lengthTmp;
				float t = 0.0f;
				while (t < lengthTmp)
				{
					toAdd.push_back(point1 + t * direc);
					t += GAP;
				}
				// get boundary points to offset
				toOffset.push_back(point1);
				toOffset.push_back(Vec3f(toAdd.back()));
			}
			Vec3f point1 = edgeList->at(boundaryEdges[i][startJ])->pvert_->position_;
			toAdd.push_back(point1);
			//ptr_support_->AddLineSupport(toAdd, visited, i + 1);
#pragma omp critical(a)
			{	ptr_support_->AddLineSupport(toAdd, boundaryOuterLoopFlag[i][i1], visited, i + 1);}
		
			// get offset boundary
			toOffset = ptr_support_->getOffsetWallPoints(toOffset, visited, i + 1, boundaryOuterLoopFlag[i][i1], 0.01);
			int off_num = toOffset.size();
			for (int ofn = 0; ofn < off_num; ofn++)
			{
				segments.push_back(new LineSegment(toOffset[ofn], toOffset[(ofn + 1) % off_num]));
			}
		}

		// fill the inside area of the boundary, use space2dKDTree to accelerate
		if (segments.size() != 0)
		{
			Space2dKDTree* sKDT = new Space2dKDTree(segments); // segments are no longer in ordered after this

			for (float j = sKDT->rootNode->AA[1] + VERTICALGAP; j < sKDT->rootNode->BB[1] - VERTICALGAP; j += VERTICALGAP)
			{
				Vec3f sPoint(-999999.0f, j, 0);
				std::vector<Vec3f> hitPointList;
				sKDT->RayIntersection2d(sPoint, sKDT->rootNode, segments, hitPointList);
				sort(hitPointList.begin(), hitPointList.end(), SortByX);

				if (hitPointList.size() == 0)
					break;

				for (int k = 0; k < hitPointList.size() - 1; k += 2)
				{
					Vec3f point1 = hitPointList[k];
					Vec3f point2 = hitPointList[k + 1];

					//sample points on the boundary and add support structure to the mesh
					float lengthTmp = point2[0] - point1[0];
					Vec3f direc = (point2 - point1) / lengthTmp;

					float t = GAP;
					float rec = 0.0f;
					std::vector<Vec3f> toAdd;
					while (t < lengthTmp)
					{
						if (rec >= SEGLENGTH)
						{
							rec = 0.0f;
							t += GAP;
#pragma omp critical(b)
							{	ptr_support_->AddLineSupport(toAdd, false);}
						
							toAdd.clear();

							continue;
						}
						if (t + RESO >= lengthTmp)
							break;

						Vec3f point1T = point1 + t * direc;
						Vec3f point1TUp = ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, 1), true, visited, i + 1);
						Vec3f point1TDo = ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, -1), true, visited, i + 1);

						point1T = point1TUp[2] - point1T[2] > point1T[2] - point1TDo[2] ? point1TDo : point1TUp;
						if (point1T.x() > point1.x() + VERTICALGAP / 2 &&
							point1T.x() < point2.x() - VERTICALGAP / 2)
							toAdd.push_back(point1T);

						rec += RESO;
						t += RESO;
					}
					Vec3f point2T = point2;
					Vec3f point2TUp = ptr_support_->GetMeshInOctree()->InteractPoint(point2T, Vec3f(0, 0, 1), true, visited, i + 1);
					Vec3f point2TDo = ptr_support_->GetMeshInOctree()->InteractPoint(point2T, Vec3f(0, 0, -1), true, visited, i + 1);
					point2T = point2TUp[2] - point2T[2] > point2T[2] - point2TDo[2] ? point2TDo : point2TUp;
					if (point2T.x() > point1.x() + VERTICALGAP / 2 &&
						point2T.x() < point2.x() - VERTICALGAP / 2)
						toAdd.push_back(point2T);
#pragma omp critical(c)
					{ptr_support_->AddLineSupport(toAdd);}
					
				}
			}

			// free memory
			for (std::vector<LineSegment*>::iterator f = segments.begin(); f != segments.end(); f++)
				SafeDelete(*f);
			SafeDelete(sKDT);
		}
	}

	SafeDeletes(boundaryEdges);
	SafeDeletes(boundaryEdgesSep);
	SafeDeletes(boundaryOuterLoopFlag);
	SafeDeletes(boundaryEdgesLeftMostX);
	QDateTime end = QDateTime::currentDateTime();//获取系统现在的时间
	int seconds = start.secsTo(end);
	QString str = "Path generation use ";
	str.append(QString::number(seconds, 10));
	str.append(" seconds");
	QMessageBox::information(this, "Path Generation Successfully!", str);

	ptr_support_->updateSupportMesh();
	ptr_support_->BuildSupportOctree();
	update();
}

void RenderingWidget::setPointD(double diameter)
{
	ptr_support_->SetPoint(diameter, 0.0);
}
void RenderingWidget::setPointH(double diameter)
{

	ptr_support_->SetPoint(0.0, diameter);
}
void RenderingWidget::setLineD(double diameter)
{
	ptr_support_->SetLine(diameter, 0.0);
}
void RenderingWidget::setLineH(double diameter)
{
	ptr_support_->SetLine(0.0, diameter);
}

//hatch operators
void RenderingWidget::objectTransformation(float * matrix)
{
	ptr_mesh_->Transformation(matrix);
	ptr_support_->GetMeshSupport()->Transformation(matrix);
}
void RenderingWidget::Translation()
{
}
void RenderingWidget::SetDirection(int id)
{
	//is_select_face = false;
	ptr_mesh_->SetDirection(id);
}
void RenderingWidget::cutinPieces()
{
	if (ptr_mesh_->num_of_vertex_list() == 0)
	{
		return;
	}
	mycut = new SliceCut(ptr_mesh_);
	//isAddLine = true;
	is_draw_face_ = false;
	//is_draw_point_ = false;
	is_draw_edge_ = false;
	//is_draw_cutpieces_ = !is_draw_cutpieces_;
	mycut->clearcut();
	mycut->storeMeshIntoSlice();
	mycut->CutInPieces();
	mycut->clipPolygon();
	mycut->storeClipIntoCube();
	//Export();
	update();
}
void RenderingWidget::cutinPiecesSup()
{
	if (ptr_support_->GetMeshSupport()->num_of_vertex_list() == 0)
	{
		return;
	}
	if (mycutsup != NULL)
	{
		mycutsup->~SliceCut();
	}
	//ptr_support_->GetMeshSupport()->UpdateMesh();
	mycutsup = new SliceCut(ptr_support_->GetMeshSupport());
	//isAddLine = true;
	is_draw_face_ = false;
	//is_draw_point_ = false;
	is_draw_edge_ = false;
	//is_draw_cutpieces_ = !is_draw_cutpieces_;
	mycutsup->clearcut();
	mycutsup->storeMeshIntoSlice();
	mycutsup->CutInPieces();
	//Export();
	update();
}

void RenderingWidget::SelectFace(int x, int y)
{
	OpenGLProjector myProjector = OpenGLProjector();

	Vec3f u(x, y, 0.0);
	qDebug() << "u:" << u.x() << u.y() << u.z();
	const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());

	double mindis = 1e6; int selectedFacet = -1;
	for (size_t i = 0; i < faces.size(); i++)
	{
		//qDebug() << "*****************" ;
		Vec3f v = myProjector.Project(faces[i]->center());

		//qDebug() << myProjector.GetDepthValue((int)v.x(), (int)v.y());
		//qDebug() << myProjector.GetDepthValue((int)v.x(), (int)v.y()) - v.z();
		if (myProjector.GetDepthValue((int)v.x(), (int)v.y()) - v.z() <= -1e-2)
		{
			continue;
		}
		//qDebug() << "v:"<<v.x() << v.y() << v.z();
		v.z() = 0;
		double dist = (u - v).length();
		if (dist < mindis)
		{
			mindis = dist;
			selectedFacet = (int)i;
		}
	}
	if (selectedFacet != -1)
	{
		current_face_ = selectedFacet;

		qDebug() << current_face_;
	}
}
void RenderingWidget::renderdoHatch()
{
	
	QDateTime start = QDateTime::currentDateTime();//获取系统现在的时间

	if (ptr_mesh_->num_of_vertex_list() == 0)
	{
		return;
	}
	delete mycut;
	delete mycutsup;
	delete  myhatch;
	delete myhatchsup;
	mycut = mycutsup = NULL;
	myhatch = myhatchsup = NULL;

	if (mycut == NULL)
	{
		cutinPieces();
	}

	if (mycutsup == NULL)
	{
		cutinPiecesSup();
	}

	switch (hatch_type_)
	{
	case NONE:
		break;
	case CHESSBOARD:
		if (mycutsup != NULL)
		{
			myhatchsup = new HatchChessboard(mycutsup);
		}

		myhatch = new HatchChessboard(mycut);
		break;
	case OFFSETFILLING:
		if (mycutsup != NULL)
		{
			myhatchsup = new HatchOffsetfilling(mycutsup);
		}
		myhatch = new HatchOffsetfilling(mycut);

		break;
	case STRIP:
		break;
	case MEANDER:
		myhatch = new HatchMeander(mycut);
		if (mycutsup!= NULL)
		{
			myhatchsup = new HatchMeander(mycutsup);
		}
		break;
	default:
		break;
	}
	if (myhatch != NULL)
	{
		myhatch->doHatch();
	}
	if (myhatchsup != NULL)
	{
		myhatchsup->doHatch();
	}
	has_lighting_ = false;
	is_draw_face_ = false;
	is_draw_support_ = false;
	update();
	QDateTime end = QDateTime::currentDateTime();//获取系统现在的时间
	int seconds=start.secsTo(end);
	QString str = "Path generation use ";
	str.append(QString::number(seconds, 10));
	str.append(" seconds");
	QMessageBox::information(this, "Path Generation Successfully!", str);
}

//set varible
void RenderingWidget::setfieldWidth(double width)
{
	field_width_ = width;
}
void RenderingWidget::setfieldHeight(double height)
{
	field_height_ = height;
}
void RenderingWidget::setlineOverlap(int lineoverlap)
{
	line_width_ = lineoverlap / 1000;
}
void RenderingWidget::setfieldOverlap(double fieldoverlap)
{
	field_overlap_ = fieldoverlap;
}
void RenderingWidget::setHatchDis(double value)
{
	line_width_ = value;
}
void RenderingWidget::setThickness(double thick)
{
	thickness_ = thick;
}
void RenderingWidget::setAngle(int angle)
{
	increment_angle_ = angle;
}
void RenderingWidget::FindRegion()
{
	is_draw_region_ = !is_draw_region_;
	if (!is_draw_region_)
	{
		parent->ui.pushButton_3->setChecked(is_draw_region_);
		parent->ui.pushButton->setChecked(is_draw_region_);
		update();
		return;
	}
	parent->ui.pushButton_3->setChecked(is_draw_region_);
	parent->ui.pushButton->setChecked(is_draw_region_);

	std::vector<HE_face*>* faceList = ptr_mesh_->get_faces_list();
	int faceNum = faceList->size();
	std::vector<HE_edge*>* edgeList = ptr_mesh_->get_edges_list();
	int edgeNum = ptr_mesh_->get_edges_list()->size();

	// for each face, set a need-support flag

	std::vector<bool> faceNeedSupportFlag(faceNum);
	std::vector<int> neighorNeedSupportNum(faceNum);

	const Vec3f _z(0, 0, -1);
	for (int i = 0; i < faceNum; i++)
	{
		std::vector<HE_vert *> tmp_face_verts;
		faceList->at(i)->face_verts(tmp_face_verts);

		Vec3f eAB = tmp_face_verts[1]->position_ - tmp_face_verts[0]->position_;
		Vec3f eAC = tmp_face_verts[2]->position_ - tmp_face_verts[0]->position_;
		Vec3f vNormal = eAB.cross(eAC);
		vNormal.normalize();

		float angle = vNormal.dot(_z);
		faceNeedSupportFlag[i] = angle > THRESHOLD ? true : false;

		// if true, try to set its neighbor to true
		if (faceNeedSupportFlag[i])
		{
			HE_edge* faceEdge = faceList->at(i)->pedge_;
			do
			{
				if (faceEdge->pface_ && faceEdge->ppair_->pface_)
				{
					int pairFace = faceEdge->ppair_->pface_->id_;
					if (faceNeedSupportFlag[pairFace])
					{
						faceEdge = faceEdge->pnext_;
						continue;
					}
					neighorNeedSupportNum[pairFace]++;
					if (neighorNeedSupportNum[pairFace] >= 2)
					{
						std::vector<HE_vert *> tmp_face_verts1;
						faceList->at(pairFace)->face_verts(tmp_face_verts1);

						Vec3f eAB1 = tmp_face_verts1[1]->position_ - tmp_face_verts1[0]->position_;
						Vec3f eAC1 = tmp_face_verts1[2]->position_ - tmp_face_verts1[0]->position_;
						Vec3f vNormal1 = eAB1.cross(eAC1);
						vNormal1.normalize();

						float angle1 = vNormal1.dot(_z);
						faceNeedSupportFlag[pairFace] = angle1 > THRESHOLD1 ? true : false;
					}

				}
				faceEdge = faceEdge->pnext_;
			} while (faceEdge != faceList->at(i)->pedge_);
		}
	}
	faceNeedSupport = faceNeedSupportFlag;
	for (int i = 0; i < faceNum; i++)
	{
		if (faceNeedSupportFlag[i])
		{
			faceList->at(i)->selected_ = true;
		}
	}
	// find need-support regions
	int regionNum = 0;
	std::vector<int> visited(faceNum); // also records the region a face belongs to (region starts from 1)
	std::queue<int> facesInQueue;
	for (int i = 0; i < faceNum; i++)
	{
		if (visited[i] == 0 && faceNeedSupportFlag[i])
		{
			facesInQueue.push(i);
			visited[i] = regionNum + 1;
			while (!facesInQueue.empty())
			{
				int topFaceInQueue = facesInQueue.front();
				facesInQueue.pop();

				HE_edge* faceEdge = faceList->at(topFaceInQueue)->pedge_;
				do
				{
					if (!faceEdge->isBoundary())
					{
						int pairFace = faceEdge->ppair_->pface_->id_;
						if (visited[pairFace] == 0 && faceNeedSupportFlag[pairFace])
						{
							facesInQueue.push(pairFace);
							visited[pairFace] = regionNum + 1;
						}
					}
					faceEdge = faceEdge->pnext_;
				} while (faceEdge != faceList->at(topFaceInQueue)->pedge_);
			}
			regionNum++;
		}
	}
	for (int iiii = 0; iiii < visited.size(); iiii++)
		qwewqe[iiii] = visited[iiii];

	// find region-boundary edge
	// and which region it is in (region's idx starts from 1)
	std::vector<int> regionBoundaryEdgeFlag(edgeNum);
	for (int i = 0; i < edgeNum; i++)
	{
		HE_face* edgeFace = edgeList->at(i)->pface_;
		if (edgeList->at(i)->isBoundary())
		{
			if (edgeFace == NULL)
				regionBoundaryEdgeFlag[i] = 0;
			else if (faceNeedSupportFlag[edgeFace->id_])
				regionBoundaryEdgeFlag[i] = visited[edgeFace->id_];
			else
				regionBoundaryEdgeFlag[i] = 0;
		}
		else if (faceNeedSupportFlag[edgeFace->id_])
		{
			int idT = edgeFace->id_;
			edgeFace = edgeList->at(i)->ppair_->pface_;
			if (faceNeedSupportFlag[edgeFace->id_])
				regionBoundaryEdgeFlag[i] = 0;
			else
				regionBoundaryEdgeFlag[i] = visited[idT];
		}
		qwewqe1[i] = regionBoundaryEdgeFlag[i];
	}
	BoundaryEdge = regionBoundaryEdgeFlag;
	update();
}
