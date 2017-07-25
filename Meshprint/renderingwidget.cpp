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
//#include "mainwindow.h"
#include "ArcBall.h"
#include "globalFunctions.h"
//#include "HE_mesh/Mesh3D.h"
#include "SliceCut.h"
#include "openGLProjector.h"
#include "QDebug"
#include "meshprint.h"
#include <fstream>
#include "Hatch.h"
#include <QTime>
// extern float laser_power_hatch_ ;
// extern float laser_speed_hatch_ ;
// extern float laser_power_polygon_ ;
// extern float laser_speed_polygon_;
class Support;

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
}

void RenderingWidget::initializeGL()
{
	glClearColor(.1, .1, .1, 0.0);
	glShadeModel(GL_SMOOTH);
	//glShadeModel(GL_FLAT);

	glEnable(GL_DOUBLEBUFFER);
	// 	glEnable(GL_POINT_SMOOTH);
	// 	glEnable(GL_LINE_SMOOTH);
	//	glEnable(GL_POLYGON_SMOOTH);
	//glEnable(GL_CULL_FACE);
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

	ptr_arcball_->reSetBound(w, h);
	ptr_arcball_module_->reSetBound(w, h);


	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0, GLdouble(w) / GLdouble(h), 0.1, 10000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void RenderingWidget::paintGL()
{
	glShadeModel(GL_SMOOTH);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (has_lighting_)
	{
		SetLight();
	}
	else
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_LIGHT0);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	register vec eyepos = eye_distance_*eye_direction_;
	gluLookAt(eyepos[0], eyepos[1], eyepos[2],
		eye_goal_[0], eye_goal_[1], eye_goal_[2],
		0.0, 1.0, 0.0);
	//glPushMatrix();

	glMultMatrixf(ptr_arcball_->GetBallMatrix());

	Render();
	//glPopMatrix();
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
		ptr_arcball_->MouseDown(e->pos());
		update();
	}
		break;
	case Qt::MidButton:
		current_position_ = e->pos();
		break;
	case  Qt::RightButton:
	{
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
		ptr_arcball_->MouseMove(e->pos());
		break;

	case Qt::MidButton:

		if (ptr_mesh_ != NULL)
		{
			eye_goal_[0] -= ptr_mesh_->getBoundingBox().at(0).at(2)*scaleV*GLfloat(e->x() - current_position_.x()) / GLfloat(width());
			eye_goal_[1] += ptr_mesh_->getBoundingBox().at(0).at(2)*scaleV*GLfloat(e->y() - current_position_.y()) / GLfloat(height());
		}
		current_position_ = e->pos();

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

		ptr_arcball_->MouseUp(e->pos());
		setCursor(Qt::ArrowCursor);

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
	DrawHatch(is_draw_hatch_);

}

void RenderingWidget::SetLight()
{
	//return;
	static GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	static GLfloat mat_shininess[] = { 50.0f };
	static GLfloat light_position0[] = { 1.0f, 1.0f, 0.5f, 0.0f };
	static GLfloat light_position1[] = { -1.0f, -1.0f, 0.5f, 0.0f };
	static GLfloat light_position2[] = { -.0f, -.0f, -0.5f, 0.0f };
	static GLfloat bright[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	static GLfloat dim_light[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	static GLfloat lmodel_ambient[] = { 0.4f, 0.4f, 0.4f, 1.0f };

	//glMaterialfv(GL_FRONT, GL_AMBIENT, mat_specular);
	//glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, bright);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, bright);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, dim_light);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, white_light);
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
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
	ptr_mesh_->ClearData();
	is_draw_grid_ = true;
	is_draw_face_ = true;
	is_draw_cutpieces_ = true;
	is_draw_hatch_ = true;
	has_lighting_ = true;
	ClearSlice();
	ClearHatch();
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Read Mesh"),
		"Resources/models", tr("Meshes (*.obj *.stl)"));

	if (filename.isEmpty())
	{
		//emit(operatorInfo(QString("Read Mesh Failed!")));
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


	//	m_pMesh->LoadFromOBJFile(filename.toLatin1().data());
	//emit(operatorInfo(QString("Read Mesh from") + filename + QString(" Done")));
	//emit(meshInfo(ptr_mesh_->num_of_vertex_list(), ptr_mesh_->num_of_edge_list(), ptr_mesh_->num_of_face_list()));


	float max_ = ptr_mesh_->getBoundingBox().at(0).at(0);
	max_ = max_ > ptr_mesh_->getBoundingBox().at(0).at(1) ? max_ : ptr_mesh_->getBoundingBox().at(0).at(1);
	max_ = max_ > ptr_mesh_->getBoundingBox().at(0).at(2) ? max_ : ptr_mesh_->getBoundingBox().at(0).at(2);

	//updateGL();
	update();
	time = QDateTime::currentDateTime();//获取系统现在的时间
	str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	//qDebug() << "read mesh end time :" << str;

	// 	qDebug() << "孔洞个数为：" << ptr_mesh_->GetBLoop().size();
	// 	qDebug() << "法向面片错误" <<sss;
	//qDebug() << "法向错误面片个数："<<sss;
	qDebug() << "load model end at" << time;
	qDebug() << ptr_mesh_->get_faces_list()->size();
	qDebug() << ptr_mesh_->getBoundingBox().at(0)[0] * 2 << ptr_mesh_->getBoundingBox().at(0)[1] * 2 << ptr_mesh_->getBoundingBox().at(0)[2];
	//ptr_arcball_->PlaceBall(scaleV);
	scaleT = scaleV;
	eye_distance_ = 2 * max_;
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
		"..", tr("Meshes (*.txt)"));

	if (filename.isEmpty())
		return;

	QByteArray byfilename = filename.toLocal8Bit();
	std::ofstream out(byfilename);
	std::vector < std::vector<cutLine>*>*tc = (mycut->GetPieces());
	//std::set<SField*, compareSField> *tc1 = (myhatch->getsmallFields());
	for (int i = 13; i < 14; i++)
	{
		for (size_t j = 0; j < tc[i].size(); j++)
		{
			for (int k = 0; k < (tc[i])[j]->size(); k++)
			{

				out << ((tc[i])[j])->at(k).position_vert[0].x() << " " << ((tc[i])[j])->at(k).position_vert[0].y() << " " << ((tc[i])[j])->at(k).position_vert[0].z() << "\n";
				out << ((tc[i])[j])->at(k).position_vert[1].x() << " " << ((tc[i])[j])->at(k).position_vert[1].y() << " " << ((tc[i])[j])->at(k).position_vert[1].z() << "\n";
			}
		}
		// 		for (auto iter = tc1[i].begin(); iter != tc1[i].end(); iter++)
		// 		{
		// 			out <<"f"<<" "<< (*iter)->x_min_*line_width_ << " " << (*iter)->y_min_*line_width_ << " " << (*iter)->z_height_ << "\n";
		// 			out << "f" << " " << (*iter)->x_max_ *line_width_ << " " << (*iter)->y_min_*line_width_ << " " << (*iter)->z_height_ << "\n";
		// 			out << "f" << " " << (*iter)->x_max_ *line_width_ << " " << (*iter)->y_max_*line_width_ << " " << (*iter)->z_height_ << "\n";
		// 			out << "f" << " " << (*iter)->x_min_*line_width_ << " " << (*iter)->y_max_*line_width_ << " " << (*iter)->z_height_ << "\n";
		// 
		// 		}
	}
	out.close();

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
	std::vector < std::vector<cutLine>* >*tc2 = (mycut->GetPieces());
	std::vector < std::vector<Vec3f>>* tc3 = myhatch->getOffsetVertex();
	std::vector<Vec3f *>* tc_s_hatch_ = myhatch->getHatch();
	std::vector < std::vector<cutLine>* >*tc_s_pieces_ = (mycut->GetPieces());
	std::vector < std::vector<Vec3f>>* tc_s_offsetvertex_ = myhatch->getOffsetVertex();
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
		if (i < myhatchsup->GetNumPieces())
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
		len_layer = len_Zpos + 6 + len_Hatch + 6 + len_Polygon + 6 + len_Hatch_s_ + 6 + len_Polygon_s_ + 6;
		newlayer.append(reinterpret_cast<const char *>(&Layer_Section), 2);
		newlayer.append(reinterpret_cast<const char *>(&len_layer), 4);
		newlayer.append(zPosition);
		newlayer.append(hatch);
		newlayer.append(polygon);
		newlayer.append(hatch_s_);
		newlayer.append(polygon_s_);
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
	update();

}
void RenderingWidget::CheckDrawFace()
{
	is_draw_face_ = !is_draw_face_;
	update();

}
void RenderingWidget::CheckLight()
{
	has_lighting_ = !has_lighting_;
	update();

}
void RenderingWidget::CheckGrid()
{
	is_draw_grid_ = !is_draw_grid_;
	update();

}
void RenderingWidget::CheckDrawTexture()
{
	is_draw_texture_ = !is_draw_texture_;
	if (is_draw_texture_)
		glEnable(GL_TEXTURE_2D);
	else
		glDisable(GL_TEXTURE_2D);
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
		glColor3f(0.0, 0.0, 0.0);
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
	glBegin(GL_TRIANGLES);

	glColor4f(.5, .5, 1.0, 0.9);
	for (size_t i = 0; i < faces.size(); ++i)
	{
		if (i == current_face_)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			HE_edge *pedge(faces.at(i)->pedge_);
			do
			{
				if (pedge == NULL)
				{
					break;
				}
				if (pedge == NULL || pedge->pface_->id() != faces.at(i)->id())
				{
					faces.at(i)->pedge_ = NULL;
					qDebug() << faces.at(i)->id() << "facet display wrong";
					break;
				}
				glNormal3fv(pedge->pvert_->normal().data());
				glVertex3fv((pedge->pvert_->position_*scaleV).data());
				pedge = pedge->pnext_;
			} while (pedge != faces.at(i)->pedge_);
		}
		else
		{
			if (faces.at(i)->selected() == SELECTED&&is_draw_region_)
			{
				glColor4f(1.0, 1.0, 0.0, 1.0);
			}
			else
			{
				//glColor4f(1.0, 1.0, 1.0, 1.0);
			}

			HE_edge *pedge(faces.at(i)->pedge_);
			do
			{
				if (pedge == NULL)
				{
					break;
				}
				if (pedge == NULL || pedge->pface_->id() != faces.at(i)->id())
				{
					faces.at(i)->pedge_ = NULL;
					qDebug() << faces.at(i)->id() << "facet display wrong";
					break;
				}
				glNormal3fv(pedge->pvert_->normal().data());
				glVertex3fv((pedge->pvert_->position()*scaleV).data());
				pedge = pedge->pnext_;
			} while (pedge != faces.at(i)->pedge_);
		}
	}
	glEnd();
	//return;

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
	std::vector < std::vector<cutLine>* >*tc = (mycut->GetPieces());
	glColor3f(0.0, 1.0, 0.0);
	//for (int i = 0; i<mycut->num_pieces_; i++)
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


// support operators
void RenderingWidget::AddPointSupport() {
	isAddPoint = !isAddPoint;
}
void RenderingWidget::AddLineSupport() {
	isAddLine = !isAddLine;
	line_points_.clear();
}
void RenderingWidget::DeleteSupport() {
	isDelete = !isDelete;
}



//hatch operators
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
	if (mycut != NULL)
	{
		mycut->~SliceCut();
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
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	//qDebug() << "do hatch start time" << str;
	qDebug() << "do hatch start at:" << time;
	if (ptr_mesh_->num_of_vertex_list() == 0)
	{
		return;
	}
	if (mycut == NULL)
	{
		cutinPieces();
	}
	if (myhatch != NULL)
	{
		myhatch->clearHatch();
	}
	
	if (myhatchsup != NULL)
	{
		myhatchsup->clearHatch();
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
		break;
	case STRIP:
		myhatch = new HatchStrip(mycut);
		if (mycutsup->GetPieces() != NULL)
		{
			myhatchsup = new HatchStrip(mycutsup);
		}

		break;
	case MEANDER:
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
	time = QDateTime::currentDateTime();//获取系统现在的时间
	str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	qDebug() << "do hatch end time :" << time;
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
void RenderingWidget::setThickness(double thick)
{
	thickness_ = thick;
}
void RenderingWidget::setAngle(int angle)
{
	increment_angle_ = angle;
}
