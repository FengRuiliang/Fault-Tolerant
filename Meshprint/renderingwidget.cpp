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
#include "HE_mesh/Mesh3D.h"
#include "openGLProjector.h"
#include "QDebug"
#include "meshprint.h"
#include <fstream>
#include <QTime>

class Support;

RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
	: QOpenGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(5.0),
	has_lighting_(true), is_draw_point_(false), is_draw_edge_(false), is_draw_face_(true)
{
	ptr_arcball_ = new CArcBall(width(), height());
	ptr_mesh_ =new Mesh3D;
	ptr_slice_ = NULL;
	ptr_hatch_ = NULL;
	is_select_face = false;
	is_draw_hatch_ = false;
	is_load_texture_ = false;
	is_draw_axes_ = false;
	is_draw_texture_ = (false);
	is_draw_grid_ = (false);
	is_draw_cutpieces_ = (false);
	is_move_module_ = (false);
	is_show_all = false;
	eye_goal_[0] = eye_goal_[1] = eye_goal_[2] = 0.0;
	eye_direction_[0] = eye_direction_[1] = 0.0;
	eye_direction_[2] = 1.0;
	slice_check_id_ = 0;
}

RenderingWidget::~RenderingWidget()
{
	SafeDelete(ptr_arcball_);
	SafeDelete(ptr_slice_);
	SafeDelete(ptr_mesh_);
}

void RenderingWidget::initializeGL()
{
	glClearColor(0.68, 0.68, 0.68, 0.0);
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
		makeCurrent();
		OpenGLProjector myProjector = OpenGLProjector();
		Vec3f myPointN(e->x(), height() - e->y(), -1.0f);
		Vec3f myPointF(e->x(), height() - e->y(), 1.0f);
		Vec3f add_pointN = myProjector.UnProject(myPointN);
		Vec3f add_pointF = myProjector.UnProject(myPointF);
		Vec3f direc = add_pointF - add_pointN;
		// when near is 0.001, here direc*10.0f, near is 0.01, then direct*1.0f
		add_pointN -= direc * 0.10f;
		direc.normalize();
		const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());
		for (int i=0;i<faces.size();i++)
		{
			Vec3f point_;
			CalPlaneLineIntersectPoint(faces.at(i)->normal(), faces.at(i)->vertices_[0]->position(),
				direc, add_pointN, point_);
			if (PointinTriangle(faces.at(i),point_))
			{
				ptr_mesh_->SetDirection(i);
				ptr_mesh_->scalemesh(1.0);
				break;
			}
		}
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

ptr_arcball_->MouseUp(e->pos());
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
	DrawSlice(true);
	DrawHatch(true);
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

void RenderingWidget::SetSliceCheckId(int id)
{
	if (ptr_slice_ == NULL)
	{
		return;
	}
	if (id >= ptr_slice_->GetNumPieces())
	{
		slice_check_id_ = ptr_slice_->GetNumPieces() - 1;
	}
	else
	{
		slice_check_id_ = id;
	}
	update();
}

void RenderingWidget::setHatchType(int type_)
{
	hatch_type_ = (hatchType)type_;
}

void RenderingWidget::ReadMesh()
{
	SafeDelete(ptr_mesh_);
	SafeDelete(ptr_slice_);
	SafeDelete(ptr_hatch_);
	ptr_mesh_ = NULL;
	ptr_slice_ = NULL;
	ptr_hatch_ = NULL;
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	ptr_arcball_->reSetBound(width(), height());
	ptr_mesh_ = new Mesh3D();
	is_draw_grid_ = true;
	is_draw_face_ = true;
	is_draw_cutpieces_ = true;
	is_draw_hatch_ = true;
	has_lighting_ = true;
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

	update();
	time = QDateTime::currentDateTime();//获取系统现在的时间
	str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	//qDebug() << "read mesh end time :" << str;
	scaleT = scaleV;
	eye_distance_ = 2 * max_;
	//ptr_mesh_->MarkEdge();
	qDebug() << "load model end at" << time;
	qDebug() << ptr_mesh_->get_faces_list()->size();
	qDebug() << ptr_mesh_->getBoundingBox().at(0)[0] * 2 << ptr_mesh_->getBoundingBox().at(0)[1] * 2 << ptr_mesh_->getBoundingBox().at(0)[2];
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
// 	const std::vector<HE_edge *>& edges = *(ptr_mesh_->get_edges_list());
// 	glLineWidth(2.0f);
// 	glColor4ub(0, 0, 0, 255);
// 	glBegin(GL_LINES);
// 	for (int i=0;i<edges.size();i++)
// 	{
// 		if (edges[i]->boundary_flag()==BOUNDARY)
// 		{
// 			glVertex3fv(edges.at(i)->start_->position());
// 			glVertex3fv(edges.at(i)->pvert_->position());
// 		}
// 	}
// 	glEnd();
	const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());
	glBegin(GL_TRIANGLES);
	glColor4ub(0, 170, 0, 255);
	for (size_t i = 0; i < faces.size(); ++i)
	{
		glNormal3fv(faces.at(i)->normal());
		glVertex3fv(faces[i]->vertices_[0]->position());
		glVertex3fv(faces[i]->vertices_[1]->position());
		glVertex3fv(faces[i]->vertices_[2]->position());
	}
	glEnd();
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
void RenderingWidget::DrawSlice(bool bv)
{
	if (!bv||ptr_slice_==NULL)
		return;
	//glLineWidth(1.0);
	glColor3f(0.0, 1.0, 0.0);
	std::map<float, std::vector<std::vector<cutLine>>> map_cut_ = ptr_slice_->getMapPieces();
	auto layer_ = ptr_slice_->thickf_;
	for (auto iterset = map_cut_.begin(); iterset != map_cut_.end(); iterset++)
	{
		std::vector<std::vector<cutLine>>& loops_ = iterset->second;
		glBegin(GL_LINES);
		for (int j = 0; j < loops_.size(); j++)
		{
			std::vector<cutLine>& one_loop_ = loops_[j];
			for (int k=0;k<one_loop_.size();k++)
			{
				glVertex3fv(one_loop_[k].position_vert[0]);
				glVertex3fv(one_loop_[k].position_vert[1]);
			}
		}
		glEnd();
	}
	return;

	std::vector < std::vector<std::pair<Vec3f,Vec3f>> >*tc = (ptr_slice_->GetPieces());
	if (tc==NULL)
	{
		return;
	}
	for (int i = 0; i < ptr_slice_->num_pieces_; i++)
	{
		glBegin(GL_LINES);
		for (size_t j = 0; j < tc[i].size(); j++)
		{
			for (int k = 0; k < (tc[i])[j].size(); k++)
			{
				if (k==0)
				{
					glColor3f(1.0, 0.0, 0.0);
				}
				else
				{
					glColor3f(0.0, 1.0, 0.0);
				}
				glVertex3fv(tc[i][j][k].first);
				glVertex3fv(tc[i][j][k].second);
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
	if (ptr_mesh_->num_of_vertex_list() == 0 || ptr_slice_ == NULL || ptr_hatch_ == NULL)
	{
		return;
	}
	std::vector<Vec3f*>* tc_hatch_ = ptr_hatch_->getHatch();
	std::vector < std::vector<Vec3f>>* tc_offset_ = ptr_hatch_->getOffsetVertex();
	if (slice_check_id_ > ptr_hatch_->num_pieces_ - 1)
	{
		return;
	}
	if (is_show_all)
	{
		for (int i = 0; i < ptr_hatch_->num_pieces_; i++)
		{
			for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
			{
				glColor3f(0.0, 1.0, 0.0);
				glBegin(GL_LINES);
				glVertex3fv((*iterline)[0] );
				glVertex3fv((*iterline)[1] );
				glEnd();
			}

			for (int j = 0; j < tc_offset_[i].size(); j++)
			{
				glColor3f(1.0, 0.0, 0.0);
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
				{
					glVertex3fv(((tc_offset_[i])[j]).at(k));
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
				glVertex3fv(((*iterline)[0]));
				glVertex3fv(((*iterline)[1]));
				glEnd();
			}

			for (int j = 0; j < tc_offset_[i].size(); j++)
			{
				glColor3f(1.0, 0.0, 0.0);
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
				{
					glVertex3fv(((tc_offset_[i])[j]).at(k));
				}
				glEnd();
			}
		}
	}
}
void RenderingWidget::DoSlice()
{
	if (ptr_slice_!=NULL)
	{
		SafeDelete(ptr_slice_);
	}
	ptr_slice_ = new SliceCut(ptr_mesh_);
 	ptr_slice_->cutThrouthVertex();
 	return;
	ptr_slice_->StoreFaceIntoSlice();
	ptr_slice_->CutInPieces();
	fillPath();
}
void RenderingWidget::fillPath()
{
	if (ptr_mesh_->num_of_vertex_list() == 0)
	{
		return;
	}
	switch (hatch_type_)
	{
	case NONE:
		SafeDelete(ptr_hatch_);
		ptr_hatch_ = NULL;
		break;
	case CHESSBOARD:
		SafeDelete(ptr_hatch_);
		ptr_hatch_ = new HatchChessboard(ptr_slice_);
		ptr_hatch_->doHatch();
		break;
	case OFFSETFILLING:
		break;
	case STRIP:
		SafeDelete(ptr_hatch_);
		ptr_hatch_ = new HatchStrip(ptr_slice_);
		ptr_hatch_->doHatch();
		break;
	case MEANDER:
		break;
	default:
		break;
	}
	update();
}


