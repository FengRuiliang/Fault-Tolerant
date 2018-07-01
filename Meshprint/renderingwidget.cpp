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
#include <utility>
#include "Library/ArcBall.h"
#include "globalFunctions.h"
#include "HE_mesh/Mesh3D.h"
#include "openGLProjector.h"
#include "QDebug"
#include "meshprint.h"
#include <fstream>
#include <QTime>
#include "Support.h"


static GLfloat win, hei;
RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
	: QOpenGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(200),
	has_lighting_(true), is_draw_point_(false), is_draw_edge_(false), is_draw_face_(true)
{
	ptr_arcball_ = new CArcBall(width(), height());
	ptr_mesh_ = new Mesh3D;
	ptr_slice_ = NULL;
	ptr_support_ = NULL;
	is_select_face = false;
	is_draw_hatch_ = false;
	is_load_texture_ = false;
	is_draw_axes_ = false;
	is_draw_texture_ = (false);
	is_draw_grid_ = (false);
	is_draw_cutpieces_ = (false);
	is_move_module_ = (false);
	is_show_all = false;
	is_draw_support_ = false;
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
	glClearColor(0.78, 0.78, 0.78, 0.0);
	glClearColor(1, 1, 1, 0.0);
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
	win = w;
	hei = h;
	ptr_arcball_->reSetBound(w, h);
	//glViewport(0, 0, w, h);
	glViewport(0, 0, (GLfloat)eye_distance_*win, (GLfloat)eye_distance_*hei);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-eye_distance_, eye_distance_, -eye_distance_ * (GLfloat)h / (GLfloat)w, eye_distance_ * (GLfloat)h / (GLfloat)w, -200.0, 200.0);
	else
		glOrtho(-eye_distance_*(GLfloat)w / (GLfloat)h, eye_distance_*(GLfloat)w / (GLfloat)h, -eye_distance_, eye_distance_, -200.0, 200.0);

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
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (win <= hei)
		glOrtho(-eye_distance_ + eye_goal_[0], eye_distance_ + eye_goal_[0],
			-eye_distance_ * (GLfloat)hei / (GLfloat)win + eye_goal_[1], eye_distance_ * (GLfloat)hei / (GLfloat)win + eye_goal_[1],
			-200.0, 200.0);
	else
		glOrtho(-eye_distance_ * (GLfloat)win / (GLfloat)hei + eye_goal_[0], eye_distance_ * (GLfloat)win / (GLfloat)hei + eye_goal_[0],
			-eye_distance_ + eye_goal_[1], eye_distance_ + eye_goal_[1], -200.0, 200.0);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	float mId[4][4];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (i == j)
					mId[i][j] = 1.f;
				else	mId[i][j] = 0.f;
			}
		}
		// 	mId[1][1] = 0;
		// 	mId[2][2] = 0;
		// 	mId[2][1] = 1;
		// 	mId[1][2] = -1;
		mId[1][1] = -1;
		mId[2][2] = -1;
		glMultMatrixf((float*)mId);
	//glMultMatrixf(ptr_arcball_->GetBallMatrix());

	Render();

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
		for (int i = 0; i < faces.size(); i++)
		{
			Vec3f point_;
			CalPlaneLineIntersectPoint(faces.at(i)->normal(), faces.at(i)->vec_ptr_vert_[0]->position(),
				direc, add_pointN, point_);
			std::vector<Vec3f> te;
			for (int i = 0; i < 3; i++)
			{
				te.push_back(faces[i]->vec_ptr_vert_[i]->position());
			}
			if (PointinTriangle(te, point_))
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
		eye_distance_ -= e->delta() / 100;
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
	draw_support_aera(is_draw_support_);
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

	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, bright);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, bright);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, bright);
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
	slice_check_id_ = id;
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
void RenderingWidget::setFildID(int id) {
	field_id = id;
	update();
}
void RenderingWidget::SetLineId(int id) {
	line_id_ = id;
	update();
}


void RenderingWidget::ReadMesh()
{
	SafeDelete(ptr_mesh_);
	SafeDelete(ptr_slice_);
	ptr_mesh_ = NULL;
	ptr_slice_ = NULL;
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
void RenderingWidget::check_support(bool bv)
{
	is_draw_support_ = bv;
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
	const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());

	//////////////////////////////////////////////////////////////////////////
	glBegin(GL_LINES);
	glColor4ub(0, 0, 0, 255);
	for (int i = 0; i < faces.size(); i++)
	{

		for (int j = 0; j < 3; j++)
		{
			glVertex3fv(faces[i]->vec_ptr_vert_[j]->position());

			glVertex3fv(faces[i]->vec_ptr_vert_[(j + 1) % 3]->position());
		}
	}
	glEnd();
	//////////////////////////////////////////////////////////////////////////
}
void RenderingWidget::DrawFace(bool bv)
{
	if (!bv || ptr_mesh_ == NULL)
		return;

	if (ptr_mesh_->num_of_face_list() == 0)
	{
		return;
	}
	;
	const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());

	glBegin(GL_TRIANGLES);
	glColor4ub(0, 170, 0, 255);
	for (size_t i = 0; i < faces.size(); ++i)
	{
		Vec3f color = SetColor(faces[i]->com_flag);
		glColor4ub((int)color.x(), (int)color.y(), (int)color.z(), 255);
		glNormal3fv(faces[i]->normal());
		glVertex3fv(faces[i]->vec_ptr_vert_[0]->position());
		glVertex3fv(faces[i]->vec_ptr_vert_[1]->position());
		glVertex3fv(faces[i]->vec_ptr_vert_[2]->position());

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
	glLineWidth(0.5);
	glBegin(GL_LINES);


	Vec3f box(ptr_mesh_->getBoundingBox().at(0)*scaleV - ptr_mesh_->getBoundingBox().at(1)*scaleV);
	for (int i = -31; i < 32; i++)
	{
		glVertex2f(-box[0], i);
		glVertex2f(box[0], i);

		glVertex2f(i, -box[1]);
		glVertex2f(i, box[1]);
	}

	glEnd();

	//glEnable(GL_LIGHTING);
}
void RenderingWidget::DrawSlice(bool bv)
{
	if (!bv || ptr_slice_ == NULL)
		return;
//slicing before offset
	for (int i = 0; i < layers[slice_check_id_].size(); i++) {
		glBegin(GL_LINE_LOOP); glColor4f(0.0, 0.0, 1.0, 1.0); glLineWidth(2);
		for (int j = 0; j < layers[slice_check_id_][i].size(); j++) {
			glVertex3f(layers[slice_check_id_][i][j].first, layers[slice_check_id_][i][j].second, slice_check_id_*thickness_);
		}
		glEnd();
	}

	//slicing after offset


	//show grids
	if (slice_check_id_)for (int i = 0; i < gridshow[slice_check_id_ - 1].size(); i++) {
		glBegin(GL_LINE_LOOP); glColor4f(1.0, 1.0, 1.0, 1.0);
		glVertex3f(gridshow[slice_check_id_ - 1][i].first*MIN_DIS, gridshow[slice_check_id_ - 1][i].second*MIN_DIS, thickness_*slice_check_id_);
		glVertex3f(gridshow[slice_check_id_ - 1][i].first*MIN_DIS + MIN_DIS, gridshow[slice_check_id_ - 1][i].second*MIN_DIS, thickness_*slice_check_id_);
		glVertex3f(gridshow[slice_check_id_ - 1][i].first*MIN_DIS + MIN_DIS, gridshow[slice_check_id_ - 1][i].second*MIN_DIS + MIN_DIS, thickness_*slice_check_id_);
		glVertex3f(gridshow[slice_check_id_ - 1][i].first*MIN_DIS, gridshow[slice_check_id_ - 1][i].second*MIN_DIS + MIN_DIS, thickness_*slice_check_id_);
		glEnd();
	}
}
void RenderingWidget::DrawHatch(bool bv)
{
}
void RenderingWidget::draw_support_aera(bool bv)
{
	if (bv&&ptr_support_ != NULL)
	{
		auto sp_flist_ = *ptr_support_->sp_mesh.get_faces_list();
		auto sup_point_ = ptr_support_->sample_points_;
		auto sup_component_region = ptr_support_->component_regions_mesh;
		// draw support point
		glColor3f(0.0, 0.0, 0.0);
		glBegin(GL_TRIANGLES);
		for (int i = -1; i < 6; i++)
		{
			if (i > slice_check_id_)
			{
				continue;
			}
			for (auto iter = sup_point_[i].begin(); iter != sup_point_[i].end(); iter++)
			{
				//Vec3f color = SetColor(i);
				//glColor4ub((int)color.x(), (int)color.y(), (int)color.z(), 255);
				for (int k = 0; k < sp_flist_.size(); k++)
				{
					HE_edge* sta = sp_flist_[k]->pedge_;
					HE_edge* cur = sta;
					do
					{
						glVertex3fv(cur->pvert_->position() + *iter-Vec3f(0,0,100));
						cur = cur->pnext_;
					} while (cur != sta);
				}
			}

		}
		glEnd();
		
		glColor4ub(0, 0, 0, 255);
		glLineWidth(3.0);
		//glLineStipple(1, 0x3F07);
		glEnable(GL_LINE_STIPPLE);

		for (int i=0;i<COUNTOFANGLE;i++)
		{
			if (i != slice_check_id_)
			{
				continue;
			}

			for (int j = 0; j < test_path[i].size(); j++)
			{
				
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < test_path[i][j].size(); k++)
				{
					glVertex3f((float)test_path[i][j][k].X / 1000, (float)test_path[i][j][k].Y / 1000, -1);
				}
				glEnd();

			}
		}
		glDisable(GL_LINE_STIPPLE);


		for (int i=0;i<sup_component_region.size();i++)
		{
			for (int j=0;j<sup_component_region[i].size();j++)
			{
				Vec3f color = SetColor(j);
				glColor4ub((int)color.x(), (int)color.y(), (int)color.z(), 255);
				for (int k=0;k<sup_component_region[i][j].size();k++)
				{
				
					glBegin(GL_TRIANGLES);
					auto face_list_ =sup_component_region[i][j][k]->get_faces_list();
					for (int ii = 0; ii < face_list_->size(); ii++)
					{
						HE_edge* sta = face_list_->at(ii)->pedge_;
						HE_edge* cur = sta;
						do
						{
							glVertex3fv(cur->pvert_->position()-Vec3f(0,0,cur->pvert_->position().z()));
							cur = cur->pnext_;
						} while (cur != sta);

					}
					glEnd();

				}
				continue;//for display
				glColor4ub(0,0,0, 255);
				glLineWidth(1.0);
				for (int k = 0; k < sup_component_region[i][j].size(); k++)
				{
					auto face_list_ = sup_component_region[i][j][k]->get_faces_list();
					for (int ii = 0; ii < face_list_->size(); ii++)
					{
						glBegin(GL_LINE_LOOP);
						HE_edge* sta = face_list_->at(ii)->pedge_;
						HE_edge* cur = sta;
						do
						{
							glVertex3fv(cur->pvert_->position() - Vec3f(0, 0, cur->pvert_->position().z() + 1.0));
							//glVertex3fv(cur->pvert_->position());
							cur = cur->pnext_;
						} while (cur != sta);
						glEnd();
					}

				}
			}
		}
	}
}
Vec3f RenderingWidget::SetColor(int j)
{
	
	switch (j)
	{
	case -1:
		return Vec3f(153.0, 153.0, 153.0);
	case 0:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(228, 26, 28);

	case 1:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(55, 126, 184);

	case 2:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(77, 175, 14);

	case 3:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(152, 78, 163);

	case 4:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(255.0, 172.0, 0.0);

	case 5:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(255.0, 172.0, 0.0);
		return Vec3f(255.0, 255.0, 51.0);

	case 6:
		return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(255.0, 255.0, 51.0);
		return Vec3f(166.0, 84.0, 40.0);
	case 7:
		//return Vec3f(0.0, 170.0, 0.0);
		return Vec3f(247.0, 129.0, 191.0);
	case 8:
		
	default:
		//return Vec3f(0.0, 184, 229);
	
		return Vec3f(0.0,170.0, 0.0);

	}
}
void RenderingWidget::DoSlice()
{
	if (ptr_slice_ != NULL)
	{
		SafeDelete(ptr_slice_);
	}
	ptr_slice_ = new SliceCut(ptr_mesh_);
	ptr_slice_->StoreFaceIntoSlice();
	ptr_slice_->CutInPieces();
}
void RenderingWidget::FillPath()
{
	update();
}
void RenderingWidget::add_support()
{
	SafeDelete(ptr_support_);
	if (ptr_mesh_ != NULL)
	{
		ptr_support_ = new Support(ptr_mesh_);
	}
	counter_++;
	ptr_support_->find_support_area();
	
	ptr_support_->support_point_sampling(counter_);
	qDebug() << "the final support point number is " << ptr_support_->num;
	return;
	QString filename = QFileDialog::
		getSaveFileName(this, tr("Write Mesh"),
			"..", tr("grs (*.grs)"));
	if (filename.isEmpty())
		return;
	QByteArray byfilename = filename.toLocal8Bit();
	ptr_support_->exportcylinder(byfilename);

	



}
