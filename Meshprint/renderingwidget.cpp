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
	ptr_hatch_ = NULL;
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
	glMultMatrixf(ptr_arcball_->GetBallMatrix());

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
	if (!bv || ptr_slice_ == NULL)
		return;
	/*glColor3f(0.0, 1.0, 0.0);
	std::vector < std::vector<std::pair<Vec3f,Vec3f>> >*tc = (ptr_slice_->GetPieces());
	for (int i = slice_check_id_; i < slice_check_id_+1; i++)
	{
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		for (size_t j = field_id; j < field_id+1; j++)
		{
			if (j>=tc[i].size())
			{
				break;
			}
			for (int k = 0; k < (tc[i])[j].size(); k++)
			{
				if (k == 0)
				{
					glColor3f(0.0, 0.0, 1.0);
				}
				else
					glColor3f(1.0, 0.0, 0.0);
				glVertex3fv(tc[i][j][k].first+Vec3f(0.0,0.0,10*thickness_));
				glVertex3fv(tc[i][j][k].second+ Vec3f(0.0, 0.0, 10*thickness_));
			}
			glColor3f(1.0, 1.0, 0.0);
			for (int k = 0; k < line_id_&&k < (tc[i])[j].size(); k++)
			{
				glVertex3fv(tc[i][j][k].first + Vec3f(0.0, 0.0, 20 * thickness_));
				glVertex3fv(tc[i][j][k].second + Vec3f(0.0, 0.0, 20 * thickness_));
			}
		}
		glColor3f(0.0, 1.0, 0.0);
		for (size_t j = 0; j < tc[i].size(); j++)
		{
			for (int k = 0; k < (tc[i])[j].size(); k++)
			{

				glVertex3fv(tc[i][j][k].first);
				glVertex3fv(tc[i][j][k].second );
			}
		}
		glEnd();
	}
*/
//slicing before offset
	for (int i = 0; i < layers[slice_check_id_].size(); i++) {
		glBegin(GL_LINE_LOOP); glColor4f(0.0, 0.0, 1.0, 1.0); glLineWidth(2);
		for (int j = 0; j < layers[slice_check_id_][i].size(); j++) {
			glVertex3f(layers[slice_check_id_][i][j].first, layers[slice_check_id_][i][j].second, slice_check_id_*thickness_);
		}
		glEnd();
	}

	//slicing after offset
	DrawPaths(res_path[slice_check_id_], slice_check_id_);


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
				glVertex3fv((*iterline)[0]);
				glVertex3fv((*iterline)[1]);
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
void RenderingWidget::draw_support_aera(bool bv)
{
	if (bv&&ptr_support_ != NULL)
	{
		auto sp_flist_ = *ptr_support_->sp_mesh.get_faces_list();
		auto sup_point_ = ptr_support_->sample_points_;
		for (auto iter = ptr_support_->sup_ptr_aera_list_.begin(); iter != ptr_support_->sup_ptr_aera_list_.end(); iter++)
		{
			if (field_id!=iter->first&&slice_check_id_==0)
			{
				continue;
			}
			for (int i=0;i<iter->second.size();i++)
			{
			
				auto face_list_ = iter->second[i]->get_faces_list();
				glBegin(GL_TRIANGLES);
				glColor3f(sin(iter->first * 10 + 1), cos(iter->first * 10 + 1), tan(iter->first * 10 + 1));
				for (int j = 0; j < face_list_->size(); j++)
				{
					HE_edge* sta = face_list_->at(j)->pedge_;
					HE_edge* cur = sta;
					do
					{
						//glVertex3fv(cur->pvert_->position()-Vec3f(0,0,cur->pvert_->position().z()));
						glVertex3fv(cur->pvert_->position());
						cur = cur->pnext_;
					} while (cur != sta);

				}
				glEnd();
				auto boundary_loop_ = iter->second[i]->GetBLoop();
				for (int j = 0; j < boundary_loop_.size(); j++)
				{
					glBegin(GL_LINE_LOOP);
					for (int k = 0; k < boundary_loop_[j].size(); k++)
					{
						glVertex3fv(boundary_loop_[j][k]->pvert_->position());
					}
					glEnd();
				}
			}
			// draw support point
			glColor3f(1.0, 0.0, 0.0);
			glBegin(GL_TRIANGLES);
			for (int j = 0; j < sup_point_[iter->first].size(); j++)
			{
				for (int k = 0; k < sp_flist_.size(); k++)
				{
					HE_edge* sta = sp_flist_[k]->pedge_;
					HE_edge* cur = sta;
					do
					{
						glVertex3fv(cur->pvert_->position() + sup_point_[iter->first][j]);
						cur = cur->pnext_;
					} while (cur != sta);
				}
			}
			glEnd();
		}

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
	FindNarrowBand();
}
void RenderingWidget::FillPath()
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
	QString filename = QFileDialog::
		getSaveFileName(this, tr("Write Mesh"),
			"..", tr("grs (*.grs)"));
	if (filename.isEmpty())
		return;
	QByteArray byfilename = filename.toLocal8Bit();
	ptr_support_->exportcylinder(byfilename);
}


void RenderingWidget::FindNarrowBand()
{
	std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_ = ptr_slice_->GetPieces();
	layers.clear(); res_path.clear();

	qDebug() << "preprocessing" << endl;
	for (int i = 0; i < ptr_slice_->GetNumPieces(); i++) {
		std::vector<std::vector<std::pair<double, double> > > contours; contours.clear(); New2Origin layerlinenew2origin;
		for (int j = 0; j < pieces_[i].size(); j++) {
			std::vector<std::pair<double, double> > contour; contour.clear();
			//CancelBugCut(pieces_,i,j);
			for (int k = 0; k < pieces_[i][j].size(); k++) {
				double x1 = pieces_[i][j][k].first[0]; double y1 = pieces_[i][j][k].first[1];
				double x2 = pieces_[i][j][k].second[0]; double y2 = pieces_[i][j][k].second[1];
				std::set<double> godis; godis.clear();
				godis.insert(0); godis.insert(1);
				for (int ii = int(min(x1, x2) / MIN_DIS); ii < int(max(x1, x2) / MIN_DIS); ii++) {
					if (!(x1 < ii*MIN_DIS&&ii*MIN_DIS < x2))continue;
					godis.insert((ii*MIN_DIS - x1) / (x2 - x1));
				}
				for (int jj = int(min(y1, y2) / MIN_DIS); jj < int(max(y1, y2) / MIN_DIS); jj++) {
					if (!(y1 < jj*MIN_DIS&&jj*MIN_DIS < y2))continue;
					godis.insert((jj*MIN_DIS - y1) / (y2 - y1));
				}
				std::set<double>::iterator it;
				double last_it;
				for (it = godis.begin(); it != godis.end(); ++it) {
					if (it != godis.begin() && abs(*it - last_it) < eps)continue;
					last_it = *it;
					layerlinenew2origin.layermap[j][contour.size()] = k;
					contour.push_back(std::make_pair(x1 + *it*(x2 - x1), y1 + *it*(y2 - y1)));
				}
			}
			contours.push_back(contour);
		}
		lines_new2origin.push_back(layerlinenew2origin);
		layers.push_back(contours);
	}

	int dir[9][2] = { { -1,1 },{ 0,1 },{ 1,1 },{ -1,0 },{ 0,0 },{ 1,0 },{ -1,-1 },{ 0,-1 },{ 1,-1 } };

	qDebug() << "begin detection" << endl;
	for (int i = 0; i < layers.size(); i++) {
		std::map<std::pair<int, int>, LayerGrid  > layer_grids;
		std::map<std::vector<int>, int> lineincontour;
		std::vector<bool> IsInner;
		LayerOffDis layer_offdis;
		//build grids
		for (int j = 0; j < layers[i].size(); j++) {
			IsInner.push_back(!JudgeLoopDir(layers[i][j]));
			for (int k = 0; k < layers[i][j].size(); k++) {
				layer_offdis.dis[j].push_back(0);
				double x1 = layers[i][j][k].first, x2 = layers[i][j][(k + 1) % layers[i][j].size()].first;
				double y1 = layers[i][j][k].second, y2 = layers[i][j][(k + 1) % layers[i][j].size()].second;
				double x = 0.5*(x1 + x2), y = 0.5*(y1 + y2);
				std::pair<int, int> g = point2grid(std::make_pair(x, y));
				std::vector<int> a; a.push_back(g.first); a.push_back(g.second); a.push_back(j); a.push_back(layer_grids[g].grids[j].size());
				lineincontour[a] = k;
				layer_grids[g].grids[j].push_back(std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
			}
		}
		std::map<std::pair<int, int>, LayerGrid  >::iterator it1, it2;
		std::set<std::pair<pa, pa> > vis;
		//detection of near distance
		for (it1 = layer_grids.begin(); it1 != layer_grids.end(); it1++) {
			int x = it1->first.first, y = it1->first.second;
			for (int k = 0; k < 9; k++) {
				int next_x = x + dir[k][0], next_y = y + dir[k][1];
				if (vis.find(std::make_pair(std::make_pair(x, y), std::make_pair(next_x, next_y))) != vis.end())continue;
				vis.insert(std::make_pair(std::make_pair(next_x, next_y), std::make_pair(x, y))); vis.insert(std::make_pair(std::make_pair(next_x, next_y), std::make_pair(x, y)));
				it2 = layer_grids.find(std::make_pair(next_x, next_y));
				if (it2 == layer_grids.end())continue;
				for (int j1 = 0; j1 < layers[i].size(); j1++) if (it1->second.grids[j1].size()) {
					for (int j2 = 0; j2 < layers[i].size(); j2++)if (it2->second.grids[j2].size()) {
						if (IsInner[j1] && IsInner[j2] && j1 != j2) {
							for (int k1 = 0; k1 < it1->second.grids[j1].size(); k1++) {
								for (int k2 = 0; k2 < it2->second.grids[j2].size(); k2++) {
									pa p[3][3], optimal_dir, offdir[3];
									p[1][1] = it1->second.grids[j1][k1].first, p[1][2] = it1->second.grids[j1][k1].second;
									p[2][1] = it2->second.grids[j2][k2].first, p[2][2] = it2->second.grids[j2][k2].second;
									optimal_dir = furthestdir(p);
									double line_dis = sqrt(dot(optimal_dir, optimal_dir));
									//line_dis = sqrt((p[1][1].first + p[1][2].first - p[2][1].first - p[2][2].first) * (p[1][1].first + p[1][2].first - p[2][1].first - p[2][2].first)* 0.25 + (p[1][1].second + p[1][2].second - p[2][1].second - p[2][2].second) *(p[1][1].second + p[1][2].second - p[2][1].second - p[2][2].second)*0.25);
									double move = (MIN_DIS - line_dis);
									if (move < 0)continue;
									std::vector<int> a; a.push_back(x); a.push_back(y); a.push_back(j1); a.push_back(k1);
									std::vector<int> b; b.push_back(next_x); b.push_back(next_y); b.push_back(j2); b.push_back(k2);
									layer_offdis.dis[j1][lineincontour[a]] = max(move*0.5, layer_offdis.dis[j1][lineincontour[a]]);
									layer_offdis.dis[j2][lineincontour[b]] = max(move*0.5, layer_offdis.dis[j2][lineincontour[b]]);
									//if (dot(p1, optimal_dir) > 0 && dot(p1, optimal_dir) < 0)
								}
							}
						}
						else if (IsInner[j1] && !IsInner[j2] && j1 != j2) {
							for (int k1 = 0; k1 < it1->second.grids[j1].size(); k1++) {
								for (int k2 = 0; k2 < it2->second.grids[j2].size(); k2++) {
									pa p[3][3], optimal_dir, offdir[3];
									p[1][1] = it1->second.grids[j1][k1].first, p[1][2] = it1->second.grids[j1][k1].second;
									p[2][1] = it2->second.grids[j2][k2].first, p[2][2] = it2->second.grids[j2][k2].second;
									optimal_dir = furthestdir(p);
									double line_dis = sqrt(dot(optimal_dir, optimal_dir));
									double move = (MIN_DIS - line_dis);
									if (move < 0)continue;
									std::vector<int> a; a.push_back(x); a.push_back(y); a.push_back(j1); a.push_back(k1);
									std::vector<int> b; b.push_back(next_x); b.push_back(next_y); b.push_back(j2); b.push_back(k2);
									layer_offdis.dis[j1][lineincontour[a]] = max(move, layer_offdis.dis[j1][lineincontour[a]]);
								}
							}
						}
						else if (!IsInner[j1] && IsInner[j2] && j1 != j2) {
							for (int k1 = 0; k1 < it1->second.grids[j1].size(); k1++) {
								for (int k2 = 0; k2 < it2->second.grids[j2].size(); k2++) {
									pa p[3][3], optimal_dir, offdir[3];
									p[1][1] = it1->second.grids[j1][k1].first, p[1][2] = it1->second.grids[j1][k1].second;
									p[2][1] = it2->second.grids[j2][k2].first, p[2][2] = it2->second.grids[j2][k2].second;
									optimal_dir = furthestdir(p);
									double line_dis = sqrt(dot(optimal_dir, optimal_dir));
									double move = (MIN_DIS - line_dis);
									if (move < 0)continue;
									std::vector<int> a; a.push_back(x); a.push_back(y); a.push_back(j1); a.push_back(k1);
									std::vector<int> b; b.push_back(next_x); b.push_back(next_y); b.push_back(j2); b.push_back(k2);
									layer_offdis.dis[j2][lineincontour[b]] = max(move, layer_offdis.dis[j2][lineincontour[b]]);
								}
							}
						}
						else if (!IsInner[j1] && !IsInner[j2] && j1 == j2) {
							for (int k1 = 0; k1 < it1->second.grids[j1].size(); k1++) {
								for (int k2 = 0; k2 < it2->second.grids[j2].size(); k2++) if (k1 != k2) {
									pa p[3][3], optimal_dir, offdir[3];
									p[1][1] = it1->second.grids[j1][k1].first, p[1][2] = it1->second.grids[j1][k1].second;
									p[2][1] = it2->second.grids[j2][k2].first, p[2][2] = it2->second.grids[j2][k2].second;
									optimal_dir = furthestdir(p);
									double line_dis = sqrt(dot(optimal_dir, optimal_dir));
									double move = (MIN_DIS - line_dis);
									if (move < 0)continue;
									pa p1, p2; p1.first = p[1][1].first - p[1][2].first; p1.second = p[1][1].second - p[1][2].second;
									p2.first = p[2][1].first - p[2][2].first; p2.second = p[2][1].second - p[2][2].second;
									if (dot(p1, p2) > 0)continue;
									std::vector<int> a; a.push_back(x); a.push_back(y); a.push_back(j1); a.push_back(k1);
									std::vector<int> b; b.push_back(next_x); b.push_back(next_y); b.push_back(j2); b.push_back(k2);
									layer_offdis.dis[j2][lineincontour[b]] = max(move, layer_offdis.dis[j2][lineincontour[b]]);
								}
							}
						}
					}
				}
			}
		}
		offdis.push_back(layer_offdis);
		//save grid
		std::vector<pa> tmp_grid;
		for (auto it = layer_grids.begin(); it != layer_grids.end(); it++) tmp_grid.push_back(it->first);
		if (tmp_grid.size())gridshow.push_back(tmp_grid);
	}
	qDebug() << "begin offset" << endl;
	for (int i = 0; i < layers.size(); i++) {
		LayerOffset(offdis[i], i);
	}

	qDebug() << "save slicing to [offsetpieces]" << endl;
	//PathToCutLine();
}
void RenderingWidget::LayerOffset(LayerOffDis layer_offdis, int layernum) {
	std::vector<bool> IsInner;
	for (int j = 0; j < layers[layernum].size(); j++) {
		IsInner.push_back(!JudgeLoopDir(layers[layernum][j]));
	}
	qDebug() << "1 guo";
	Paths final_clippers, subs, tmp_res, tmp_res1, res;
	for (int i = 0; i < layers[layernum].size(); i++) if (IsInner[i]) {
		Path sub, clipper; ClipperOffset offset; Clipper difference; Paths clippers;
		double max_d = 0;
		for (int j = 0; j < layers[layernum][i].size(); j++) {
			double d = layer_offdis.dis[i][j];
			ClipperLib::cInt x = ScaleNumber*layers[layernum][i][j].first;
			ClipperLib::cInt y = ScaleNumber*layers[layernum][i][j].second;
			max_d = max(max_d, d);
			sub << IntPoint(x, y);
			if (max_d > eps)clipper << IntPoint(x, y);

			if (fabs(d) < eps&&max_d > eps) {
				offset.AddPath(clipper, jtMiter, etOpenButt);
				offset.Execute(clippers, max_d*ScaleNumber); ClipperLib::CleanPolygons(clippers);
				difference.AddPaths(clippers, ptClip, TRUE);
				max_d = 0;
				clipper.clear();
				continue;
			}
		}
		if (clipper.size()) {
			offset.AddPath(clipper, jtMiter, etOpenButt);
			offset.Execute(clippers, max_d*ScaleNumber); ClipperLib::CleanPolygons(clippers);
			difference.AddPaths(clippers, ptClip, TRUE);
			clipper.clear();
		}
		if (sub.size())subs.push_back(sub);/* DrawPaths(subs, layernum);*/
		difference.AddPath(sub, ptSubject, TRUE);
		difference.Execute(ctDifference, tmp_res, pftNonZero, pftNonZero);  ClipperLib::CleanPolygons(tmp_res);
		tmp_res1.clear(); for (int j = 0; j < tmp_res.size(); j++)if (fabs(Area(tmp_res[j]))*1.0 / ScaleNumber / ScaleNumber >= MIN_DIS*MIN_DIS*0.2)tmp_res1.push_back(tmp_res[j]);
		for (int j = 0; j < tmp_res1.size(); j++)if (tmp_res1[j].size())res.push_back(tmp_res1[j]);
	}
	qDebug() << "2 guo";
	for (int i = 0; i < layers[layernum].size(); i++) if (!IsInner[i]) {
		Path sub, clipper; ClipperOffset offset; Clipper difference; Paths clippers;
		for (int j = 0; j < layers[layernum][i].size(); j++) {
			ClipperLib::cInt x = ScaleNumber*layers[layernum][i][j].first;
			ClipperLib::cInt y = ScaleNumber*layers[layernum][i][j].second;
			sub << IntPoint(x, y);
		}
		//ClipperLib::CleanPolygon(sub);
		res.push_back(sub);
	}
	qDebug() << "3 guo";
	res_path.push_back(res);
}
void RenderingWidget::PathToCutLine() {
	offsetpieces = new std::vector<std::vector<CutLine>*>[res_path.size()];
	//qDebug() << "size of res_path = " << res_path.size() << endl;
	for (int i = 0; i < res_path.size(); i++)
	{
		//qDebug() << "res_path[" << i << "].size()=" << res_path[i].size() << endl;
		for (int j = 0; j < res_path[i].size(); j++)
		{
			std::vector<CutLine>* circle_list_ = new std::vector<CutLine>;
			if (res_path[i][j].size() <= 2)continue;
			//qDebug() << "res_path[" << i<<","<<j << "].size()=" << res_path[i][j].size() << endl;
			for (int k = 0; k < res_path[i][j].size(); k++) {
				double x1, y1, z1 = i*thickness_, x2, y2, z2 = i*thickness_;
				x1 = res_path[i][j][k].X*1.0 / ScaleNumber; y1 = res_path[i][j][k].Y*1.0 / ScaleNumber;
				x2 = res_path[i][j][(k + 1) % res_path[i][j].size()].X*1.0 / ScaleNumber; y2 = res_path[i][j][(k + 1) % res_path[i][j].size()].Y*1.0 / ScaleNumber;
				point pos1(x1, y1, z1), pos2(x2, y2, z2);
				CutLine new_CutLine(pos1, pos2);
				circle_list_->push_back(new_CutLine);
			}
			offsetpieces[i].push_back(circle_list_);
			//delete[]circle_list_;
		}
	}
}
bool RenderingWidget::IsNestedIn(Path a, Path b) {
	Clipper difference; Paths solutions;
	difference.AddPath(b, ptClip, TRUE); difference.AddPath(a, ptSubject, TRUE);
	difference.Execute(ctDifference, solutions, pftNonZero, pftNonZero);
	return solutions.size() == 0 ? TRUE : FALSE;
}
bool RenderingWidget::ModelThicken(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* tc, int numlayer) {
	/* if (mycutthicken == NULL)mycutthicken = new std::vector<std::vector<CutLine>*>[mycut->GetNumPieces()];*/
	if (tc == NULL)return FALSE;
	ClipperLib::Paths contours, res;
	for (size_t j = 0; j < tc[numlayer].size(); j++) {
		ClipperLib::Path p;
		for (int k = 0; k < (tc[numlayer])[j].size(); k++) {
			ClipperLib::cInt x = ScaleNumber*((tc[numlayer])[j]).at(k).first[0];
			ClipperLib::cInt y = ScaleNumber*((tc[numlayer])[j]).at(k).first[1];
			p << IntPoint(x, y);
		}
		contours.push_back(p);
	}
	//DrawPaths(contours);
	for (int i = contours.size() - 1; i >= 0; i--) {
		if (Orientation(contours[i]) == FALSE) {
			Clipper difference; Clipper intersection; Paths tmpsolution, solution;
			for (int j = 0; j < contours.size(); j++)if (j != i) {
				if (Orientation(contours[j]) == TRUE&&IsNestedIn(contours[i], contours[j])) {
					ClipperOffset co; Paths solution;
					co.AddPath(contours[j], jtMiter, etClosedPolygon);
					co.Execute(solution, -ScaleNumber*MIN_DIS); CleanPolygons(solution);
					intersection.AddPaths(solution, ptClip, TRUE);
					/*DrawPaths(solution);*/
				}
				else if (Orientation(contours[j]) == FALSE) {
					ClipperOffset co; Paths solution;
					co.AddPath(contours[j], jtMiter, etClosedPolygon);
					co.Execute(solution, ScaleNumber*MIN_DIS); CleanPolygons(solution);
					difference.AddPaths(solution, ptClip, TRUE);
					/*DrawPaths(solution);*/
				}
			}
			intersection.AddPath(contours[i], ptSubject, TRUE);
			intersection.Execute(ctIntersection, tmpsolution, pftNonZero, pftNonZero);
			difference.AddPaths(tmpsolution, ptSubject, TRUE);
			difference.Execute(ctDifference, solution, pftNonZero, pftNonZero);
			for (int j = 0; j < solution.size(); j++)res.push_back(solution[j]);
		}
		else if (Orientation(contours[i]) == TRUE) {
			res.push_back(contours[i]);
		}
	}

	DrawPaths(res, numlayer);
	Contour2Layer(res, numlayer);
	return TRUE;
}
bool RenderingWidget::DrawPaths(ClipperLib::Paths contours, int numlayer) {
	for (int i = 0; i < contours.size(); i++) {
		ClipperLib::Path p = contours[i];
		glBegin(GL_LINE_LOOP);
		glColor4f(1.0, 0.0, 0.0, 1.0);
		for (int j = 0; j < p.size(); j++) {
			glVertex3f(p[j].X*1.0 / ScaleNumber, p[j].Y*1.0 / ScaleNumber, numlayer*thickness_);
		}
		glEnd();
	}
	return TRUE;
}
bool RenderingWidget::Contour2Layer(ClipperLib::Paths contours, int numlayer) {
	//if (mycutthicken[numlayer].size() != 0)return FALSE;
	std::vector<CutLine>* circle_list_ = new std::vector<CutLine>;
	for (int i = 0; i < contours.size(); i++) {
		for (int j = 0; j < contours[i].size(); j++) {
			point pos1, pos2;
			pos1[0] = contours[i][j].X*1.0 / ScaleNumber; pos1[1] = contours[i][j].Y*1.0 / ScaleNumber; pos1[2] = numlayer*thickness_;
			pos2[0] = contours[i][j].X*1.0 / ScaleNumber; pos2[1] = contours[i][j].Y*1.0 / ScaleNumber; pos2[2] = numlayer*thickness_;
			CutLine new_CutLine(pos1, pos2);
			circle_list_->push_back(new_CutLine);
		}
	}
	//mycutthicken[numlayer].push_back(circle_list_);
	return TRUE;
}
std::pair<int, int> RenderingWidget::point2grid(std::pair<double, double> p) {
	double x = p.first, y = p.second;
	return std::make_pair(int(x / MIN_DIS - (x < 0 ? 1 : 0)), int(y / MIN_DIS - (y < 0 ? 1 : 0)));
}
bool RenderingWidget::JudgeLoopDir(std::vector<std::pair<double, double> > Loop) {
	ClipperLib::Path p;
	for (int i = 0; i < Loop.size(); i++) {
		ClipperLib::cInt x = ScaleNumber*(int)Loop[i].first;
		ClipperLib::cInt y = ScaleNumber*(int)Loop[i].second;
		p << IntPoint(x, y);
	}
	return Orientation(p);
}
pa RenderingWidget::furthestdir(pa p[3][3]) {
	double l2 = (p[2][1].first - p[2][2].first)*(p[2][1].first - p[2][2].first) + (p[2][1].second - p[2][2].second)*(p[2][1].second - p[2][2].second);
	double t = max(0.0, min(1.0, ((p[1][1].first - p[2][1].first)*(p[2][2].first - p[2][1].first) + (p[1][1].second - p[2][1].second)*(p[2][2].second - p[2][1].second)) / l2));
	pa p11, p12, p21, p22, res, pnear;
	pnear.first = p[2][1].first + t*(p[2][2].first - p[2][1].first);
	pnear.second = p[2][1].second + t*(p[2][2].second - p[2][1].second);
	p11.first = p[1][1].first - pnear.first; p11.second = p[1][1].second - pnear.second;

	t = max(0.0, min(1.0, ((p[1][2].first - p[2][1].first)*(p[2][2].first - p[2][1].first) + (p[1][2].second - p[2][1].second)*(p[2][2].second - p[2][1].second)) / l2));
	pnear.first = p[2][1].first + t*(p[2][2].first - p[2][1].first);
	pnear.second = p[2][1].second + t*(p[2][2].second - p[2][1].second);
	p12.first = p[1][2].first - pnear.first; p12.second = p[1][2].second - pnear.second;
	res = p11.first*p11.first + p11.second*p11.second < p12.first*p12.first + p12.second*p12.second ? p11 : p12;

	l2 = (p[1][1].first - p[1][2].first)*(p[1][1].first - p[1][2].first) + (p[1][1].second - p[1][2].second)*(p[1][1].second - p[1][2].second);
	t = max(0.0, min(1.0, ((p[2][1].first - p[1][1].first)*(p[1][2].first - p[1][1].first) + (p[2][1].second - p[1][1].second)*(p[1][2].second - p[1][1].second)) / l2));
	pnear.first = p[1][1].first + t*(p[1][2].first - p[1][1].first);
	pnear.second = p[1][1].second + t*(p[1][2].second - p[1][1].second);
	p21.first = -1 * (p[2][1].first - pnear.first); p21.second = -1 * (p[2][1].second - pnear.second);
	res = res.first*res.first + res.second*res.second < p21.first*p21.first + p21.second*p21.second ? res : p21;

	l2 = (p[1][1].first - p[1][2].first)*(p[1][1].first - p[1][2].first) + (p[1][1].second - p[1][2].second)*(p[1][1].second - p[1][2].second);
	t = max(0.0, min(1.0, ((p[2][2].first - p[1][1].first)*(p[1][2].first - p[1][1].first) + (p[2][2].second - p[1][1].second)*(p[1][2].second - p[1][1].second)) / l2));
	pnear.first = p[1][1].first + t*(p[1][2].first - p[1][1].first);
	pnear.second = p[1][1].second + t*(p[1][2].second - p[1][1].second);
	p22.first = -1 * (p[2][2].first - pnear.first); p22.second = -1 * (p[2][2].second - pnear.second);
	res = res.first*res.first + res.second*res.second < p22.first*p22.first + p22.second*p22.second ? res : p22;
	return res;
	//return make_pair(res.first/sqrt(res.first*res.first+res.second*res.second),res.second/sqrt(res.first*res.first+res.second*res.second));
}
pa RenderingWidget::rotate(pa p, double angle) {
	pa res;
	res.first = p.first*cos(angle) - p.second*sin(angle);
	res.second = p.first*sin(angle) + p.second*cos(angle);
	return res;
}
double RenderingWidget::dot(pa p1, pa p2) {
	return p1.first*p2.first + p2.second*p1.second;
}
pa RenderingWidget::normailize(pa p) {
	return std::make_pair(p.first / sqrt(p.first*p.first + p.second*p.second), p.second / sqrt(p.first*p.first + p.second*p.second));
}
void RenderingWidget::CancelBugCut(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_, int i, int j) {
	if (!pieces_[i][j].size())return;
	if (pieces_[i][j].at(0).first != pieces_[i][j].at(pieces_[i][j].size() - 1).second) {
		std::pair<Vec3f, Vec3f> new_CutLine(pieces_[i][j].at(pieces_[i][j].size() - 1).second, pieces_[i][j].at(0).first);
		pieces_[i][j].push_back(new_CutLine);
	}
}
