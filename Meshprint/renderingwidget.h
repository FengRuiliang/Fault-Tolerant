#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H
//////////////////////////////////////////////////////////////////////////
#include <QOpenGLWidget>
#include "globalFunctions.h"
#include <QEvent>
#include "HE_mesh/Vec.h"
#include "Hatch.h"
using trimesh::vec;
using trimesh::point;
typedef trimesh::vec3  Vec3f;

class Meshprint;
class MainWindow;
class CArcBall;
class Mesh3D;
class Support;
class SliceCut;
class Hatch;
class HatchChessboard;
enum hatchType
{
	NONE=0,
	CHESSBOARD ,
	OFFSETFILLING,
	MEANDER,
	STRIP
	
};
class RenderingWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	Meshprint* parent;

	RenderingWidget(QWidget *parent, MainWindow* mainwindow=0);
	~RenderingWidget();

protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	void timerEvent(QTimerEvent *e);

	// mouse events
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseDoubleClickEvent(QMouseEvent *e);
	void wheelEvent(QWheelEvent *e);

public:
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);

signals:
	//void meshInfo(int, int, int);
	//void operatorInfo(QString);

private:
	void Render();
	void SetLight();
	bool is_draw_region_;
	bool is_draw_support_;
	public slots:
	void SetBackground();
	void ReadMesh();
	void WriteMesh();
	void Export();
	void LoadTexture();
	void SetSliceCheckId(int id);
	void CheckDrawPoint();
	void CheckDrawEdge();
	void CheckDrawFace();
	void CheckLight();
	void CheckGrid();
	void CheckDrawTexture();
	void CheckDrawAxes();
	void CheckDrawCutPieces();
	void Checkmoduletranslate();
	void CheckSetFace();

	void CheckRegion(bool bv);
	
	void CheckSupport(bool bv);
	void CheckRotateModel(bool bv);
	// support operators
	void AddPointSupport();
	void AddLineSupport();
	void DeleteSupport();
	void AutoSupport();
	

	void setPointD(double diameter);
	void setPointH(double diameter);
	void setLineD(double diameter);
	void setLineH(double diameter);
	
	void setThreshold(double threshold) { THRESHOLD = cos(3.1415926 * threshold / 180); };
	void setGap(double gap) { GAP = gap; };
	void setSeglength(double length) { SEGLENGTH = length; };
	void setReso(double reso) { RESO = reso; };
	void setVerticalgap(double verticalgap) { VERTICALGAP = verticalgap; };
	void setfieldWidth(double width);
	void setfieldHeight(double height);
	void setlineOverlap(int lineoverlap);
	void setfieldOverlap(double fieldoverlap);
	void setHatchDis(double value);
	void setThickness(double thick);
	void setAngle(int angle);
	void SetAllHatch(bool bv) { is_show_all = bv; }
	void FindRegion();
	void setFildID(int id) {
		fildID = id; 
		update();
	};
	
	//hatch operator
	void setHatchType(int type_) 
	{ 
		hatch_type_=(hatchType)type_; 
	};
	void setLaserPower(int power) { laser_power_hatch_ = power; };
	void setLaserSpeed(int speed) { laser_speed_hatch_ = speed; };
	void setLaserPowerPolygon(int power) { laser_power_polygon_ = power; };
	void setLaserSpeedPolygon(int speed) { laser_speed_polygon_ = speed; };
	void setOffset(double dis)
	{
		offset_dis_ = dis; 
	};
	// SLICE operators
	void cutinPieces();
	void cutinPiecesSup();
	void renderdoHatch();

	void objectTransformation(float * matrix);
	void Translation();
	void SetDirection(int id);
	void setThickness(float thickness){thickness_ = thickness;};
	public slots:
	void SelectFace(int x, int y);
	void SetScaleT(double size) {
		scaleT = size;
	}
	void SclaleModel() {
		ptr_mesh_->scalemesh(scaleT);
	};

private:
	void DrawAxes(bool bv);
	void DrawPoints(bool);
	void DrawEdge(bool);
	void DrawFace(bool);
	void DrawSupport(bool bv);
	void DrawSupFace(bool bv);
	void DrawOctree(bool bv);
	void DrawSegments(bool bv);
	void DrawInnerLoop(bool bv);
	void DrawOuterLoop(bool bv);
	void DrawSupFaceZero(bool bv);
	void DrawTexture(bool);
	void DrawGrid(bool bV);
	void DrawCutPieces(bool bv);
	void DrawCutPiecesSup(bool bv);
	void DrawHatch(bool bv);
	void DrawHatchsup(bool bv);
public:
	MainWindow					*ptr_mainwindow_;
	CArcBall					*ptr_arcball_;
	CArcBall					*ptr_arcball_module_;
	Mesh3D						*ptr_mesh_;
	SliceCut					*mycut;
	SliceCut					*mycutsup;
	Hatch						*myhatch;
	Hatch						*myhatchsup;
	//hatch operator
	hatchType					hatch_type_;
	// my Support-operator
	//Mesh3D						*ptr_mesh_support_;
	Support						*ptr_support_;
	bool						isAddPoint;
	bool						isAddLine;
	bool						isDelete;
	std::vector<int>			delete_points_;
	std::vector<Vec3f>			line_points_;

	// Texture
	GLuint						texture_[1];
	bool						is_load_texture_;

	// eye
	GLfloat						eye_distance_;
	point						eye_goal_;
	vec							eye_direction_;
	QPoint						current_position_;

	// Render information
	bool						is_draw_point_;
	bool						is_draw_edge_;
	bool						is_move_module_;
	bool						is_draw_face_;
	bool						is_draw_texture_;
	bool						has_lighting_;
	bool						is_draw_axes_;
	bool						is_draw_grid_;
	bool						is_draw_cutpieces_;
	bool						is_select_face;
	bool						is_draw_hatch_;
	bool						is_show_all;
private:
	int slice_check_id_;
	int current_face_;
	std::vector<bool> faceNeedSupport;
	std::vector<int> BoundaryEdge;
	void ClearSlice();
	void ClearHatch();
	void ClearSupport();
};

#endif // RENDERINGWIDGET_H
