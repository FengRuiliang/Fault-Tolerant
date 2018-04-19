#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H
//////////////////////////////////////////////////////////////////////////
#include <QOpenGLWidget>
#include "globalFunctions.h"
#include <QEvent>
#include "HE_mesh/Vec.h"
#include "HE_mesh/Mesh3D.h"
#include "SliceCut.h"
#include "Hatch.h"
using trimesh::vec;
using trimesh::point;
typedef trimesh::vec3  Vec3f;
class Meshprint;
class MainWindow;
class CArcBall;
class Mesh3D;
class SliceCut;
class Support;
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
	void sendMsgtoDialog(QString);
	//void meshInfo(int, int, int);
	//void operatorInfo(QString);

private:
	void Render();
	void SetLight();

	bool is_draw_support_;
	int counter_;
	public slots:
	void ResetView();
	void RecvMsg(QString);
	void ApplyMaintenance();
	void SetBackground();
	void SetSliceCheckId(int id);
	void setFildID(int id);
	void SetLineId(int id);
	void ReadMesh();
	void WriteMesh();
	void CheckDrawPoint();
	void CheckDrawEdge();
	void CheckDrawFace();
	void CheckLight();
	void CheckGrid();
	void CheckDrawTexture();
	void CheckDrawAxes();
	void check_support(bool bv);
	void DoSlice();
	void FillPath();
	void add_support();
	void FindNarrowBand();
	void LayerOffset(LayerOffDis layer_offdis, int layernum);
	void PathToCutLine();
	bool IsNestedIn(Path a, Path b);
	bool ModelThicken(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* tc, int numlayer);
	bool DrawPaths(ClipperLib::Paths contours, int numlayer);
	bool Contour2Layer(ClipperLib::Paths contours, int numlayer);
	std::pair<int, int> point2grid(std::pair<double, double> p);
	bool JudgeLoopDir(std::vector<std::pair<double, double>> Loop);
	pa furthestdir(pa p[3][3]);
	pa rotate(pa p, double angle);
	double dot(pa p1, pa p2);
	pa normailize(pa p);
	void CancelBugCut(std::vector<std::vector<std::pair<Vec3f, Vec3f>>>* pieces_, int i, int j);
	void setHatchType(int type_);
	void setThickness(double t) { 
		thickness_ = t; }
private:
	void DrawAxes(bool bv);
	void DrawPoints(bool);
	void DrawEdge(bool);
	void DrawFace(bool);
	void DrawTexture(bool);
	void DrawGrid(bool bV);
	void DrawSlice(bool bv);
	void DrawHatch(bool bv);

	void draw_support_aera(bool bv);
	
public:
	MainWindow					*ptr_mainwindow_;
	CArcBall					*ptr_arcball_;
	Mesh3D						*ptr_mesh_;
	SliceCut					*ptr_slice_;
	Hatch						*ptr_hatch_;
	hatchType					hatch_type_;
	Support						*ptr_support_;
	// Texture
	GLuint						texture_[1];
	bool						is_load_texture_;

	// eye
	GLfloat						eye_distance_{5.0};
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
	int							slice_check_id_;
	int field_id{0};
	int line_id_{0};
public:
	std::vector<Paths>			res_path;
	std::vector<std::vector<CutLine>*>* offsetpieces;
	std::vector<std::vector<std::vector<std::pair<double, double> > > > layers;
	std::vector<std::vector<Vec3f>>			offset_points;
	std::vector<New2Origin>     lines_new2origin;
	std::vector<int>			delete_points_;
	std::vector<Vec3f>			line_points_;
	std::vector<std::pair<int, int>>	cutline_points_;
	std::vector<std::vector<pa>> gridshow;
	std::vector<LayerOffDis> offdis;
	std::vector<std::vector<CutLine>>  offsetcircul_;
	std::vector<std::vector<CutLine>>  offset_cutline;

	std::vector<Vec3f>  points;
};

#endif // RENDERINGWIDGET_H
