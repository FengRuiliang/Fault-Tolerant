#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_oepndialog.h"
#include "HE_mesh/Mesh3D.h"
class MeshInfo :
	public QDialog
{
	Q_OBJECT
public:
	MeshInfo(QDialog *parent = Q_NULLPTR);
	~MeshInfo();

	void setMeshinfo(int param1, std::vector<Vec3f> param2);
public:
	Ui::open_dia ui;
};

