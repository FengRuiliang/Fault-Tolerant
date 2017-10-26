#include "MeshInfo.h"


MeshInfo::MeshInfo(QDialog *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}


MeshInfo::~MeshInfo()
{
}

void MeshInfo::setMeshinfo(int param1, std::vector<Vec3f> param2)
{
	QString a = " has ";
	ui.label->setText(a);
}
