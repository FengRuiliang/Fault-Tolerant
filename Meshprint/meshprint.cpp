#include "meshprint.h"

Meshprint::Meshprint(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	ui.openGLWidget->parent = this;
}

Meshprint::~Meshprint()
{

}
