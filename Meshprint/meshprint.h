#ifndef MESHPRINT_H
#define MESHPRINT_H

#include <QtWidgets/QMainWindow>
#include "ui_meshprint.h"

class Meshprint : public QMainWindow
{
	Q_OBJECT

public:
	Ui::MeshprintClass ui;

	Meshprint(QWidget *parent = 0);
	~Meshprint();

private:
};

class Stldialog : public QMainWindow
{
	//Q_OBJECT

public:
	Ui::MeshprintClass ui;

	Stldialog(QWidget *parent = 0) {};
	~Stldialog() {};
};

#endif // MESHPRINT_H
