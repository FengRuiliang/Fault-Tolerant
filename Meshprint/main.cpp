#include "meshprint.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Meshprint w;
	w.show();
	return a.exec();
}
