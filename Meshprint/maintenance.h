#ifndef MYDIALOG_H  
#define MYDIALOG_H 

#include <QDialog>  
#include "QProgressBar"
namespace Ui {
	class MyDialog;
}

class MyDialog : public QDialog  
{  
    Q_OBJECT  
public:
	explicit MyDialog(QWidget *parent = 0);
	~MyDialog();

signals:
	void SendMsg(QString);
	//void sendUpdate();
public slots:	
	void on_updateButton_clicked();
	void on_maintenanceButton_clicked();
	void on_clearButton_clicked();
	void on_markcolorbutton_clicked();
	void on_createTriangle_clicked();
	void on_repairhole_clicked();
	void on_intersectAndoverlapCheckBox();
	void on_markbadisdebutton_clicked();

	void DialogClose();
	void receiveData(QString);


private:
	Ui::MyDialog *ui;

};  
#endif // MYDIALOG_H  