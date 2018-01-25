#include "maintenance.h"  
#include "ui_maintenance.h"  
#include "globalFunctions.h"
#include "QDebug"

MyDialog::MyDialog(QWidget *parent) :
	QDialog(parent),ui(new Ui::MyDialog)
{
	ui->setupUi(this);
}

MyDialog::~MyDialog()
{
	delete ui;
}

void MyDialog::on_updateButton_clicked()
{	
	//重置显示记录
	intersect_num = overlap_num = reverse_num = badside_num = hole_num  = 0;

	if (ui->intersect_checkBox->isChecked()) {
		emit SendMsg(tr("intersectDetection"));
	}
	if (ui->overlap_checkBox->isChecked())
	{
		emit SendMsg(tr("overlapDetection"));
	}
	if (ui->reverse_checkBox->isChecked())
	{
		emit SendMsg(tr("reverseDetection"));
	}
	if (ui->badside_checkBox->isChecked())
	{
		emit SendMsg(tr("badsideDetection"));
	}
	if (ui->hole_checkBox->isChecked())
	{
		holeMark = true;
		emit SendMsg(tr("holeDetection"));
	}
	else
	{
		holeMark = false;
	}
	if (ui->shell_checkBox->isChecked())
	{
		emit SendMsg(tr("shellDetection"));
	}
	emit SendMsg(tr("update"));
}
//自动修复按钮
void MyDialog::on_maintenanceButton_clicked()
{
	emit SendMsg(tr("maintenance"));
}
//清除标记按钮
void MyDialog::on_clearButton_clicked()
{
	emit SendMsg(tr("clearmark"));
}
//相交标记按钮
void MyDialog::on_markcolorbutton_clicked()
{
	emit SendMsg(tr("intersectmark"));
}
//创建三角面片按钮
void MyDialog::on_createTriangle_clicked()
{
	emit SendMsg(tr("createtriangle"));
}
//孔洞修复按钮
void MyDialog::on_repairhole_clicked()
{
	emit SendMsg(tr("repairhole"));
}
//交叉和重叠的复选框关联
void MyDialog::on_intersectAndoverlapCheckBox() {
	if (ui->intersect_checkBox->isChecked())
	{
		ui->overlap_checkBox->setChecked(true);
	}
	else
	{
		ui->overlap_checkBox->setChecked(false);
	}
}

//标记坏边
void MyDialog::on_markbadisdebutton_clicked()
{
	emit SendMsg(tr("markbadside"));
}

//控制maintenance只产生一个Dialog
void MyDialog::DialogClose() {
	actionMaintenanceFlag = true;
}
//pass information for sendMsgtoDialog
void MyDialog::receiveData(QString str)
{	//选中交叉checkBox
	if (ui->intersect_checkBox->isChecked())
	{
		ui->intersect_label->setText(QString::number(intersect_num, 10));
	}
	else
	{	//如果进行两次检测，第二次取消勾选，应当重置
		ui->intersect_label->setText(QString::number(0, 10));
	}
	//选中重复checkBox
	if (ui->overlap_checkBox->isChecked())
	{
		ui->overlap_label->setText(QString::number(overlap_num, 10));
	}
	else
	{	
		ui->overlap_label->setText(QString::number(0, 10));
	}
	//选中反向checkBox
	if (ui->reverse_checkBox->isChecked())
	{
		ui->reverse_label->setText(QString::number(reverse_num, 10));
	}
	else
	{
		ui->reverse_label->setText(QString::number(0, 10));
	}
	//选中坏边checkBox
	if (ui->badside_checkBox->isChecked())
	{
		ui->badside_label->setText(QString::number(badside_num, 10));
	}
	else
	{
		ui->badside_label->setText(QString::number(0, 10));
	}
	//选中孔洞checkBox
	if (ui->hole_checkBox->isChecked())
	{
		ui->hole_label->setText(QString::number(hole_num, 10));
	}
	else
	{
		ui->hole_label->setText(QString::number(0, 10));
	}
	//选中壳体checkBox
	if (ui->shell_checkBox->isChecked())
	{
		ui->shell_label->setText(QString::number(shell_num, 10));
	}
	else
	{
		ui->shell_label->setText(QString::number(0, 10));
	}
}
