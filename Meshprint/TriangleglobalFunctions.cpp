#include "TriangleglobalFunctions.h"
//by Triangle Maintenance
int num = 0;		//�䵱�����ı��������䵱��������ID ��ֵ �ڶ�ȡ����(��Ƭ)��ʱ��
int insStartPoint = 0;	//�ཻ�ߵ�start�˵�
int insEndPoint = 0;		//�ཻ�ߵ�End�˵�
int insRedunPoint = 0;	//�ཻ�ߵ������

float INSPOINT_X;				//��ȫ�ֱ������ã��洢���ཻ�Ľ���X����
float INSPOINT_Y;				//��ȫ�ֱ������ã��洢���ཻ�Ľ���Y����
float INSPOINT_Z;				//��ȫ�ֱ������ã��洢���ཻ�Ľ���Z����

bool actionMaintenanceFlag = true;

int intersect_num = 0;
int overlap_num = 0;
int reverse_num = 0;
int badside_num = 0;
int hole_num = 0;
int shell_num = 0;

bool holeMark = false;
bool badsideMark = false;