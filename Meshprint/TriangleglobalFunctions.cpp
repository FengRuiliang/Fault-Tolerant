#include "TriangleglobalFunctions.h"
//by Triangle Maintenance
int num = 0;		//充当计数的变量，还充当给三角面ID 赋值 在读取它们(面片)的时候
int insStartPoint = 0;	//相交边的start端点
int insEndPoint = 0;		//相交边的End端点
int insRedunPoint = 0;	//相交边的冗余点

float INSPOINT_X;				//做全局变量来用，存储边相交的交点X坐标
float INSPOINT_Y;				//做全局变量来用，存储边相交的交点Y坐标
float INSPOINT_Z;				//做全局变量来用，存储边相交的交点Z坐标

bool actionMaintenanceFlag = true;

int intersect_num = 0;
int overlap_num = 0;
int reverse_num = 0;
int badside_num = 0;
int hole_num = 0;
int shell_num = 0;

bool holeMark = false;
bool badsideMark = false;