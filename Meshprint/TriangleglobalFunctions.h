#ifndef __TRIANGLEGLOBALFUNCATIONS_
#define __TRIANGLEGLOBALFUNCATIONS_
//by Triangle Maintenancer
extern int num;				//零碎的一些记录
extern int insStartPoint;	//相交边的start端点
extern int insEndPoint;		//相交边的End端点
extern int insRedunPoint;	//相交边的冗余点

extern float INSPOINT_X;	//做全局变量来用，存储边相交的交点X坐标
extern float INSPOINT_Y;	//做全局变量来用，存储边相交的交点Y坐标
extern float INSPOINT_Z;	//做全局变量来用，存储边相交的交点Z坐标

extern bool actionMaintenanceFlag;

extern int intersect_num;	//dialog中显示相交面的数量
extern int overlap_num;		//dialog中显示重叠面的数量
extern int reverse_num;		//dialog中显示反向面的数量
extern int badside_num;		//dialog中显示坏边的数量
extern int hole_num;		//dialog中显示孔洞的数量
extern int shell_num;		//dialog中显示壳体的数量

extern bool holeMark;		//标记孔洞
extern bool badsideMark;	//标记坏边


#endif
