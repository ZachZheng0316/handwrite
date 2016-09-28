#ifndef _DMATH_HEADER
#define _DMATH_HEADER

#ifdef __cplusplus
extern "C" {
#endif

//MX28AT舵机属性值(12V)
#define AngleRange	  (float)(360.0)       //舵机运动的角度范围
#define PositionRange (float)(4095.0)	   //舵机运动的刻度范围
#define PositionUnit  (float)(0.087912088) //舵机每个刻度对应0.088°
#define SpeedRange    (float)(117.07)      //速度最大为114rpm
#define SpeedUnit     (float)(0.114437927) //每个刻度对应0.111rpm
#define SpeedKRange	  (float)(1023.0)      //舵机速度刻度范围

//常量
#define PI (float)(3.141592653) //常数PI的值

//舵机编号
#define MXID1 (int)(1)
#define MXID2 (int)(2)

//基准角度(°)
#define IDAngle	 (float)(0.00)
//#define ID1Angle (float)(0.00)
//#define ID2Angle (float)(0.00)

//基准刻度
#define ID1POSK  (3537)
#define ID2POSK  (605)

//减速比
#define REDUCTION_RATE (float)(0.500)  //1:2.0

//移动极限(°)
#define MINANGLE  (float)(-50.0) //最小运动范围
#define MAXANGLE  (float)(110.0) //最大运动范围

//机械臂工作范围
#define HEIGHT 	(float)(160.00)
#define WIDTH	(float)(160.00)
#define STARTX  (float)(-80.00)
#define STARTY	(float)(145.00)

//工具函数
float AngleFromPI(float pi);         //弧度转角度
float PIFromAngle(float alpha);      //角度转弧度

int servoPosKFromJointAngle(int id, float jointAngle); //关节角度转化为舵机刻度
float jointAngleFromServoPosK(int id, int servoPosK);  //舵机刻度转化为关节角度

int servoSpeKFromJointAngleSpe(float jointAngleSpe); //关节角速度转化为舵机速度刻度
float jointAngleSpeFromServoSpeK(int servoSpeK);	 //舵机速度刻度转化为关节角速度

void coor_translate(float *originanal, float *result);//把inkscape的基准坐标转化为机械臂的工作坐标

float disPoint(float p1[], float p2[]); //计算两点之间的距离

float cal_triangle_angle(float a, float b, float c);//计算三角形a对边的角度

void delay_us(unsigned long usec); //微妙延迟函数

#ifdef __cplusplus
}
#endif

#endif
