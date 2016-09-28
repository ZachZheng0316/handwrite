#include "dmath.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

//弧度转角度
float AngleFromPI(float pi)
{
	return (pi * 180.0 / PI);
}
//角度转弧度
float PIFromAngle(float alpha)
{
	return (alpha * PI / 180.0);	
}

//关节角度转化为舵机刻度
int servoPosKFromJointAngle(int id, float jointAngle)
{
	float diffAngle;
	int posK;
	
	//计算出舵机角度差
	diffAngle = (jointAngle - IDAngle) / REDUCTION_RATE;
	
	//计算舵机刻度
	if(MXID1 == id)
		posK = (int)(ID1POSK * 1.0 - diffAngle / PositionUnit + 0.35);
	else
		posK = (int)(ID2POSK * 1.0 + diffAngle / PositionUnit + 0.35);
	
	return posK;
}
//舵机刻度转化为关节角度
float jointAngleFromServoPosK(int id, int servoPosK)
{
	float diffAngle, jointAngle;
	
	if(MXID1 == id) {
		//计算出关节角度差
		diffAngle = (float)(ID1POSK - servoPosK) * PositionUnit * REDUCTION_RATE;
		jointAngle = diffAngle + IDAngle;
	}
	else{
		diffAngle = (float)(servoPosK - ID2POSK) * PositionUnit * REDUCTION_RATE;
		jointAngle = IDAngle + diffAngle;
	}
	
	return jointAngle;
}

//关节角速度转化为舵机速度刻度
int servoSpeKFromJointAngleSpe(float jointAngleSpe)
{
	float radSpe, rpm;
	int speK;
	
	//关节角速度转化为关节弧速度
	radSpe = PIFromAngle(fabs(jointAngleSpe));
	//关节弧速度转化为rpm
	rpm = radSpe * 60 / (2 * PI);
	//关节rpm转化为舵机rpm
	rpm /= REDUCTION_RATE;
	//舵机rpm转化为舵机速度
	speK = (int)(rpm / SpeedUnit + 0.35);
	if(speK <= 1) speK = 3;
	else if(speK >= 1023) speK = 1020;
	else{
	}
	
	return speK;
	
}
//舵机速度刻度转化为关节角速度
//关节的速度正负由运动方向决定
float jointAngleSpeFromServoSpeK(int servoSpeK)
{
	float radSpe, rpm;
	float angleSpe;
	
	//舵机速度刻度转化为舵机rpm
	rpm = servoSpeK * SpeedUnit;
	//舵机rpm转化为舵机弧速度
	radSpe = rpm * PI * 2.0 / 60.0;
	//舵机弧速度转化舵机角速度
	angleSpe = AngleFromPI(radSpe);
	//舵机角速度转化为关节角速度
	return (angleSpe * REDUCTION_RATE);
}


//把inkscape的基准坐标转化为机械臂的工作坐标
void coor_translate(float *originanal, float *result)
{
	result[0] = originanal[0] + STARTX;
	result[1] = originanal[1] + STARTY;
}

//计算两点之间的距离
float disPoint(float p1[], float p2[])
{
	float disx, disy, k;

	disx = p1[0] - p2[0];
	disy = p1[1] - p2[1];
	disx = pow(disx, 2);
	disy = pow(disy, 2);
	k = sqrt(disx + disy);

	return k;
}


//计算三角形a对边的角度
float cal_triangle_angle(float a, float b, float c)
{
	float cos_angle, angle;
	float temp1, temp2;

	temp1 = pow(b, 2);
	temp1 += pow(c, 2);
	temp1 -= pow(a, 2);
	temp2 = 2 * b * c;
	cos_angle = temp1 / temp2;

	angle = acos(cos_angle);

	//把弧度转化为角度
	angle = AngleFromPI(angle);

	return angle;
}

//微妙延迟函数
void delay_us(unsigned long usec)
{
	struct timeval start, end;
	unsigned long diff;
	gettimeofday(&start, NULL);
	do{
		gettimeofday(&end, NULL);
		diff = 1000000*(end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
	}while(diff <= usec);
}
