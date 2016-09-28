#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "kinesiology.h"
#include "dmath.h"

//固定坐标
//static float A[2] = {0.0, 0.0};
static float B[2] = {-J/2, 0.0};
static float C[2] = {J/2, 0.0};

//常用变量
static float XY[2] = {0.0, 0.0};
static float ANGLE[2] = {0.0, 0.0};

//逆运动学和速度
//已知坐标求关节角度
void cal_xy_angle(float *xy, float *angle)
{
	float k, l;
	float EBF, DCF;
	float GBF, HCF;

	//设置此时的坐标
	XY[0] = xy[0];
	XY[1] = xy[1];

	//计算k, l
	k = disPoint(XY, B);
	l = disPoint(XY, C);
	//printf("(k l)(%f %f)\n", k, l);

	//计算角度EBF、DCF
	EBF = cal_triangle_angle(H, G, k);
	DCF = cal_triangle_angle(H, G, l);
	//printf("(EBF, DCF)(%f, %f)\n", EBF, DCF);

	//计算GBF, HCF
	GBF = (XY[0] - B[0]) / (XY[1] - B[1]);
	HCF = (XY[0] - C[0]) / (XY[1] - C[1]);
	GBF = atan(GBF);
	HCF = atan(HCF);
	GBF = AngleFromPI(GBF);
	HCF = AngleFromPI(HCF);
	//printf("(GBF HCF)(%f %f)\n", GBF, HCF);

	//关节角度
	angle[0] = EBF - GBF;
	angle[1] = DCF + HCF;

	//更新当前关节角度
	ANGLE[0] = angle[0];
	ANGLE[1] = angle[1];
}

//已知关节角度求坐标
void cal_angle_xy(float *angle, float *xy)
{
	float E[2], D[2];
	float a, b, c, d;
	float m, n, temp1, temp2;

	//更新当前角度
	ANGLE[0] = angle[0];
	ANGLE[1] = angle[1];

	//E, D的坐标
	E[0] = B[0] - G * sin(PIFromAngle(ANGLE[0]));
	E[1] = B[1] + G * cos(PIFromAngle(ANGLE[0]));
	D[0] = C[0] + G * sin(PIFromAngle(ANGLE[1]));
	D[1] = C[1] + G * cos(PIFromAngle(ANGLE[1]));
	//printf("E(%f %f) D(%f %f)\n", E[0], E[1], D[0], D[1]);

	//start cal
	a = D[0] - E[0];
	b = E[0] + D[0];
	c = D[1] - E[1];
	d = E[1] + D[1];
	temp1 = a * b + c * d;
	temp2 = 2 * a;
	m = temp1 / temp2;
	n = c / a;

	a = pow(n, 2) + 1;
	temp1 = E[0] * n;
	temp1 -= m * n;
	b = temp1 - E[1];
	temp1 = pow(H, 2);
	temp1 -= pow(E[1], 2);
	temp2 = m - E[0];
	temp2 = pow(temp2, 2);
	c = temp1 - temp2;

	temp1 = pow(b, 2);
	temp1 += a * c;
	temp1 = sqrt(temp1);
	temp1 -= b;
	temp2 = a;
	xy[1] = temp1 / temp2;
	xy[0] = m - n * xy[1];

	//更新当前坐标
	XY[0] = xy[0];
	XY[1] = xy[1];
}


//已知线速度求关节角速度
void cal_lspe_aspe(float *lspe, float *aspe)
{
	float a, b, c, d;
	float temp1, temp2;
	float alpha, beta;
	float Ex, Ey, Dx, Dy;
	
	alpha = PIFromAngle(ANGLE[0]);
	beta = PIFromAngle(ANGLE[1]);
	
	Ex = B[0] - G * sin(alpha);
	Ey = B[1] + G * cos(alpha);
	Dx = C[0] + G * sin(beta);
	Dy = C[1] + G * cos(beta);
	
	a = 2 * (XY[0] - Ex);
	b = 2 * (XY[1] - Ey);
	c = 2 * (XY[0] - Dx);
	d = 2 * (XY[1] - Dy);
	
	temp1 = 0.0 - (a * lspe[0] + b * lspe[1]);
	temp2 = a * G * cos(alpha);
	temp2 += b * G * sin(alpha);
	aspe[0] = temp1 / temp2;
	
	temp1 = c * lspe[0] + d * lspe[1];
	temp2 = c * G * cos(beta);
	temp2 -= d * G * sin(beta);
	aspe[1] = temp1 / temp2;

	//把弧速度转化为角速度
	aspe[0] = AngleFromPI(aspe[0]);
	aspe[1] = AngleFromPI(aspe[1]);
}

//已知关节角速度求线速度
void cal_aspe_lspe(float *aspe, float *lspe)
{
	float a, b, c, d, m, n;
	float temp1, temp2;
	float alpha, beta;
	float Wa, Wb;
	float Ex, Ey, Dx, Dy;
	
	//关节角速度转化为关节弧速度
	Wa = PIFromAngle(aspe[0]);
	Wb = PIFromAngle(aspe[1]);
	alpha = PIFromAngle(ANGLE[0]);
	beta = PIFromAngle(ANGLE[1]);
	
	Ex = B[0] - G * sin(alpha);
	Ey = B[1] + G * cos(alpha);
	Dx = C[0] + G * sin(beta);
	Dy = C[1] + G * cos(beta);
	
	a = 2 * (XY[0] - Ex);
	b = 2 * (XY[1] - Ey);
	c = 2 * (XY[0] - Dx);
	d = 2 * (XY[1] - Dy);
	
	temp1 = a * G * cos(alpha);
	temp1 += b * G * sin(alpha);
	temp1 *= Wa;
	m = 0.0 - temp1;
	temp2 = c * G * cos(beta);
	temp2 -= d * G * sin(beta);
	n = temp2 * Wb;
	
	temp1 = m * d - n * b;
	temp2 = a * d - c * b;
	lspe[0] = temp1 / temp2;
	
	temp1 = m * c - n * a;
	temp2 = b * c - a * d;
	lspe[1] = temp1 / temp2;
}

