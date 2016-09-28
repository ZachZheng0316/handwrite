#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include "MX28AT.h"
#include "dmath.h"
#include "kinesiology.h"

#define SPEEDMIN (10.0) //10.0mm/s
#define SPEEDMAX (100.0)//100.0mm/s
#define SPEEDMX  (100)  //95.0mm/s
#define TMin	 (10)   //10ms
#define TMax	 (100)  //100ms
#define TM		 (16)   //30ms数据传输所需的时间

static float Current_xy[2];

void tran_coor_to_k(float *xyz, int *k); //坐标转化为刻度值
//已知坐标线速度，求舵机刻度和速度
void tran_covec_to_K(float *xy, float *lineVec, int *posK, int *vecK);

void initial_sys(); //系统初始化
void tra_ink_to_sour(char *sourcefile, char *resultfile);
void tra_sour_to_coor(char *tra1, char *tra2);
void tra_coor_to_spe(char *coor, char *spe);
void tra_coor_to_posk(char *coor, char *spe, char *posk);
void excute_posk(char *tra_k);

void set_default_pos();

int main()
{
	char inkfile[] = "A_data.txt", coorfile[] = "source.txt";
	char tra_coor[] = "tracoor.txt", tra_spe[] = "traspe.txt", tra_k[] = "trak.txt";
	
	initial_sys();
	
	tra_ink_to_sour(inkfile, coorfile);
	tra_sour_to_coor(coorfile, tra_coor);
	tra_coor_to_spe(tra_coor, tra_spe);
	tra_coor_to_posk(tra_coor, tra_spe, tra_k);
	
	printf("start ...\n");
	getchar();
	//while(1)
	excute_posk(tra_k);
	
	relax_arms();
	
	return 1;
}

//系统初始化
void initial_sys()
{
	int deviceIndex = 0, baudnum = 1, value[2];
	//float angle[2];

	printf("\ninitial_sys ...\n");
	if(!dxl_initialize(deviceIndex, baudnum)){
		printf("Failed to open USB2Dynamixel!\n");
		exit(1);
	}
	else
		printf("Succeed to open USB2Dynamixel!\n");
		
	//属性初始化
	//PID control
	value[0] = 46; value[1] = 46;
	set_many_servo_byte(P_Gain, value);
	value[0] = 32; value[1] = 32;
	set_many_servo_byte(I_Gain, value);
	value[0] = 30; value[1] = 30;
	set_many_servo_byte(D_Gain, value);
	
	//enable
	set_enable_arms();
	
	//set pen up
	set_pen_height(1);
	
	/*
	//cal current position
	//get preServoK
	value[0] = get_one_servo_word(1, Present_Position);
	value[1] = get_one_servo_word(2, Present_Position);
	//get preAngle
	angle[0] = jointAngleFromServoPosK(1, value[0]);
	angle[1] = jointAngleFromServoPosK(2, value[1]);
	//get prePosition
	cal_angle_xy(angle, Current_xy);
	*/
	Current_xy[0] = 0.00; Current_xy[1] = 145;
	
	set_default_pos();
}

void tra_ink_to_sour(char *sourcefile, char *resultfile)
{
	char flag, penFlag;
	float ps[2], pd[2], cur[2];
	FILE *fp_source, *fp_result;
	
	fp_source = fopen(sourcefile, "r");
	fp_result = fopen(resultfile, "w");
	
	//set cur[2]
	cur[0] = Current_xy[0];
	cur[1] = Current_xy[1];
	
	while(!feof(fp_source)){
		if(!fscanf(fp_source, "%c (%f %f)\n", &flag, &ps[0], &ps[1])) {
			printf("err:tra_ink_to_sour failed!\n");
			exit(1);
		}
		
		coor_translate(ps, pd);
		
		if('m' == flag) {
			//set pen up
			penFlag = 'u';
			fprintf(fp_result, "%c (%f %f) %c\n", flag, cur[0], cur[1], penFlag);
			
			//move to ps[2]
			penFlag = 'u';
			fprintf(fp_result, "%c (%f %f) %c\n", flag, pd[0], pd[1], penFlag);
			
			//set pen down
			penFlag = 'd';
			fprintf(fp_result, "%c (%f %f) %c\n", flag, pd[0], pd[1], penFlag);
		}
		else {
			penFlag = 'd';
			fprintf(fp_result, "%c (%f %f) %c\n", flag, pd[0], pd[1], penFlag);
		}
		cur[0] = pd[0];
		cur[1] = pd[1];
	}
	
	//pen up
	flag = 'm';
	penFlag = 'u';
	fprintf(fp_result, "%c (%f %f) %c\n", flag, cur[0], cur[1], penFlag);
	
	fclose(fp_source);
	fclose(fp_result);
}


void tra_sour_to_coor(char *tra1, char *tra2)
{
	float ps[2], pd[2], px[2]; //坐标对
	float dis, unit, t; 	   //线性函数控制变量
	char flag[2], penFlag[2];
	FILE  *fp1, *fp2;
	
	fp1 = fopen(tra1, "r");
	fp2 = fopen(tra2, "w");
	
	//搜索第一个起始点(必为'm')
	//取当前点(第一个‘m’)
	if(!fscanf(fp1, "%c (%f %f) %c\n", &flag[0], &ps[0], &ps[1], &penFlag[0]))
		exit(1);
	
	while(!feof(fp1)) {
		if(!fscanf(fp1, "%c (%f %f) %c\n", &flag[1], &pd[0], &pd[1], &penFlag[1]))
			exit(1);
			
		//第一种情况(mu)(mu):由起始点运动到目标位置
		if((flag[0] == 'm') && (penFlag[0] == 'u') && (flag[1] == 'm') && (penFlag[1] == 'u')) {
			dis = disPoint(ps, pd);
			if(dis > 0.0) {
				unit = 1.0 / dis;
				for(t = 0; t < 1.0; t += unit) {
					px[0] = t * (pd[0] - ps[0]) + ps[0];
					px[1] = t * (pd[1] - ps[1]) + ps[1];
					fprintf(fp2, "%c (%f %f) %c\n",flag[0], px[0], px[1], penFlag[0]);
				}
			}
			fprintf(fp2, "%c (%f %f) %c\n",flag[0], pd[0], pd[1], penFlag[0]);
			
			//更新
			flag[0] = flag[1];
			penFlag[0] = penFlag[1];
			ps[0] = pd[0];
			ps[1] = pd[1];
		}
		//第二种情况(mu)(md)：单纯的下降动作
		if((flag[0] == 'm') && (penFlag[0] == 'u') && (flag[1] == 'm') && (penFlag[1] == 'd')) {
			fprintf(fp2, "%c (%f %f) %c\n",flag[1], pd[0], pd[1], penFlag[1]);
			//更新
			flag[0] = flag[1];
			penFlag[0] = penFlag[1];
			ps[0] = pd[0];
			ps[1] = pd[1];
		}
		//第三种情况(md)(ld)：由起始点进行轨迹规划
		if((flag[0] == 'm') && (penFlag[0] == 'd') && (flag[1] == 'l') && (penFlag[1] == 'd')) {
			dis = disPoint(ps, pd);
			if(dis >= 1.0) { //线段之间的距离最小 >= 1.0
				unit = 1.0 / dis;
				for(t = unit; t < 1.0; t += unit) {
					px[0] = t * (pd[0] - ps[0]) + ps[0];
					px[1] = t * (pd[1] - ps[1]) + ps[1];
					fprintf(fp2, "%c (%f %f) %c\n",flag[1], px[0], px[1], penFlag[1]);
				}
				//更新
				flag[0] = flag[1];
				penFlag[0] = penFlag[1];
				ps[0] = pd[0];
				ps[1] = pd[1];
			}
		}
		
		//第四种情况(md)(mu)：距离太短，由起始点直接运动到目标的点
		if((flag[0] == 'm') && (penFlag[0] == 'd') && (flag[1] == 'm') && (penFlag[1] == 'u')) {
			dis = disPoint(ps, pd);
			if(dis >= 1.0) { //线段之间的距离最小 >= 1.0
				unit = 1.0 / dis;
				for(t = unit; t < 1.0; t += unit) {
					px[0] = t * (pd[0] - ps[0]) + ps[0];
					px[1] = t * (pd[1] - ps[1]) + ps[1];
					flag[0] = 'l'; penFlag[1] = 'd';
					fprintf(fp2, "%c (%f %f) %c\n",flag[0], px[0], px[1], penFlag[0]);
				}
			}
			flag[0] = 'l'; penFlag[0] = 'd';
			fprintf(fp2, "%c (%f %f) %c\n",flag[0], pd[0], pd[1], penFlag[0]);
			flag[0] = 'm'; penFlag[0] = 'u';
			fprintf(fp2, "%c (%f %f) %c\n",flag[0], pd[0], pd[1], penFlag[0]);
			
			//更新
			flag[0] = flag[1];
			penFlag[0] = penFlag[1];
			ps[0] = pd[0];
			ps[1] = pd[1];
		}
		
		//第四种情况(ld)(ld)：轨迹规划
		if((flag[0] == 'l') && (penFlag[0] == 'd') && (flag[1] == 'l') && (penFlag[1] == 'd')) {
			dis = disPoint(ps, pd);
			if(dis >= 1.0) { //线段之间的距离最小 >= 1.0
				unit = 1.0 / dis;
				for(t = 0; t < 1.0; t += unit) {
					px[0] = t * (pd[0] - ps[0]) + ps[0];
					px[1] = t * (pd[1] - ps[1]) + ps[1];
					fprintf(fp2, "%c (%f %f) %c\n",flag[1], px[0], px[1], penFlag[1]);
				}
				
				//更新
				flag[0] = flag[1];
				penFlag[0] = penFlag[1];
				ps[0] = pd[0];
				ps[1] = pd[1];
			}
		}
		//第五种情况(ld)(mu)：单纯的上升动作(一段路径规划完成)
		if((flag[0] == 'l') && (penFlag[0] == 'd') && (flag[1] == 'm') && (penFlag[1] == 'u')) {
			fprintf(fp2, "%c (%f %f) %c\n",flag[0], ps[0], ps[1], penFlag[0]);
			fprintf(fp2, "%c (%f %f) %c\n",flag[1], pd[0], pd[1], penFlag[1]);
			
			//更新
			flag[0] = flag[1];
			penFlag[0] = penFlag[1];
			ps[0] = pd[0];
			ps[1] = pd[1];
		}
	}
			
	fclose(fp1);
	fclose(fp2);
}


void tra_coor_to_spe(char *tra_coor, char *tra_vec)
{
	float ps[2], pd[2]; //坐标对
    float dis, vec[2];
    long tm;
    char flag[2], penFlag[2];
    FILE  *fp1, *fp2;
    
    fp1 = fopen(tra_coor, "r");
	fp2 = fopen(tra_vec, "w");
	
	//搜索到第一个起始点
	if(!fscanf(fp1, "%c (%f %f) %c\n", &flag[0], &ps[0], &ps[1], &penFlag[0]))
	    exit(1);
	while(!feof(fp1)) {
	    if(!fscanf(fp1, "%c (%f %f) %c\n", &flag[1], &pd[0], &pd[1], &penFlag[1]))
			exit(1);
		
		dis = disPoint(ps, pd); //两点之间的距离
		if(dis <= 0.0)
		    dis = 1.0;
		vec[0] = (pd[0] - ps[0]) / dis;
		vec[1] = (pd[1] - ps[1]) / dis;
		vec[0] *= SPEEDMX;
		vec[1] *= SPEEDMX;
		tm = (long)((dis * 1000) / SPEEDMX + 0.35);
		
		fprintf(fp2, "%c (%f %f) %c %ld\n", flag[1], vec[0], vec[1], penFlag[1], tm);
		
		//更新ps
		flag[0] = flag[1];
		penFlag[0] = penFlag[1];
		ps[0] = pd[0];
		ps[1] = pd[1];
	}
	
	fclose(fp1);
	fclose(fp2);
}

void tra_coor_to_posk(char *tra_coor, char *tra_vec, char *tra_k)
{
	FILE *fp1, *fp2, *fp3;
	float ps[2], vec[2];
	int vecK[2], posK[2];
	char flag[2], penFlag[2];
	long tm;
	
	fp1 = fopen(tra_coor, "r");
	fp2 = fopen(tra_vec, "r");
	fp3 = fopen(tra_k, "w");
	
	//取第一个坐标点
	if(EOF == fscanf(fp1, "%c (%f %f) %c\n", &flag[0], &ps[0], &ps[1], &penFlag[0])) {
		printf("get tra k failed 1 in\n");
		exit(1);
	}
	tran_coor_to_k(ps, posK); //trans coor to servoK
	tm = 0;
	vecK[0] = 30; vecK[1] = 30;
	fprintf(fp3, "%c %c (%d %d) (%d %d) %ld\n",flag[0], penFlag[0], vecK[0], vecK[1], posK[0], posK[1], tm);
	
	while(!feof(fp2)) {
		if(!fscanf(fp1, "%c (%f %f) %c\n", &flag[0], &ps[0], &ps[1], &penFlag[0])) {
			printf("get tra k failed 2 in\n");
			exit(1);
		}
		
		if(!fscanf(fp2, "%c (%f %f) %c %ld\n", &flag[1], &vec[0], &vec[1], &penFlag[1], &tm)) {
			printf("get tra k failed 3 in\n");
			exit(1);
		}
		
		//坐标ps[2]， 先速度vec[2];
		tran_covec_to_K(ps, vec, posK, vecK);
		fprintf(fp3, "%c %c (%d %d) (%d %d) %ld\n",flag[0], penFlag[0], vecK[0], vecK[1], posK[0], posK[1], tm);
	}
	
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
}

void tran_coor_to_k(float *xy, int *servoK)
{
	float angle[2];
	
	//trans coor to angle
	cal_xy_angle(xy, angle);
	
	//trans angle to servoK
	servoK[0] = servoPosKFromJointAngle(1, angle[0]);
	servoK[1] = servoPosKFromJointAngle(2, angle[1]);
}

//已知线性速度，求舵机刻度和速度
void tran_covec_to_K(float *xy, float *lineVec, int *posK, int *vecK)
{
	float angle[2], aspe[2];
	
	//计算出关节角度
	cal_xy_angle(xy, angle);
	
	//计算出关节角速度
	cal_lspe_aspe(lineVec, aspe);
	
	//计算出舵机posK
	posK[0] = servoPosKFromJointAngle(1, angle[0]);
	posK[1] = servoPosKFromJointAngle(2, angle[1]);
	
	//计算出舵机speK
	vecK[0] = servoSpeKFromJointAngleSpe(aspe[0]);
	vecK[1] = servoSpeKFromJointAngleSpe(aspe[1]);
}

void excute_posk(char *tra_k)
{
	FILE *fp;
	char flag, penFlag, index[2] = {'s', 's'};
	int posK[2], vecK[2];
	long tm;
	
	fp = fopen(tra_k, "r");
	while(!feof(fp)) {
		if(EOF == fscanf(fp, "%c %c (%d %d) (%d %d) %ld\n" ,&flag, &penFlag, &vecK[0], &vecK[1], &posK[0], &posK[1], &tm)) {
			printf("excute_tra_k failed in\n");
			exit(1);
		}
		
		//(mu)
		if((flag == 'm') && (penFlag == 'u')) {
			if('s' == index[0]) {
				set_pen_height(1);
				index[0] = 'r';
				index[1] = 's';
			}
			
			//执行数据
			set_many_servo_word(Moving_Speed, vecK);
			set_many_servo_word(Goal_Position, posK);
		}
		//(md)
		if((flag == 'm') && (penFlag == 'd')) {
			if('s' == index[1]) {
				set_pen_height(0);
				index[1] = 'r';
				index[0] = 's';
			}
		}
		//(ld)
		if((flag == 'l') && (penFlag == 'd')) {
			//执行数据
			set_many_servo_word(Moving_Speed, vecK);
			set_many_servo_word(Goal_Position, posK);
			
			index[0] = 's';
			index[1] = 's';  
		}
		
		//延时控制
		tm += TM;
		tm *= 1000;
		delay_us(tm); //延迟50ms
	}
	fclose(fp);
}


void set_default_pos()
{	
	int value[2];
	float ink_xy[2] = {80.00, -20.0}, xy[2], angle[2];
	
	//set servo spe
	value[0] = 100; value[1] = 100;
	set_many_servo_word(Moving_Speed, value);
	
	//set goal pos
	coor_translate(ink_xy, xy);	//from ink to xy
	cal_xy_angle(xy, angle); 	//from xy to angle
	//from angle to servoK
	value[0] = servoPosKFromJointAngle(1, angle[0]); 
	value[1] = servoPosKFromJointAngle(2, angle[1]);
	set_many_servo_word(Goal_Position, value);
	
	//wait servos stop
	value[0] = 5; value[1] = 5;
	wait_for_many_servo_exten(value);
}

