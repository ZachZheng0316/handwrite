#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include "MX28AT.h"
#include "dmath.h"
#include "kinesiology.h"

static int servoK[2];

void initial_sys(); //系统初始化
void check_torque();//检测扭矩是否>=80
void renable();		//重新上刚度

int main()
{
	initial_sys();
	
	while(1) {
		check_torque(); //check servo's reload >= 80 or not
		renable(); 		//if servo's pre-position is stable , enable amrs; or , not;
	}
	
	return 0;
}


//系统初始化
void initial_sys()
{
	int deviceIndex = 0, baudnum = 1, value[2];
	float xy[2], angle[2];
	
	printf("\ninitial_sys ...\n");
	if(!dxl_initialize(deviceIndex, baudnum)){
		printf("Failed to open USB2Dynamixel!\n");
		exit(1);
	}
	else
		printf("Succeed to open USB2Dynamixel!\n");
		
	//属性初始化
	//PID控制
	value[0] = 46; value[1] = 46;
	set_many_servo_byte(P_Gain, value);
	value[0] = 32; value[1] = 32;
	set_many_servo_byte(I_Gain, value);
	value[0] = 30; value[1] = 30;
	set_many_servo_byte(D_Gain, value);
	
	//enable
	set_enable_arms();
	
	//get preServoK
	value[0] = get_one_servo_word(1, Present_Position);
	value[1] = get_one_servo_word(2, Present_Position);
	//get preAngle
	angle[0] = jointAngleFromServoPosK(1, value[0]);
	angle[1] = jointAngleFromServoPosK(2, value[1]);
	//get prePosition
	cal_angle_xy(angle, xy);
	printf("preServoK(%d %d) \npreAngle(%f %f) \nprePosition(%f %f)\n", value[0], value[1], angle[0], angle[1], xy[0], xy[1]);
	
	//更新当前刻度
	servoK[0] = value[0];
	servoK[1] = value[1];
	
}

//检测扭矩是否>=80
void check_torque()
{
	int reload[2];
	
	reload[0] = get_one_servo_word(1, Present_Load)%1024;
	reload[1] = get_one_servo_word(2, Present_Load)%1024;
	
	while(1) {
		if((reload[0] >= 80) || (reload[1] >= 80)) {
			relax_arms();
			break;
		}
		else{
			reload[0] = get_one_servo_word(1, Present_Load)%1024;
			reload[1] = get_one_servo_word(2, Present_Load)%1024;
			printf("\r id(1) reload(%d); id(2) reload(%d)", reload[0], reload[1]);
			fflush(stdout);
		}
		
		delay_us(200 * 1000);
	}
	printf("\n");
}

//重新上刚度
void renable()
{
	int preK[2], goalK[2];
	float angle[2], xy[2];
	
	while(1){
		preK[0] = get_one_servo_word(1, Present_Position);
		preK[1] = get_one_servo_word(2, Present_Position);
		
		delay_us(4 * 1000 * 1000);
		
		goalK[0] = get_one_servo_word(1, Present_Position);
		goalK[1] = get_one_servo_word(2, Present_Position);
		
		//abs([preK[0], preK[1]] - [goalK[0], goalK[1]]) < [10, 10]
		if(abs(preK[0] - goalK[0]) <= 10) {
			if(abs(preK[1] - goalK[1]) <= 10) {
				set_enable_arms();
					
				angle[0] = jointAngleFromServoPosK(1, goalK[0]);
				angle[1] = jointAngleFromServoPosK(2, goalK[1]);
				
				cal_angle_xy(angle, xy);
				
				printf("preServoK(%d %d) \n", goalK[0], goalK[1]);
				printf("preAngle(%f %f) \n", angle[0], angle[1]);
				printf("prePosition(%f %f)\n",  xy[0], xy[1]);
				
				break;
			}
		}
	}
}
