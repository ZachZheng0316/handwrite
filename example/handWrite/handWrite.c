#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "MX28AT.h"
#include "dmath.h"
#include "kinesiology.h"
#include "serialCommuni.h"

#define TM (16);//>=16(大于数据传输的时间间隔)

void initial_sys(); //系统初始化
void set_default_pos();
void receive_instruct(unsigned char *instruct);
void execute_instruct(unsigned char *instruct);
void finish_instruct(unsigned char *feedback);
void finish_sys();

void draw_word(FILE *fp);
void fill_word(FILE *fp);
void write_word(int wordNum);


int main()
{
	unsigned char packet[10] = {0, };
	
	initial_sys();
	
	set_default_pos();
	
	do{
		receive_instruct(packet);
		
		execute_instruct(packet);
		
	}while(1);
	
	finish_sys();
	
	return 1;
}

//系统初始化
void initial_sys()
{
	int deviceIndex = 0, baudnum = 1, value[2];
	//float xy[2], angle[2];
	
	//open sericCommunical
	if(!serial_open(5, 1200)) {
		printf("err:initial_sys serial_open failed\n");
		exit(0);
	}
	
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
	
	//set pen up
	set_pen_height(1);
}

void set_default_pos()
{
	int value[2];
	float ink_xy[2] = {80.00, -20.00}, xy[2], angle[2];
	
	//set servo spe
	value[0] = 50; value[1] = 50;
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

void receive_instruct(unsigned char *instruct)
{
	int flag;
    
    while(1) {
        //1.获取字符
        flag = receiveMessage(instruct, 10);
        if(-1 == flag) {
            printf("\rreceive data failed!");
            fflush(stdout);
        }
        else if(0 == flag) {
            printf("\rcontinue receiving data ...");
            fflush(stdout);
        }
        else if(flag > 0) {
            printf("\nprint data ...");
            fflush(stdout);
            if(flag >= 9)
                instruct[9] = '\0';
            else
                instruct[flag] = '\0';
            printf("the receiving data (%s)\n", instruct);
            fflush(stdout);
            break;
        }
        else{
        }
    }
}
void execute_instruct(unsigned char *instruct)
{
	static int num;
	char temp[10] = {0, };
	
	if('N' == instruct[0]) {
		strcpy(temp, (char *)&instruct[1]);
		num = atoi(temp);
		printf("the word num:%d\n", num);
		
		write_word(num);
		relax_arms();
	}
	else
		finish_instruct((unsigned char *)"default");
}
void finish_instruct(unsigned char *feedback)
{
	int feedNum, realLen;

	feedNum = strlen((char *)feedback);
	
	while(1) {
        realLen = sendMessage(feedback, feedNum);
		if(realLen != feedNum) {
			printf("failed send data\r"); fflush(stdout);
		}
		else {
			printf("success send data:%s\n", feedback); fflush(stdout);
			break;
		}	
     }
}
void finish_sys()
{
	serial_close();
    dxl_terminate();
}

void draw_word(FILE *fp)
{
	char flag, penFlag, index[2] = {'s', 's'};
	int posK[2], vecK[2], speCtrNum = 10;
	long tm;
	
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
			if(speCtrNum >= 0) {
				vecK[0] = (int)((float)vecK[0] * 0.30);
				vecK[1] = (int)((float)vecK[1] * 0.30);
				set_many_servo_word(Moving_Speed, vecK);
				speCtrNum --;
			}
			else{
				set_many_servo_word(Moving_Speed, vecK);
			}
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
}

void fill_word(FILE *fp)
{
	char flag;
	int value[2];
	float angle[2], ink_xy[2], xy[2];	
	
	//set move spe
	value[0] = 40; value[1] = 40;
	set_many_servo_word(Moving_Speed, value);
	
	while(!feof(fp)){
		if(EOF != fscanf(fp, "%c (%f %f)\n", &flag, &ink_xy[0], &ink_xy[1])) {
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
		
			//set pen down
			set_pen_height(0);
			
			//printf("fill(%f %f)\n", ink_xy[0], ink_xy[1]);
		
			//delay
			delay_us(1000 * 1000);
		
			//set pen up
			set_pen_height(1);
		}
	}

	//move back to init pos
	set_default_pos();
}

void write_word(int wordNum)
{
	FILE *fpDraw, *fpFill;
	char path[20] = {0, };
	
	//set draw path
	sprintf(path, "words/d%d.txt", wordNum);
	fpDraw = fopen(path, "r");
	if(fpDraw) {
		//set fill path
		sprintf(path, "words/f%d.txt", wordNum);
		fpFill = fopen(path, "r");
		if(fpFill) {
			
			//set pen up
			//set_pen_height(1);
		
			//send ready signal
			delay_us(1000 * 1000);
			finish_instruct((unsigned char *)"ook");
			delay_us(500 * 1000);
			
			//draw word
			draw_word(fpDraw);
			
			//send draw over signal
			finish_instruct((unsigned char *)"ppover");
			delay_us(500 * 1000);
			
			//fill word
			fill_word(fpFill);
			
			//send fill over signal
			finish_instruct((unsigned char *)"ffover");
			
			//set up down
			//set_pen_height(0);
			
			fclose(fpFill);
		}
		fclose(fpDraw);
	}
}
