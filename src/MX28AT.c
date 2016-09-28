#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include "MX28AT.h"
#include "dynamixel.h"
#include "dmath.h"
#include "dxl_hal.h"

//全局变量
static int servo_num = 2;          //舵机个数默认为3
static int servo_unit[2] = {1, 2}; //舵机组
static int pre_goal_pos[2];        //舵机组的当前目标刻度值

//设置单个舵机单字节属性
//成功写入，返回1; 写入失败返回-1；
void set_one_servo_byte(int id, int address, int value)
{
    int commStatus, write_num = 2000;

	while(write_num-- >= 0) {
    	//设置属性值
    	dxl_write_byte(id, address, value);
    	//判断是否成功写入
    	commStatus = dxl_get_result();
    	if( commStatus == COMM_RXSUCCESS ) {
        	PrintErrorCode();
        	return;
    	}
    	else {
        	PrintCommStatus(commStatus);
        	delay_us(10000);
    	}
    }

   printf("error:MX28AT:set_one_servo_byte failed\n");
   exit(1);
}

//设置单个舵机多字节属性
//写入成功，返回1; 写入失败， 返回-1
void set_one_servo_word(int id, int address, int value)
{
    int commStatus, write_num = 2000;

    while(write_num-- >= 0) {
    	dxl_write_word(id, address, value);
    	commStatus = dxl_get_result();
    	if( commStatus == COMM_RXSUCCESS ) {
        	PrintErrorCode();
        	return;
    	}
    	else {
        	PrintCommStatus(commStatus);
        	delay_us(10000);
    	}
    }

   printf("error:MX28AT:set_one_servo_word failed\n");
   exit(1);
}

//设置多个舵机单字节属性
//写入成功, 返回1; 写入失败, 返回-1。
void set_many_servo_byte(int address, int value[])
{
    int commStatus, i, write_num = 2000;

    while(write_num-- >= 0) {
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, address);
		dxl_set_txpacket_parameter(1, 1);
		for( i = 0; i < servo_num; i++ ) {
		    dxl_set_txpacket_parameter(2 + 2 * i, servo_unit[i]);
		    dxl_set_txpacket_parameter(2 + 2 * i + 1, value[i]);
		}
		dxl_set_txpacket_length((1 + 1) * servo_num + 4);
		dxl_txrx_packet();//printf("\n");
		commStatus = dxl_get_result();
		if( commStatus == COMM_RXSUCCESS ) {
		    PrintErrorCode();
		    return;
		}
		else{
		    PrintCommStatus(commStatus);
		    delay_us(10000);
		}
	}

	printf("error:MX28AT:set_many_servo_byte failed\n");
   	exit(1);
}

//设置多个舵机多自己属性
//写入成功，返回1; 写入失败, 返回-1.
void set_many_servo_word(int address, int value[])
{
    int commStatus, i, write_num = 2000;

    while(write_num-- >= 0) {
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, address);
		dxl_set_txpacket_parameter(1, 2);
		for( i = 0; i < servo_num; i++ ) {
		    dxl_set_txpacket_parameter(2 + 3 * i, servo_unit[i]);
		    dxl_set_txpacket_parameter(2 + 3 * i + 1, dxl_get_lowbyte(value[i]));
		    dxl_set_txpacket_parameter(2 + 3 * i + 2, dxl_get_highbyte(value[i]));
		}
		dxl_set_txpacket_length((2 + 1) * servo_num + 4);
		dxl_txrx_packet(); //printf("\n");
		commStatus = dxl_get_result();
		if(commStatus == COMM_RXSUCCESS ) {
		    PrintErrorCode();
		    if(Goal_Position == address) {
		    	for(i = 0; i < servo_num; i++)
		    		pre_goal_pos[i] = value[i];
		    }
		    return;
		}
		else{
		    PrintCommStatus(commStatus);
		    delay_us(10000);
		}
	}

	printf("error:MX28AT:set_many_servo_byte failed\n");
   	exit(1);
}

//获取单个舵机单字节属性
//读取成功，返回大于0的值;读入失败, 返回-1；
int get_one_servo_byte(int id, int address)
{
    int value, commStatus;

    while(1) {
        value = dxl_read_byte(id, address);
        commStatus = dxl_get_result();
        if(commStatus == COMM_RXSUCCESS) {
            PrintErrorCode();
            break;
        }
        else {
            PrintCommStatus(commStatus);
        }
    }

    return value;
}

//获取单个舵机多字节属性
//读取成功，返回大于0的值;读入失败, 返回-1；
int get_one_servo_word(int id, int address)
{
    int value, commStatus;

    while(1) {
        value = dxl_read_word(id, address);
        commStatus = dxl_get_result();
        if(COMM_RXSUCCESS == commStatus) {
            PrintErrorCode();
            break;
        }
        else {
            PrintCommStatus(commStatus);
        }
    }

    return value;
}

//等待1个舵机运动停止
//执行成功返回1；执行失败返回-1
void wait_for_one_servo(int id)
{
    int moving;
    int value, read_num = 0;

    //读取目标刻度
    value = get_one_servo_word(id, Goal_Position);
    printf("MX28AT::wait_for_one_servo:%d servo's Goal_Position is %d\n", id, value);fflush(stdout);//显示读取的目标刻度

    do{
        moving = get_one_servo_byte(id, Moving); //获取运动结果
        if(0 == moving)
        	read_num++;
        else
        	read_num = 0;
    }while(read_num <= 20); //Goal_Position正在执行

    //读取舵机的当前刻度
    value = get_one_servo_word(id, Present_Position);
    printf("MX28AT::wait_for_one_servo:%d servo's Present_Position %d\n", id, value);

    //return 1;
}

//等待所有舵机停止运动
void wait_for_many_servo()
{
	int i;

	for(i = 0; i < servo_num; i++)
		wait_for_one_servo(servo_unit[i]);

}

//等待1个舵机运动到exten范围内
void wait_for_one_servo_exten(int id, int exten)
{
	int goal_pos, value, diff;

	goal_pos = get_one_servo_word(id, Goal_Position);

	//获得目标位置
	do{
		value = get_one_servo_word(id, Present_Position);
		diff = goal_pos - value;
	}while(abs(diff) >= exten);

}

//等待所有舵机运动到exten范围内
void wait_for_many_servo_exten(int exten[])
{
	int i;

	for(i = 0; i <servo_num; i++)
		wait_for_one_servo_exten(servo_unit[i], exten[i]);
}

//设置笔的高度
void set_pen_height(int heightFlag)
{
	//设置速度
	delay_us(500*1000);
	if(0 == heightFlag) {
		dxl_set_pen_height(5, Goal_Position, 61441);
		delay_us(10 * 1000);
		dxl_set_pen_height(5, Goal_Position, 61441);
		delay_us(10 * 1000);
		dxl_set_pen_height(5, Goal_Position, 61441);
		
		//dxl_write_word(5, Goal_Position, 61441);
		//printf("hello\n");
	}
	else {
		dxl_set_pen_height(5, Goal_Position, 36865);
		delay_us(10 * 1000);
		dxl_set_pen_height(5, Goal_Position, 36865);
		delay_us(10 * 1000);
		dxl_set_pen_height(5, Goal_Position, 36865);
		//dxl_write_word(5, Goal_Position, 36865);
	}
	delay_us(1000*1000);
}

//设置机械臂的刚度
void set_enable_arms()
{
	int value[2] = {1, 1};
	set_many_servo_byte(Torque_Enable, value);
}
//放松机械臂
void relax_arms()
{
	int value[2] = {0, 0};
	set_many_servo_byte(Torque_Enable, value);
}
//获取机械臂的温度
void get_arms_temprature(int temprature[])
{
	temprature[0] = get_one_servo_word(1, Present_Temperature);
	temprature[1] = get_one_servo_word(2, Present_Temperature);
}
