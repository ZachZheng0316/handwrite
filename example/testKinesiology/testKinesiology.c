//##########################################################
//##                      R O B O T I S                   ##
//##          ReadWrite Example code for Dynamixel.       ##
//##                                           2009.11.10 ##
//##########################################################
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

int main()
{
	float xy[2], angle[2];  //执行端坐标和关节角度
	float lspe[2], aspe[2]; //速度计算

	//已知关节角度,求坐标
	angle[0] = 66.88;
	angle[1] = 66.39;
	cal_angle_xy(angle, xy);
	printf("angle(%f %f) xy(%f %f)\n",angle[0], angle[1], xy[0], xy[1]);
	
	//已知坐标， 求关节角度
	xy[0] = -0.66; xy[1] = 176.38;
	cal_xy_angle(xy, angle);
	printf("xy(%f %f) angle(%f %f)\n", xy[0], xy[1], angle[0], angle[1]);
	
	//已知线速度，求角速度
	lspe[0] = 0.0; lspe[1] = 100.0;
	cal_lspe_aspe(lspe, aspe);
	printf("lspe(%f %f) aspe(%f %f)\n", lspe[0], lspe[1], aspe[0], aspe[1]);

	//已知角速度，求线速度
	aspe[0] = -18.731716; aspe[1] = -18.452038;
	cal_aspe_lspe(aspe, lspe);
	printf("aspe(%f %f) lspe(%f %f)\n", aspe[0], aspe[1], lspe[0], lspe[1]);
	

	return 1;
}
