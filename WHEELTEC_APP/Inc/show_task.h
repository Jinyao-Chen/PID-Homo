#ifndef __SHOW_TASK_H
#define __SHOW_TASK_H

#include <stdint.h>

typedef struct{
	float roll;
	float pitch;
	float yaw;
	float gyrox;
	float gyroy;
	float gyroz;
	float m1;
	float m2;
	float m3;
	float m4;
	float balanceTaskFreq;
	float height;
	float c_roll;
	float c_pitch;
	float c_yaw;
	float c_height;
	short posx;
	short posy;
}APPShowType_t;

#endif

