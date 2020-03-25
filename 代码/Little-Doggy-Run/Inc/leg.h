/**
  ******************************************************************************
  * File Name          : LEG.h
  * Description        : 腿部运动的舵机实现及舵机初始化.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 Little-Doggy.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */
#ifndef __LEG_H
#define __LEG_H

#include "main.h"
#include "math.h"
#include "tim.h"
#include "gpio.h"

/*
	舵机布局图
	
	1 2     3 4
	
	5 6     7 8
	*/

#define PI 3.1415926
#define rad_2_deg(X) ( X / PI * 180.0 )
#define deg_2_rad(X) ( X / 180.0 * PI )

typedef enum{
	RF_LEG = 0,
	RB_LEG,
	LF_LEG,
	LB_LEG
} Leg_Index;

typedef enum{
	RF_SERVO_1 = 0,
	RF_SERVO_2,
	RB_SERVO_1,
	RB_SERVO_2,
	LF_SERVO_1,
	LF_SERVO_2,
	LB_SERVO_1,
	LB_SERVO_2
} Servo;

typedef struct __Leg_t
{
	uint32_t index;
	float Ax;
	float Ay;
	float zero_x;
	float zero_y;
}Leg_t;

#define LEG_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	12,\
}\

extern Leg_t LF, RF, LB, RB;

void Leg_Init(void);
void Set_ZeroPos(void);
void Set_Servo_Angle(Servo servo, double degree);
void Set_Leg_Angle(Leg_Index leg, double degree1, double degree2);
void Set_Feet_Pos(Leg_Index leg, double x, double y);

#endif /*__LEG_H*/

