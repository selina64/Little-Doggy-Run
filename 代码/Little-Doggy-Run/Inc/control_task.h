/**
  ******************************************************************************
  * File Name          : CONTROL_TASK.c
  * Description        : 具体控制算法实现.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 Little-Doggy.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */
#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "main.h"
#include "math.h"
#include "tim.h"
#include "gpio.h"
#include "leg.h"

#define T 1.0

#define YH 0.5
#define YL -0.2
#define XH 1.5
#define XL -0.5

typedef struct __Body_t
{
	float yaw;
	float pitch;
	float roll;
	float z_lf_rb;
	float z_rf_lb;
} Body_t;

void Trot_Leg(Leg_t* in, double phase);
void Main_Control_Task(void);
void BodyPos_Control_Task(void);
void BodyPos_Calc_Task(void);

#endif /*__CONTROL_TASK_H*/
