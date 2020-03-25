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
	
#include "pid.h"
#include "main.h"
#include "control_task.h"	
#include "RemoteTask.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
	
Body_t body;


void Main_Control_Task()
{
	static double t=0.0;
	double trot_period = 0.45;
	double rotate_spd = 0, forward_spd = 0 ;
	
	if(RC_CtrlData.rc.RightSwitch == RC_SWITCH_TOP)
	{
		forward_spd = 1.0*(RC_CtrlData.rc.RightY - 0x400)/(0x694-0x400);
		rotate_spd  = -1.0*(RC_CtrlData.rc.RightX - 0x400)/(0x694-0x400);
	}
	else if(RC_CtrlData.rc.RightSwitch == RC_SWITCH_MIDDLE)
	{
		rotate_spd  = -1.0*(usb_face_x)/(300);
		forward_spd = 0;
	}
	if (fabs(forward_spd) > 0.1 || fabs(rotate_spd) > 0.1)
	{
		LF.Ay = RB.Ay = LB.Ay = RF.Ay = 1.9;
		RB.Ax = RF.Ax = 2.7*(forward_spd+rotate_spd);
		LF.Ax = LB.Ax = 2.7*(forward_spd-rotate_spd);
		LF.zero_x = LF.Ax > 0? -1.2: 1.2;
		RB.zero_x = RB.Ax > 0? -1.2: 1.2;
		LB.zero_x = LB.Ax > 0? -1.2: 1.2;
		RF.zero_x = RF.Ax > 0? -1.2: 1.2;
		
	} else {
		
		LF.Ay = RB.Ay = LB.Ay = RF.Ay = 0;
		RB.Ax = RF.Ax = 0;
		LF.Ax = LB.Ax = 0;
		LF.zero_x = 0;
		RB.zero_x = 0;
		LB.zero_x = 0;
		RF.zero_x = 0;
		
	}
	
	
	t = (HAL_GetTick()%(int)(trot_period*1000))/1000.0;
	Trot_Leg(&LF, 		 2 * PI / trot_period * t);
	Trot_Leg(&RB, 		 2 * PI / trot_period * t);
	Trot_Leg(&LB, PI + 2 * PI / trot_period * t);
	Trot_Leg(&RF, PI + 2 * PI / trot_period * t);
}

void BodyPos_Control_Task()
{
	pid_body_pos_lf_rb.ref = 0; //后期如果需要俯仰，则更改这个位置
	pid_body_pos_lf_rb.fdb = body.z_lf_rb;
	if (pid_body_pos_lf_rb.output > 0)
	{
		
	}
	
}

void BodyPos_Calc_Task()
{
	
}


/**
  * @brief  Trot步态驱动函数
  * @param  in: 需要操作的Leg_Index
  * @param  Ax: x方向上的增益
  * @param  Ay: y方向上的增益
  * @param  zero_x: x方向的零位置
  * @param  zero_y: y方向的零位置
  * @retval None
  */

void Trot_Leg(Leg_t* in, double phase)
{
	double Ax = in->Ax, Ay = in->Ay, zero_x = in->zero_x, zero_y = in->zero_y;
	static double x = 0;
	static double y = 0;
	
	if (phase > 2*PI) phase = phase - 2*PI;
	
	if ((phase<=PI*11/12)) //跨
	{
		x = ((XH-XL)/2 *( - cos(phase/11*12)))+(XH+XL)/2;
		y = YH*sin(phase/(11/12));
	}
	else if ((phase>PI*11/12) & (phase<=PI))//换象进行中
	{
		
	}
	else if ((phase>PI) && (phase<=(1+5.0/6)*PI)) //蹬
	{
		x = ((XH-XL)/2 * cos((phase-PI)*6/5)+(XH+XL)/2);
		y = YL*sin(6/5 /2*(phase-PI));
	}
	else if ((phase>(1+5.0/6)*PI) & (phase<=(1+11.0/12)*PI))//换象进行中
	{
		
	}
	else if((phase>(1+11.0/12)*PI)&(phase<=2*PI)) //抬
	{
		x = XL;
		y = -YL*sin(12/2*(phase-2*PI));
	}
	
	x = zero_x + x*Ax;
	y = zero_y + y*Ay;
	
	Set_Feet_Pos(in->index,x, y);
}