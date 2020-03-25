#include "pid.h"

PID_Regulator_t pid_body_pos_lf_rb;
PID_Regulator_t pid_body_pos_rf_lb;

void PID_Calc(PID_Regulator_t *pid);

void PID_Init()
{
	//pid_body_pos_lf_rb initialization
	pid_body_pos_lf_rb.kp 				= 1.2;
	pid_body_pos_lf_rb.ki 				= 0;
	pid_body_pos_lf_rb.kd					= 0;
	pid_body_pos_lf_rb.outputMax	= 2;
	pid_body_pos_lf_rb.Calc 			= PID_Calc;
	
	//pid_body_pos_rf_lb initialization
	pid_body_pos_rf_lb.kp 				= 1.2;
	pid_body_pos_rf_lb.ki 				= 0;
	pid_body_pos_rf_lb.kd					= 0;
	pid_body_pos_rf_lb.outputMax	= 2;
	pid_body_pos_rf_lb.Calc 			= PID_Calc;
}

void PID_Calc(PID_Regulator_t *pid)
{
  float dError, Error;
	
	Error = pid->ref-pid->fdb;
	pid->sum += Error;
	dError = pid->err[0] - pid->err[1];
	pid->err[1] = pid->err[0];
	pid->err[0] = Error;
	
	pid->output = pid->kp*Error + pid->ki*pid->sum + pid->kd*dError;
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if(pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;
}
