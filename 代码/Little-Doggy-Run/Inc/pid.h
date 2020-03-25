#ifndef __PID_H__
#define __PID_H__

typedef struct __PID_Regulator_t
{
	//input
	float ref;
	float fdb;
	
	//output
	float output;
	
	//private
	double sum;
	float err[2];
	
	// initializations
	float kp;
	float ki;
	float kd;
	float outputMax;
	void (*Calc)(struct __PID_Regulator_t *pid);
	
}PID_Regulator_t;

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	8.0f,\
	0.0f,\
	0.1f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
}\

extern PID_Regulator_t pid_body_pos_lf_rb;

#endif
