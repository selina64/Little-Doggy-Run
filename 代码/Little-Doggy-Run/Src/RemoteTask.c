
#include "RemoteTask.h"
#include "math.h"

uint8_t rc_rxbuff[20];
int cnt = 0;


uint32_t time_tick_1ms = 0;//1ms定时计数，系统会在指定的毫秒时刻进行不同的控制
uint8_t remoteDataLock = 0;
extern uint32_t Get_Time_Micros();
RC_Ctl_t RC_CtrlData;   //remote control data
void RemoteDataProcess(uint8_t rc_rxdata)
{
	
	
	
	
	
	
	
	
	
	
	
	
	static unsigned int last_time_tick_1ms = 0,i = 0;
	time_tick_1ms = HAL_GetTick();
	
	if(abs((int32_t)time_tick_1ms - (int32_t)last_time_tick_1ms) > 1)
		i = 0;
	last_time_tick_1ms = time_tick_1ms;
	rc_rxbuff[i++] = rc_rxdata;
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	if(i == 18) {
		remoteDataLock = 1;
		RC_CtrlData.rc.RightX = ((int16_t)rc_rxbuff[0] | ((int16_t)rc_rxbuff[1] << 8)) & 0x07FF; 
		RC_CtrlData.rc.RightY = (((int16_t)rc_rxbuff[1] >> 3) | ((int16_t)rc_rxbuff[2] << 5)) & 0x07FF;
		RC_CtrlData.rc.LeftX = (((int16_t)rc_rxbuff[2] >> 6) | ((int16_t)rc_rxbuff[3] << 2) | ((int16_t)rc_rxbuff[4] << 10)) & 0x07FF;
		RC_CtrlData.rc.LeftY = (((int16_t)rc_rxbuff[4] >> 1) | ((int16_t)rc_rxbuff[5]<<7)) & 0x07FF;
		RC_CtrlData.rc.LeftSwitch = ((rc_rxbuff[5] >> 4) & 0x000C) >> 2;
		RC_CtrlData.rc.RightSwitch = ((rc_rxbuff[5] >> 4) & 0x0003);
		RC_CtrlData.mouse.x = ((int16_t)rc_rxbuff[6]) | ((int16_t)rc_rxbuff[7] << 8);
		RC_CtrlData.mouse.y = ((int16_t)rc_rxbuff[8]) | ((int16_t)rc_rxbuff[9] << 8);
		RC_CtrlData.mouse.z = ((int16_t)rc_rxbuff[10]) | ((int16_t)rc_rxbuff[11] << 8);    
		RC_CtrlData.mouse.press_l = rc_rxbuff[12];//左键
		RC_CtrlData.mouse.press_r = rc_rxbuff[13];//右键
		RC_CtrlData.key.v = ((int16_t)rc_rxbuff[14]);
		remoteDataLock = 0;
		i = 0;
	} 
}