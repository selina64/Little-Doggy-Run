#ifndef __REMOTETAST_H__
#define __REMOTETAST_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"

//RC_CtrlData
typedef struct{
	uint8_t rc_bytes[18];
}RC_Raw_t;

typedef __packed struct
{
	int16_t RightX;
	int16_t RightY;
	int16_t LeftX;
	int16_t LeftY;
	int8_t LeftSwitch;
	int8_t RightSwitch;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed enum
{
	RC_ON,
	RC_OFF
} RC_State;

typedef __packed enum
{
	RC_SWITCH_TOP = 1,
	RC_SWITCH_BUTTOM,
	RC_SWITCH_MIDDLE
} RC_Switch_State;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
	RC_State state;
} RC_Ctl_t;

extern RC_Ctl_t RC_CtrlData;

void RemoteDataProcess(uint8_t rc_rxdata);


#endif
