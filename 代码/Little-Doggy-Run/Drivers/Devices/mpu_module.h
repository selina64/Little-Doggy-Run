#ifndef __MPU_MODULE__
#define __MPU_MODULE__

/* Includes ------------------------------------------------------------------*/


/* Function typedef -----------------------------------------------------------*/
/** 
 *  @brief MPU???????
 *  @return ?
 *  @attention 
 */
int module_mpu_init(void);

/** 
 *  @brief ?MPU?FIFO?????????
           ??????mpl??????????
 *  @return ????,????
 *  @attention ?????10ms????
 */
int mpu_module_sampling(void);

/** 
 *  @brief ???,??inv_get_sensor_type_euler()??
 *  @return 1 if data was updated. 
 *  @attention 
 */
signed char mpu_read_euler(long *data, unsigned long *timestamp);

#endif