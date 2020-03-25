/* Includes ------------------------------------------------------------------*/
//#include "inv_mpu20608.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "inv_mpu.h"

#include "mpu_module.h"

/* Private define ------------------------------------------------------------*/
#ifdef MPU_DEBUG
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)
#endif

/* Switch */
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

/* Starting sampling rate. */
#define DEFAULT_mpu_HZ  (1000)
#define TEMP_READ_TICK    (500)


/* Private typedef -----------------------------------------------------------*/
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char lp_6axis_mode;
    unsigned char sensors;
    unsigned char watermark;
    //volatile unsigned char new_sensor;
    unsigned char motion_int_mode;
    unsigned long next_temp_tick;
    unsigned int report;
};

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};


/* Private variables ---------------------------------------------------------*/
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the driver(s).
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct hal_s hal = {0};

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
#ifdef MPU_DEBUG
/** 
 *  @brief ?MPL????????
 *  @return ?
 *  @attention ??UART??
 */
static void read_from_mpl(void)
{
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
       /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
    }
    if (hal.report & PRINT_ACCEL) {
        if (inv_get_sensor_type_accel(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO) {
        if (inv_get_sensor_type_gyro(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);
    }
    if (hal.report & PRINT_EULER) {
        if (inv_get_sensor_type_euler(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_EULER, data);
    }
    if (hal.report & PRINT_ROT_MAT) {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING) {
        if (inv_get_sensor_type_heading(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy,
            (inv_time_t*)&timestamp))
            MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
                    float_data[0], float_data[1], float_data[2]);
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) {
            if (inv_get_sensor_type_gravity(float_data, &accuracy,
                (inv_time_t*)&timestamp))
                MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
                        float_data[0], float_data[1], float_data[2]);
    }
}
#endif

/** 
 *  @brief ???,??inv_get_sensor_type_euler()??
 *  @return 1 if data was updated. 
 *  @attention 
 */
int8_t mpu_read_euler(long *data, unsigned long *timestamp) {
  int8_t tmp;
  inv_get_sensor_type_euler(data, &tmp, timestamp);
  return tmp;
}

/** 
 *  @brief MPU???????
 *  @return ?
 *  @attention 
 */
int module_mpu_init(void)
{ 
  inv_error_t result;
  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;


  result = mpu_init();
  if (result) {
      MPL_LOGE("Could not initialize sensors.\n");
  }

  result = inv_init_mpl();
  if (result) {
      MPL_LOGE("Could not initialize MPL.\n");
  }

  /* Compute 6-axis quaternions. ??6?????????? */
  inv_enable_quaternion();
  inv_enable_9x_sensor_fusion();

  /* Update gyro biases when not in motion.
   * ?????????????????????????
   */
  inv_enable_fast_nomot();
  /* inv_enable_motion_no_motion(); */
  /* inv_set_no_motion_time(1000); */

  /* Update gyro biases when temperature changes. ????????????*/
  inv_enable_gyro_tc();

  /* Allows use of the MPL APIs in read_from_mpl. ?????????*/
  inv_enable_eMPL_outputs();

  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
      while (1) {
          MPL_LOGE("Not authorized.\n");
      }
  }
  if (result) {
      MPL_LOGE("Could not start the MPL.\n");
  }

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_mpu_HZ);
  /* Read back configuration in case it was set improperly. */
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  /* Sync driver configuration with MPL. */
  /* Sample rate expected in microseconds. */
  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);
  /* Set chip-to-body orientation matrix.
   * Set hardware units to dps/g's/degrees scaling factor.
   */
  inv_set_gyro_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)gyro_fsr<<15);
  inv_set_accel_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)accel_fsr<<15);
  /* Initialize HAL state variables. */
  hal.sensors = ACCEL_ON | GYRO_ON;
  hal.report = 0;
  hal.next_temp_tick = 0;
  return 0;
}


/** 
 *  @brief ?MPU?FIFO?????????
           ??????mpl??????????
 *  @return ?
 *  @attention ?????10ms????
 */
int mpu_module_sampling()
{
  unsigned char sensors, more,new_temp = 0;
  static unsigned long ticktime = 0;
  unsigned long sensor_timestamp,cycletime = 0;

  int new_data = 0;

  ticktime++;

  /* ?????????????????????? */
  if (ticktime > hal.next_temp_tick) {
      hal.next_temp_tick = ticktime + TEMP_READ_TICK;
      new_temp = 1; //start task temp;
  }

  /* ????sensor??? */
  if (!hal.sensors) return 0;

  do {
    short gyro[3], accel_short[3];
    long accel[3], temperature;

    cycletime++;

    mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
        &sensors, &more);
    if (sensors & INV_XYZ_GYRO) {
        /* ?????MPL. */
        inv_build_gyro(gyro, sensor_timestamp);
        new_data = 1;
        if (new_temp) {
            new_temp = 0;
            /* Temperature only used for gyro temp comp. */
            mpu_get_temperature(&temperature, &sensor_timestamp);
            inv_build_temp(temperature, sensor_timestamp);
        }
    }
    if (sensors & INV_XYZ_ACCEL) {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
    }

    if (new_data) {
        if(inv_execute_on_data()) {
            MPL_LOGE("ERROR execute on data\n");
        }
        #ifdef MPU_DEBUG
          read_from_mpl(); 
        #endif
    }
  }
  while(more);
  return cycletime;
}
