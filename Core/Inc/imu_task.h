#ifndef IMU_TASK_H
#define IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi088.h"

typedef enum
{
  IMU_STATE_UNINITIALIZED = 0,
  IMU_STATE_CALIBRATING,
  IMU_STATE_RUNNING,
  IMU_STATE_ERROR
} IMU_State;

typedef struct
{
  IMU_State state;
  BMI088_Status error;
  uint8_t accel_chip_id;
  uint8_t gyro_chip_id;
  char verify_device;
  uint8_t verify_reg;
  uint8_t verify_expected;
  uint8_t verify_actual;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float accel_norm_mps2;
  float gyro_norm_rps;
  uint32_t gyro_drdy_count;
  uint32_t accel_drdy_count;
  uint32_t gyro_overrun_count;
  uint32_t accel_overrun_count;
  uint32_t spi_error_count;
  uint32_t attitude_update_count;
} IMU_Status;

void IMU_Init(SPI_HandleTypeDef *spi);
void IMU_Task(void);
void IMU_OnDataReady(uint16_t gpio_pin);
const IMU_Status *IMU_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif