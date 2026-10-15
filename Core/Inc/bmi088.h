#ifndef BMI088_H
#define BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
  BMI088_OK = 0,
  BMI088_ERROR_ARGUMENT,
  BMI088_ERROR_SPI,
  BMI088_ERROR_ACCEL_ID,
  BMI088_ERROR_GYRO_ID,
  BMI088_ERROR_VERIFY,
  BMI088_ERROR_DRDY_TIMEOUT
} BMI088_Status;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} BMI088_RawVector;

typedef struct
{
  SPI_HandleTypeDef *spi;
  uint8_t accel_chip_id;
  uint8_t gyro_chip_id;
  char verify_device;
  uint8_t verify_reg;
  uint8_t verify_expected;
  uint8_t verify_actual;
} BMI088_Device;

BMI088_Status BMI088_Init(BMI088_Device *device, SPI_HandleTypeDef *spi);
BMI088_Status BMI088_ReadAccel(BMI088_Device *device, BMI088_RawVector *raw);
BMI088_Status BMI088_ReadGyro(BMI088_Device *device, BMI088_RawVector *raw);

#ifdef __cplusplus
}
#endif

#endif