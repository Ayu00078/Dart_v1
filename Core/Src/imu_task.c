#include "imu_task.h"
#include "attitude.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define IMU_ACCEL_SCALE_MPS2       (6.0f * 9.80665f / 32768.0f)
#define IMU_GYRO_SCALE_RPS         (2000.0f * 0.01745329252f / 32768.0f)
#define IMU_SAMPLE_PERIOD_S        0.001f
#define IMU_CALIBRATION_SAMPLES    1000U
#define IMU_STILL_GYRO_LIMIT_RPS   0.15f
#define IMU_STILL_ACCEL_MIN_MPS2   8.3f
#define IMU_STILL_ACCEL_MAX_MPS2   11.3f
#define IMU_GYRO_TIMEOUT_MS         20U

static BMI088_Device bmi088;
static Attitude attitude;
static IMU_Status imu_status;
static volatile uint32_t gyro_drdy_sequence;
static volatile uint32_t accel_drdy_sequence;
static uint32_t processed_gyro_sequence;
static uint32_t processed_accel_sequence;
static uint32_t calibration_count;
static float gyro_bias_sum[3];
static float gyro_bias[3];
static float latest_accel[3];
static uint8_t accel_valid;
static uint32_t last_gyro_drdy_tick;

static void MapToBody(const BMI088_RawVector *raw, float scale, float body[3])
{
  body[0] = (float)raw->y * scale;
  body[1] = -(float)raw->x * scale;
  body[2] = (float)raw->z * scale;
}

static uint8_t IsCalibrationSampleStill(const float gyro[3], const float accel[3])
{
  float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                           accel[2] * accel[2]);
  return ((fabsf(gyro[0]) < IMU_STILL_GYRO_LIMIT_RPS) &&
          (fabsf(gyro[1]) < IMU_STILL_GYRO_LIMIT_RPS) &&
          (fabsf(gyro[2]) < IMU_STILL_GYRO_LIMIT_RPS) &&
          (accel_norm > IMU_STILL_ACCEL_MIN_MPS2) &&
          (accel_norm < IMU_STILL_ACCEL_MAX_MPS2)) ? 1U : 0U;
}

void IMU_Init(SPI_HandleTypeDef *spi)
{
  uint8_t attempt;

  memset(&imu_status, 0, sizeof(imu_status));
  memset(&attitude, 0, sizeof(attitude));
  gyro_drdy_sequence = 0U;
  accel_drdy_sequence = 0U;
  processed_gyro_sequence = 0U;
  processed_accel_sequence = 0U;
  calibration_count = 0U;
  accel_valid = 0U;
  imu_status.state = IMU_STATE_UNINITIALIZED;

  for (attempt = 0U; attempt < 3U; attempt++)
  {
    imu_status.error = BMI088_Init(&bmi088, spi);
    if (imu_status.error == BMI088_OK)
    {
      break;
    }
    HAL_Delay(10U);
  }
  imu_status.accel_chip_id = bmi088.accel_chip_id;
  imu_status.gyro_chip_id = bmi088.gyro_chip_id;
  imu_status.verify_device = bmi088.verify_device;
  imu_status.verify_reg = bmi088.verify_reg;
  imu_status.verify_expected = bmi088.verify_expected;
  imu_status.verify_actual = bmi088.verify_actual;
  if (imu_status.error == BMI088_OK)
  {
    __disable_irq();
    gyro_drdy_sequence = 0U;
    accel_drdy_sequence = 0U;
    processed_gyro_sequence = 0U;
    processed_accel_sequence = 0U;
    imu_status.gyro_drdy_count = 0U;
    imu_status.accel_drdy_count = 0U;
    __enable_irq();
    last_gyro_drdy_tick = HAL_GetTick();
    imu_status.state = IMU_STATE_CALIBRATING;
  }
  else
  {
    imu_status.state = IMU_STATE_ERROR;
  }
}

void IMU_OnDataReady(uint16_t gpio_pin)
{
  if (gpio_pin == BMI088_GYRO_INT_Pin)
  {
    gyro_drdy_sequence++;
    imu_status.gyro_drdy_count++;
  }
  else if (gpio_pin == BMI088_ACCEL_INT_Pin)
  {
    accel_drdy_sequence++;
    imu_status.accel_drdy_count++;
  }
}

void IMU_Task(void)
{
  uint32_t gyro_sequence;
  uint32_t accel_sequence;
  BMI088_RawVector raw;
  float gyro[3];

  if ((imu_status.state != IMU_STATE_CALIBRATING) &&
      (imu_status.state != IMU_STATE_RUNNING))
  {
    return;
  }

  if ((uint32_t)(HAL_GetTick() - last_gyro_drdy_tick) > IMU_GYRO_TIMEOUT_MS)
  {
    imu_status.error = BMI088_ERROR_DRDY_TIMEOUT;
    imu_status.state = IMU_STATE_ERROR;
    return;
  }

  gyro_sequence = gyro_drdy_sequence;
  if (gyro_sequence == processed_gyro_sequence)
  {
    return;
  }
  if ((uint32_t)(gyro_sequence - processed_gyro_sequence) > 1U)
  {
    imu_status.gyro_overrun_count +=
      (uint32_t)(gyro_sequence - processed_gyro_sequence - 1U);
  }
  processed_gyro_sequence = gyro_sequence;
  last_gyro_drdy_tick = HAL_GetTick();

  accel_sequence = accel_drdy_sequence;
  if (accel_sequence != processed_accel_sequence)
  {
    if ((uint32_t)(accel_sequence - processed_accel_sequence) > 1U)
    {
      imu_status.accel_overrun_count +=
        (uint32_t)(accel_sequence - processed_accel_sequence - 1U);
    }
    processed_accel_sequence = accel_sequence;
    if (BMI088_ReadAccel(&bmi088, &raw) == BMI088_OK)
    {
      MapToBody(&raw, IMU_ACCEL_SCALE_MPS2, latest_accel);
      accel_valid = 1U;
    }
    else
    {
      imu_status.spi_error_count++;
    }
  }

  if (BMI088_ReadGyro(&bmi088, &raw) != BMI088_OK)
  {
    imu_status.spi_error_count++;
    return;
  }
  MapToBody(&raw, IMU_GYRO_SCALE_RPS, gyro);
  if (accel_valid == 0U)
  {
    return;
  }

  if (imu_status.state == IMU_STATE_CALIBRATING)
  {
    if (IsCalibrationSampleStill(gyro, latest_accel) == 0U)
    {
      calibration_count = 0U;
      gyro_bias_sum[0] = 0.0f;
      gyro_bias_sum[1] = 0.0f;
      gyro_bias_sum[2] = 0.0f;
      return;
    }
    gyro_bias_sum[0] += gyro[0];
    gyro_bias_sum[1] += gyro[1];
    gyro_bias_sum[2] += gyro[2];
    calibration_count++;
    if (calibration_count >= IMU_CALIBRATION_SAMPLES)
    {
      gyro_bias[0] = gyro_bias_sum[0] / (float)IMU_CALIBRATION_SAMPLES;
      gyro_bias[1] = gyro_bias_sum[1] / (float)IMU_CALIBRATION_SAMPLES;
      gyro_bias[2] = gyro_bias_sum[2] / (float)IMU_CALIBRATION_SAMPLES;
      Attitude_Init(&attitude, latest_accel);
      imu_status.state = IMU_STATE_RUNNING;
    }
    return;
  }

  gyro[0] -= gyro_bias[0];
  gyro[1] -= gyro_bias[1];
  gyro[2] -= gyro_bias[2];
  Attitude_Update(&attitude, gyro, latest_accel, IMU_SAMPLE_PERIOD_S);
  imu_status.roll_deg = attitude.roll_deg;
  imu_status.pitch_deg = attitude.pitch_deg;
  imu_status.yaw_deg = attitude.yaw_deg;
  imu_status.attitude_update_count++;
}

const IMU_Status *IMU_GetStatus(void)
{
  return &imu_status;
}