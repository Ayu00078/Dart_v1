#include "bmi088.h"
#include "main.h"

#define BMI088_SPI_TIMEOUT_MS            2U
#define BMI088_SPI_READ                  0x80U

#define BMI088_ACCEL_CHIP_ID_REG         0x00U
#define BMI088_ACCEL_DATA_REG            0x12U
#define BMI088_ACCEL_CONF_REG            0x40U
#define BMI088_ACCEL_RANGE_REG           0x41U
#define BMI088_ACCEL_INT1_IO_CONF_REG    0x53U
#define BMI088_ACCEL_INT_MAP_DATA_REG    0x58U
#define BMI088_ACCEL_PWR_CONF_REG        0x7CU
#define BMI088_ACCEL_PWR_CTRL_REG        0x7DU
#define BMI088_ACCEL_SOFTRESET_REG       0x7EU

#define BMI088_GYRO_CHIP_ID_REG          0x00U
#define BMI088_GYRO_DATA_REG             0x02U
#define BMI088_GYRO_RANGE_REG            0x0FU
#define BMI088_GYRO_BANDWIDTH_REG        0x10U
#define BMI088_GYRO_LPM1_REG             0x11U
#define BMI088_GYRO_SOFTRESET_REG        0x14U
#define BMI088_GYRO_INT_CTRL_REG         0x15U
#define BMI088_GYRO_INT_IO_CONF_REG      0x16U
#define BMI088_GYRO_INT_MAP_REG          0x18U

#define BMI088_ACCEL_CHIP_ID             0x1EU
#define BMI088_GYRO_CHIP_ID              0x0FU
#define BMI088_SOFTRESET_COMMAND         0xB6U
#define BMI088_ACCEL_CONF_1600HZ_NORMAL  0xACU
#define BMI088_ACCEL_RANGE_6G            0x01U
#define BMI088_ACCEL_INT1_ACTIVE_HIGH_PP 0x0AU
#define BMI088_ACCEL_MAP_DRDY_INT1       0x04U
#define BMI088_ACCEL_POWER_ACTIVE        0x00U
#define BMI088_ACCEL_POWER_ENABLE        0x04U
#define BMI088_GYRO_RANGE_2000DPS        0x00U
#define BMI088_GYRO_1000HZ_116HZ         0x02U
#define BMI088_GYRO_RANGE_MASK           0x07U
#define BMI088_GYRO_BANDWIDTH_MASK       0x0FU
#define BMI088_GYRO_POWER_NORMAL         0x00U
#define BMI088_GYRO_DRDY_ENABLE          0x80U
#define BMI088_GYRO_INT3_ACTIVE_HIGH_PP  0x01U
#define BMI088_GYRO_MAP_DRDY_INT3        0x01U

static void AccelSelect(void)
{
  HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET);
}

static void AccelDeselect(void)
{
  HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET);
}

static void GyroSelect(void)
{
  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}

static void GyroDeselect(void)
{
  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

static BMI088_Status SpiTransfer(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx,
                                 uint16_t length)
{
  return (HAL_SPI_TransmitReceive(spi, tx, rx, length, BMI088_SPI_TIMEOUT_MS) == HAL_OK)
           ? BMI088_OK : BMI088_ERROR_SPI;
}

static BMI088_Status AccelWrite(BMI088_Device *device, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };
  uint8_t rx[2];
  BMI088_Status status;

  AccelSelect();
  status = SpiTransfer(device->spi, tx, rx, 2U);
  AccelDeselect();
  return status;
}

static BMI088_Status GyroWrite(BMI088_Device *device, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };
  uint8_t rx[2];
  BMI088_Status status;

  GyroSelect();
  status = SpiTransfer(device->spi, tx, rx, 2U);
  GyroDeselect();
  return status;
}

/* BMI088 accelerometer SPI reads contain one dummy byte after the address. */
static BMI088_Status AccelRead(BMI088_Device *device, uint8_t reg, uint8_t *data,
                               uint8_t length)
{
  uint8_t tx[8] = { 0U };
  uint8_t rx[8];
  uint8_t index;
  BMI088_Status status;

  if ((data == NULL) || (length > 6U))
  {
    return BMI088_ERROR_ARGUMENT;
  }
  tx[0] = (uint8_t)(reg | BMI088_SPI_READ);
  AccelSelect();
  status = SpiTransfer(device->spi, tx, rx, (uint16_t)(length + 2U));
  AccelDeselect();
  if (status == BMI088_OK)
  {
    for (index = 0U; index < length; index++)
    {
      data[index] = rx[index + 2U];
    }
  }
  return status;
}

static BMI088_Status GyroRead(BMI088_Device *device, uint8_t reg, uint8_t *data,
                              uint8_t length)
{
  uint8_t tx[7] = { 0U };
  uint8_t rx[7];
  uint8_t index;
  BMI088_Status status;

  if ((data == NULL) || (length > 6U))
  {
    return BMI088_ERROR_ARGUMENT;
  }
  tx[0] = (uint8_t)(reg | BMI088_SPI_READ);
  GyroSelect();
  status = SpiTransfer(device->spi, tx, rx, (uint16_t)(length + 1U));
  GyroDeselect();
  if (status == BMI088_OK)
  {
    for (index = 0U; index < length; index++)
    {
      data[index] = rx[index + 1U];
    }
  }
  return status;
}

static BMI088_Status VerifyAccel(BMI088_Device *device, uint8_t reg, uint8_t expected)
{
  uint8_t value;
  BMI088_Status status = AccelRead(device, reg, &value, 1U);
  if ((status == BMI088_OK) && (value != expected))
  {
    device->verify_device = 'A';
    device->verify_reg = reg;
    device->verify_expected = expected;
    device->verify_actual = value;
    return BMI088_ERROR_VERIFY;
  }
  return status;
}

static BMI088_Status VerifyGyro(BMI088_Device *device, uint8_t reg, uint8_t expected,
                                uint8_t mask)
{
  uint8_t value;
  BMI088_Status status = GyroRead(device, reg, &value, 1U);
  if ((status == BMI088_OK) && ((value & mask) != (expected & mask)))
  {
    device->verify_device = 'G';
    device->verify_reg = reg;
    device->verify_expected = expected;
    device->verify_actual = value;
    return BMI088_ERROR_VERIFY;
  }
  return status;
}

BMI088_Status BMI088_Init(BMI088_Device *device, SPI_HandleTypeDef *spi)
{
  BMI088_Status status;
  uint8_t discard;

  if ((device == NULL) || (spi == NULL))
  {
    return BMI088_ERROR_ARGUMENT;
  }
  device->spi = spi;
  device->accel_chip_id = 0U;
  device->gyro_chip_id = 0U;
  device->verify_device = '-';
  device->verify_reg = 0U;
  device->verify_expected = 0U;
  device->verify_actual = 0U;
  AccelDeselect();
  GyroDeselect();
  HAL_Delay(5U);

  /* A first read changes the accelerometer interface from I2C to SPI. */
  status = AccelRead(device, BMI088_ACCEL_CHIP_ID_REG, &discard, 1U);
  if (status == BMI088_OK)
  {
    status = AccelRead(device, BMI088_ACCEL_CHIP_ID_REG, &device->accel_chip_id, 1U);
  }
  if (status != BMI088_OK)
  {
    return status;
  }
  if (device->accel_chip_id != BMI088_ACCEL_CHIP_ID)
  {
    return BMI088_ERROR_ACCEL_ID;
  }
  status = AccelWrite(device, BMI088_ACCEL_SOFTRESET_REG, BMI088_SOFTRESET_COMMAND);
  HAL_Delay(2U);
  if (status == BMI088_OK)
  {
    status = AccelRead(device, BMI088_ACCEL_CHIP_ID_REG, &discard, 1U);
  }
  if (status == BMI088_OK)
  {
    status = AccelWrite(device, BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_POWER_ENABLE);
  }
  HAL_Delay(5U);
  if (status == BMI088_OK)
  {
    status = AccelWrite(device, BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_POWER_ACTIVE);
  }
  HAL_Delay(5U);
  if (status == BMI088_OK)
  {
    status = AccelWrite(device, BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_1600HZ_NORMAL);
  }
  HAL_Delay(10U);
  if (status == BMI088_OK)
  {
    status = AccelWrite(device, BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_6G);
  }
  HAL_Delay(10U);
  if (status == BMI088_OK)
  {
    status = AccelWrite(device, BMI088_ACCEL_INT1_IO_CONF_REG,
                        BMI088_ACCEL_INT1_ACTIVE_HIGH_PP);
  }
  if (status == BMI088_OK)
  {
    status = AccelWrite(device, BMI088_ACCEL_INT_MAP_DATA_REG,
                        BMI088_ACCEL_MAP_DRDY_INT1);
  }
  HAL_Delay(40U);
  if (status == BMI088_OK)
  {
    status = VerifyAccel(device, BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_1600HZ_NORMAL);
  }
  if (status == BMI088_OK)
  {
    status = VerifyAccel(device, BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_6G);
  }
  if (status != BMI088_OK)
  {
    return status;
  }

  status = GyroRead(device, BMI088_GYRO_CHIP_ID_REG, &device->gyro_chip_id, 1U);
  if (status != BMI088_OK)
  {
    return status;
  }
  if (device->gyro_chip_id != BMI088_GYRO_CHIP_ID)
  {
    return BMI088_ERROR_GYRO_ID;
  }
  status = GyroWrite(device, BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_COMMAND);
  HAL_Delay(30U);
  if (status == BMI088_OK)
  {
    status = GyroWrite(device, BMI088_GYRO_LPM1_REG, BMI088_GYRO_POWER_NORMAL);
  }
  HAL_Delay(30U);
  if (status == BMI088_OK)
  {
    status = GyroWrite(device, BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS);
  }
  if (status == BMI088_OK)
  {
    status = GyroWrite(device, BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_1000HZ_116HZ);
  }
  HAL_Delay(10U);
  if (status == BMI088_OK)
  {
    status = GyroWrite(device, BMI088_GYRO_INT_IO_CONF_REG,
                       BMI088_GYRO_INT3_ACTIVE_HIGH_PP);
  }
  if (status == BMI088_OK)
  {
    status = GyroWrite(device, BMI088_GYRO_INT_MAP_REG, BMI088_GYRO_MAP_DRDY_INT3);
  }
  if (status == BMI088_OK)
  {
    status = GyroWrite(device, BMI088_GYRO_INT_CTRL_REG, BMI088_GYRO_DRDY_ENABLE);
  }
  if (status == BMI088_OK)
  {
    status = VerifyGyro(device, BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS,
                        BMI088_GYRO_RANGE_MASK);
  }
  if (status == BMI088_OK)
  {
    status = VerifyGyro(device, BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_1000HZ_116HZ,
                        BMI088_GYRO_BANDWIDTH_MASK);
  }
  return status;
}

BMI088_Status BMI088_ReadAccel(BMI088_Device *device, BMI088_RawVector *raw)
{
  uint8_t data[6];
  BMI088_Status status;
  if ((device == NULL) || (raw == NULL))
  {
    return BMI088_ERROR_ARGUMENT;
  }
  status = AccelRead(device, BMI088_ACCEL_DATA_REG, data, 6U);
  if (status == BMI088_OK)
  {
    raw->x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    raw->y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    raw->z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
  }
  return status;
}

BMI088_Status BMI088_ReadGyro(BMI088_Device *device, BMI088_RawVector *raw)
{
  uint8_t data[6];
  BMI088_Status status;
  if ((device == NULL) || (raw == NULL))
  {
    return BMI088_ERROR_ARGUMENT;
  }
  status = GyroRead(device, BMI088_GYRO_DATA_REG, data, 6U);
  if (status == BMI088_OK)
  {
    raw->x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    raw->y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    raw->z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
  }
  return status;
}