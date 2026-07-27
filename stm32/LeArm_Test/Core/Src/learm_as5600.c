#include "learm_as5600.h"

#define LEARM_AS5600_I2C_ADDRESS      0x36U
#define LEARM_AS5600_REG_STATUS       0x0BU
#define LEARM_AS5600_REG_RAW_ANGLE    0x0CU
#define LEARM_AS5600_RAW_ANGLE_MASK   0x0FFFU

static uint16_t latestRawAngle;
static uint8_t latestStatus;
static uint8_t latestValid;
static uint8_t initialized;

static void LeArm_As5600Delay(void)
{
  volatile uint32_t index;

  for (index = 0U; index < 80U; index++)
  {
    __NOP();
  }
}

static void LeArm_As5600SclHigh(void)
{
  HAL_GPIO_WritePin(as5600_scl_GPIO_Port, as5600_scl_Pin, GPIO_PIN_SET);
}

static void LeArm_As5600SclLow(void)
{
  HAL_GPIO_WritePin(as5600_scl_GPIO_Port, as5600_scl_Pin, GPIO_PIN_RESET);
}

static void LeArm_As5600SdaHigh(void)
{
  HAL_GPIO_WritePin(as5600_sda_GPIO_Port, as5600_sda_Pin, GPIO_PIN_SET);
}

static void LeArm_As5600SdaLow(void)
{
  HAL_GPIO_WritePin(as5600_sda_GPIO_Port, as5600_sda_Pin, GPIO_PIN_RESET);
}

static uint8_t LeArm_As5600ReadSda(void)
{
  return (HAL_GPIO_ReadPin(as5600_sda_GPIO_Port, as5600_sda_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void LeArm_As5600Start(void)
{
  LeArm_As5600SdaHigh();
  LeArm_As5600SclHigh();
  LeArm_As5600Delay();
  LeArm_As5600SdaLow();
  LeArm_As5600Delay();
  LeArm_As5600SclLow();
  LeArm_As5600Delay();
}

static void LeArm_As5600Stop(void)
{
  LeArm_As5600SdaLow();
  LeArm_As5600Delay();
  LeArm_As5600SclHigh();
  LeArm_As5600Delay();
  LeArm_As5600SdaHigh();
  LeArm_As5600Delay();
}

static uint8_t LeArm_As5600WriteByte(uint8_t value)
{
  uint8_t mask;
  uint8_t acknowledged;

  for (mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    if ((value & mask) != 0U)
    {
      LeArm_As5600SdaHigh();
    }
    else
    {
      LeArm_As5600SdaLow();
    }

    LeArm_As5600Delay();
    LeArm_As5600SclHigh();
    LeArm_As5600Delay();
    LeArm_As5600SclLow();
    LeArm_As5600Delay();
  }

  LeArm_As5600SdaHigh();
  LeArm_As5600Delay();
  LeArm_As5600SclHigh();
  LeArm_As5600Delay();
  acknowledged = (LeArm_As5600ReadSda() == 0U) ? 1U : 0U;
  LeArm_As5600SclLow();
  LeArm_As5600Delay();

  return acknowledged;
}

static uint8_t LeArm_As5600ReadByte(uint8_t acknowledge)
{
  uint8_t mask;
  uint8_t value = 0U;

  LeArm_As5600SdaHigh();
  for (mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    LeArm_As5600Delay();
    LeArm_As5600SclHigh();
    LeArm_As5600Delay();
    if (LeArm_As5600ReadSda() != 0U)
    {
      value |= mask;
    }
    LeArm_As5600SclLow();
    LeArm_As5600Delay();
  }

  if (acknowledge != 0U)
  {
    LeArm_As5600SdaLow();
  }
  else
  {
    LeArm_As5600SdaHigh();
  }
  LeArm_As5600Delay();
  LeArm_As5600SclHigh();
  LeArm_As5600Delay();
  LeArm_As5600SclLow();
  LeArm_As5600SdaHigh();
  LeArm_As5600Delay();

  return value;
}

static uint8_t LeArm_As5600ReadRegister(uint8_t registerAddress, uint8_t *data, uint8_t length)
{
  uint8_t index;

  if ((data == 0) || (length == 0U))
  {
    return 0U;
  }

  LeArm_As5600Start();
  if (LeArm_As5600WriteByte((uint8_t)(LEARM_AS5600_I2C_ADDRESS << 1)) == 0U)
  {
    LeArm_As5600Stop();
    return 0U;
  }
  if (LeArm_As5600WriteByte(registerAddress) == 0U)
  {
    LeArm_As5600Stop();
    return 0U;
  }

  LeArm_As5600Start();
  if (LeArm_As5600WriteByte((uint8_t)((LEARM_AS5600_I2C_ADDRESS << 1) | 1U)) == 0U)
  {
    LeArm_As5600Stop();
    return 0U;
  }

  for (index = 0U; index < length; index++)
  {
    data[index] = LeArm_As5600ReadByte((index + 1U) < length);
  }
  LeArm_As5600Stop();
  return 1U;
}

void LeArm_As5600Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = as5600_scl_Pin | as5600_sda_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LeArm_As5600SclHigh();
  LeArm_As5600SdaHigh();
  latestRawAngle = 0U;
  latestStatus = 0U;
  latestValid = 0U;
  initialized = 1U;
}

void LeArm_As5600Task(void)
{
  uint8_t status;
  uint8_t rawBytes[2];

  if (initialized == 0U)
  {
    return;
  }

  if ((LeArm_As5600ReadRegister(LEARM_AS5600_REG_STATUS, &status, 1U) == 0U) ||
    (LeArm_As5600ReadRegister(LEARM_AS5600_REG_RAW_ANGLE, rawBytes, 2U) == 0U))
  {
    latestValid = 0U;
    return;
  }

  latestStatus = status;
  latestRawAngle = (uint16_t)((((uint16_t)rawBytes[0] << 8) | (uint16_t)rawBytes[1]) &
    LEARM_AS5600_RAW_ANGLE_MASK);
  latestValid = 1U;
}

uint8_t LeArm_As5600GetLatest(uint16_t *rawAngle, uint8_t *status)
{
  if (rawAngle != 0)
  {
    *rawAngle = latestRawAngle;
  }
  if (status != 0)
  {
    *status = latestStatus;
  }
  return latestValid;
}
