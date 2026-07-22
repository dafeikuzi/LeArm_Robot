#include "learm_servo.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
} LeArmServoOutput;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define LEARM_SERVO_UPDATE_MS   20U
#define LEARM_SERVO_MAX_TIME_MS 30000U
#define LEARM_SERVO_Q15_ONE     32768U

static const LeArmServoOutput servoOutputs[LEARM_SERVO_MAX_ID + 1U] =
{
  {0, 0},
  {&htim3, TIM_CHANNEL_1},
  {&htim3, TIM_CHANNEL_2},
  {&htim3, TIM_CHANNEL_3},
  {&htim3, TIM_CHANNEL_4},
  {&htim4, TIM_CHANNEL_1},
  {&htim4, TIM_CHANNEL_2}
};

static const uint16_t servoInitialPulseUs[LEARM_SERVO_MAX_ID + 1U] =
{
  0U, 1200U, 1500U, 1500U, 1500U, 1500U, 1500U
};

static uint16_t servoStartPulseUs[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoCurrentPulseUs[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoTargetPulseUs[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoTotalSteps[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoStep[LEARM_SERVO_MAX_ID + 1U];
static uint8_t servoMoving[LEARM_SERVO_MAX_ID + 1U];

static uint16_t LeArm_ServoSmoothstepQ15(uint16_t step, uint16_t totalSteps)
{
  uint32_t t;
  uint32_t tSquared;
  uint32_t tCubed;

  if ((totalSteps == 0U) || (step >= totalSteps))
  {
    return LEARM_SERVO_Q15_ONE;
  }

  t = ((uint32_t)step << 15) / totalSteps;
  tSquared = (t * t) >> 15;
  tCubed = (tSquared * t) >> 15;
  return (uint16_t)((3U * tSquared) - (2U * tCubed));
}

static uint16_t LeArm_ClampPulse(uint16_t pulseUs)
{
  if (pulseUs < LEARM_SERVO_MIN_PULSE_US)
  {
    return LEARM_SERVO_MIN_PULSE_US;
  }

  if (pulseUs > LEARM_SERVO_MAX_PULSE_US)
  {
    return LEARM_SERVO_MAX_PULSE_US;
  }

  return pulseUs;
}

static void LeArm_ServoApply(uint8_t id)
{
  __HAL_TIM_SET_COMPARE(servoOutputs[id].timer, servoOutputs[id].channel, servoCurrentPulseUs[id]);
}

void LeArm_ServoInit(void)
{
  uint8_t id;

  for (id = LEARM_SERVO_MIN_ID; id <= LEARM_SERVO_MAX_ID; id++)
  {
    if (HAL_TIM_PWM_Start(servoOutputs[id].timer, servoOutputs[id].channel) != HAL_OK)
    {
      Error_Handler();
    }

    servoStartPulseUs[id] = servoInitialPulseUs[id];
    servoCurrentPulseUs[id] = servoInitialPulseUs[id];
    servoTargetPulseUs[id] = servoInitialPulseUs[id];
    servoTotalSteps[id] = 0U;
    servoStep[id] = 0U;
    servoMoving[id] = 0U;
    LeArm_ServoApply(id);
  }
}

void LeArm_ServoSetPulseAndTime(uint8_t id, uint16_t pulseUs, uint16_t timeMs)
{
  uint16_t steps;

  if ((id < LEARM_SERVO_MIN_ID) || (id > LEARM_SERVO_MAX_ID))
  {
    return;
  }

  if (timeMs < LEARM_SERVO_UPDATE_MS)
  {
    timeMs = LEARM_SERVO_UPDATE_MS;
  }
  else if (timeMs > LEARM_SERVO_MAX_TIME_MS)
  {
    timeMs = LEARM_SERVO_MAX_TIME_MS;
  }

  pulseUs = LeArm_ClampPulse(pulseUs);
  steps = (uint16_t)((timeMs + LEARM_SERVO_UPDATE_MS - 1U) / LEARM_SERVO_UPDATE_MS);
  if (steps == 0U)
  {
    steps = 1U;
  }

  servoStartPulseUs[id] = servoCurrentPulseUs[id];
  servoTargetPulseUs[id] = pulseUs;
  servoTotalSteps[id] = steps;
  servoStep[id] = 0U;
  servoMoving[id] = 1U;
}

void LeArm_ServoUpdate20ms(void)
{
  uint8_t id;

  for (id = LEARM_SERVO_MIN_ID; id <= LEARM_SERVO_MAX_ID; id++)
  {
    if (servoMoving[id] == 0U)
    {
      continue;
    }

    servoStep[id]++;
    if (servoStep[id] >= servoTotalSteps[id])
    {
      servoCurrentPulseUs[id] = servoTargetPulseUs[id];
      servoMoving[id] = 0U;
    }
    else
    {
      int32_t delta = (int32_t)servoTargetPulseUs[id] - (int32_t)servoStartPulseUs[id];
      int32_t progress = (int32_t)LeArm_ServoSmoothstepQ15(servoStep[id], servoTotalSteps[id]);
      int32_t pulse = (int32_t)servoStartPulseUs[id]
                    + ((delta * progress) / (int32_t)LEARM_SERVO_Q15_ONE);
      servoCurrentPulseUs[id] = (uint16_t)pulse;
    }

    LeArm_ServoApply(id);
  }
}

void LeArm_ServoEmergencyStop(void)
{
  uint8_t id;

  for (id = LEARM_SERVO_MIN_ID; id <= LEARM_SERVO_MAX_ID; id++)
  {
    servoStartPulseUs[id] = servoCurrentPulseUs[id];
    servoTargetPulseUs[id] = servoCurrentPulseUs[id];
    servoTotalSteps[id] = 0U;
    servoStep[id] = 0U;
    servoMoving[id] = 0U;
    LeArm_ServoApply(id);
  }
}

uint16_t LeArm_ServoGetCurrentPulse(uint8_t id)
{
  if ((id < LEARM_SERVO_MIN_ID) || (id > LEARM_SERVO_MAX_ID))
  {
    return 0U;
  }

  return servoCurrentPulseUs[id];
}

uint16_t LeArm_ServoGetTargetPulse(uint8_t id)
{
  if ((id < LEARM_SERVO_MIN_ID) || (id > LEARM_SERVO_MAX_ID))
  {
    return 0U;
  }

  return servoTargetPulseUs[id];
}

uint8_t LeArm_ServoGetMovingMask(void)
{
  uint8_t id;
  uint8_t mask = 0U;

  for (id = LEARM_SERVO_MIN_ID; id <= LEARM_SERVO_MAX_ID; id++)
  {
    if (servoMoving[id] != 0U)
    {
      mask |= (uint8_t)(1U << (id - 1U));
    }
  }

  return mask;
}
