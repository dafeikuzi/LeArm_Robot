#include "learm_servo.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
} LeArmServoOutput;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define LEARM_SERVO_UPDATE_MS    20U
#define LEARM_SERVO_MIN_TIME_MS  1000U
#define LEARM_SERVO_MAX_TIME_MS  30000U
#define LEARM_SERVO_Q15_ONE      32768L

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
  /* Power-on zero pose: gripper closed, joint_3/joint_4 at the measured -40 deg pose. */
  0U, 1400U, 1500U, 1056U, 1056U, 1500U, 1500U
};

/* Maximum PWM change per 20 ms update. IDs map to gripper, joint_2..joint_6. */
static const uint16_t servoMaxStepUs[LEARM_SERVO_MAX_ID + 1U] =
{
  0U, 20U, 8U, 8U, 10U, 12U, 12U
};

/* Positions use Q15 PWM microseconds for sub-microsecond continuity. */
static uint32_t servoStartPositionQ15[LEARM_SERVO_MAX_ID + 1U];
static uint32_t servoCurrentPositionQ15[LEARM_SERVO_MAX_ID + 1U];
static uint32_t servoTargetPositionQ15[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoCurrentPulseUs[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoTargetPulseUs[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoTotalSteps[LEARM_SERVO_MAX_ID + 1U];
static uint16_t servoStep[LEARM_SERVO_MAX_ID + 1U];
static uint8_t servoMoving[LEARM_SERVO_MAX_ID + 1U];

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

static uint32_t LeArm_PulseToQ15(uint16_t pulseUs)
{
  return (uint32_t)pulseUs * (uint32_t)LEARM_SERVO_Q15_ONE;
}

static uint16_t LeArm_Q15ToPulse(uint32_t positionQ15)
{
  return (uint16_t)((positionQ15 + ((uint32_t)LEARM_SERVO_Q15_ONE / 2U)) /
    (uint32_t)LEARM_SERVO_Q15_ONE);
}

static uint32_t LeArm_ClampPosition(int64_t positionQ15, uint32_t firstQ15, uint32_t secondQ15)
{
  uint32_t minimum = (firstQ15 < secondQ15) ? firstQ15 : secondQ15;
  uint32_t maximum = (firstQ15 > secondQ15) ? firstQ15 : secondQ15;

  if (positionQ15 < (int64_t)minimum)
  {
    return minimum;
  }
  if (positionQ15 > (int64_t)maximum)
  {
    return maximum;
  }
  return (uint32_t)positionQ15;
}

static uint16_t LeArm_DurationToSteps(uint16_t timeMs)
{
  uint16_t steps;

  if (timeMs < LEARM_SERVO_MIN_TIME_MS)
  {
    timeMs = LEARM_SERVO_MIN_TIME_MS;
  }
  else if (timeMs > LEARM_SERVO_MAX_TIME_MS)
  {
    timeMs = LEARM_SERVO_MAX_TIME_MS;
  }

  steps = (uint16_t)((timeMs + LEARM_SERVO_UPDATE_MS - 1U) / LEARM_SERVO_UPDATE_MS);
  if (steps == 0U)
  {
    steps = 1U;
  }
  return steps;
}

static uint16_t LeArm_MinStepsForMaxStep(uint8_t id, int64_t deltaQ15)
{
  uint64_t deltaAbsQ15;
  uint64_t maxStepQ15;
  uint64_t numerator;
  uint64_t denominator;
  uint64_t steps;

  if (deltaQ15 == 0L)
  {
    return 1U;
  }

  deltaAbsQ15 = (deltaQ15 < 0L) ? (uint64_t)(-deltaQ15) : (uint64_t)deltaQ15;
  maxStepQ15 = (uint64_t)servoMaxStepUs[id] * (uint64_t)LEARM_SERVO_Q15_ONE;
  if (maxStepQ15 == 0ULL)
  {
    return 1U;
  }

  /* Quintic smoothstep peak speed is 1.875x average speed, i.e. 15/8. */
  numerator = deltaAbsQ15 * 15ULL;
  denominator = maxStepQ15 * 8ULL;
  steps = (numerator + denominator - 1ULL) / denominator;
  if (steps == 0ULL)
  {
    steps = 1ULL;
  }
  if (steps > ((uint64_t)LEARM_SERVO_MAX_TIME_MS / LEARM_SERVO_UPDATE_MS))
  {
    steps = (uint64_t)LEARM_SERVO_MAX_TIME_MS / LEARM_SERVO_UPDATE_MS;
  }
  return (uint16_t)steps;
}

static uint32_t LeArm_EvaluateSmoothStepQ15(uint32_t startQ15,
  uint32_t targetQ15, uint16_t step, uint16_t totalSteps)
{
  int64_t t;
  int64_t tSquared;
  int64_t tCubed;
  int64_t tFourth;
  int64_t tFifth;
  int64_t smooth;
  int64_t delta;
  int64_t position;

  if ((totalSteps == 0U) || (step >= totalSteps))
  {
    return targetQ15;
  }

  t = ((int64_t)step << 15) / totalSteps;
  tSquared = (t * t) >> 15;
  tCubed = (tSquared * t) >> 15;
  tFourth = (tCubed * t) >> 15;
  tFifth = (tFourth * t) >> 15;
  smooth = (10LL * tCubed) - (15LL * tFourth) + (6LL * tFifth);
  delta = (int64_t)targetQ15 - (int64_t)startQ15;
  position = (int64_t)startQ15 + ((delta * smooth) >> 15);

  return LeArm_ClampPosition(position, startQ15, targetQ15);
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
    uint32_t initialPositionQ15 = LeArm_PulseToQ15(servoInitialPulseUs[id]);

    servoStartPositionQ15[id] = initialPositionQ15;
    servoCurrentPositionQ15[id] = initialPositionQ15;
    servoTargetPositionQ15[id] = initialPositionQ15;
    servoCurrentPulseUs[id] = servoInitialPulseUs[id];
    servoTargetPulseUs[id] = servoInitialPulseUs[id];
    servoTotalSteps[id] = 0U;
    servoStep[id] = 0U;
    servoMoving[id] = 0U;
    LeArm_ServoApply(id);

    /* Apply the configured power-on zero pulse before enabling the output channel. */
    if (HAL_TIM_PWM_Start(servoOutputs[id].timer, servoOutputs[id].channel) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void LeArm_ServoSetPulseAndTime(uint8_t id, uint16_t pulseUs, uint16_t timeMs)
{
  uint16_t steps;
  uint16_t minimumSteps;
  uint32_t targetPositionQ15;
  int64_t deltaQ15;

  if ((id < LEARM_SERVO_MIN_ID) || (id > LEARM_SERVO_MAX_ID))
  {
    return;
  }

  pulseUs = LeArm_ClampPulse(pulseUs);
  targetPositionQ15 = LeArm_PulseToQ15(pulseUs);
  deltaQ15 = (int64_t)targetPositionQ15 - (int64_t)servoCurrentPositionQ15[id];
  steps = LeArm_DurationToSteps(timeMs);
  minimumSteps = LeArm_MinStepsForMaxStep(id, deltaQ15);
  if (steps < minimumSteps)
  {
    steps = minimumSteps;
  }

  servoStartPositionQ15[id] = servoCurrentPositionQ15[id];
  servoTargetPositionQ15[id] = targetPositionQ15;
  servoTargetPulseUs[id] = pulseUs;
  servoTotalSteps[id] = steps;
  servoStep[id] = 0U;
  servoMoving[id] = (deltaQ15 != 0L) ? 1U : 0U;
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
      servoCurrentPositionQ15[id] = servoTargetPositionQ15[id];
      servoMoving[id] = 0U;
    }
    else
    {
      servoCurrentPositionQ15[id] = LeArm_EvaluateSmoothStepQ15(
        servoStartPositionQ15[id], servoTargetPositionQ15[id],
        servoStep[id], servoTotalSteps[id]);
    }

    servoCurrentPulseUs[id] = LeArm_Q15ToPulse(servoCurrentPositionQ15[id]);
    LeArm_ServoApply(id);
  }
}

void LeArm_ServoEmergencyStop(void)
{
  uint8_t id;

  for (id = LEARM_SERVO_MIN_ID; id <= LEARM_SERVO_MAX_ID; id++)
  {
    servoStartPositionQ15[id] = servoCurrentPositionQ15[id];
    servoTargetPositionQ15[id] = servoCurrentPositionQ15[id];
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
