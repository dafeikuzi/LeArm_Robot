#ifndef LEARM_SERVO_H
#define LEARM_SERVO_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEARM_SERVO_MIN_ID       1U
#define LEARM_SERVO_MAX_ID       6U
#define LEARM_SERVO_MIN_PULSE_US 500U
#define LEARM_SERVO_MAX_PULSE_US 2500U

void LeArm_ServoInit(void);
void LeArm_ServoSetPulseAndTime(uint8_t id, uint16_t pulseUs, uint16_t timeMs);
void LeArm_ServoUpdate20ms(void);
void LeArm_ServoEmergencyStop(void);
uint16_t LeArm_ServoGetCurrentPulse(uint8_t id);
uint16_t LeArm_ServoGetTargetPulse(uint8_t id);
uint8_t LeArm_ServoGetMovingMask(void);

#ifdef __cplusplus
}
#endif

#endif
