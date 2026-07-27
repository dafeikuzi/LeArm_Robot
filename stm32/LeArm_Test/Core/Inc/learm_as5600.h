#ifndef LEARM_AS5600_H
#define LEARM_AS5600_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEARM_AS5600_RAW_MAX 4095U

void LeArm_As5600Init(void);
void LeArm_As5600Task(void);
uint8_t LeArm_As5600GetLatest(uint16_t *rawAngle, uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif
