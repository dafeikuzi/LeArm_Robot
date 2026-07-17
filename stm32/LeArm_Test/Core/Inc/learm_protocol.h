#ifndef LEARM_PROTOCOL_H
#define LEARM_PROTOCOL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void LeArm_ProtocolInit(UART_HandleTypeDef *huart);
void LeArm_ProtocolTask(void);

#ifdef __cplusplus
}
#endif

#endif
