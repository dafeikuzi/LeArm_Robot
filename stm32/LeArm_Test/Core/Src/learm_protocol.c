#include "learm_protocol.h"

#include "learm_servo.h"
#include <stdio.h>
#include <string.h>

#define LEARM_FRAME_HEADER_1      0xA5U
#define LEARM_FRAME_HEADER_2      0x5AU
#define LEARM_PROTOCOL_VERSION    0x01U
#define LEARM_FRAME_MIN_LEN       8U
#define LEARM_FRAME_MAX_LEN       64U
#define LEARM_FRAME_QUEUE_SIZE    4U
#define LEARM_TX_FRAME_QUEUE_SIZE 8U
#define LEARM_MAX_PAYLOAD_LEN     (LEARM_FRAME_MAX_LEN - LEARM_FRAME_MIN_LEN)

#define LEARM_CMD_MOVE_PULSES     0x10U
#define LEARM_CMD_GET_STATUS      0x11U
#define LEARM_CMD_EMERGENCY_STOP  0x12U
#define LEARM_CMD_CLEAR_ESTOP     0x13U
#define LEARM_CMD_ACK             0x80U
#define LEARM_CMD_STATUS          0x81U

#define LEARM_STATUS_OK                 0U
#define LEARM_STATUS_UNSUPPORTED_COMMAND 2U
#define LEARM_STATUS_INVALID_PAYLOAD     3U
#define LEARM_STATUS_ESTOP_ACTIVE        4U

typedef enum
{
  LEARM_RX_WAIT_HEADER_1 = 0,
  LEARM_RX_WAIT_HEADER_2,
  LEARM_RX_WAIT_BODY
} LeArmRxState;

static UART_HandleTypeDef *protocolUart;
static volatile uint8_t uartRxByte;

static LeArmRxState rxState = LEARM_RX_WAIT_HEADER_1;
static uint8_t rxFrame[LEARM_FRAME_MAX_LEN];
static uint8_t rxIndex;
static uint8_t rxExpectedLength;

static uint8_t frameQueue[LEARM_FRAME_QUEUE_SIZE][LEARM_FRAME_MAX_LEN];
static uint8_t frameQueueLength[LEARM_FRAME_QUEUE_SIZE];
static volatile uint8_t frameQueueHead;
static volatile uint8_t frameQueueTail;
static uint8_t txFrameQueue[LEARM_TX_FRAME_QUEUE_SIZE][LEARM_FRAME_MAX_LEN];
static uint8_t txFrameQueueLength[LEARM_TX_FRAME_QUEUE_SIZE];
static uint8_t txFrameQueueHead;
static uint8_t txFrameQueueTail;
static volatile uint8_t uartTxBusy;
static volatile uint8_t uartTxComplete;
static uint8_t estopActive;

static uint16_t LeArm_ReadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void LeArm_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t LeArm_Crc16Ccitt(const uint8_t *data, uint8_t length)
{
  uint16_t crc = 0xFFFFU;
  uint8_t index;

  for (index = 0U; index < length; index++)
  {
    uint8_t bit;
    crc ^= (uint16_t)data[index] << 8;
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void LeArm_ProtocolResetParser(void)
{
  rxState = LEARM_RX_WAIT_HEADER_1;
  rxIndex = 0U;
  rxExpectedLength = 0U;
}

static void LeArm_ProtocolQueueFrame(void)
{
  uint8_t nextHead = (uint8_t)((frameQueueHead + 1U) % LEARM_FRAME_QUEUE_SIZE);

  if (nextHead == frameQueueTail)
  {
    return;
  }

  memcpy(frameQueue[frameQueueHead], rxFrame, rxExpectedLength);
  frameQueueLength[frameQueueHead] = rxExpectedLength;
  frameQueueHead = nextHead;
}

static uint8_t LeArm_ProtocolPopFrame(uint8_t *frame, uint8_t *length)
{
  uint8_t available = 0U;

  __disable_irq();
  if (frameQueueTail != frameQueueHead)
  {
    *length = frameQueueLength[frameQueueTail];
    memcpy(frame, frameQueue[frameQueueTail], *length);
    frameQueueTail = (uint8_t)((frameQueueTail + 1U) % LEARM_FRAME_QUEUE_SIZE);
    available = 1U;
  }
  __enable_irq();

  return available;
}

static void LeArm_ProtocolAcceptByte(uint8_t value)
{
  switch (rxState)
  {
    case LEARM_RX_WAIT_HEADER_1:
      if (value == LEARM_FRAME_HEADER_1)
      {
        rxFrame[0] = value;
        rxIndex = 1U;
        rxState = LEARM_RX_WAIT_HEADER_2;
      }
      break;

    case LEARM_RX_WAIT_HEADER_2:
      if (value == LEARM_FRAME_HEADER_2)
      {
        rxFrame[1] = value;
        rxIndex = 2U;
        rxState = LEARM_RX_WAIT_BODY;
      }
      else if (value == LEARM_FRAME_HEADER_1)
      {
        rxFrame[0] = value;
        rxIndex = 1U;
      }
      else
      {
        LeArm_ProtocolResetParser();
      }
      break;

    case LEARM_RX_WAIT_BODY:
      rxFrame[rxIndex] = value;
      rxIndex++;
      if (rxIndex == 6U)
      {
        if (rxFrame[5] > LEARM_MAX_PAYLOAD_LEN)
        {
          LeArm_ProtocolResetParser();
          break;
        }
        rxExpectedLength = (uint8_t)(LEARM_FRAME_MIN_LEN + rxFrame[5]);
      }
      if ((rxExpectedLength != 0U) && (rxIndex >= rxExpectedLength))
      {
        LeArm_ProtocolQueueFrame();
        LeArm_ProtocolResetParser();
      }
      break;

    default:
      LeArm_ProtocolResetParser();
      break;
  }
}

static uint8_t LeArm_ProtocolCanQueueTx(void)
{
  uint8_t nextHead = (uint8_t)((txFrameQueueHead + 1U) % LEARM_TX_FRAME_QUEUE_SIZE);

  return (nextHead != txFrameQueueTail);
}

static void LeArm_ProtocolServiceTx(void)
{
  if (protocolUart == 0)
  {
    return;
  }

  if (uartTxComplete != 0U)
  {
    uartTxComplete = 0U;
    txFrameQueueTail = (uint8_t)((txFrameQueueTail + 1U) % LEARM_TX_FRAME_QUEUE_SIZE);
  }

  if ((uartTxBusy != 0U) || (txFrameQueueTail == txFrameQueueHead))
  {
    return;
  }

  uartTxBusy = 1U;
  if (HAL_UART_Transmit_IT(protocolUart, txFrameQueue[txFrameQueueTail],
    txFrameQueueLength[txFrameQueueTail]) != HAL_OK)
  {
    uartTxBusy = 0U;
  }
}

static uint8_t LeArm_ProtocolSendFrame(uint8_t sequence, uint8_t command,
  const uint8_t *payload, uint8_t payloadLength)
{
  uint8_t frame[LEARM_FRAME_MAX_LEN];
  uint8_t length;
  uint8_t nextHead;
  uint16_t crc;

  if ((protocolUart == 0) || (payloadLength > LEARM_MAX_PAYLOAD_LEN) ||
    (LeArm_ProtocolCanQueueTx() == 0U))
  {
    return 0U;
  }

  frame[0] = LEARM_FRAME_HEADER_1;
  frame[1] = LEARM_FRAME_HEADER_2;
  frame[2] = LEARM_PROTOCOL_VERSION;
  frame[3] = sequence;
  frame[4] = command;
  frame[5] = payloadLength;
  if (payloadLength > 0U)
  {
    memcpy(&frame[6], payload, payloadLength);
  }
  crc = LeArm_Crc16Ccitt(&frame[2], (uint8_t)(4U + payloadLength));
  frame[6U + payloadLength] = (uint8_t)(crc & 0xFFU);
  frame[7U + payloadLength] = (uint8_t)((crc >> 8) & 0xFFU);
  length = (uint8_t)(LEARM_FRAME_MIN_LEN + payloadLength);
  memcpy(txFrameQueue[txFrameQueueHead], frame, length);
  txFrameQueueLength[txFrameQueueHead] = length;
  nextHead = (uint8_t)((txFrameQueueHead + 1U) % LEARM_TX_FRAME_QUEUE_SIZE);
  txFrameQueueHead = nextHead;
  return 1U;
}

static uint8_t LeArm_ProtocolSendRaw(const uint8_t *data, uint8_t length)
{
  uint8_t nextHead;

  if ((protocolUart == 0) || (data == 0) || (length == 0U) ||
    (length > LEARM_FRAME_MAX_LEN) || (LeArm_ProtocolCanQueueTx() == 0U))
  {
    return 0U;
  }

  memcpy(txFrameQueue[txFrameQueueHead], data, length);
  txFrameQueueLength[txFrameQueueHead] = length;
  nextHead = (uint8_t)((txFrameQueueHead + 1U) % LEARM_TX_FRAME_QUEUE_SIZE);
  txFrameQueueHead = nextHead;
  return 1U;
}

static void LeArm_ProtocolSendAck(uint8_t sequence, uint8_t requestCommand, uint8_t status)
{
  uint8_t payload[2] = {requestCommand, status};
  LeArm_ProtocolSendFrame(sequence, LEARM_CMD_ACK, payload, sizeof(payload));
}

static void LeArm_ProtocolSendStatus(uint8_t sequence)
{
  uint8_t payload[26];
  uint8_t id;
  uint8_t movingMask = LeArm_ServoGetMovingMask();

  payload[0] = 0U;
  if (movingMask != 0U)
  {
    payload[0] |= 0x01U;
  }
  if (estopActive != 0U)
  {
    payload[0] |= 0x02U;
  }
  payload[1] = movingMask;
  for (id = LEARM_SERVO_MIN_ID; id <= LEARM_SERVO_MAX_ID; id++)
  {
    LeArm_WriteU16(&payload[2U + ((id - 1U) * 2U)], LeArm_ServoGetCurrentPulse(id));
    LeArm_WriteU16(&payload[14U + ((id - 1U) * 2U)], LeArm_ServoGetTargetPulse(id));
  }
  LeArm_ProtocolSendFrame(sequence, LEARM_CMD_STATUS, payload, sizeof(payload));
}

static int16_t LeArm_ProtocolPulseToDegrees(uint16_t pulseUs)
{
  int32_t numerator = ((int32_t)pulseUs - 1500L) * 90L;

  if (numerator >= 0L)
  {
    return (int16_t)((numerator + 500L) / 1000L);
  }
  return (int16_t)((numerator - 500L) / 1000L);
}

void LeArm_ProtocolSendAngleReport(void)
{
  char message[64];
  int length;

  length = snprintf(message, sizeof(message),
    "ANG J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d deg\r\n",
    LeArm_ProtocolPulseToDegrees(LeArm_ServoGetCurrentPulse(1U)),
    LeArm_ProtocolPulseToDegrees(LeArm_ServoGetCurrentPulse(2U)),
    LeArm_ProtocolPulseToDegrees(LeArm_ServoGetCurrentPulse(3U)),
    LeArm_ProtocolPulseToDegrees(LeArm_ServoGetCurrentPulse(4U)),
    LeArm_ProtocolPulseToDegrees(LeArm_ServoGetCurrentPulse(5U)),
    LeArm_ProtocolPulseToDegrees(LeArm_ServoGetCurrentPulse(6U)));
  if ((length <= 0) || (length >= (int)sizeof(message)))
  {
    return;
  }

  (void)LeArm_ProtocolSendRaw((const uint8_t *)message, (uint8_t)length);
}

static uint8_t LeArm_ProtocolHandleMovePulses(const uint8_t *payload, uint8_t payloadLength)
{
  uint16_t moveTime;
  uint8_t servoCount;
  uint8_t seenMask = 0U;
  uint8_t index;

  if (estopActive != 0U)
  {
    return LEARM_STATUS_ESTOP_ACTIVE;
  }
  if (payloadLength < 3U)
  {
    return LEARM_STATUS_INVALID_PAYLOAD;
  }

  moveTime = LeArm_ReadU16(payload);
  servoCount = payload[2];
  if ((moveTime < 20U) || (moveTime > 30000U) ||
    (servoCount == 0U) || (servoCount > LEARM_SERVO_MAX_ID) ||
    (payloadLength != (uint8_t)(3U + (servoCount * 3U))))
  {
    return LEARM_STATUS_INVALID_PAYLOAD;
  }

  for (index = 0U; index < servoCount; index++)
  {
    uint8_t offset = (uint8_t)(3U + (index * 3U));
    uint8_t id = payload[offset];
    uint16_t pulse = LeArm_ReadU16(&payload[offset + 1U]);
    uint8_t idMask;

    if ((id < LEARM_SERVO_MIN_ID) || (id > LEARM_SERVO_MAX_ID) ||
      (pulse < LEARM_SERVO_MIN_PULSE_US) || (pulse > LEARM_SERVO_MAX_PULSE_US) ||
      ((id == 1U) && (pulse > 1500U)))
    {
      return LEARM_STATUS_INVALID_PAYLOAD;
    }
    idMask = (uint8_t)(1U << (id - 1U));
    if ((seenMask & idMask) != 0U)
    {
      return LEARM_STATUS_INVALID_PAYLOAD;
    }
    seenMask |= idMask;
  }

  for (index = 0U; index < servoCount; index++)
  {
    uint8_t offset = (uint8_t)(3U + (index * 3U));
    LeArm_ServoSetPulseAndTime(payload[offset], LeArm_ReadU16(&payload[offset + 1U]), moveTime);
  }
  return LEARM_STATUS_OK;
}

static void LeArm_ProtocolHandleFrame(const uint8_t *frame, uint8_t frameLength)
{
  uint8_t sequence;
  uint8_t command;
  uint8_t payloadLength;
  const uint8_t *payload;
  uint16_t expectedCrc;
  uint16_t actualCrc;

  if (frameLength < LEARM_FRAME_MIN_LEN)
  {
    return;
  }
  payloadLength = frame[5];
  if (frameLength != (uint8_t)(LEARM_FRAME_MIN_LEN + payloadLength))
  {
    return;
  }
  expectedCrc = LeArm_ReadU16(&frame[frameLength - 2U]);
  actualCrc = LeArm_Crc16Ccitt(&frame[2], (uint8_t)(4U + payloadLength));
  if (expectedCrc != actualCrc)
  {
    return;
  }

  sequence = frame[3];
  command = frame[4];
  payload = &frame[6];
  if ((frame[2] == LEARM_PROTOCOL_VERSION) && (command == LEARM_CMD_EMERGENCY_STOP) &&
    (payloadLength == 0U))
  {
    LeArm_ServoEmergencyStop();
    estopActive = 1U;
    if (LeArm_ProtocolCanQueueTx() != 0U)
    {
      LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_OK);
    }
    return;
  }
  if (LeArm_ProtocolCanQueueTx() == 0U)
  {
    return;
  }
  if (frame[2] != LEARM_PROTOCOL_VERSION)
  {
    LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_UNSUPPORTED_COMMAND);
    return;
  }

  switch (command)
  {
    case LEARM_CMD_MOVE_PULSES:
      LeArm_ProtocolSendAck(sequence, command, LeArm_ProtocolHandleMovePulses(payload, payloadLength));
      break;

    case LEARM_CMD_GET_STATUS:
      if (payloadLength == 0U)
      {
        LeArm_ProtocolSendStatus(sequence);
      }
      else
      {
        LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_INVALID_PAYLOAD);
      }
      break;

    case LEARM_CMD_EMERGENCY_STOP:
      LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_INVALID_PAYLOAD);
      break;

    case LEARM_CMD_CLEAR_ESTOP:
      if (payloadLength == 0U)
      {
        estopActive = 0U;
        LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_OK);
      }
      else
      {
        LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_INVALID_PAYLOAD);
      }
      break;

    default:
      LeArm_ProtocolSendAck(sequence, command, LEARM_STATUS_UNSUPPORTED_COMMAND);
      break;
  }
}

void LeArm_ProtocolInit(UART_HandleTypeDef *huart)
{
  protocolUart = huart;
  LeArm_ProtocolResetParser();
  frameQueueHead = 0U;
  frameQueueTail = 0U;
  txFrameQueueHead = 0U;
  txFrameQueueTail = 0U;
  uartTxBusy = 0U;
  uartTxComplete = 0U;
  estopActive = 0U;

  if (protocolUart != 0)
  {
    (void)HAL_UART_Receive_IT(protocolUart, (uint8_t *)&uartRxByte, 1U);
  }
}

void LeArm_ProtocolTask(void)
{
  uint8_t frame[LEARM_FRAME_MAX_LEN];
  uint8_t frameLength;

  /* HAL stops interrupt reception after a blocking UART error such as ORE. */
  if ((protocolUart != 0) && (protocolUart->RxState == HAL_UART_STATE_READY))
  {
    LeArm_ProtocolResetParser();
    (void)HAL_UART_Receive_IT(protocolUart, (uint8_t *)&uartRxByte, 1U);
  }

  LeArm_ProtocolServiceTx();
  while (LeArm_ProtocolPopFrame(frame, &frameLength) != 0U)
  {
    LeArm_ProtocolHandleFrame(frame, frameLength);
  }
  LeArm_ProtocolServiceTx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == protocolUart)
  {
    uartTxBusy = 0U;
    uartTxComplete = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == protocolUart)
  {
    LeArm_ProtocolAcceptByte(uartRxByte);
    (void)HAL_UART_Receive_IT(protocolUart, (uint8_t *)&uartRxByte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == protocolUart)
  {
    /* Discard a partial frame and restore the one-byte interrupt receiver. */
    LeArm_ProtocolResetParser();
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    if (huart->RxState == HAL_UART_STATE_READY)
    {
      (void)HAL_UART_Receive_IT(protocolUart, (uint8_t *)&uartRxByte, 1U);
    }
  }
}
