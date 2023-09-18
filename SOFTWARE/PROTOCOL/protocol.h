#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "sys.h"
#include "main.h"
#include "dma.h"

#define DATA_MAX_LEN 256

#define EMPTY 0
#define NOT_EMPTY 1

#define COM1 1
#define COM2 2
#define COM3 3

#define HEAD0 0x3e
#define TAIL0 0x0d
#define TAIL1 0x0a

#define STATUS_OK 0
#define START_ERR 1
#define QOS_ERR 2
#define LEN_ERR 3
#define TAIL0_ERR 4
#define TAIL1_ERR 5
#define UNKNOWN_ERR 6

typedef struct FrameType
{
    uint8_t COM1_BUF[DATA_MAX_LEN];
    uint8_t COM2_BUF[DATA_MAX_LEN];
    uint8_t COM3_BUF[DATA_MAX_LEN];
    uint16_t COM1_STA;
    uint16_t COM2_STA;
    uint16_t COM3_STA;

} FrameTypeDef;

extern FrameTypeDef UART_Frame; // |0x3e|QoS|Data|0x0d|0x0a|

uint8_t FrameProcess(FrameTypeDef *Frame, uint8_t COMID, uint8_t data);

#endif
