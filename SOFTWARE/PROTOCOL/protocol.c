#include "protocol.h"

FrameTypeDef UART_Frame; // |0x3e|QoS|Data|0x0d|0x0a|

uint8_t EMPTY_BUF[DATA_MAX_LEN];

uint8_t FrameProcess(FrameTypeDef *Frame, uint8_t COMID, uint8_t data)
{
    static uint8_t state1;                  // 当前的帧处理状态
    static uint8_t QoSTemp1;                // 暂存的QoS等级
    static uint16_t count1;                 // 已经接收到的数据长度
    static uint8_t dataTemp1[DATA_MAX_LEN]; // 已经接收到的数据缓冲

    static uint8_t state2;                  // 当前的帧处理状态
    static uint8_t QoSTemp2;                // 暂存的QoS等级
    static uint16_t count2;                 // 已经接收到的数据长度
    static uint8_t dataTemp2[DATA_MAX_LEN]; // 已经接收到的数据缓冲

    static uint8_t state3;                  // 当前的帧处理状态
    static uint8_t QoSTemp3;                // 暂存的QoS等级
    static uint16_t count3;                 // 已经接收到的数据长度
    static uint8_t dataTemp3[DATA_MAX_LEN]; // 已经接收到的数据缓冲

    switch (COMID)
    {
    case COM1:
    {
        switch (state1)
        {
        case 0:
        {
            if (data == HEAD0)
            {
                state1 = 1;
                return STATUS_OK;
            }
            else
            {
                state1 = 0;
                return START_ERR;
            }
        }
        case 1:
        {
            if (data == '0' || data == '1')
            {
                QoSTemp1 = data;
                state1 = 2;
                return STATUS_OK;
            }
            else
            {
                state1 = 0;
                return QOS_ERR;
            }
        }
        case 2:
        {
            if (data == TAIL0)
            {
                state1 = 3;
                return STATUS_OK;
            }
            else
            {
                if (count1 < DATA_MAX_LEN - 1)
                {
                    dataTemp1[count1++] = data;
                    state1 = 2;
                    return STATUS_OK;
                }
                else
                {
                    state1 = 0;
                    QoSTemp1 = 0;
                    count1 = 0;
                    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp1, DATA_MAX_LEN); // 清空缓冲区
                    if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                    {
                        printf("DMA transfer error\r\n");
                    }
                    __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                    return LEN_ERR;
                }
            }
        }
        case 3:
        {
            if (data == TAIL1)
            {
                dataTemp1[count1] = '\0';
                HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)dataTemp1, (uint32_t)Frame->COM1_BUF, DATA_MAX_LEN); // 接收一个完整帧
                if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                {
                    printf("DMA transfer error\r\n");
                }
                __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                Frame->COM1_STA = NOT_EMPTY;
                if (QoSTemp1 == '1')
                {
                    printf("Received!\r\n");
                }
                state1 = 0;
                QoSTemp1 = 0;
                count1 = 0;
                HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp1, DATA_MAX_LEN); // 清空缓冲区
                if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                {
                    printf("DMA transfer error\r\n");
                }
                __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                return STATUS_OK;
            }
            else
            {
                if (count1 < DATA_MAX_LEN - 2)
                {
                    dataTemp1[count1++] = TAIL0;
                    dataTemp1[count1++] = data;
                    state1 = 2;
                    return STATUS_OK;
                }
                else
                {
                    state1 = 0;
                    QoSTemp1 = 0;
                    count1 = 0;
                    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp1, DATA_MAX_LEN); // 清空缓冲区
                    if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                    {
                        printf("DMA transfer error\r\n");
                    }
                    __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                    return LEN_ERR;
                }
            }
        }
        default:
        {
            return UNKNOWN_ERR;
        }
        }
    }
    case COM2:
    {
        switch (state2)
        {
        case 0:
        {
            if (data == HEAD0)
            {
                state2 = 1;
                return STATUS_OK;
            }
            else
            {
                state2 = 0;
                return START_ERR;
            }
        }
        case 1:
        {
            if (data == '0' || data == '1')
            {
                QoSTemp2 = data;
                state2 = 2;
                return STATUS_OK;
            }
            else
            {
                state2 = 0;
                return QOS_ERR;
            }
        }
        case 2:
        {
            if (data == TAIL0)
            {
                state2 = 3;
                return STATUS_OK;
            }
            else
            {
                if (count2 < DATA_MAX_LEN - 1)
                {
                    dataTemp2[count2++] = data;
                    state2 = 2;
                    return STATUS_OK;
                }
                else
                {
                    state2 = 0;
                    QoSTemp2 = 0;
                    count2 = 0;
                    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp2, DATA_MAX_LEN); // 清空缓冲区
                    if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                    {
                        printf("DMA transfer error\r\n");
                    }
                    __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                    return LEN_ERR;
                }
            }
        }
        case 3:
        {
            if (data == TAIL1)
            {
                dataTemp2[count2] = '\0';
                HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)dataTemp2, (uint32_t)Frame->COM2_BUF, DATA_MAX_LEN); // 接收一个完整帧
                if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                {
                    printf("DMA transfer error\r\n");
                }
                __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                Frame->COM2_STA = NOT_EMPTY;
                if (QoSTemp2 == '1')
                {
                    printf("Received!\r\n");
                }
                state2 = 0;
                QoSTemp2 = 0;
                count2 = 0;
                HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp2, DATA_MAX_LEN); // 清空缓冲区
                if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                {
                    printf("DMA transfer error\r\n");
                }
                __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                return STATUS_OK;
            }
            else
            {
                if (count2 < DATA_MAX_LEN - 2)
                {
                    dataTemp2[count2++] = TAIL0;
                    dataTemp2[count2++] = data;
                    state2 = 2;
                    return STATUS_OK;
                }
                else
                {
                    state2 = 0;
                    QoSTemp2 = 0;
                    count2 = 0;
                    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp2, DATA_MAX_LEN); // 清空缓冲区
                    if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                    {
                        printf("DMA transfer error\r\n");
                    }
                    __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                    return LEN_ERR;
                }
            }
        }
        default:
        {
            return UNKNOWN_ERR;
        }
        }
    }
    case COM3:
    {
        switch (state3)
        {
        case 0:
        {
            if (data == HEAD0)
            {
                state3 = 1;
                return STATUS_OK;
            }
            else
            {
                state3 = 0;
                return START_ERR;
            }
        }
        case 1:
        {
            if (data == '0' || data == '1')
            {
                QoSTemp3 = data;
                state3 = 2;
                return STATUS_OK;
            }
            else
            {
                state3 = 0;
                return QOS_ERR;
            }
        }
        case 2:
        {
            if (data == TAIL0)
            {
                state3 = 3;
                return STATUS_OK;
            }
            else
            {
                if (count3 < DATA_MAX_LEN - 1)
                {
                    dataTemp3[count3++] = data;
                    state3 = 2;
                    return STATUS_OK;
                }
                else
                {
                    state3 = 0;
                    QoSTemp3 = 0;
                    count3 = 0;
                    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp3, DATA_MAX_LEN); // 清空缓冲区
                    if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                    {
                        printf("DMA transfer error\r\n");
                    }
                    __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                    return LEN_ERR;
                }
            }
        }
        case 3:
        {
            if (data == TAIL1)
            {
                dataTemp3[count3] = '\0';
                HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)dataTemp3, (uint32_t)Frame->COM3_BUF, DATA_MAX_LEN); // 接收一个完整帧
                if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                {
                    printf("DMA transfer error\r\n");
                }
                __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                Frame->COM3_STA = NOT_EMPTY;
                if (QoSTemp3 == '1')
                {
                    printf("Received!\r\n");
                }
                state3 = 0;
                QoSTemp3 = 0;
                count3 = 0;
                HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp3, DATA_MAX_LEN); // 清空缓冲区
                if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                {
                    printf("DMA transfer error\r\n");
                }
                __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                return STATUS_OK;
            }
            else
            {
                if (count3 < DATA_MAX_LEN - 2)
                {
                    dataTemp3[count3++] = TAIL0;
                    dataTemp3[count3++] = data;
                    state3 = 2;
                    return STATUS_OK;
                }
                else
                {
                    state3 = 0;
                    QoSTemp3 = 0;
                    count3 = 0;
                    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)EMPTY_BUF, (uint32_t)dataTemp3, DATA_MAX_LEN); // 清空缓冲区
                    if (HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0, HAL_DMA_FULL_TRANSFER, 1000) != HAL_OK)
                    {
                        printf("DMA transfer error\r\n");
                    }
                    __HAL_DMA_CLEAR_FLAG(&hdma_memtomem_dma2_stream0, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_memtomem_dma2_stream0));
                    return LEN_ERR;
                }
            }
        }
        default:
        {
            return UNKNOWN_ERR;
        }
        }
    }
    default:
    {
        printf("Invalid COMID!\r\n");
        return UNKNOWN_ERR;
    }
    }
}
