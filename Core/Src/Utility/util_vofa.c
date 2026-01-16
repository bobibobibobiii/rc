#include "util_vofa.h"

// 1. 定义全局的发送缓冲区
Frame_t g_vofa_frame_buff;

// 2. 定义 UART 和 DMA 句柄 (如果您没有在 vofa.h 中 extern 它们)
// 如果您在 vofa.h 中 extern 了，请确保它们在别处 (如 main.c) 是全局可见的
// extern UART_HandleTypeDef huart2;
// extern DMA_HandleTypeDef hdma_usart2_tx;

/**
 * @brief 初始化 VOFA 帧的帧尾
 * @note  必须在 main 函数中调用一次
 */
void Vofa_Frame_Init(void)
{
    g_vofa_frame_buff.tail[0] = 0x00;
    g_vofa_frame_buff.tail[1] = 0x00;
    g_vofa_frame_buff.tail[2] = 0x80;
    g_vofa_frame_buff.tail[3] = 0x7f;
}

/**
 * @brief 准备数据并通过 DMA 发送
 * @param ch1 通道1数据 (float)
 * @param ch2 通道2数据 (float)
 * @param ch3 通道3数据 (float)
 * @param ch4 通道4数据 (float)
 * @param ch5 通道5数据 (float)
 * @param ch6 通道6数据 (float)
 * @param ch7 通道7数据 (float)
 */
void Vofa_Send_Data(float ch1, float ch2, float ch3, float ch4 )
{
    // 1. (!!! 关键的DMA安全检查 !!!)
    // 检查上一次 DMA 传输是否已经完成
    // 如果 DMA 计数器不为 0，说明 UART 正忙，则本次不发送，立即返回
    if (VOFA_DMA_REMAIN_SIZE != 0)
    {
        return; // 退出，防止数据冲突
    }

    // 2. 填充数据
    g_vofa_frame_buff.fdata[0] = ch1;
    g_vofa_frame_buff.fdata[1] = ch2;
    g_vofa_frame_buff.fdata[2] = ch3;
    g_vofa_frame_buff.fdata[3] = ch4;
    // g_vofa_frame_buff.fdata[4] = ch5;
    // g_vofa_frame_buff.fdata[5] = ch6;
    
    // 帧尾在 Vofa_Frame_Init() 中已填充，此处无需重复填充

    // 3. 启动 DMA 传输
    // (注意：您必须用 (uint8_t*) 来发送)
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&g_vofa_frame_buff, sizeof(Frame_t));
}