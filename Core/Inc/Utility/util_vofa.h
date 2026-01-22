/*
 * @Author: WenXin Tan 3086080053@qq.com
 * @Date: 2025-11-11 20:47:04
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2026-01-11 17:48:33
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Inc\Utility\util_vofa.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-11-11 20:47:04
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2025-12-23 11:02:22
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Inc\Utility\util_vofa.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _VOFA_H_
#define _VOFA_H_

#include "main.h" // 确保包含了 STM32 HAL 库的头文件

// 1. 定义数据通道数量
#define CH_COUNT (6)

// 2. 定义数据帧结构体 (3个float + 4字节帧尾 = 16字节)
typedef struct Frame {
    float fdata[CH_COUNT];   // 3个 float 数据 (12 字节)
    unsigned char tail[5]; // 4 字节帧尾 (0x00, 0x00, 0x80, 0x7f)
} Frame_t;

// 3. 声明外部变量和函数
// 声明您的 UART 和 DMA 句柄 (请确保它们在 main.c 或其他地方定义为全局)
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_tx; // 假设 huart7 对应的 DMA TX 是这个

// 声明全局的发送缓冲区
extern Frame_t g_vofa_frame_buff;

// 4. 定义 DMA 剩余计数器宏 (!!! 关键的DMA安全检查 !!!)
// 适配您的 DMA 句柄
#define VOFA_DMA_REMAIN_SIZE (__HAL_DMA_GET_COUNTER(&hdma_uart7_tx))

// 5. 声明初始化和发送函数
void Vofa_Frame_Init(void);
void Vofa_Send_Data(float ch1, float ch2, float ch3, float ch4 , float ch5 , float ch6 );

#endif