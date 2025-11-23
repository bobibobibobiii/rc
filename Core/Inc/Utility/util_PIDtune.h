/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-11-11 21:53:27
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-11-11 21:54:55
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Inc\Utility\util_PIDtune.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _UTIL_PISTUNE_H_
#define _UTIL_PIDTUNE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" // 包含它，以便我们可以传递 DataBuff


extern volatile uint8_t g_vofa_command_ready; // 中断设置的“标志位”
extern volatile uint8_t DataBuff[200];      // 中断存入的“数据缓冲”
extern volatile uint16_t RxLine;            // 中断记录的“数据长度”

/**
 * @brief 解析来自 VOFA+ 的命令字符串
 * @param buff 包含完整命令 (例如 "LSP=1.23!") 的 DataBuff
 */
void Tune_Manager_Parse_Command(uint8_t* buff);

#ifdef __cplusplus
}
#endif

#endif