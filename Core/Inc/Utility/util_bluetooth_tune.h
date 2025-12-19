/*
 * @Author: WenXin Tan 3086080053@qq.com
 * @Date: 2025-12-11 17:52:10
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2025-12-11 23:47:37
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Inc\Utility\util_bluetooth_tune.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/*
 * @file: module_bluetooth.h
 * @brief: 蓝牙调试模块头文件 - 包含所有指令解析接口
 */

#ifndef __UTIL_BLUETOOTH_TUNE_H__
#define __UTIL_BLUETOOTH_TUNE_H__

#include <stdint.h>

// 蓝牙接收缓冲区大小
#define BT_RX_BUF_SIZE 128 //稍微改大点，防止长指令溢出

/**
 * @brief 解析蓝牙发送过来的字符串指令
 * @param cmd_str: 接收到的以 '#' 结尾的字符串
 */
void Bluetooth_Parse_Command(char *cmd_str);
void BT_Log(const char *format, ...);

#endif /* __MODULE_BLUETOOTH_H__ */