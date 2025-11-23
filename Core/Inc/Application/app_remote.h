/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-05-25 18:03:38
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-10-31 20:50:22
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Inc\Application\app_remote.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_remote.h
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:57
 *  LastEditTime : 2023-09-23 21:35:07
 */



#ifndef GIM_REMOTE_CTRL_H
#define GIM_REMOTE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define REMOTE_TASK_PERIOD  1

typedef struct {
    uint8_t pending;
} Remote_RemoteControlTypeDef;

void Remote_Task(void const * argument);
void Remote_RemotrControlInit(void);
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr(void);
void Remote_ControlCom(void);
void Remote_RemoteProcess(void);
void Remote_RemoteProcessPlus(void);
void Remote_KeyMouseProcess(void);
void Remote_NucProcess(void);
void Remote_RiseProcess(void);
uint8_t Remote_Gesturejudge(void);
void Remote_Gesture(void);
void Remote_GestureFunction_1(void);
void Remote_GestureFunction_2(void);
void Remote_GestureFunction_3(void);
void Remote_GestureFunction_4(void);


#endif

#ifdef __cplusplus
}
#endif
