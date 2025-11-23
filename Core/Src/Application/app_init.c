/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-05-25 20:29:34
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-10-31 20:42:27
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Src\Application\app_init.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#include "app_init.h"
#include "app_remote.h"
#include "sys_dwt.h"
#include "util_can.h"
#include "module_platform.h"
#include "module_communicate.h"
#include "periph_motor.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "periph_remote.h"
#include "periph_dt35.h"
#include "comm_common.h"
#include "app_serve.h"
#include "app_gimbal.h"
#include "module_chassis.h"
#include "module_serve.h"
#include "app_rise.h"
#include "module_rise.h"
 
 void Init_InitAll() {
	 // system init
	 DWT_Init(168);

	 // uart init
	 Remote_InitRemote();
	 Comm_InitComm();
	 
	 // util init
	 Can_InitFilterAndStart(&hcan1);
	 Can_InitFilterAndStart(&hcan2);
	 
	 // periph init
	 DT35_Init();
	 Motor_InitAllMotors();
	 HAL_Delay(500);
	 
	 Module_ServeInit();

	 // App init
	 Platform_Init();
	 Remote_RemotrControlInit();
	 Chassis_Init();
	 Gimbal_Init();
	 Serve_Init();
	 Rise_Init();


 } 



 void Init_Task(){



    for(;;){
       
		osDelay(1000);
    }

 }
