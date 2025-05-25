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

	 // App init
	 Platform_Init();
	 Remote_RemotrControlInit();
	 Chassis_Init();
	 Gimbal_Init();
 } 



 void Init_Task(){



    for(;;){
       
		osDelay(1000);
    }

 }
