/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-10-31 18:52:29
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-11-22 19:41:15
 * @FilePath: \MDK-ARM\app_rise.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/*
 *  Project      :DeltaPlatform
 * 
 *  file         : app_platform.c
 *  Description  : This file contains DeltaPlatform task function
 *  LastEditors  : twx
 *  Date         : 2025-10-31
 *  LastEditTime : 
 */
 
#include "module_rise.h"
#include "app_rise.h"
#include "sys_dwt.h"
#include "util_vofa.h"
 /**
  * @brief          Rise task
  * @param          NULL
  * @retval         NULL
  */

	void Rise_Task(void const * argument) {
		 Rise_DataTypeDef *Rise = Rise_GetRisePtr();
		osDelay(500);
    for(;;) {   

		Vofa_Send_Data(
		Motor_Rise_Chop_Left_Motor.encoder.speed,
		Motor_Rise_Chop_Right_Motor.encoder.speed,
		Motor_Rise_Chop_Front_Motor.encoder.speed,
		Motor_Rise_Lift_Motor.encoder.standard_speed,
		Motor_Rise_Hit_Motor.encoder.angle
        );

      Rise_Update_Fdb();

			Rise_Control();
			Rise_Output();

			    
      osDelay(2);
    }
}



