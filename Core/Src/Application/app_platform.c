/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-05-27 18:52:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-10-31 19:09:47
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Src\Application\app_platform.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-05-27 18:52:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-10-31 19:03:49
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Src\Application\app_platform.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/*
 *  Project      :DeltaPlatform
 * 
 *  file         : app_platform.c
 *  Description  : This file contains DeltaPlatform task function
 *  LastEditors  : lkn
 *  Date         : 2024-7-2
 *  LastEditTime : 2024-7-3
 */
 
#include "module_platform.h"
#include "app_platform.h"
#include "sys_dwt.h"
 /**
  * @brief          Platform task
  * @param          NULL
  * @retval         NULL
  */

	void Platform_Task(void const * argument) {
		 Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
		osDelay(500);
    for(;;) {   

      Platform_Update_Fdb();
//			Platform_Check();
//			if(Platform->error_code == 0){
//      Platform_Control();	 
//      }
//			else{
//      Platform_Set_Torque_Output(0,0,0);
//			}
			Platform_Control();
			Platform_Output();
			    
      osDelay(100);
    }
}
