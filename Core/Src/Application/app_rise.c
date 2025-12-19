/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-10-31 18:52:29
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2025-12-19 10:19:49
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Src\Application\app_rise.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/*
 *  Project      :Rise
 * 
 *  file         : app_rise.c
 *  Description  : This file contains Rise task function
 *  LastEditors  : twx
 *  Date         : 2025-10-31
 *  LastEditTime : 
 */
 
#include "module_rise.h"
#include "app_rise.h"
#include "sys_dwt.h"
#include "util_vofa.h"
#include <string.h>
#include "util_bluetooth_tune.h"
 /**
  * @brief          Rise task
  * @param          NULL
  * @retval         NULL
  */

extern volatile uint8_t bt_cmd_received_flag; 
extern char bt_rx_buffer[];
extern void Bluetooth_Parse_Command(char *cmd_str);

	void Rise_Task(void const * argument) {
		 Rise_DataTypeDef *Rise = Rise_GetRisePtr();
		osDelay(500);
    for(;;) {   
		// 定义一个计数器
    	static uint32_t heartbeat_cnt = 0;

		if (bt_cmd_received_flag == 1) {
          // 这里调用解析函数是安全的
          Bluetooth_Parse_Command(bt_rx_buffer); 
          
          // 处理完清除标志
          bt_cmd_received_flag = 0;
          
          // 可选：清空buffer防止脏数据
          memset(bt_rx_buffer, 0, BT_RX_BUF_SIZE);
      }
	  // ==========================================
        // ★★★ 新增测试代码：每秒强制发一次 ★★★
        // ==========================================
        heartbeat_cnt++;
        if (heartbeat_cnt % 1000 == 0) { // 假设 osDelay(1) 是 1ms，这里就是 1秒
             BT_Log("STM32 Alive\r\n"); 
        }
        // ==========================================

		// Vofa_Send_Data(
		// Motor_Rise_Chop_Left_Motor.encoder.speed,
		// Motor_Rise_Chop_Right_Motor.encoder.speed,
		// Motor_Rise_Chop_Front_Motor.encoder.speed,
		// Motor_Rise_Lift_Motor.encoder.standard_speed,
		// Motor_Rise_Hit_Motor.encoder.angle
        // );

        Rise_Update_Fdb();

        Rise_Check();
        if(Rise->error_code == 0){
            Rise_Control();	 
          }
        else{
            Rise_Set_Torque_Output(0,0,0,0,0);
          }
		// Rise_Control();	
		
		Rise_Output();

			    
      osDelay(1);
    }
}



