/*
 * @file: module_bluetooth.c
 * @brief: 蓝牙调试指令解析实现 (全功能版)
 * @author: Gemini Assistant
 */

#include "util_bluetooth_tune.h"
#include "module_rise.h" // 引入 Rise 的枚举定义
#include <stdio.h>       // sscanf
#include <string.h>      // strstr, strcmp
#include <stdlib.h>      // atof
#include <stdarg.h> // 需要引入这个头文件
/* ===================================================================
 * 引用 module_rise.c 中的全局变量 
 * (变量名必须与 module_rise.c 中完全一致)
 * =================================================================== */

// --- 核心控制变量 ---
extern float Rise_K_Sync;

// --- 击打电机 (Hit) ---
extern float Rise_Hit_Target_Angle;
extern float Rise_Hit_Return_Angle;
extern float Rise_Hit_Target_Speed;
extern float Rise_Hit_Return_Time;
extern float RISE_HIT_ACCEL_LIMIT;

// --- 搓球电机 (Chop) ---
extern float Rise_Chop_Front_Target_Speed;
extern float Rise_Chop_Right_Target_Speed;
extern float Rise_Chop_Left_Target_Speed;

// --- 抬升电机 (Lift) ---
extern float Rise_Lift_Target_Speed;
extern float LIFT_RETURN_KP;
extern float LIFT_MAX_RETURN_SPEED;

// --- 自动流程时间参数 ---
extern float pre_spin_time;
extern float lift_time;
extern float drop_time;
extern float hit_action_time;

// --- PID 参数数组 (必须在原文件中去掉 const) ---
extern float Const_HitPosMotorParam[4][5];
extern float Const_HitSpdMotorParam[4][5];
extern float Const_LiftSpdMotorParam[4][5];
// 如果还需要调搓球PID，请把 Const_Chop... 也 extern 进来

// --- 外部函数引用 ---
extern void Rise_Set_ControlMode(uint8_t mode);
extern void Rise_Set_Torque_Output(float t1, float t2, float t3, float t4, float t5);
extern void Rise_Refresh_PID_Params(void); // 刷新 PID 参数到结构体
extern UART_HandleTypeDef huart2;

/**
 * @brief 蓝牙专用打印函数 (用法同 printf)
 * @example BT_Log("Speed Set: %.2f\r\n", 300.0f);
 */
void BT_Log(const char *format, ...) {
    char str_buf[128];
    va_list args;
    
    // 1. 格式化字符串
    va_start(args, format);
    vsnprintf(str_buf, sizeof(str_buf), format, args);
    va_end(args);
    
    // 2. 发送出去 (阻塞式发送，简单可靠)
    // 参数: 句柄, 数据指针, 长度, 超时时间
    HAL_UART_Transmit(&huart2, (uint8_t*)str_buf, strlen(str_buf), 100);
}

/* ===================================================================
 * ⚙️ 解析函数实现
 * =================================================================== */

/**
 * @brief 解析蓝牙指令
 * @note  指令格式: [Key]=[Value]#  (必须以 # 结尾)
 */
void Bluetooth_Parse_Command(char *cmd_str) {
    float val_f = 0.0f;
    int val_i = 0;
    int need_refresh_pid = 0; // 标记是否需要刷新 PID
	
	  BT_Log("CMD Recv: %s\r\n", cmd_str);

    // ================= [1. 模式切换] =================
    // 指令: M=1# (1:自动, 2:击球, 3:搓球, 4:抬升, 5:停止)
    if (sscanf(cmd_str, "M=%d", &val_i) == 1) {
        Rise_Set_ControlMode((uint8_t)val_i);
				BT_Log("OK!MODE=%d\r\n", val_i);
        // 安全保护：如果是切到停止模式，强制清零力矩
        if (val_i == Rise_Stop) {
            Rise_Set_Torque_Output(0, 0, 0, 0, 0);
        }
    }

// ================= [2. 击打参数 (Hit)] =================
    // A=45.0#  -> 目标角度
    else if (sscanf(cmd_str, "A=%f", &val_f) == 1) { 
        Rise_Hit_Target_Angle = val_f; 
        BT_Log("Hit Angle: %.1f\r\n", val_f); 
    }
    // S=300#   -> 目标速度
    else if (sscanf(cmd_str, "S=%f", &val_f) == 1) { 
        Rise_Hit_Target_Speed = val_f; 
        BT_Log("Hit Speed: %.1f\r\n", val_f); 
    }
    // Acc=5000# -> 最大加速度
    else if (sscanf(cmd_str, "Acc=%f", &val_f) == 1) { 
        RISE_HIT_ACCEL_LIMIT = val_f; 
        BT_Log("Hit Acc: %.1f\r\n", val_f); 
    }
    // T_Hit=0.5# -> 击打保持时间
    else if (sscanf(cmd_str, "T_Hit=%f", &val_f) == 1) { 
        hit_action_time = val_f; 
        BT_Log("Hit Time: %.2f s\r\n", val_f); 
    } 
    // T_Ret=1.0# -> 归位等待时间
    else if (sscanf(cmd_str, "T_Ret=%f", &val_f) == 1) { 
        Rise_Hit_Return_Time = val_f; 
        BT_Log("Return Time: %.2f s\r\n", val_f); 
    }

    // ================= [3. 搓球 & 抬升 (Chop & Lift)] =================
    // C=100#   -> 搓球速度
    else if (sscanf(cmd_str, "C=%f", &val_f) == 1) { 
        Rise_Chop_Front_Target_Speed = val_f; 
        BT_Log("Chop Front Spd: %.1f\r\n", val_f); 
    }
    else if (sscanf(cmd_str, "C=%f", &val_f) == 1) { 
        Rise_Chop_Right_Target_Speed = val_f; 
        BT_Log("Chop Right Spd: %.1f\r\n", val_f); 
    }
    else if (sscanf(cmd_str, "C=%f", &val_f) == 1) { 
        Rise_Chop_Left_Target_Speed = val_f; 
        BT_Log("Chop Left Spd: %.1f\r\n", val_f); 
    }
    // L=10#    -> 抬升速度
    else if (sscanf(cmd_str, "L=%f", &val_f) == 1) { 
        Rise_Lift_Target_Speed = val_f; 
        BT_Log("Lift Spd: %.1f\r\n", val_f); 
    }
    // K_Lift=0.02# -> 抬升归位 KP
    else if (sscanf(cmd_str, "K_Lift=%f", &val_f) == 1) { 
        LIFT_RETURN_KP = val_f; 
        BT_Log("Lift KP: %.4f\r\n", val_f); 
    }
    else if (sscanf(cmd_str, "Lift_time=%f", &val_f) == 1) { 
        lift_time = val_f; 
        BT_Log("Lift Time: %.4f\r\n", val_f); 
    }
    else if (sscanf(cmd_str, "Drop_time=%f", &val_f) == 1) { 
        drop_time = val_f; 
        BT_Log("Drop Time: %.4f\r\n", val_f); 
    }

    // ================= [4. PID 在线调参 (Hit Speed)] =================
    // HSP=1.5# -> Hit Speed P
    else if (sscanf(cmd_str, "HSP=%f", &val_f) == 1) { 
        Const_HitSpdMotorParam[0][0] = val_f; need_refresh_pid = 1; 
        BT_Log("Hit Spd P: %.5f\r\n", val_f);
    }
    // HSI=0.01# -> Hit Speed I
    else if (sscanf(cmd_str, "HSI=%f", &val_f) == 1) { 
        Const_HitSpdMotorParam[0][1] = val_f; need_refresh_pid = 1; 
        BT_Log("Hit Spd I: %.5f\r\n", val_f);
    }
    // HSD=0.0# -> Hit Speed D
    else if (sscanf(cmd_str, "HSD=%f", &val_f) == 1) { 
        Const_HitSpdMotorParam[0][2] = val_f; need_refresh_pid = 1; 
        BT_Log("Hit Spd D: %.5f\r\n", val_f);
    }

    // ================= [5. PID 在线调参 (Hit Position)] =================
    // HPP=0.9# -> Hit Pos P
    else if (sscanf(cmd_str, "HPP=%f", &val_f) == 1) { 
        Const_HitPosMotorParam[0][0] = val_f; need_refresh_pid = 1; 
        BT_Log("Hit Pos P: %.5f\r\n", val_f);
    }
    // HPI=0.9# -> Hit Pos I 
    else if (sscanf(cmd_str, "HPI=%f", &val_f) == 1) { 
        Const_HitPosMotorParam[0][1] = val_f; need_refresh_pid = 1; 
        BT_Log("Hit Pos I: %.5f\r\n", val_f);
    }
    // HPD=10.0# -> Hit Pos D
    else if (sscanf(cmd_str, "HPD=%f", &val_f) == 1) { 
        Const_HitPosMotorParam[0][2] = val_f; need_refresh_pid = 1; 
        BT_Log("Hit Pos D: %.5f\r\n", val_f);
    }

    // ================= [6. PID 在线调参 (Lift Speed)] =================
    // LSP=0.9# -> Lift Speed P
    else if (sscanf(cmd_str, "LSP=%f", &val_f) == 1) { 
        Const_LiftSpdMotorParam[0][0] = val_f; need_refresh_pid = 1; 
        BT_Log("Lift Spd P: %.5f\r\n", val_f);
    }
    // LIP=0.9# -> Lift Speed I
    else if (sscanf(cmd_str, "LIP=%f", &val_f) == 1) { 
        Const_LiftSpdMotorParam[0][1] = val_f; need_refresh_pid = 1; 
        BT_Log("Lift Spd I: %.5f\r\n", val_f);
    }
    // LSD=0.9# -> Lift Speed D
    else if (sscanf(cmd_str, "LSD=%f", &val_f) == 1) { 
        Const_LiftSpdMotorParam[0][2] = val_f; need_refresh_pid = 1; 
        BT_Log("Lift Spd D: %.5f\r\n", val_f);
    }

    // ================= [7. 特殊指令] =================
    // TEST# -> 立即触发击球测试
    else if (strstr(cmd_str, "TEST")) {
        Rise_Set_ControlMode(Rise_Jiqiu);
        BT_Log("TEST START!\r\n");
    }
    // STOP# -> 急停
    else if (strstr(cmd_str, "STOP")) {
        Rise_Set_ControlMode(Rise_Stop);
        Rise_Set_Torque_Output(0, 0, 0, 0, 0);
        BT_Log("!!! EMERGENCY STOP !!!\r\n");
    }

    // ================= [8. 刷新生效] =================
    if (need_refresh_pid) {
        Rise_Refresh_PID_Params();
        BT_Log("PID Refreshed OK!\r\n");
    }
}