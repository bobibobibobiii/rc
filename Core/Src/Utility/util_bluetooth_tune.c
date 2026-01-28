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
#include <stdarg.h>      // 需要引入这个头文件
/* ===================================================================
 * 引用 module_rise.c 中的全局变量
 * (变量名必须与 module_rise.c 中完全一致)
 * =================================================================== */
// --- 击打电机 (Hit) ---
extern float Rise_Hit_Ready_Speed;
extern float Rise_Hit_Ready_Angle;
extern float Rise_Hit_Burst_Speed;

// --- 搓球电机 (Chop) ---
extern float Rise_Chop_Front_Target_Speed;
extern float Rise_Chop_Right_Target_Speed;
extern float Rise_Chop_Left_Target_Speed;

// --- 抬升电机 (Lift) ---
extern float Rise_Lift_Target_Speed;
extern float Rise_Lift_Target_Dist;
extern float Rise_Lift_Kp;
extern float LIFT_MAX_RETURN_SPEED;

// --- 自动流程时间 & 阈值参数 ---
extern float pre_spin_time;
extern float lift_time;
extern float drop_time;
extern float hit_action_time;
extern float Lift_torque_threshold;

// --- 重力补偿 & 零点 ---
extern float Rise_Gravity_Comp_Max_Current;
extern float Rise_Zero_Angle_Offset;

// --- 外部函数引用 ---
extern void Rise_Set_ControlMode(uint8_t mode);
extern void Rise_Set_Torque_Output(float t0, float t1, float t2, float t3, float t4, float t5);
extern UART_HandleTypeDef huart2;

/**
 * @brief 蓝牙专用打印函数 (用法同 printf)
 * @example BT_Log("Speed Set: %.2f\r\n", 300.0f);
 */
void BT_Log(const char *format, ...)
{
    char str_buf[128];
    va_list args;

    // 1. 格式化字符串
    va_start(args, format);
    vsnprintf(str_buf, sizeof(str_buf), format, args);
    va_end(args);

    // 2. 发送出去 (阻塞式发送，简单可靠)
    // 参数: 句柄, 数据指针, 长度, 超时时间
    HAL_UART_Transmit(&huart2, (uint8_t *)str_buf, strlen(str_buf), 100);
}

/* ===================================================================
 * ⚙️ 解析函数实现
 * =================================================================== */

/**
 * @brief 解析蓝牙指令
 * @note  指令格式: [Key]=[Value]#  (必须以 # 结尾)
 */
void Bluetooth_Parse_Command(char *cmd_str)
{
    float val_f = 0.0f;
    int val_i = 0;
    int need_refresh_pid = 0; // 标记是否需要刷新 PID

    BT_Log("CMD Recv: %s\r\n", cmd_str);

    // ================= [1. 模式切换] =================
    // 指令: M=1# (1:自动, 2:击球, 3:搓球, 4:抬升, 5:停止)
    if (sscanf(cmd_str, "M=%d", &val_i) == 1)
    {
        Rise_Set_ControlMode((uint8_t)val_i);
        BT_Log("OK!MODE=%d\r\n", val_i);
        // 安全保护：如果是切到停止模式，强制清零力矩
        if (val_i == Rise_Stop)
        {
            Rise_Set_Torque_Output(0, 0, 0, 0, 0,0);
        }
    }

// ================= [2. 击打电机参数 (Hit)] =================
    // HS_Burst=33.0#  -> 击打爆发速度
    else if (sscanf(cmd_str, "HS_Burst=%f", &val_f) == 1)
    {
        Rise_Hit_Burst_Speed = val_f;
        BT_Log("Hit Burst Spd: %.1f\r\n", val_f);
    }
    // HS_Ready=2.0#   -> 击打准备速度 (复位速度)
    else if (sscanf(cmd_str, "HS_Ready=%f", &val_f) == 1)
    {
        Rise_Hit_Ready_Speed = val_f;
        BT_Log("Hit Ready Spd: %.1f\r\n", val_f);
    }
    // HA_Ready=180.0# -> 击打复位角度
    else if (sscanf(cmd_str, "HA_Ready=%f", &val_f) == 1)
    {
        Rise_Hit_Ready_Angle = val_f;
        BT_Log("Hit Ready Ang: %.1f\r\n", val_f);
    }

    // ================= [3. 搓球参数 (Chop)] =================
    // CF=95.0# -> 前摩擦轮速度
    else if (sscanf(cmd_str, "CF=%f", &val_f) == 1)
    {
        Rise_Chop_Front_Target_Speed = val_f;
        BT_Log("Chop Front: %.1f\r\n", val_f);
    }
    // CR=120.0# -> 右摩擦轮速度
    else if (sscanf(cmd_str, "CR=%f", &val_f) == 1)
    {
        Rise_Chop_Right_Target_Speed = val_f;
        BT_Log("Chop Right: %.1f\r\n", val_f);
    }
    // CL=95.0# -> 左摩擦轮速度
    else if (sscanf(cmd_str, "CL=%f", &val_f) == 1)
    {
        Rise_Chop_Left_Target_Speed = val_f;
        BT_Log("Chop Left: %.1f\r\n", val_f);
    }

    // ================= [4. 抬升参数 (Lift)] =================
    // LS=2.0# -> 抬升速度
    else if (sscanf(cmd_str, "LS=%f", &val_f) == 1)
    {
        Rise_Lift_Target_Speed = val_f;
        BT_Log("Lift Spd: %.1f\r\n", val_f);
    }
    // LD=200.0# -> 抬升距离 (角度值)
    else if (sscanf(cmd_str, "LD=%f", &val_f) == 1)
    {
        Rise_Lift_Target_Dist = val_f;
        BT_Log("Lift Dist: %.1f\r\n", val_f);
    }
    // LKP=0.05# -> 抬升到位缓冲 KP
    else if (sscanf(cmd_str, "LKP=%f", &val_f) == 1)
    {
        Rise_Lift_Kp = val_f;
        BT_Log("Lift Up KP: %.3f\r\n", val_f);
    }
    // LS_MaxRet=10.0# -> 最大归位速度限制
    else if (sscanf(cmd_str, "LS_MaxRet=%f", &val_f) == 1)
    {
        LIFT_MAX_RETURN_SPEED = val_f;
        BT_Log("Lift Max Ret Spd: %.1f\r\n", val_f);
    }

    // ================= [5. 时间与阈值 (Time & Threshold)] =================
    // T_Pre=0.5# -> 预旋转时间
    else if (sscanf(cmd_str, "T_Pre=%f", &val_f) == 1)
    {
        pre_spin_time = val_f;
        BT_Log("Pre-Spin Time: %.2f\r\n", val_f);
    }
    // T_Lift=1.4# -> 抬升超时时间
    else if (sscanf(cmd_str, "T_Lift=%f", &val_f) == 1)
    {
        lift_time = val_f;
        BT_Log("Lift Time: %.2f\r\n", val_f);
    }
    // T_Drop=0.09# -> 下落时间
    else if (sscanf(cmd_str, "T_Drop=%f", &val_f) == 1)
    {
        drop_time = val_f;
        BT_Log("Drop Time: %.3f\r\n", val_f);
    }
    // T_Hit=2.0# -> 击打动作时间
    else if (sscanf(cmd_str, "T_Hit=%f", &val_f) == 1)
    {
        hit_action_time = val_f;
        BT_Log("Hit Action Time: %.2f\r\n", val_f);
    }
    // LTT=2.0# -> 抬升力矩阈值 (检测触底)
    else if (sscanf(cmd_str, "LTT=%f", &val_f) == 1)
    {
        Lift_torque_threshold = val_f;
        BT_Log("Lift Torque Thres: %.2f\r\n", val_f);
    }

    // ================= [6. 补偿参数] =================
    // G_Comp=0.5# -> 重力补偿电流
    else if (sscanf(cmd_str, "G_Comp=%f", &val_f) == 1)
    {
        Rise_Gravity_Comp_Max_Current = val_f;
        BT_Log("Gravity Comp: %.2f\r\n", val_f);
    }
    // Z_Off=0.0# -> 零点角度偏移
    else if (sscanf(cmd_str, "Z_Off=%f", &val_f) == 1)
    {
        Rise_Zero_Angle_Offset = val_f;
        BT_Log("Zero Offset: %.1f\r\n", val_f);
    }

    // ================= [7. 特殊指令] =================
    // STOP# -> 急停
    else if (strstr(cmd_str, "STOP"))
    {
        Rise_Set_ControlMode(Rise_Stop);
        Rise_Set_Torque_Output(0, 0, 0, 0, 0, 0);
        BT_Log("!!! EMERGENCY STOP !!!\r\n");
    }
}
