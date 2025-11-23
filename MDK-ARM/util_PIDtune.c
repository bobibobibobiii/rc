#include "util_PIDtune.h"
#include <stdio.h>
#include <string.h>

// 1. !!! 包含所有您想调的模块 !!!
#include "module_rise.h" // (这样我们才能访问 Rise_Data)
// #include "module_gimbal.h" // (未来您可以取消注释)
// #include "module_lift.h"   // (未来...)

#include "alg_pid.h"


/***************************************************************************
 *
 * “可调参数”的结构体
 *
 ***************************************************************************/
typedef struct {
    const char* command_string; 
    volatile float* variable_pointer; 
} TuneableParam_t;


/***************************************************************************
 *
 * “主查找表”
 * (!!!) 这是您未来唯一需要编辑的地方 (!!!)
 *
 ***************************************************************************/
static const TuneableParam_t g_tuneable_params[] = {

    // --- Rise 模块的参数 ---
    {"LSP", (volatile float*)&Rise_Data.pid.Chop_Left_Spd_Middle_PIDParam.kp},
    {"LSI", (volatile float*)&Rise_Data.pid.Chop_Left_Spd_Middle_PIDParam.ki},
    {"LSD", (volatile float*)&Rise_Data.pid.Chop_Left_Spd_Middle_PIDParam.kd},
    {"LAP", (volatile float*)&Rise_Data.pid.Chop_Left_Ang_Middle_PIDParam.kp},
    {"L_A", &Rise_Data.tar.Chop_left_pitch_angle},

    {"RSP", (volatile float*)&Rise_Data.pid.Chop_Right_Spd_Middle_PIDParam.kp},
    {"RSI", (volatile float*)&Rise_Data.pid.Chop_Right_Spd_Middle_PIDParam.ki},
    {"RSD", (volatile float*)&Rise_Data.pid.Chop_Right_Spd_Middle_PIDParam.kd},
    {"RAP", (volatile float*)&Rise_Data.pid.Chop_Right_Ang_Middle_PIDParam.kp},
    {"R_A", &Rise_Data.tar.Chop_right_pitch_angle},
    
    {"FSP", (volatile float*)&Rise_Data.pid.Chop_Front_Spd_Middle_PIDParam.kp},
    {"FSI", (volatile float*)&Rise_Data.pid.Chop_Front_Spd_Middle_PIDParam.ki},
    {"FSD", (volatile float*)&Rise_Data.pid.Chop_Front_Spd_Middle_PIDParam.kd},
    {"FAP", (volatile float*)&Rise_Data.pid.Chop_Front_Ang_Middle_PIDParam.kp},
    {"F_A", &Rise_Data.tar.Chop_front_pitch_angle},

    /*
    // --- (未来) Gimbal 模块的参数 ---
    {"G_Y_P", (volatile float*)&Gimbal_Data.pid.Yaw_Spd_PIDParam.kp},
    {"G_Y_I", (volatile float*)&Gimbal_Data.pid.Yaw_Spd_PIDParam.ki},
    */
		
		{"MODE", (volatile float*)&Rise_Data.ctrl_mode},
};

// 自动计算查找表的大小
static const uint32_t g_num_tuneable_params = sizeof(g_tuneable_params) / sizeof(TuneableParam_t); 

 
/***************************************************************************
 *
 * 函数: 通用的 PID 调整函数 (这个函数永远不需要改动)
 *
 ***************************************************************************/
void Tune_Manager_Parse_Command(uint8_t* buff)
{
    char cmd_buffer[10]; // 用于存放 "LSP", "A_A" 等
    float value_buffer = 0;

    // 1. 尝试解析 "CMD=VALUE!" 格式
    if (sscanf((char*)buff, "%[^=]=%f", cmd_buffer, &value_buffer) == 2)
    {
        // 2. 解析成功! 
        //    例如: cmd_buffer = "A_A", value_buffer = 45.0
        
        // --------------------------------------------------
        // --- 3. (新) 检查“特殊”的全局命令 ---
        // --------------------------------------------------

        // 命令 1: "A_A" (All Angles)
        // (设置 "角度环" 的目标值, 这是推荐的控制方式)
        if (strcmp(cmd_buffer, "A_A") == 0)
        {
            Rise_Data.tar.Chop_left_pitch_angle = value_buffer;
            Rise_Data.tar.Chop_right_pitch_angle = value_buffer;
            Rise_Data.tar.Chop_front_pitch_angle = value_buffer;
            return; // 任务完成
        }

        // 命令 2: "A_S" (All Speeds)
        // (直接设置 "速度环" 的目标值, 仅用于单独测试速度环)
        if (strcmp(cmd_buffer, "A_S") == 0)
        {
            // (注意: `pid` 是 `Rise_Data` 的成员, Spd_PID 是 `pid` 的成员)
            PID_SetPIDRef(&Rise_Data.pid.Chop_Left_Spd_PID, value_buffer);
            PID_SetPIDRef(&Rise_Data.pid.Chop_Right_Spd_PID, value_buffer);
            PID_SetPIDRef(&Rise_Data.pid.Chop_Front_Spd_PID, value_buffer);
            return; // 任务完成
        }

        // --------------------------------------------------
        // --- 4. 如果不是特殊命令, 搜索 "PID" 查找表 ---
        // --------------------------------------------------
        for (uint32_t i = 0; i < g_num_tuneable_params; i++)
        {
            if (strcmp(cmd_buffer, g_tuneable_params[i].command_string) == 0)
            {
                // 找到了! (例如 "LSP") 通过指针写入 RAM
                *(g_tuneable_params[i].variable_pointer) = value_buffer;
                return; // 任务完成
            }
        }
        
        // (如果循环结束都没找到，自动忽略)
    }
    // (如果 sscanf 解析失败，自动忽略)
}