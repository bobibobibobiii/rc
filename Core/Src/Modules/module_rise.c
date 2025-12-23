/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-10-31 18:53:16
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2025-12-23 19:44:53
 * @FilePath: \MDK-ARMd:\Files\xiaobing_origin\xiaobing\Core\Src\Modules\module_rise.c
 * @Description:
 *
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved.
 */
/*
 *  Project      :Rise
 *
 *  file         : module_rise.c
 *  Description  :
 *  LastEditors  : twx
 *  Date         : 2025-10-31
 *  LastEditTime :
 */
#include "sys_dwt.h"
#include "module_communicate.h"
#include "module_rise.h"
#include "periph_motor.h"
#include "sys_const.h"
#include "alg_math.h"
#include "cmsis_os.h"
#include "app_remote.h"
#include "periph_remote.h"

/* ===================================================================
 * 🛠️ 全局调试参数
 * =================================================================== */

// --- 交叉耦合 (同步) 参数 ---
// 调试步骤 1: 设为 0.0f, 先调稳 PID
// 调试步骤 2: 设为 0.05f ~ 0.1f, 测试同步效果
float Rise_K_Sync = 0.0f;

// --- 击打电机 (Hit) 参数 ---

float Rise_Hit_Target_Angle = 35.0f;  // 击打目标角度
float Rise_Hit_Return_Angle = 0.0f;   // 返回角度
float Rise_Hit_Target_Speed = 25.0f; // 击打目标速度
float Rise_Hit_Return_Time = 2.0f;    // 归位等待时间 (s)
float RISE_HIT_ACCEL_LIMIT = 3000.0f; // 击打最大加速度限制
float RISE_HIT_V_MAX = 30.0f; // 击打最大速度

// --- 搓球电机 (Chop) 参数 ---
float Rise_Chop_Front_Target_Speed = 100.0f; // 搓球目标转速
float Rise_Chop_Right_Target_Speed = 100.0f; // 搓球目标转速
float Rise_Chop_Left_Target_Speed = 100.0f; // 搓球目标转速

// --- 抬升电机 (Lift) 参数 ---
float Rise_Lift_Target_Speed = 2.0f;  // 抬升目标转速
// float Rise_Lift_Target_Dist = 270.0f; // 想要上升的高度（角度值）
float Rise_Lift_Target_Dist = 200.0f; // 想要上升的高度（角度值）
float Rise_Lift_Kp_Up = 0.05f;        // 上升到位时的柔和度

float pre_spin_time = 0.5f;   // 预旋转时间
float lift_time = 0.3f;       // 抬升时间
float drop_time = 0.7f;       // 下落时间
float hit_action_time = 2.0f; // 击打动作时间
float Lift_torque_threshold = 2.0f; // 击打检测阈值
float Hit_torque_threshold = 2.0f; // 击打检测阈值

float LIFT_RETURN_KP = 0.02f;        // 归位力度 (值越大回得越快，太大会震荡)
float LIFT_MAX_RETURN_SPEED = 10.0f; // 限制最大归位速度，防止太快撞到底

/* =================================================================== */
float torque_Hit, torque_CF, torque_CR, torque_CL, torque_Lift;

Rise_DataTypeDef Rise_Data;
/**
 * @brief      Get the pointer of Rise control object
 * @param      NULL
 * @retval     Pointer to Rise control object
 */
Rise_DataTypeDef *Rise_GetRisePtr()
{
    return &Rise_Data;
}

void Rise_Init()
{
    Rise_DataTypeDef *Rise = &Rise_Data;

    Rise->output_state = Rise_middle;
    Rise->ctrl_mode = Rise_Stop;
    Rise->fdb.Hit_pitch_angle = 0;

    Motor_DM_Basic_Output(&Motor_Rise_Chop_Front_Motors, Motor_Enable);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Right_Motors, Motor_Enable);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Left_Motors, Motor_Enable);

    Rise->lift_zero_offset = Motor_Rise_Lift_Motor.encoder.consequent_angle;

    PID_InitPIDParam(&Rise->pid.Hit_Ang_Middle_PIDParam,
                     Const_HitPosMotorParam[0][0],
                     Const_HitPosMotorParam[0][1],
                     Const_HitPosMotorParam[0][2],
                     Const_HitPosMotorParam[0][3],
                     Const_HitPosMotorParam[0][4],
                     Const_HitPosMotorParam[1][0],
                     Const_HitPosMotorParam[1][1],
                     Const_HitPosMotorParam[2][0],
                     Const_HitPosMotorParam[2][1],
                     Const_HitPosMotorParam[3][0],
                     Const_HitPosMotorParam[3][1],
                     PID_POSITION);
    PID_InitPIDParam(&Rise->pid.Hit_Spd_Middle_PIDParam,
                     Const_HitSpdMotorParam[0][0],
                     Const_HitSpdMotorParam[0][1],
                     Const_HitSpdMotorParam[0][2],
                     Const_HitSpdMotorParam[0][3],
                     Const_HitSpdMotorParam[0][4],
                     Const_HitSpdMotorParam[1][0],
                     Const_HitSpdMotorParam[1][1],
                     Const_HitSpdMotorParam[2][0],
                     Const_HitSpdMotorParam[2][1],
                     Const_HitSpdMotorParam[3][0],
                     Const_HitSpdMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Chop_Front_Ang_Middle_PIDParam,
                     Const_ChopPosMotorParam[0][0],
                     Const_ChopPosMotorParam[0][1],
                     Const_ChopPosMotorParam[0][2],
                     Const_ChopPosMotorParam[0][3],
                     Const_ChopPosMotorParam[0][4],
                     Const_ChopPosMotorParam[1][0],
                     Const_ChopPosMotorParam[1][1],
                     Const_ChopPosMotorParam[2][0],
                     Const_ChopPosMotorParam[2][1],
                     Const_ChopPosMotorParam[3][0],
                     Const_ChopPosMotorParam[3][1],
                     PID_POSITION);
    PID_InitPIDParam(&Rise->pid.Chop_Front_Spd_Middle_PIDParam,
                     Const_ChopFrontSpdMotorParam[0][0],
                     Const_ChopFrontSpdMotorParam[0][1],
                     Const_ChopFrontSpdMotorParam[0][2],
                     Const_ChopFrontSpdMotorParam[0][3],
                     Const_ChopFrontSpdMotorParam[0][4],
                     Const_ChopFrontSpdMotorParam[1][0],
                     Const_ChopFrontSpdMotorParam[1][1],
                     Const_ChopFrontSpdMotorParam[2][0],
                     Const_ChopFrontSpdMotorParam[2][1],
                     Const_ChopFrontSpdMotorParam[3][0],
                     Const_ChopFrontSpdMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Chop_Right_Ang_Middle_PIDParam,
                     Const_ChopPosMotorParam[0][0],
                     Const_ChopPosMotorParam[0][1],
                     Const_ChopPosMotorParam[0][2],
                     Const_ChopPosMotorParam[0][3],
                     Const_ChopPosMotorParam[0][4],
                     Const_ChopPosMotorParam[1][0],
                     Const_ChopPosMotorParam[1][1],
                     Const_ChopPosMotorParam[2][0],
                     Const_ChopPosMotorParam[2][1],
                     Const_ChopPosMotorParam[3][0],
                     Const_ChopPosMotorParam[3][1],
                     PID_POSITION);
    PID_InitPIDParam(&Rise->pid.Chop_Right_Spd_Middle_PIDParam,
                     Const_ChopRightSpdMotorParam[0][0],
                     Const_ChopRightSpdMotorParam[0][1],
                     Const_ChopRightSpdMotorParam[0][2],
                     Const_ChopRightSpdMotorParam[0][3],
                     Const_ChopRightSpdMotorParam[0][4],
                     Const_ChopRightSpdMotorParam[1][0],
                     Const_ChopRightSpdMotorParam[1][1],
                     Const_ChopRightSpdMotorParam[2][0],
                     Const_ChopRightSpdMotorParam[2][1],
                     Const_ChopRightSpdMotorParam[3][0],
                     Const_ChopRightSpdMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Chop_Left_Ang_Middle_PIDParam,
                     Const_ChopPosMotorParam[0][0],
                     Const_ChopPosMotorParam[0][1],
                     Const_ChopPosMotorParam[0][2],
                     Const_ChopPosMotorParam[0][3],
                     Const_ChopPosMotorParam[0][4],
                     Const_ChopPosMotorParam[1][0],
                     Const_ChopPosMotorParam[1][1],
                     Const_ChopPosMotorParam[2][0],
                     Const_ChopPosMotorParam[2][1],
                     Const_ChopPosMotorParam[3][0],
                     Const_ChopPosMotorParam[3][1],
                     PID_POSITION);
    PID_InitPIDParam(&Rise->pid.Chop_Left_Spd_Middle_PIDParam,
                     Const_ChopLeftSpdMotorParam[0][0],
                     Const_ChopLeftSpdMotorParam[0][1],
                     Const_ChopLeftSpdMotorParam[0][2],
                     Const_ChopLeftSpdMotorParam[0][3],
                     Const_ChopLeftSpdMotorParam[0][4],
                     Const_ChopLeftSpdMotorParam[1][0],
                     Const_ChopLeftSpdMotorParam[1][1],
                     Const_ChopLeftSpdMotorParam[2][0],
                     Const_ChopLeftSpdMotorParam[2][1],
                     Const_ChopLeftSpdMotorParam[3][0],
                     Const_ChopLeftSpdMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Lift_Ang_Middle_PIDParam,
                     Const_LiftPosMotorParam[0][0],
                     Const_LiftPosMotorParam[0][1],
                     Const_LiftPosMotorParam[0][2],
                     Const_LiftPosMotorParam[0][3],
                     Const_LiftPosMotorParam[0][4],
                     Const_LiftPosMotorParam[1][0],
                     Const_LiftPosMotorParam[1][1],
                     Const_LiftPosMotorParam[2][0],
                     Const_LiftPosMotorParam[2][1],
                     Const_LiftPosMotorParam[3][0],
                     Const_LiftPosMotorParam[3][1],
                     PID_POSITION);
    PID_InitPIDParam(&Rise->pid.Lift_Spd_Middle_PIDParam,
                     Const_LiftSpdMotorParam[0][0],
                     Const_LiftSpdMotorParam[0][1],
                     Const_LiftSpdMotorParam[0][2],
                     Const_LiftSpdMotorParam[0][3],
                     Const_LiftSpdMotorParam[0][4],
                     Const_LiftSpdMotorParam[1][0],
                     Const_LiftSpdMotorParam[1][1],
                     Const_LiftSpdMotorParam[2][0],
                     Const_LiftSpdMotorParam[2][1],
                     Const_LiftSpdMotorParam[3][0],
                     Const_LiftSpdMotorParam[3][1],
                     PID_POSITION);

    Rise->update_dt = 0;
    Rise->last_update_tick = DWT_GetTimeline_us();
    Rise->error_code = 0;
}

void Rise_Refresh_PID_Params(void)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // --- 1. 刷新击打电机 (Hit) ---
    // 位置环 Hit Pos (只刷 P, D)
    Rise->pid.Hit_Ang_Middle_PIDParam.kp = Const_HitPosMotorParam[0][0];
    Rise->pid.Hit_Ang_Middle_PIDParam.ki = Const_HitPosMotorParam[0][1]; // 如果你没给位置环调I，这行也可以注释
    Rise->pid.Hit_Ang_Middle_PIDParam.kd = Const_HitPosMotorParam[0][2];

    // 速度环 Hit Spd (刷 P, I, D)
    Rise->pid.Hit_Spd_Middle_PIDParam.kp = Const_HitSpdMotorParam[0][0];
    Rise->pid.Hit_Spd_Middle_PIDParam.ki = Const_HitSpdMotorParam[0][1];
    Rise->pid.Hit_Spd_Middle_PIDParam.kd = Const_HitSpdMotorParam[0][2];

    // 【这里也删掉了】

    // --- 2. 刷新抬升电机 (Lift) ---
    // 速度环 Lift Spd (只刷 P)
    Rise->pid.Lift_Spd_Middle_PIDParam.kp = Const_LiftSpdMotorParam[0][0];
    Rise->pid.Lift_Spd_Middle_PIDParam.ki = Const_LiftSpdMotorParam[0][1];
    Rise->pid.Lift_Spd_Middle_PIDParam.kd = Const_LiftSpdMotorParam[0][2];

    // 其他的都不动，保持原样
}
/**
 * @brief 清空 PID 状态（用于模式切换时消除历史影响）
 * @param pid 指向 PID 结构体的指针
 */
void PID_Clear(PID_PIDTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    // 1. 清空目标值和反馈值 (可选，但在重置时清零更安全)
    pid->ref = 0.0f;
    pid->fdb = 0.0f;

    // 2. 清空误差历史 (关键！防止 D 项微分在切换瞬间跳变)
    pid->err[0] = 0.0f; // 当前误差
    pid->err[1] = 0.0f; // 上一次误差
    pid->err[2] = 0.0f; // 上上次误差

    // 3. 【最关键】清空积分累积值 (解决"抽搐"和"暴冲"的核心)
    pid->sum = 0.0f;

    // 4. 清空前馈相关的历史数据
    pid->err_fdf[0] = 0.0f;
    pid->err_fdf[1] = 0.0f;
    pid->err_fdf[2] = 0.0f;
    pid->out_fdf = 0.0f;

    // 5. 清空最终输出
    pid->output = 0.0f;

    // 6. 清空调试变量
    pid->err_watch = 0.0f;

    // 7. 清空滤波器状态
    // 注意：不要使用 memset 清空整个 Filter 结构体，因为里面可能包含滤波系数(fc, dt等)
    // 只需要清空输出值和历史值即可。
    // (假设你的 Filter_LowPassTypeDef 结构体里存储输出的变量叫 out 或 output，请根据实际情况修改)

    // 示例：假设滤波器结构体里有一个 float out;
    // pid->d_fil.out = 0.0f;
    // pid->delta_fil.out = 0.0f;
    // pid->kf1_fil.out = 0.0f;
    // pid->kf2_fil.out = 0.0f;

    // 如果你的滤波器有 Reset 函数，最好调用它，例如：
    // Filter_Reset(&pid->d_fil);
}

void Rise_Update_Fdb()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // 零点待标定
    Rise->fdb.Hit_pitch_angle = Motor_Rise_Hit_Motor.encoder.angle;
    Rise->fdb.Hit_pitch_speed = Motor_Rise_Hit_Motor.encoder.speed;
    Rise->fdb.Hit_pitch_torque = Motor_Rise_Hit_Motor.encoder.torque;
    Rise->fdb.Chop_front_pitch_angle = Motor_Rise_Chop_Front_Motor.encoder.angle;
    Rise->fdb.Chop_front_pitch_speed = Motor_Rise_Chop_Front_Motor.encoder.speed;
    Rise->fdb.Chop_right_pitch_angle = Motor_Rise_Chop_Right_Motor.encoder.angle;
    Rise->fdb.Chop_right_pitch_speed = Motor_Rise_Chop_Right_Motor.encoder.speed;
    Rise->fdb.Chop_left_pitch_angle = Motor_Rise_Chop_Left_Motor.encoder.angle;
    Rise->fdb.Chop_left_pitch_speed = Motor_Rise_Chop_Left_Motor.encoder.speed;
    Rise->fdb.Lift_pitch_speed = Motor_Rise_Lift_Motor.encoder.standard_speed;
    float raw_total_angle = Motor_Rise_Lift_Motor.encoder.consequent_angle;
    Rise->fdb.Lift_pitch_angle = raw_total_angle - Rise->lift_zero_offset;
    Rise->fdb.Lift_pitch_torque = Motor_Rise_Lift_Motor.encoder.torque;

    Motor_Rise_Hit_Motor.watchdog += 1;
    Motor_Rise_Chop_Front_Motor.watchdog += 1;
    Motor_Rise_Chop_Right_Motor.watchdog += 1;
    Motor_Rise_Chop_Left_Motor.watchdog += 1;
    Motor_Rise_Lift_Motor.watchdog += 1;

    Rise->update_dt = DWT_GetDeltaT(&Rise->last_update_tick);
}
uint16_t watchdog_rise2 = 0;
/**
 * @brief      Check Rise motors status
 * @param      NULL
 * @retval     NULL
 */
void Rise_Check()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    if (Motor_Rise_Hit_Motor.watchdog > 20)
    {
        Rise->error_code = 1;
    }

    if (Motor_Rise_Chop_Front_Motor.watchdog > 20)
    {
        Rise->error_code = 2;
    }

    if (Motor_Rise_Chop_Right_Motor.watchdog > 20)
    {
        Rise->error_code = 3;
    }

    if (Motor_Rise_Chop_Left_Motor.watchdog > 20)
    {
        Rise->error_code = 4;
    }

    if (Motor_Rise_Lift_Motor.watchdog > 20)
    {
        Rise->error_code = 5;
    }

    if (fabsf(Motor_Rise_Lift_Motor.encoder.torque) > 20.0f)
    {
        watchdog_rise2++;
    }
    else
    {
        watchdog_rise2 = 0;
    }
    if (watchdog_rise2 > 20)
    {
        Rise->error_code = 6;
    }
}

/**
 * @brief      Set Rise motors torque output
 * @param      torque_Hit: Hit motor torque
 * @param      torque_CF: Chop front motor torque
 * @param      torque_CR: Chop right motor torque
 * @param      torque_CL: Chop left motor torque
 * @param      torque_Lift: Lift motor torque
 * @retval     NULL
 */
void Rise_Set_Torque_Output(float torque_Hit, float torque_CF, float torque_CR, float torque_CL, float torque_Lift)
{

    Motor_SetMotorOutput(&Motor_Rise_Hit_Motor, torque_Hit);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Front_Motor, torque_CF);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Right_Motor, torque_CR);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Left_Motor, torque_CL);
    Motor_SetMotorOutput(&Motor_Rise_Lift_Motor, -torque_Lift);
}

void Rise_Set_Angle_Output(float ang1, float ang2, float ang3, float ang4, float ang5)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    // float torque1, torque2, torque3, torque4;
    LimitMaxMin(ang1, 180.0f, -30.0f);
    LimitMaxMin(ang2, 360.0f, -360.0f);
    LimitMaxMin(ang3, 360.0f, -360.0f);
    LimitMaxMin(ang4, 360.0f, -360.0f);
    LimitMaxMin(ang5, 360.0f, -360.0f);

    switch (Rise->output_state)
    {

    case Rise_middle:
        PID_SetPIDRef(&Rise->pid.Hit_Ang_PID, ang1);
        PID_SetPIDFdb(&Rise->pid.Hit_Ang_PID, Rise->fdb.Hit_pitch_angle);
        PID_CalcPID(&Rise->pid.Hit_Ang_PID, &Rise->pid.Hit_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Hit_Spd_PID, PID_GetPIDOutput(&Rise->pid.Hit_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed);
        PID_CalcPID(&Rise->pid.Hit_Spd_PID, &Rise->pid.Hit_Spd_Middle_PIDParam);
        torque_Hit = PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID) - 0.0f;

        float angle = Rise->fdb.Hit_pitch_angle;
        if (angle >= 150.0f)
        {
            // 禁止任何“继续向前”的力矩
            if (torque_Hit > 0.0f)
                torque_Hit = -10.0f;
        }

        PID_SetPIDRef(&Rise->pid.Chop_Front_Ang_PID, ang2);
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Ang_PID, Rise->fdb.Chop_front_pitch_angle);
        PID_CalcPID(&Rise->pid.Chop_Front_Ang_PID, &Rise->pid.Chop_Front_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Chop_Front_Spd_PID, PID_GetPIDOutput(&Rise->pid.Chop_Front_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, -Motor_Rise_Chop_Front_Motor.encoder.speed);
        PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
        torque_CF = PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID);

        PID_SetPIDRef(&Rise->pid.Chop_Right_Ang_PID, ang3);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Ang_PID, Rise->fdb.Chop_right_pitch_angle);
        PID_CalcPID(&Rise->pid.Chop_Right_Ang_PID, &Rise->pid.Chop_Right_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, PID_GetPIDOutput(&Rise->pid.Chop_Right_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, -Motor_Rise_Chop_Right_Motor.encoder.speed);
        PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
        torque_CR = PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID);

        PID_SetPIDRef(&Rise->pid.Chop_Left_Ang_PID, ang4);
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Ang_PID, Rise->fdb.Chop_left_pitch_angle);
        PID_CalcPID(&Rise->pid.Chop_Left_Ang_PID, &Rise->pid.Chop_Left_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID, PID_GetPIDOutput(&Rise->pid.Chop_Left_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, -Motor_Rise_Chop_Left_Motor.encoder.speed);
        PID_CalcPID(&Rise->pid.Chop_Left_Spd_PID, &Rise->pid.Chop_Left_Spd_Middle_PIDParam);
        torque_CL = PID_GetPIDOutput(&Rise->pid.Chop_Left_Spd_PID);

        PID_SetPIDRef(&Rise->pid.Lift_Ang_PID, ang5);
        PID_SetPIDFdb(&Rise->pid.Lift_Ang_PID, Rise->fdb.Lift_pitch_angle);
        PID_CalcPID(&Rise->pid.Lift_Ang_PID, &Rise->pid.Lift_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Lift_Spd_PID, PID_GetPIDOutput(&Rise->pid.Lift_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Lift_Spd_PID, -Motor_Rise_Lift_Motor.encoder.speed);
        PID_CalcPID(&Rise->pid.Lift_Spd_PID, &Rise->pid.Lift_Spd_Middle_PIDParam);
        torque_Lift = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID) - 0.0f;
        break;
    default:
        break;
    }

    Rise_Set_Torque_Output(torque_Hit, torque_CF, torque_CR, torque_CL, torque_Lift);
}

/**
 * @brief 统一的速度控制函数 (包含 PID 计算和交叉耦合)
 * @param hit_speed   击打电机目标速度
 * @param chop_speed  搓球电机(3个)目标速度
 * @param lift_speed  抬升电机目标速度
 */

void Rise_Set_Speed_Output(float hit_speed, float chop1_speed, float chop2_speed, float chop3_speed, float lift_speed)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    // float torque1, torque2, torque3, torque4;
    LimitMaxMin(hit_speed, RISE_HIT_V_MAX, -RISE_HIT_V_MAX);
    LimitMaxMin(chop1_speed, 500.0f,  -500.0f);
    LimitMaxMin(chop2_speed, 500.0f,  -500.0f);
    LimitMaxMin(chop3_speed, 500.0f,  -500.0f);
    LimitMaxMin(lift_speed, 469.0f, -469.0f);

    switch (Rise->output_state)
    {

    case Rise_middle:

        PID_SetPIDRef(&Rise->pid.Hit_Spd_PID, hit_speed);
        PID_SetPIDFdb(&Rise->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed);
        PID_CalcPID(&Rise->pid.Hit_Spd_PID, &Rise->pid.Hit_Spd_Middle_PIDParam);
        torque_Hit = PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID);

        // chop的逻辑

        float avg_speed = (Rise->fdb.Chop_front_pitch_speed + Rise->fdb.Chop_right_pitch_speed + Rise->fdb.Chop_left_pitch_speed) / 3.0f;

        float front_sync_err = Rise->fdb.Chop_front_pitch_speed - avg_speed;
        float right_sync_err = Rise->fdb.Chop_right_pitch_speed - avg_speed;
        float left_sync_err = Rise->fdb.Chop_left_pitch_speed - avg_speed;

        PID_SetPIDRef(&Rise->pid.Chop_Front_Spd_PID, chop1_speed);
        // 2. 设置*反馈*速度 (Fdb) 为*电机真实速度*
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, Rise->fdb.Chop_front_pitch_speed);
        // 3. 计算PID
        PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
        // 4. 获取力矩
        torque_CF = PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID) - (Rise_K_Sync * front_sync_err);

        // --- Right Motor (ID: 0x02, Group: 9) ---
        PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, chop2_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, Rise->fdb.Chop_right_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
        torque_CR = PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID) - (Rise_K_Sync * right_sync_err);

        // --- Left Motor (ID: 0x03, Group: 10) ---
        PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID, chop3_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, Rise->fdb.Chop_left_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Left_Spd_PID, &Rise->pid.Chop_Left_Spd_Middle_PIDParam);
        torque_CL = PID_GetPIDOutput(&Rise->pid.Chop_Left_Spd_PID) - (Rise_K_Sync * left_sync_err);

        PID_SetPIDRef(&Rise->pid.Lift_Spd_PID, lift_speed);
        PID_SetPIDFdb(&Rise->pid.Lift_Spd_PID, Rise->fdb.Lift_pitch_speed);
        PID_CalcPID(&Rise->pid.Lift_Spd_PID, &Rise->pid.Lift_Spd_Middle_PIDParam);
        torque_Lift = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID);
        break;
    default:
        break;
    }

    Rise_Set_Torque_Output(torque_Hit, torque_CF, torque_CR, torque_CL, torque_Lift);
}

void Rise_Set_ControlMode(uint8_t mode)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    Rise->ctrl_mode = mode;
}
void Rise_Set_OutputState(uint8_t state)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    Rise->output_state = state;
}

/**
 * @brief 终极混合控制函数
 * @param hit_angle   击打电机 -> 目标角度 (位置闭环)
 * @param chop_speed  搓球电机 -> 目标速度 (速度闭环 + 交叉耦合)
 * @param lift_speed  抬升电机 -> 目标速度 (速度闭环)
 */
void Rise_Set_Hybrid_Output(float hit_angle, float chop_front_speed, float chop_right_speed, float chop_left_speed, float lift_speed)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // 1. 限幅
    LimitMaxMin(hit_angle, 180.0f, -30.0f);
    LimitMaxMin(chop_front_speed, 500.0f,  -500.0f);
    LimitMaxMin(chop_right_speed, 500.0f,  -500.0f);
    LimitMaxMin(chop_left_speed, 500.0f,  -500.0f);
    LimitMaxMin(lift_speed, 469.0f, -469.0f);

    // 局部变量
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;

    if (Rise->output_state == Rise_middle)
    {
        // --- 1. 击打电机 (Hit): 使用【串级 PID】 (角度环 -> 速度环) ---
        // 计算角度环
        PID_SetPIDRef(&Rise->pid.Hit_Ang_PID, hit_angle);
        PID_SetPIDFdb(&Rise->pid.Hit_Ang_PID, Rise->fdb.Hit_pitch_angle);
        PID_CalcPID(&Rise->pid.Hit_Ang_PID, &Rise->pid.Hit_Ang_Middle_PIDParam);

        // 角度环输出作为速度环输入
        PID_SetPIDRef(&Rise->pid.Hit_Spd_PID, PID_GetPIDOutput(&Rise->pid.Hit_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed); // 注意: 这里用 speed 而不是 -speed，取决于方向
        PID_CalcPID(&Rise->pid.Hit_Spd_PID, &Rise->pid.Hit_Spd_Middle_PIDParam);

        t1 = PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID); // 计算出力矩

        // --- 2. 搓球电机 (Chop): 使用【速度 PID】 + 交叉耦合 ---

        // 计算平均速度和误差 (用于同步)
        float avg_speed = (Rise->fdb.Chop_front_pitch_speed +
                           Rise->fdb.Chop_right_pitch_speed +
                           Rise->fdb.Chop_left_pitch_speed) /
                          3.0f;
        float err_f = Rise->fdb.Chop_front_pitch_speed - avg_speed;
        float err_r = Rise->fdb.Chop_right_pitch_speed - avg_speed;
        float err_l = Rise->fdb.Chop_left_pitch_speed - avg_speed;

        // Front
        PID_SetPIDRef(&Rise->pid.Chop_Front_Spd_PID, chop_front_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, Rise->fdb.Chop_front_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
        t2 = PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID) - (Rise_K_Sync * err_f);

        // Right
        PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, chop_right_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, Rise->fdb.Chop_right_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
        t3 = PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID) - (Rise_K_Sync * err_r);

        // Left
        PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID, chop_left_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, Rise->fdb.Chop_left_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Left_Spd_PID, &Rise->pid.Chop_Left_Spd_Middle_PIDParam);
        t4 = PID_GetPIDOutput(&Rise->pid.Chop_Left_Spd_PID) - (Rise_K_Sync * err_l);

        // --- 3. 抬升电机 (Lift): 使用【速度 PID】 ---
        PID_SetPIDRef(&Rise->pid.Lift_Spd_PID, lift_speed);
        PID_SetPIDFdb(&Rise->pid.Lift_Spd_PID, Rise->fdb.Lift_pitch_speed);
        PID_CalcPID(&Rise->pid.Lift_Spd_PID, &Rise->pid.Lift_Spd_Middle_PIDParam);
        t5 = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID);
    }

    // 统一发送
    Rise_Set_Torque_Output(t1, t2, t3, t4, t5);
}

// 简单的线性插值或分段拟合
float Get_Speed_By_Distance(float distance_m)
{
    // 假设实测数据：3m->200rpm, 9m->500rpm
    // y = kx + b
    // 这只是个例子，实际大概率是非线性的，可能需要二次函数
    float speed = 50.0f * distance_m + 50.0f;

    // 限制安全范围
    if (speed > RISE_HIT_V_MAX-5.0f)
        speed = RISE_HIT_V_MAX-5.0f;
    if (speed < 1.0f)
        speed = 1.0f;

    return speed;
}

/**
 * @brief      速度保持型轨迹更新器（过点速度不归零）
 * @param      goal        目标位置（击球点角度）
 * @param      dt          时间步长
 * @param      v_pass      期望通过目标时的速度（即“击打力度” deg/s）
 * @param      a_limit     最大加速度能力
 * @param      state       当前位置指针
 * @param      traj_v      当前轨迹速度指针
 * @retval     1 到达目标（且达到速度），0 未到达
 * @note       适用于“击球”场景：在到达 goal 瞬间，速度尽量等于 v_pass，而不是 0
 */
int Traj_Update_VelPass(float goal, float dt, float v_pass, float a_limit,
                        float *state, float *traj_v)
{
    // 1. 参数预处理
    if (dt <= 0.0f || state == NULL || traj_v == NULL)
        return 0;

    // 确保加速度非零
    float a_cap = (fabsf(a_limit) > 1e-3f) ? fabsf(a_limit) : 1e-3f;
    // 期望速度带符号（正负取决于 v_pass 本身，不需要像之前那样根据误差方向判断）
    // 但通常我们希望 v_pass 的符号与运动方向一致

    // 计算当前位置误差
    float e = goal - (*state);

    // 2. 核心逻辑：始终全力趋近于 v_pass
    // 既然目的是以 v_pass 通过 goal，那么在到达 goal 之前，
    // 我们不需要计算刹车距离，只需要把速度拉向 v_pass 即可。

    // 判断运动是否完成：
    // 如果 v_pass > 0 (正向击打)，且当前位置已经超过 goal，则认为完成
    // 如果 v_pass < 0 (反向击打)，且当前位置已经小于 goal，则认为完成
    if ((v_pass > 0.0f && *state >= goal) || (v_pass < 0.0f && *state <= goal))
    {
        // 到达后，可以选择保持速度继续冲，或者返回完成标志让上层处理
        // 这里选择维持通过速度，防止瞬间掉速
        *traj_v = v_pass;
        return 1;
    }

    // 3. 速度规划
    // 这里的逻辑比原版简单：不需要刹车，只需要加速限制
    float dv = v_pass - (*traj_v);
    float dv_max = a_cap * dt;

    // 限制单步加速度
    if (dv > dv_max)
        dv = dv_max;
    if (dv < -dv_max)
        dv = -dv_max;

    // 更新速度
    *traj_v += dv;

    // 4. 更新位置
    *state += (*traj_v) * dt;

    return 0; // 未到达
}

// 定义全局或静态变量来保存轨迹状态
static float hit_traj_pos = 0.0f;
static float hit_traj_vel = 0.0f;
static uint8_t hit_traj_init = 0; // 初始化标志

void Rise_Reset_Hit_Traj(void)
{
    hit_traj_init = 0;
    hit_traj_pos = 0.0f;
    hit_traj_vel = 0.0f;
}

int g_hit_finished_flag = 0;
/**
 * @brief      变力度击打控制
 * @param      target_angle   击球点的角度 (例如 45度)
 * @param      hit_velocity   击打力度 (例如 300 deg/s)
 */
float Rise_Hit_Control_Variable(float target_angle, float hit_velocity)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    float dt = Rise->update_dt;
    // --- 【修改 1】定义最大允许角度和保护区 ---
    const float ABSOLUTE_MAX_ANGLE = 180.0f; // 机械极限或绝对软限位
    const float SAFETY_MARGIN = 20.0f;        // 安全余量，防止恰好撞到90度

    // --- 【修改 2】强制输入限幅 ---
    // 如果上层指令要求去 100 度，这里强制把它改为 88 度 (90-2)
    // 这样轨迹规划器只会规划到 88 度，不会试图冲向 90 度以上
    if (target_angle > (ABSOLUTE_MAX_ANGLE - SAFETY_MARGIN))
    {
        target_angle = (ABSOLUTE_MAX_ANGLE - SAFETY_MARGIN);
    }

    // 1. 初始化：如果是第一次进入这个模式，把轨迹起点设为电机当前真实位置
    if (hit_traj_init == 0)
    {
        hit_traj_pos = Rise->fdb.Hit_pitch_angle;
        hit_traj_vel = Rise->fdb.Hit_pitch_speed;
        hit_traj_init = 1;
    }

    // 2. 运行轨迹更新器
    int is_reached = Traj_Update_VelPass(target_angle, dt, hit_velocity, RISE_HIT_ACCEL_LIMIT,
                                         &hit_traj_pos, &hit_traj_vel);

    // 3. 串级 PID 控制 (关键：引入前馈)
    // 3.1 位置环
    PID_SetPIDRef(&Rise->pid.Hit_Ang_PID, hit_traj_pos);
    PID_SetPIDFdb(&Rise->pid.Hit_Ang_PID, Rise->fdb.Hit_pitch_angle);
    PID_CalcPID(&Rise->pid.Hit_Ang_PID, &Rise->pid.Hit_Ang_Middle_PIDParam);

    // 3.2 速度环 (目标 = 位置环输出 + 轨迹规划的速度前馈)
    // 加上 hit_traj_vel 非常重要！这意味着PID不需要等有了误差才加速，
    // 而是直接要把电机速度拉到规划速度。
    float pid_out = PID_GetPIDOutput(&Rise->pid.Hit_Ang_PID);
    float spd_ref;

    // 到达击打目标之前：允许前馈
    if (!is_reached)
    {
        spd_ref = pid_out + hit_traj_vel;
    }
    else
    {
        // 到点后：立刻关闭前馈，防止继续冲
        spd_ref = pid_out;
    }
    float angle = Rise->fdb.Hit_pitch_angle;
    float speed = Rise->fdb.Hit_pitch_speed;

    const float HIT_MAX_ANGLE = 150.0f;
    const float BRAKE_START_ANGLE = 125.0f;

    if (angle > BRAKE_START_ANGLE)
    {
        // 1. 基础减速：线性降低目标速度
        float scale = (HIT_MAX_ANGLE - angle) / (HIT_MAX_ANGLE - BRAKE_START_ANGLE);
        if (scale < 0.0f)
            scale = 0.0f;

        if (spd_ref > 0.0f)
            spd_ref *= scale;

        // 2. 进阶：【主动反向制动】 (Active Braking)
        if (angle > (BRAKE_START_ANGLE - 10) && speed > RISE_HIT_V_MAX - 5.0f)
        {
            spd_ref = -(RISE_HIT_V_MAX-10.0f); // 猛烈倒车！值越大刹车越狠
        }

        // 3. 救命：【绝对死线】
        if (angle > BRAKE_START_ANGLE)
        {
            spd_ref = -RISE_HIT_V_MAX;
        }
    }

    // 5. 后处理：如果到达了
    if (is_reached)
    {
        // 可以在这里切换状态机，进入“随挥”或“复位”阶段
        // 下次进来前记得把 hit_traj_init 置 0
        g_hit_finished_flag = 1;
    }

    return spd_ref;
}

static uint8_t test_state = 0;
// 定义一个静态变量来记录进入测试模式时的初始高度
static float test_start_height = 0.0f;
static float return_ramp_angle = 0.0f; // 复位过程中的中间目标
static float ramp_start_angle = 0.0f;
static float ramp_start_time = 0.0f;
static float current_hit_pos = 0.0f;

void Rise_Hit_Cal(void)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    float current_time = DWT_GetTimeline_s();

    // 设置为 Middle 状态以启用 PID 计算
    Rise_Set_OutputState(Rise_middle);

    static float start_time = 0.0f;

    switch (test_state)
    {
    case 0: // [启动 & 初始化]
        // 1. 复位轨迹
        Rise_Reset_Hit_Traj();
        g_hit_finished_flag = 0;

        start_time = current_time;
        test_state = 1;
        break;

    case 1: // [模拟 Auto 的击打阶段]
    {

        // --- A. 计算击打速度 (轨迹规划) ---
        float hit_cmd_speed = Rise_Hit_Control_Variable(Rise_Hit_Target_Angle, Rise_Hit_Target_Speed);

        if (hit_cmd_speed < 0.0f)
        {
            hit_cmd_speed = 0.0f;
        }

        // --- C. 发送指令 (模仿 Auto) ---
        // Hit: 规划速度
        // Chop: 目标转速 (Auto里击打时搓球是转的)
        // Lift: 锁止速度
        Rise_Set_Speed_Output(hit_cmd_speed, 0, 0, 0, 0);

        // --- D. 判断退出 ---
        if ((g_hit_finished_flag == 1) || (current_time - start_time > 2.0f))
        {
            g_hit_finished_flag = 0;
            PID_Clear(&Rise->pid.Hit_Spd_PID);
            start_time = current_time;
            return_ramp_angle = Rise->fdb.Hit_pitch_angle;
            test_state = 2;
        }
    }
    break;

    case 2: // [复位阶段 - 改为软复位]
    {
        // --- B. 【核心修改】平滑轨迹规划 (线性插值) ---
        // 每次进入这个循环，让目标角度向 0 度靠近一点点
        // 调节这个 step 0.1f，值越小回得越慢，值越大回得越快
        float return_step = 0.05f;

        if (return_ramp_angle > Rise_Hit_Return_Angle + return_step)
        {
            return_ramp_angle -= return_step; // 慢慢减小
        }
        else if (return_ramp_angle < Rise_Hit_Return_Angle - return_step)
        {
            return_ramp_angle += return_step; // 慢慢增加（如果是反向回）
        }
        else
        {
            return_ramp_angle = Rise_Hit_Return_Angle; // 到了就锁死
        }

        // --- C. 发送指令 ---
        // 注意：这里传进去的是 return_ramp_angle (慢慢变的值)，而不是 0.0f
        Rise_Set_Hybrid_Output(return_ramp_angle, 0.0f, 0.0f, 0.0f, 0.0f);

        // --- D. 等待复位完成 ---
        if (fabs(Rise->fdb.Hit_pitch_angle - Rise_Hit_Return_Angle) < 0.5f)
        {
            test_state = 3; // 任务完成
        }
    }
    break;

    case 3: // [测试结束]
    {
        Rise_Set_Torque_Output(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        break;
    }
    }
}

void Rise_Chop_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    float current_hit_target = Rise->pid.Hit_Spd_PID.ref;   // 读取击打电机的当前目标
    float current_lift_target = Rise->pid.Lift_Spd_PID.ref; // 读取抬升电机的当前目标

    Rise_Set_OutputState(Rise_middle);
    // Rise_Set_Speed_Output(current_hit_target, Rise_Chop_Target_Speed, current_lift_target);
    Rise_Set_Speed_Output(0.0f, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, 0.0f);
}

void Rise_Lift_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    // float current_lift_target = Rise->pid.Lift_Spd_PID.ref; // 读取抬升电机的当前目标
    Rise_Set_OutputState(Rise_middle);
    // Rise_Set_Speed_Output(current_lift_target,Rise_Chop_Target_Speed, Rise_Lift_Target_Speed);
    Rise_Set_Speed_Output(0.0f, 0.0f, 0.0f, 0.0f, Rise_Lift_Target_Speed);
    // Rise_Set_Torque_Output(0.0f,0.0f,0.0f,0.0f,torque_test);
}

float current_time;
float current_height;
uint32_t g_case0_entry_count = 0;
static uint8_t g_auto_state = 0; // 初始状态 0: 空闲/启动
float g_auto_start_time = 0.0f;
float g_auto_start_height = 0.0f;
static uint8_t g_last_auto_mode = Rise_Stop; // 假设默认为 Stop

void Rise_Auto_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    Rise_Set_OutputState(Rise_middle);
    current_time = DWT_GetTimeline_s();

    switch (g_auto_state)
    {

    case 0: // 启动
        Rise_Reset_Hit_Traj();
        g_hit_finished_flag = 0;
        g_case0_entry_count++;
        g_auto_start_time = current_time;
        g_auto_state = 1;
        g_auto_start_height = Rise->fdb.Lift_pitch_angle;
        break;

    case 1: // 预旋转 (Pre-spin)
        // Hit: 保持0度 | Chop: 转 | Lift: 0
        Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, 0.0f);

        if (current_time - g_auto_start_time >= pre_spin_time)
        {
            g_auto_start_time = current_time;
            g_auto_state = 2;
        }
        break;

    case 2: // 抬升 (Lift)
        // Hit: 保持0度 | Chop: 转 | Lift: 转
        float lift_up_target = g_auto_start_height + Rise_Lift_Target_Dist;
        float cmd_speed = 0.0f;
        float slow_up_zone = Rise_Lift_Target_Dist * 0.2;
        float lift_current_pos = Rise->fdb.Lift_pitch_angle;

        {

            // 2. 速度规划逻辑 (仿照 Case 3)
            // 如果当前位置 < (目标 - 减速区)，说明离目标还很远 -> 全速上升
            if (lift_current_pos < (lift_up_target - slow_up_zone))
            {
                cmd_speed = Rise_Lift_Target_Speed;
            }
            else
            {
                // 进入减速区，使用 P 控制慢慢靠近
                // 误差 = (目标 - 当前) = 正数
                // 速度 = 正数 * KP = 正速度 (向上)
                cmd_speed = (lift_up_target - lift_current_pos) * Rise_Lift_Kp_Up;

                // 最小速度钳制（防止太慢到不了）
                if (cmd_speed < 1.0f && cmd_speed > 0.1f)
                    cmd_speed = 1.0f;
            }

            // 3. 发送指令
            Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, cmd_speed);

            // 4. 退出条件：到达目标高度 (误差小于 10 度)
            // 原来的超时判断 (lift_time) 可以保留作为“保底超时”，防止卡住
            if ((fabs(lift_current_pos - lift_up_target) < 10.0f) || (current_time - g_auto_start_time >= lift_time))
            {
                g_auto_start_time = current_time; // 重置时间给下一阶段
                g_auto_state = 3;
            }

            // 【可选保底】如果时间太长还没到位，也强制下一步
            // if (current_time - g_auto_start_time > 5.0f) { ... }
        }
        break;

    case 3: // 等待下落 (Drop)
            // 获取当前高度 (假设上方是负数，例如 -7000; 目标是 0)
        float current_torque = Rise->fdb.Lift_pitch_torque;
        static float last_torque = 0.0f;
        current_height = Rise->fdb.Lift_pitch_angle;
        float return_speed = 0.0f;

        // --- 设定参数 ---
        float slow_down_zone = 500.0f;   // 减速区：距离 0 点 500 个单位时开始减速
        float fixed_down_speed = -10.0f; // 匀速下降的速度 (负数代表向下)
        float landing_kp = 0.02f;        // 着陆时的柔和度

        // --- 逻辑判断 ---

        // --- 1. 力矩突变检测 (受力停止逻辑) ---
        // 计算力矩的变化率或偏差
        float torque_diff = fabs(current_torque - last_torque);
        last_torque = current_torque; // 更新历史值

        // 如果检测到力矩突然增大（撞击或触底），强制停止
        if (torque_diff > Lift_torque_threshold && fabs(current_height - g_auto_start_height) > 100.0f)
        {
            return_speed = 0.0f;
            break;
        }

        if (current_height > g_auto_start_height + slow_down_zone)
        {
            // 1. 如果离 0 点还很远 (比如 -7000)，就以固定速度下降
            // 这样避免了在最高点产生巨大的速度指令
            return_speed = fixed_down_speed;
        }
        else
        {
            // 2. 如果进入减速区 (比如 -300)，开始 P 控制软着陆
            // 公式：速度 = 距离 * Kp
            // (角度 - 0) * Kp = 负数 * Kp = 负速度 (向下)
            return_speed = -(current_height - g_auto_start_height) * landing_kp;

            // 增加死区，防止在 0 点抖动
            if (fabs(current_height - g_auto_start_height) < 10.0f)
                return_speed = 0.0f;
        }

        // 3. 发送指令
        // 注意：这里不需要再用 LimitMaxMin 了，因为我们已经手动控制了速度
        Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, return_speed);

        // 4. 状态转换
        if (current_time - g_auto_start_time >= drop_time)
        {
            g_auto_start_time = current_time;
            Rise_Reset_Hit_Traj();
            g_hit_finished_flag = 0; // 确保标志位干净
            g_auto_state = 4;
        }

        Rise_Reset_Hit_Traj();
        break;
    case 4: // 击打 (Hit)
    {

        // 继续计算归位速度，让它死死锁在 0 点，防止松动
        float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
        float hold_speed = -pos_error * LIFT_RETURN_KP;
        LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

        float hit_cmd_speed = Rise_Hit_Control_Variable(Rise_Hit_Target_Angle, Rise_Hit_Target_Speed);

        // 发送
        Rise_Set_Speed_Output(hit_cmd_speed, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, hold_speed);

        if (current_time - g_auto_start_time >= hit_action_time || g_hit_finished_flag == 1)
        {
            g_hit_finished_flag = 0;
            PID_Clear(&Rise->pid.Hit_Spd_PID);
            return_ramp_angle = Rise->fdb.Hit_pitch_angle;
            g_auto_state = 5;
        }
    }
    break;

    case 5: // 复位 (Reset)
        // 动作：Hit 归零 | Chop 停 | Lift 停 (或继续锁零)
        {
            // 保持抬升机构锁在 0 点 (防止复位震动导致托盘掉下来)
            float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
            float hold_speed = -pos_error * LIFT_RETURN_KP;
            LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

            float return_step = 0.05f;
            if (return_ramp_angle > Rise_Hit_Return_Angle + return_step)
            {
                return_ramp_angle -= return_step; // 慢慢减小
                // } else if (return_ramp_angle < Rise_Hit_Return_Angle - return_step) {
                //     return_ramp_angle += return_step; // 慢慢增加（如果是反向回）
            }
            else
            {
                return_ramp_angle = Rise_Hit_Return_Angle; // 到了就锁死
            }

            // 发送：Hit 回 0 度
            Rise_Set_Hybrid_Output(return_ramp_angle, 0.0f, 0.0f, 0.0f, hold_speed);
        }

        // ！！！ 关键修改：等待复位完成 ！！！
        // 使用全局变量 Rise_Hit_Return_Time (例如 1.0秒)
        if (fabs(Rise->fdb.Hit_pitch_angle - Rise_Hit_Return_Angle) < 1.0f)
        {
            g_auto_state = 6; // 时间到了，才进入“完成态”
        }
        break;

    case 6: // 阶段6：完成等待 (DONE)
        // 计算锁住位置需要的力/速度
        float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
        float hold_speed = -pos_error * LIFT_RETURN_KP;
        LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

        // 持续发送保持指令 (假设 Lift 是第3个参数)
        // Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, 0.0f, 0.0f, 0.0f, hold_speed);
        Rise_Set_Torque_Output(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        break;

    default:
        g_auto_state = 0;
        break;
    }
}

void Rise_Without_Hit_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    Rise_Set_OutputState(Rise_middle);
    current_time = DWT_GetTimeline_s();

    switch (g_auto_state)
    {
    case 0: // 启动与初始化
        Rise_Reset_Hit_Traj();
        g_hit_finished_flag = 0;
        g_case0_entry_count++;
        g_auto_start_time = current_time;
        g_auto_start_height = Rise->fdb.Lift_pitch_angle;
        g_auto_state = 1;
        break;

    case 1: // 预旋转 (Pre-spin)
        // Hit: 锁在返回角 | Chop: 搓球电机转动 | Lift: 0
        Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Front_Target_Speed, 
                               Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, 0.0f);

        if (current_time - g_auto_start_time >= pre_spin_time)
        {
            g_auto_start_time = current_time;
            g_auto_state = 2;
        }
        break;

    case 2: // 抬升 (Lift)
    {
        float lift_up_target = g_auto_start_height + Rise_Lift_Target_Dist;
        float lift_current_pos = Rise->fdb.Lift_pitch_angle;
        float slow_up_zone = Rise_Lift_Target_Dist * 0.2f;
        float cmd_speed = 0.0f;

        // 速度规划
        if (lift_current_pos < (lift_up_target - slow_up_zone))
        {
            cmd_speed = Rise_Lift_Target_Speed;
        }
        else
        {
            cmd_speed = (lift_up_target - lift_current_pos) * Rise_Lift_Kp_Up;
            if (cmd_speed < 1.0f && cmd_speed > 0.1f) cmd_speed = 1.0f;
        }

        Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Front_Target_Speed, 
                               Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, cmd_speed);

        if (current_time - g_auto_start_time >= lift_time)
        {
            g_auto_start_time = current_time;
            g_auto_state = 3;
        }
    }
    break;

    case 3: // 等待下落 (Drop) + 力矩突变停止
    {
        float current_torque = Rise->fdb.Lift_pitch_torque;
        static float last_torque = 0.0f;
        current_height = Rise->fdb.Lift_pitch_angle;
        float return_speed = 0.0f;

        // --- 设定参数 ---
        float slow_down_zone = 500.0f;   
        float fixed_down_speed = -10.0f; 
        float landing_kp = 0.02f;

        // --- 1. 力矩突变检测 ---
        float torque_diff = fabs(current_torque - last_torque);
        last_torque = current_torque;

        // 检测受力突变（触底或障碍物）
        if (torque_diff > Lift_torque_threshold && fabs(current_height - g_auto_start_height) > 100.0f)
        {
            return_speed = 0.0f;
            g_auto_state = 4;
            break;
        }

        // --- 2. 软着陆逻辑 ---
        if (current_height > g_auto_start_height + slow_down_zone)
        {
            return_speed = fixed_down_speed;
        }
        else
        {
            return_speed = -(current_height - g_auto_start_height) * landing_kp;
            if (fabs(current_height - g_auto_start_height) < 10.0f)
            {
                return_speed = 0.0f;
            }
        }

        Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Front_Target_Speed, 
                               Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, return_speed);


    }
    break;

    case 4: // 完成态 (DONE) - 锁死零点并停止搓球
    {
        // 维持在 g_auto_start_height 位置，防止机构因重力下滑
        float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
        float hold_speed = -pos_error * LIFT_RETURN_KP;
        LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

        // 搓球电机设为 0，停止旋转
        Rise_Set_Torque_Output(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    break;

    default:
        g_auto_state = 0;
        break;
    }
}

static uint8_t last_ctrl_mode = Rise_Stop;

void Rise_Control()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    // 1. 检测模式切换边沿
    if (Rise->ctrl_mode != last_ctrl_mode)
    {

        // 1.1 如果是从其他模式 -> 切入【击球模式】
        if (Rise->ctrl_mode == Rise_Jiqiu)
        {
            test_state = 0; // 重置击球状态机
        }

        // 1.2 如果是从其他模式 -> 切入【自动模式】
        if (Rise->ctrl_mode == Rise_Auto || Rise->ctrl_mode == Rise_Without_Hit)
        {
            g_auto_state = 0; // 重置自动状态机
        }

        // ★★★ 关键修复：必须更新历史状态，否则会死循环 ★★★
        last_ctrl_mode = Rise->ctrl_mode;
    }

    switch (Rise->ctrl_mode)
    {
    case Rise_Auto:
        Rise_Auto_Cal();
        break;
    case Rise_Jiqiu:
        Rise_Hit_Cal();
        break;
    case Rise_Cuoqiu:
        Rise_Chop_Cal();
        break;
    case Rise_Taisheng:
        Rise_Set_OutputState(Rise_middle);
        Rise_Lift_Cal();
        break;
    case Rise_Stop:
        Rise_Set_OutputState(Rise_stop);
        Rise_Set_Torque_Output(0, 0, 0, 0, 0);
        break;
    case Rise_Without_Hit:
        Rise_Without_Hit_Cal();

    default:
        break;
    }
}

void Rise_Output()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
    Motor_SendMotorGroupOutput(Motor_groupHandle[7]);
    DWT_Delayus(150);
    Motor_SendMotorGroupOutput(Motor_groupHandle[8]);
    DWT_Delayus(150);
    Motor_SendMotorGroupOutput(Motor_groupHandle[9]);
    DWT_Delayus(150);
    Motor_SendMotorGroupOutput(Motor_groupHandle[10]);
    DWT_Delayus(150);
    Motor_SendMotorGroupOutput(Motor_groupHandle[11]);

    // last_output_time = DWT_GetTimeline_us();
    DWT_Delayus(150);
}
