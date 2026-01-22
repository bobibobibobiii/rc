/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-10-31 18:53:16
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2026-01-22 23:38:40
 * @FilePath: \xiaobing\Core\Src\Modules\module_rise.c
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

 Rise_Params Params_Middle = {
// --- 击打电机 (Hit) 参数 ---
    .Rise_Hit_Ready_Speed = 4.0f,
    .Rise_Hit_Ready_Angle = 180.0f,
    .Rise_Hit_Burst_Speed = 38.0f, // 击打预备速度
    .Hit_Test_Speed = 20.0f, // 击打测试速度

    //搓球电机 (Chop) 参数 ---
    .Rise_Chop_Front_Target_Speed = 72.0f, // 搓球目标转速
    .Rise_Chop_Right_Target_Speed = 120.0f, // 搓球目标转速
    .Rise_Chop_Left_Target_Speed = 72.0f, // 搓球目标转速

    //时间参数 ---
    .pre_spin_time = 0.5f,   // 预旋转时间
    .lift_time = 1.1f,       // 抬升时间
    .drop_time = 0.075f,       // 下落时间
    .hit_action_time = 2.0f // 击打动作时间
 };

// --- 击打电机 (Hit) 参数 ---
float Rise_Hit_Ready_Speed = 4.0f;
float Rise_Hit_Ready_Angle = 180.0f;
float Rise_Hit_Burst_Speed = 38.0f; // 击打预备速度
float Hit_Test_Speed = 20.0f; // 击打测试速度

// --- 搓球电机 (Chop) 参数 ---
float Rise_Chop_Front_Target_Speed = 72.0f; // 搓球目标转速
float Rise_Chop_Right_Target_Speed = 120.0f; // 搓球目标转速
float Rise_Chop_Left_Target_Speed = 72.0f; // 搓球目标转速

// --- 时间参数 ---
float pre_spin_time = 0.5f;   // 预旋转时间
float lift_time = 1.1f;       // 抬升时间
float drop_time = 0.075f;       // 下落时间
float hit_action_time = 2.0f; // 击打动作时间

// --- 抬升电机 (Lift) 参数 ---
float Rise_Lift_Target_Speed = 2.0f;  // 抬升目标转速
float Rise_Lift_Target_Dist = 200.0f; // 想要上升的高度（角度值）
float Rise_Lift_Kp = 0.02f;        // 上升到位时的柔和度
float Lift_torque_threshold = 1.0f; // 归位阈值
float LIFT_MAX_RETURN_SPEED = 10.0f; // 限制s最大归位速度，防止太快撞到底

float Rise_Gravity_Comp_Max_Current = 0.6f; // 实验测得水平时保持不掉所需的力矩值
float Rise_Zero_Angle_Offset = 0.0f; // 如果你的0度不是水平位置，需要补偿

/* =================================================================== */

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

    HAL_Delay(1000);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Front_Motors, Motor_Enable);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Right_Motors, Motor_Enable);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Left_Motors, Motor_Enable);


    Rise->lift_zero_offset = Motor_Rise_Lift_Motor.encoder.consequent_angle;

    PID_InitPIDParam(&Rise->pid.Hit_Left_Ang_PIDParam,
                     Const_HitLeftPosMotorParam[0][0],
                     Const_HitLeftPosMotorParam[0][1],
                     Const_HitLeftPosMotorParam[0][2],
                     Const_HitLeftPosMotorParam[0][3],
                     Const_HitLeftPosMotorParam[0][4],
                     Const_HitLeftPosMotorParam[1][0],
                     Const_HitLeftPosMotorParam[1][1],
                     Const_HitLeftPosMotorParam[2][0],
                     Const_HitLeftPosMotorParam[2][1],
                     Const_HitLeftPosMotorParam[3][0],
                     Const_HitLeftPosMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Hit_Left_Spd_PIDParam,
                     Const_HitLeftSpdMotorParam[0][0],
                     Const_HitLeftSpdMotorParam[0][1],
                     Const_HitLeftSpdMotorParam[0][2],
                     Const_HitLeftSpdMotorParam[0][3],
                     Const_HitLeftSpdMotorParam[0][4],
                     Const_HitLeftSpdMotorParam[1][0],
                     Const_HitLeftSpdMotorParam[1][1],
                     Const_HitLeftSpdMotorParam[2][0],
                     Const_HitLeftSpdMotorParam[2][1],
                     Const_HitLeftSpdMotorParam[3][0],
                     Const_HitLeftSpdMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Hit_Right_Ang_PIDParam,
                     Const_HitRightPosMotorParam[0][0],
                     Const_HitRightPosMotorParam[0][1],
                     Const_HitRightPosMotorParam[0][2],
                     Const_HitRightPosMotorParam[0][3],
                     Const_HitRightPosMotorParam[0][4],
                     Const_HitRightPosMotorParam[1][0],
                     Const_HitRightPosMotorParam[1][1],
                     Const_HitRightPosMotorParam[2][0],
                     Const_HitRightPosMotorParam[2][1],
                     Const_HitRightPosMotorParam[3][0],
                     Const_HitRightPosMotorParam[3][1],
                     PID_POSITION);

    PID_InitPIDParam(&Rise->pid.Hit_Right_Spd_PIDParam,
                     Const_HitRightSpdMotorParam[0][0],
                     Const_HitRightSpdMotorParam[0][1],
                     Const_HitRightSpdMotorParam[0][2],
                     Const_HitRightSpdMotorParam[0][3],
                     Const_HitRightSpdMotorParam[0][4],
                     Const_HitRightSpdMotorParam[1][0],
                     Const_HitRightSpdMotorParam[1][1],
                     Const_HitRightSpdMotorParam[2][0],
                     Const_HitRightSpdMotorParam[2][1],
                     Const_HitRightSpdMotorParam[3][0],
                     Const_HitRightSpdMotorParam[3][1],
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

void Rise_Load_Params(Rise_Params *target)
{
    // 这里做搬运工作： 左边是全局变量 = 右边是结构体成员
        Rise_Hit_Ready_Speed = target->Rise_Hit_Ready_Speed ;
        Rise_Hit_Ready_Angle = target->Rise_Hit_Ready_Angle;
        Rise_Hit_Burst_Speed = target->Rise_Hit_Burst_Speed; // 击打预备速度
        Hit_Test_Speed = target->Hit_Test_Speed; // 击打测试速度

        // --- 搓球电机 (Chop) 参数 ---
        Rise_Chop_Front_Target_Speed = target->Rise_Chop_Front_Target_Speed; // 搓球目标转速
        Rise_Chop_Right_Target_Speed = target->Rise_Chop_Right_Target_Speed; // 搓球目标转速
        Rise_Chop_Left_Target_Speed = target->Rise_Chop_Left_Target_Speed; // 搓球目标转速

        // --- 时间参数 ---
        pre_spin_time = target->pre_spin_time;   // 预旋转时间
        lift_time = target->lift_time;       // 抬升时间
        drop_time = target->drop_time;       // 下落时间
        hit_action_time = target->hit_action_time; // 击打动作时间

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
}

void Rise_Update_Fdb()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // 零点待标定
	Rise->fdb.Hit_left_angle = Motor_Rise_Hit_LeftMotor.encoder.consequent_angle;//上电归零
    Rise->fdb.Hit_left_speed = Motor_Rise_Hit_LeftMotor.encoder.standard_speed;
	Rise->fdb.Hit_right_angle = Motor_Rise_Hit_RightMotor.encoder.consequent_angle;//上电归零
    Rise->fdb.Hit_right_speed = Motor_Rise_Hit_RightMotor.encoder.standard_speed;
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

    Motor_Rise_Hit_LeftMotor.watchdog += 1;
    Motor_Rise_Hit_RightMotor.watchdog += 1;
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

    if (Motor_Rise_Hit_LeftMotor.watchdog > 40)
    {
        Rise->error_code = 1;
    }

    if (Motor_Rise_Hit_RightMotor.watchdog > 40)
    {
        Rise->error_code = 2;
    }

    if (Motor_Rise_Chop_Front_Motor.watchdog > 40)
    {
        Rise->error_code = 3;
    }

    if (Motor_Rise_Chop_Right_Motor.watchdog > 40)
    {
        Rise->error_code = 4;
    }

    if (Motor_Rise_Chop_Left_Motor.watchdog > 40)
    {
        Rise->error_code = 5;
    }

    if (Motor_Rise_Lift_Motor.watchdog > 40)
    {
        Rise->error_code = 6;
    }

    if (fabsf(Motor_Rise_Hit_LeftMotor.encoder.torque) > 4.5f || 
            fabsf(Motor_Rise_Hit_RightMotor.encoder.torque) > 4.5f)
        {
            watchdog_rise2++;
        }
        else
        {
            watchdog_rise2 = 0;
        }
    if (watchdog_rise2 > 20)
    {
        Rise->error_code = 7;
    }
}


void Rise_Set_Torque_Output(float torque_HL,float torque_HR, float torque_CF, float torque_CR, float torque_CL, float torque_Lift)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

        // =======================
        //  重力补偿逻辑
        // =======================

        // 1. 获取当前角度 (弧度制)
        // 假设 0度是水平，90度是垂直向上。
        // cos(0) = 1 (力矩最大), cos(90) = 0 (力矩为0)
        float current_angle_deg = Rise->fdb.Hit_left_angle;
        float angle_rad = (current_angle_deg + Rise_Zero_Angle_Offset) * (3.1415926f / 180.0f);

        // 2. 计算补偿力矩
        // 方向说明：如果重力是把板子往下拉（角度变小），你需要给正向力矩把它托住。
        // 请根据你的电机方向调整符号 (+ 或 -)
        float sin_angle = sinf(angle_rad);
        float gravity_comp = Rise_Gravity_Comp_Max_Current * sin_angle;

        torque_HL += gravity_comp/2.0f; // 左侧击打电机补偿
        torque_HR -= gravity_comp/2.0f; // 右侧击打

    Motor_SetMotorOutput(&Motor_Rise_Hit_LeftMotor, torque_HL);
    Motor_SetMotorOutput(&Motor_Rise_Hit_RightMotor, torque_HR);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Front_Motor, torque_CF);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Right_Motor, torque_CR);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Left_Motor, torque_CL);
    Motor_SetMotorOutput(&Motor_Rise_Lift_Motor, -torque_Lift);
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
// 定义静态变量，记录状态和时间 (这样函数退出后变量还在)
int g_hit_finished_flag = 0;
uint8_t g_auto_state = 0; // 初始状态 0: 空闲/启动
uint8_t Rise_Hit_Finished()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    // 引用状态机变量（如果在同一个文件可以直接用，不在就 extern）
    // 假设 g_auto_state 在这个文件里可见
    
    // 特殊处理：如果是自动击打阶段 (Case 4)
    if (g_auto_state == 4)
    {
        // 只判断正向终点！负向起点 -180 直接无视！
        if (Rise->fdb.Hit_left_angle > 165.0f) 
            return 1;
        else 
            return 0;
    }
    
    // 其他状态（比如 Case 5 复位，或者是手动模式）
    // 保持原来的双向判断，或者根据需要改
    else 
    {
        // 复位时可能是回 -180，也可能是回 0
        // 如果是回 0，这个函数可能用不上
        // 如果是双向判定：
        if (fabsf(Rise->fdb.Hit_left_angle) > 165.0f) return 1;
    }
    
    return 0;
}


void Rise_Set_Hybrid_Output(uint8_t Set_Pos_Stop , float hit_speed, float hit_angle, float chop_front_speed, float chop_right_speed, float chop_left_speed, float lift_speed)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // 1. 限幅
    LimitMaxMin(chop_front_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop_right_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop_left_speed, 280.0f,  -280.0f);
    LimitMaxMin(lift_speed, 5.0f, -5.0f);

    // 局部变量
    float t0 = 0 , t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;


    // --- 1. 击打电机 (Hit)
	if (Rise_Hit_Finished() == 0) 
	{
		PID_SetPIDRef(&Rise->pid.Hit_Left_Spd_PID, hit_speed);
		PID_SetPIDFdb(&Rise->pid.Hit_Left_Spd_PID, Rise->fdb.Hit_left_speed);
		PID_CalcPID(&Rise->pid.Hit_Left_Spd_PID, &Rise->pid.Hit_Left_Spd_PIDParam);
        	
        t0 = PID_GetPIDOutput(&Rise->pid.Hit_Left_Spd_PID);
	
		PID_SetPIDRef(&Rise->pid.Hit_Right_Spd_PID, -hit_speed);
		PID_SetPIDFdb(&Rise->pid.Hit_Right_Spd_PID, Rise->fdb.Hit_right_speed);
		PID_CalcPID(&Rise->pid.Hit_Right_Spd_PID, &Rise->pid.Hit_Right_Spd_PIDParam);

        t1 = PID_GetPIDOutput(&Rise->pid.Hit_Right_Spd_PID);
	} 
	else
    {
        if(Set_Pos_Stop == 1)
	    {
		PID_SetPIDRef(&Rise->pid.Hit_Left_Ang_PID, hit_angle);
		PID_SetPIDFdb(&Rise->pid.Hit_Left_Ang_PID, Rise->fdb.Hit_left_angle);
		PID_CalcPID(&Rise->pid.Hit_Left_Ang_PID, &Rise->pid.Hit_Left_Ang_PIDParam);
	
		PID_SetPIDRef(&Rise->pid.Hit_Left_Spd_PID, PID_GetPIDOutput(&Rise->pid.Hit_Left_Ang_PID));
		PID_SetPIDFdb(&Rise->pid.Hit_Left_Spd_PID, Rise->fdb.Hit_left_speed);
		PID_CalcPID(&Rise->pid.Hit_Left_Spd_PID, &Rise->pid.Hit_Left_Spd_PIDParam);

        t0 = PID_GetPIDOutput(&Rise->pid.Hit_Left_Spd_PID);
	
		PID_SetPIDRef(&Rise->pid.Hit_Right_Spd_PID, -hit_angle);
		PID_SetPIDFdb(&Rise->pid.Hit_Right_Spd_PID, Rise->fdb.Hit_right_angle);
		PID_CalcPID(&Rise->pid.Hit_Right_Spd_PID, &Rise->pid.Hit_Right_Spd_PIDParam);
	
		PID_SetPIDRef(&Rise->pid.Hit_Right_Spd_PID, PID_GetPIDOutput(&Rise->pid.Hit_Right_Ang_PID));
		PID_SetPIDFdb(&Rise->pid.Hit_Right_Spd_PID, Rise->fdb.Hit_right_speed);
		PID_CalcPID(&Rise->pid.Hit_Right_Spd_PID, &Rise->pid.Hit_Right_Spd_PIDParam);
        
        t1 = PID_GetPIDOutput(&Rise->pid.Hit_Right_Spd_PID);
	
	    }    

        else{
		PID_SetPIDRef(&Rise->pid.Hit_Left_Spd_PID, 0.0f);
		PID_SetPIDFdb(&Rise->pid.Hit_Left_Spd_PID, Rise->fdb.Hit_left_speed);
		PID_CalcPID(&Rise->pid.Hit_Left_Spd_PID, &Rise->pid.Hit_Left_Spd_PIDParam);

        t0 = PID_GetPIDOutput(&Rise->pid.Hit_Left_Spd_PID);
	
	
		PID_SetPIDRef(&Rise->pid.Hit_Right_Spd_PID, 0.0f);
		PID_SetPIDFdb(&Rise->pid.Hit_Right_Spd_PID, Rise->fdb.Hit_right_speed);
		PID_CalcPID(&Rise->pid.Hit_Right_Spd_PID, &Rise->pid.Hit_Right_Spd_PIDParam);
        
        t1 = PID_GetPIDOutput(&Rise->pid.Hit_Right_Spd_PID);
        }
    }       

        // Front
        PID_SetPIDRef(&Rise->pid.Chop_Front_Spd_PID, chop_front_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, Rise->fdb.Chop_front_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
        t2 = PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID) ;

        // Right
        PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, chop_right_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, Rise->fdb.Chop_right_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
        t3 = PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID) ;

        // Left
        PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID, chop_left_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, Rise->fdb.Chop_left_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Left_Spd_PID, &Rise->pid.Chop_Left_Spd_Middle_PIDParam);
        t4 = PID_GetPIDOutput(&Rise->pid.Chop_Left_Spd_PID) ;

        // --- 3. 抬升电机 (Lift): 使用【速度 PID】 ---
        PID_SetPIDRef(&Rise->pid.Lift_Spd_PID, lift_speed);
        PID_SetPIDFdb(&Rise->pid.Lift_Spd_PID, Rise->fdb.Lift_pitch_speed);
        PID_CalcPID(&Rise->pid.Lift_Spd_PID, &Rise->pid.Lift_Spd_Middle_PIDParam);
        t5 = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID);

    // 统一发送
    Rise_Set_Torque_Output(t0, t1, t2, t3, t4, t5);
}




void Rise_Hit_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
 
    Rise_Set_Hybrid_Output(1, Hit_Test_Speed , 0.0f , 0.0f, 0.0f, 0.0f, 0.0f);
}

void Rise_Chop_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    Rise_Set_Hybrid_Output( 0, 0.0f , Rise->fdb.Hit_left_angle, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, 0.0f);
}

void Rise_Lift_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    Rise_Set_Hybrid_Output(0, 0.0f, Rise->fdb.Hit_left_angle, 0.0f, 0.0f, 0.0f, Rise_Lift_Target_Speed);

}



float current_time;
float current_height;
float g_auto_start_time = 0.0f;
float g_auto_start_height = 0.0f;
float return_ramp_angle = 0.0f;
int lift_break = 0;
float cmd_speed = 0.0f;

void Rise_Auto_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    Rise_Set_OutputState(Rise_middle);
    current_time = DWT_GetTimeline_s();

    switch (g_auto_state)
    {

    case 0: // 启动
        // Rise_Reset_Hit_Traj();
        g_hit_finished_flag = 0;
        g_auto_start_time = current_time;
        g_auto_state = 1;
        g_auto_start_height = Rise->fdb.Lift_pitch_angle;
        break;

    case 1: // 预旋转 (Pre-spin)
        // Hit: 保持0度 | Chop: 转 | Lift: 0
    {
        Rise_Set_Hybrid_Output(1, -Rise_Hit_Ready_Speed, -Rise_Hit_Ready_Angle , Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, 0.0f);

        if (current_time - g_auto_start_time >= pre_spin_time)
        {
            g_auto_start_time = current_time;
            g_auto_state = 2;
            PID_Clear(&Rise->pid.Lift_Spd_PID); // 清除抬升电机 PID 状态，防止历史影响
        }
    }
        break;

    case 2: // 抬升 (Lift)
    {   // Hit: 保持0度 | Chop: 转 | Lift: 转
        float lift_up_target = g_auto_start_height + Rise_Lift_Target_Dist;
        cmd_speed = 0.0f;
        float slow_up_zone = Rise_Lift_Target_Dist * 0.2;
        float lift_current_pos = Rise->fdb.Lift_pitch_angle;


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
            cmd_speed = (lift_up_target - lift_current_pos) * Rise_Lift_Kp;

            // 最小速度钳制（防止太慢到不了）
            if (cmd_speed < 1.0f && cmd_speed > 0.1f)
                cmd_speed = 1.0f;
        }

        // 3. 发送指令
        Rise_Set_Hybrid_Output(1, -Rise_Hit_Ready_Speed, -Rise_Hit_Ready_Angle, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, cmd_speed);

        // 4. 退出条件：超时判断 (lift_time)
        if (current_time - g_auto_start_time >= lift_time)
        {
            g_auto_start_time = current_time; // 重置时间给下一阶段
            g_auto_state = 3;
        }

    }    
    break;

    case 3: // 等待下落 (Drop)
    {        // 获取当前高度 (假设上方是负数，例如 -7000; 目标是 0)
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

        if(current_height < g_auto_start_height)
        {
            return_speed = 0.0f;
        }
        
        // 3. 发送指令
        // 注意：这里不需要再用 LimitMaxMin 了，因为我们已经手动控制了速度
        Rise_Set_Hybrid_Output(1, -Rise_Hit_Ready_Speed, -Rise_Hit_Ready_Angle, Rise_Chop_Front_Target_Speed, Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, return_speed);

        // 4. 状态转换
        if (current_time - g_auto_start_time >= drop_time)
        {
            g_auto_start_time = current_time;
            g_hit_finished_flag = 0; // 确保标志位干净
            g_auto_state = 4;
        }
    }
        break;

    case 4: // 击打 (Hit)
    {

        // 继续计算归位速度，让它死死锁在 0 点，防止松动
        float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
        float hold_speed = -pos_error * Rise_Lift_Kp;
        LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

        Rise_Set_Hybrid_Output(0, Rise_Hit_Burst_Speed, NAN,
                                  Rise_Chop_Front_Target_Speed, 
                                  Rise_Chop_Right_Target_Speed, 
                                  Rise_Chop_Left_Target_Speed, 
                                  hold_speed);

        if (current_time - g_auto_start_time > 0.2f) 
        {
            if (Rise_Hit_Finished() == 1 || current_time - g_auto_start_time >= hit_action_time)
            {
                g_auto_state = 5;
                g_auto_start_time = current_time;
            }
        }
    }
    break;

    case 5: // 复位 (Reset)
    {    // 动作：Hit 归零 | Chop 停 | Lift 停 (或继续锁零)
        
            // 保持抬升机构锁在 0 点 (防止复位震动导致托盘掉下来)
            float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
            float hold_speed = -pos_error * Rise_Lift_Kp;
            LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);
 
            Rise_Set_Hybrid_Output(1, -1.5f ,0.0f, 0.0f, 0.0f, 0.0f, hold_speed);

        // ！！！ 关键修改：等待复位完成 ！！！
        // 使用全局变量 Rise_Hit_Return_Time (例如 1.0秒)
            if (fabsf(Rise->fdb.Hit_left_angle - 0.0f) <= 1.0f || (current_time - g_auto_start_time >= 10.0f))
            {
                g_auto_state = 6; // 时间到了，才进入“完成态”
            }
    }
     break;

    case 6: // 阶段6：完成等待 (DONE)
    {    // 计算锁住位置需要的力/速度
        float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
        float hold_speed = -pos_error * Rise_Lift_Kp;
        LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

        Rise_Set_Torque_Output(0, 0, 0.0f, 0.0f, 0.0f, 0.0f);      // 抬升保持
        break;

    default:
        g_auto_state = 0;
    } 
    break;
    }
}


static float hit_start_angle = 0.0f;
void Rise_Without_Hit_Cal()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    Rise_Set_OutputState(Rise_middle);
    current_time = DWT_GetTimeline_s();

    switch (g_auto_state)
    {
    case 0: // 启动与初始化
        g_hit_finished_flag = 0;
        g_auto_start_time = current_time;
        g_auto_start_height = Rise->fdb.Lift_pitch_angle;
        g_auto_state = 1;
        hit_start_angle = Rise->fdb.Hit_left_angle;
        break;

    case 1: // 预旋转 (Pre-spin)
        // Hit: 锁在返回角 | Chop: 搓球电机转动 | Lift: 0
        Rise_Set_Hybrid_Output(0 , 0.0f, NAN, Rise_Chop_Front_Target_Speed, 
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
            cmd_speed = (lift_up_target - lift_current_pos) * Rise_Lift_Kp;
            if (cmd_speed < 1.0f && cmd_speed > 0.1f) cmd_speed = 1.0f;
        }

        Rise_Set_Hybrid_Output(0 , 0.0f, NAN, Rise_Chop_Front_Target_Speed, 
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

        Rise_Set_Hybrid_Output(0 , 0.0f, NAN, Rise_Chop_Front_Target_Speed, 
                               Rise_Chop_Right_Target_Speed, Rise_Chop_Left_Target_Speed, return_speed);


    }
    break;

    case 4: // 完成态 (DONE) - 锁死零点并停止搓球
    {
        // 搓球电机设为 0，停止旋转
        Rise_Set_Torque_Output(0.0f , 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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
        Rise_Set_OutputState(Rise_middle);
        Rise_Auto_Cal();
        break;
    case Rise_Cuoqiu:
        Rise_Set_OutputState(Rise_middle);
        Rise_Chop_Cal();
        break;
    case Rise_Taisheng:
        Rise_Set_OutputState(Rise_middle);
        Rise_Lift_Cal();
        break;
    case Rise_Stop:
        Rise_Set_OutputState(Rise_stop);
        Rise_Set_Torque_Output(0 , 0, 0, 0, 0, 0);
        break;
    case Rise_Without_Hit:
         Rise_Set_OutputState(Rise_middle);
        Rise_Without_Hit_Cal();
        break;
    case Rise_Hit:
        Rise_Set_OutputState(Rise_middle);
        Rise_Hit_Cal();
        break;
    default:
        break;
    }
}

void Rise_Output()
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
    Motor_SendMotorGroupOutput(Motor_groupHandle[7]);
    DWT_Delayus(100);
    Motor_SendMotorGroupOutput(Motor_groupHandle[8]);
    DWT_Delayus(100);
    Motor_SendMotorGroupOutput(Motor_groupHandle[9]);
    DWT_Delayus(100);
    Motor_SendMotorGroupOutput(Motor_groupHandle[10]);
    DWT_Delayus(100);
    Motor_SendMotorGroupOutput(Motor_groupHandle[11]);

    // last_output_time = DWT_GetTimeline_us();
    DWT_Delayus(100);
}
