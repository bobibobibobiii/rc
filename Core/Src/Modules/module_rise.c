/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-10-31 18:53:16
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2026-01-16 22:43:39
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

float Rise_Hit_Target_Angle = 80.0f;  // 击打目标角度
float Rise_Hit_Return_Angle = -45.0f;   // 返回角度
float Rise_Hit_Target_Speed = 18.0f; // 击打目标速度
float RISE_HIT_ACCEL_LIMIT = 5000.0f; // 击打最大加速度限制
float RISE_HIT_V_MAX = 20.0f; // 击打最大速度
float REAL_HIT_ANGLE = 70.0f;
float HIT_INIT_ANGLE = 0.0f;

// --- 搓球电机 (Chop) 参数 ---
float Rise_Chop_Front_Target_Speed = 95.0f; // 搓球目标转速
float Rise_Chop_Right_Target_Speed = 120.0f; // 搓球目标转速
float Rise_Chop_Left_Target_Speed = 95.0f; // 搓球目标转速

// --- 抬升电机 (Lift) 参数 ---
float Rise_Lift_Target_Speed = 2.0f;  // 抬升目标转速
float Rise_Lift_Target_Dist = 200.0f; // 想要上升的高度（角度值）
float Rise_Lift_Kp_Up = 0.05f;        // 上升到位时的柔和度

// --- 时间参数 ---
float pre_spin_time = 0.5f;   // 预旋转时间
float lift_time = 1.4f;       // 抬升时间
float drop_time = 0.09f;       // 下落时间
float hit_action_time = 2.0f; // 击打动作时间


float Lift_torque_threshold = 2.0f; // 归位阈值
float Hit_torque_threshold = 2.0f; // 归位阈值

float LIFT_RETURN_KP = 0.02f;        // 归位力度 (值越大回得越快，太大会震荡)
float LIFT_MAX_RETURN_SPEED = 10.0f; // 限制最大归位速度，防止太快撞到底


float Rise_Gravity_Comp_Max_Current = 2.0f; // 实验测得水平时保持不掉所需的力矩值
float Rise_Zero_Angle_Offset = 0.0f; // 如果你的0度不是水平位置，需要补偿

// --- 阻抗控制参数 ---
float Rise_Imp_FF_Hit      = 20.0f;  // 击球时的额外爆发力矩
float Rise_Hit_Zone_Width  = 90.0f;  // 击球区宽度

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
    Rise->fdb.Hit_left_angle = Motor_Rise_Hit_Motor.encoder.consequent_angle-0.0;//标记竖直向下的位置为0点
    Rise->fdb.Hit_left_speed = Motor_Rise_Hit_Motor.encoder.standard_speed;
    Rise->fdb.Hit_right_angle = Motor_Rise_Hit_Motor.encoder.consequent_angle-0.0;//标记竖直向下的位置为0点
    Rise->fdb.Hit_right_speed = Motor_Rise_Hit_Motor.encoder.standard_speed;
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

    if (Motor_Rise_Hit_LeftMotor.watchdog > 20)
    {
        Rise->error_code = 1;
    }

    if (Motor_Rise_Hit_RightMotor.watchdog > 20)
    {
        Rise->error_code = 2;
    }

    if (Motor_Rise_Chop_Front_Motor.watchdog > 20)
    {
        Rise->error_code = 3;
    }

    if (Motor_Rise_Chop_Right_Motor.watchdog > 20)
    {
        Rise->error_code = 4;
    }

    if (Motor_Rise_Chop_Left_Motor.watchdog > 20)
    {
        Rise->error_code = 5;
    }

    if (Motor_Rise_Lift_Motor.watchdog > 20)
    {
        Rise->error_code = 6;
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
        Rise->error_code = 7;
    }
}

// --- 上电回零相关参数 ---
uint8_t g_system_homed = 0;         // 全局标志：0=未回零，1=已回零
static uint8_t homing_state = 0; 
static float homing_start_time = 0; // 超时计时

// 回零参数配置
const float Homing_Timeout = 8.0f;    // 超时保护 (秒)

/**
 * @brief 上电自动回零主循环 (非阻塞)
 */
void Rise_Homing_Loop(float hit_init_pos)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    float current_time = DWT_GetTimeline_s();
    static uint8_t stable_count = 0; // 防抖计数器
    
    static float traj_cmd_pos = 0.0f; // 轨迹指令 (原 curr_angle)
    static float final_target = 0.0f; // 锁定终点

    // 强制输出状态为 Middle
    Rise_Set_OutputState(Rise_middle);

    switch (homing_state)
    {
    case 0: // --- 初始化 ---
    {
        // 等待解包函数运行

        // 1. 获取当前真实位置作为起点
        traj_cmd_pos = Rise->fdb.Hit_pitch_angle; 
        
        // 2. 计算最短路径
        float diff = hit_init_pos - traj_cmd_pos;
        
        float err = fmodf(diff, 360.0f); 
        
        // 标准化到 -180 ~ +180
        if (err > 180.0f) err -= 360.0f;
        if (err < -180.0f) err += 360.0f;

        // 3. 锁定最终目标
        final_target = traj_cmd_pos + err;

        homing_start_time = current_time;
        homing_state = 1;
        stable_count = 0;
        break;
    }

    case 1: // --- 移动归零 ---
    {
        float return_step = 0.3f; // 建议改小一点，0.5 对应 500度/秒，可能太快

        // 1. 轨迹生成 (Ramping)
        // 让 traj_cmd_pos 慢慢接近 final_target
        if (traj_cmd_pos < final_target - return_step) {
            traj_cmd_pos += return_step; 
        } 
        else if (traj_cmd_pos > final_target + return_step) {
            traj_cmd_pos -= return_step; 
        } 
        else {
            traj_cmd_pos = final_target; // 锁死
        }

        // 2. 发送指令
        Rise_Set_Hybrid_Output(traj_cmd_pos, 0.0f, 0.0f, 0.0f, 0.0f);

        // 3. 判断到位
        // 必须比较：【真实反馈值】 <-> 【锁定终点】
        // 这里的 < 0.2f 是允许误差
        if (Rise->fdb.Hit_pitch_angle - final_target < 0.2f && Rise->fdb.Hit_pitch_angle - final_target >= 0.0f && 
            fabsf(Rise->fdb.Hit_pitch_speed) < 0.2f)
        {
            stable_count++;
            if (stable_count > 50) // 持续 50ms 稳定
            {
                homing_state = 2; 
            }
        }
        else
        {
            stable_count = 0;
        }

        // 4. 超时保护
        if (current_time - homing_start_time > Homing_Timeout)
        {
            Rise->error_code = 7; 
            homing_state = 2; 
        }
        break;
    }

    case 2: // --- 完成 ---
    {
        g_system_homed = 1; // 标记完成
        
        Rise->ctrl_mode = Rise_Stop; 
        break;
    }
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
// 
float KICK_TORQUE_VAL=2.0f;  // 踹这一脚的力度 (比如 PID 给 30, 这里加 20, 总共 50)
#define KICK_SPEED_THRES   0.5f   // 速度低于多少认为“没动” (rad/s)
#define CMD_DEAD_ZONE      0.1f   // 上层指令死区 (防止微小噪音触发)
void Rise_Set_Torque_Output(float torque_Hit, float torque_CF, float torque_CR, float torque_CL, float torque_Lift)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

        //脉冲启动
        float curr_speed = Rise->fdb.Hit_pitch_speed; // 获取当前真实速度

        // 1. 判断是否需要介入
        // 条件A: 上层给的力气大于死区 (说明想动)
        // 条件B: 实际速度非常小 (说明被静摩擦锁住了)
        if (fabsf(torque_Hit) > CMD_DEAD_ZONE && fabsf(curr_speed) < KICK_SPEED_THRES)
        {
            // 2. 根据方向施加“踹力”
            if (torque_Hit > 0) 
            {
                // 正向指令 -> 正向踹
                torque_Hit += KICK_TORQUE_VAL;
            }
            else 
            {
                // 负向指令 -> 负向踹 (让负值更负)
                torque_Hit -= KICK_TORQUE_VAL;
            }
    }

        // =======================
        // ⚖️ 新增：重力补偿逻辑
        // =======================

        // 1. 获取当前角度 (弧度制)
        // 假设 0度是水平，90度是垂直向上。
        // cos(0) = 1 (力矩最大), cos(90) = 0 (力矩为0)
        float current_angle_deg = Rise->fdb.Hit_pitch_angle;
        float angle_rad = (current_angle_deg + Rise_Zero_Angle_Offset) * (3.1415926f / 180.0f);

        // 2. 计算补偿力矩
        // 方向说明：如果重力是把板子往下拉（角度变小），你需要给正向力矩把它托住。
        // 请根据你的电机方向调整符号 (+ 或 -)
        float sin_angle = sinf(angle_rad);
        float gravity_comp = Rise_Gravity_Comp_Max_Current * sin_angle;

        torque_Hit += gravity_comp;

    Motor_SetMotorOutput(&Motor_Rise_Hit_Motor, torque_Hit);
    // Motor_SetMotorOutput(&Motor_Rise_Hit_Motor, gravity_comp);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Front_Motor, torque_CF);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Right_Motor, torque_CR);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Left_Motor, torque_CL);
    Motor_SetMotorOutput(&Motor_Rise_Lift_Motor, -torque_Lift);
}

void Rise_Set_Angle_Output(float ang1, float ang2, float ang3, float ang4, float ang5)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    // float torque1, torque2, torque3, torque4;
    LimitMaxMin(ang1, 360.0f, -360.0f);
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
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, Motor_Rise_Chop_Front_Motor.encoder.speed);
        PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
        torque_CF = PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID);

        PID_SetPIDRef(&Rise->pid.Chop_Right_Ang_PID, ang3);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Ang_PID, Rise->fdb.Chop_right_pitch_angle);
        PID_CalcPID(&Rise->pid.Chop_Right_Ang_PID, &Rise->pid.Chop_Right_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, PID_GetPIDOutput(&Rise->pid.Chop_Right_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, Motor_Rise_Chop_Right_Motor.encoder.speed);
        PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
        torque_CR = PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID);

        PID_SetPIDRef(&Rise->pid.Chop_Left_Ang_PID, ang4);
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Ang_PID, Rise->fdb.Chop_left_pitch_angle);
        PID_CalcPID(&Rise->pid.Chop_Left_Ang_PID, &Rise->pid.Chop_Left_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID, PID_GetPIDOutput(&Rise->pid.Chop_Left_Ang_PID));
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, Motor_Rise_Chop_Left_Motor.encoder.speed);
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
    LimitMaxMin(chop1_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop2_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop3_speed, 280.0f,  -280.0f);
    LimitMaxMin(lift_speed,  469.0f,  -469.0f);

        float avg_speed = (Rise->fdb.Chop_front_pitch_speed + Rise->fdb.Chop_right_pitch_speed + Rise->fdb.Chop_left_pitch_speed) / 3.0f;

        float front_sync_err = Rise->fdb.Chop_front_pitch_speed - avg_speed;
        float right_sync_err = Rise->fdb.Chop_right_pitch_speed - avg_speed;
        float left_sync_err = Rise->fdb.Chop_left_pitch_speed - avg_speed;

    switch (Rise->output_state)
    {

    case Rise_fast:

        PID_SetPIDRef(&Rise->pid.Hit_Spd_PID, hit_speed);
        PID_SetPIDFdb(&Rise->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed);
        PID_CalcPID(&Rise->pid.Hit_Spd_PID, &Rise->pid.Hit_Spd_Fast_PIDParam);
        torque_Hit = PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID);

        // chop的逻辑;

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

    case Rise_middle:

        PID_SetPIDRef(&Rise->pid.Hit_Spd_PID, hit_speed);
        PID_SetPIDFdb(&Rise->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed);
        PID_CalcPID(&Rise->pid.Hit_Spd_PID, &Rise->pid.Hit_Spd_Middle_PIDParam);
        torque_Hit = PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID);


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
    LimitMaxMin(hit_angle, 360.0f, -360.0f);
    LimitMaxMin(chop_front_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop_right_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop_left_speed, 280.0f,  -280.0f);
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


/**
 * @brief 终极混合控制函数
 * @param hit_angle   击打电机 -> 目标角度 (位置闭环)
 * @param chop_speed  搓球电机 -> 目标速度 (速度闭环 + 交叉耦合)
 * @param lift_speed  抬升电机 -> 目标速度 (速度闭环)
 */
void Rise_Set_Hybrid_FF_Output(float target_angle,float extra_torque_ff ,float max_speed_limit, float chop_front_speed, float chop_right_speed, float chop_left_speed, float lift_speed)
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    // 1. 限幅
    LimitMaxMin(target_angle, 360.0f, -360.0f);
    LimitMaxMin(chop_front_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop_right_speed, 280.0f,  -280.0f);
    LimitMaxMin(chop_left_speed, 280.0f,  -280.0f);
    LimitMaxMin(lift_speed, 469.0f, -469.0f);

    // 局部变量
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
    float t_hit_final = 0.0f;

    if (Rise->output_state == Rise_middle)
    {
    
        // --- 1. 计算原本的串级 PID (负责稳) ---
        // 位置环
        PID_SetPIDRef(&Rise->pid.Hit_Ang_PID, target_angle);
        PID_SetPIDFdb(&Rise->pid.Hit_Ang_PID, Rise->fdb.Hit_pitch_angle);
        PID_CalcPID(&Rise->pid.Hit_Ang_PID, &Rise->pid.Hit_Ang_Fast_PIDParam); // 用 Middle 或 Fast 参数
        // 2. 获取位置环输出 (这就是期望速度)
        float expected_speed = PID_GetPIDOutput(&Rise->pid.Hit_Ang_PID);

        // 【核心修改】在这里对期望速度进行限幅！
        // 如果 PID 想跑 1000 deg/s，但你限制了 300，它就只能输出 300
        LimitMaxMin(expected_speed, max_speed_limit, -max_speed_limit);

        // 速度环
        PID_SetPIDRef(&Rise->pid.Hit_Spd_PID,expected_speed);
        PID_SetPIDFdb(&Rise->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed);
        PID_CalcPID(&Rise->pid.Hit_Spd_PID, &Rise->pid.Hit_Spd_Fast_PIDParam); // 用 Middle 或 Fast 参数

        // PID 计算出的基础力矩
        float torque_from_pid = PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID);

        // 判断是否处于“强力击打模式”
        // 依据：如果前馈力矩很大（比如超过 5.0），说明我们在梯形的平顶区或爬升区
        if (fabsf(extra_torque_ff) > 3.0f) 
        {
            // 【纯开环模式】
            // 直接输出前馈，完全无视 PID
            t_hit_final = extra_torque_ff; 
            
            // 【可选优化：混合模式】
            // 如果你还是有点怕，可以保留 10% 的 PID 稍微稳一下
            // t_hit_final = extra_torque_ff + (torque_from_pid * 0.1f);
        }
        else
        {
            // 【闭环模式】
            // 在非击打区（准备、随挥结束），依然用 PID + FF
            // 这样能保证静止时的定位精度和复位时的柔顺
            t_hit_final = torque_from_pid + extra_torque_ff;
        }

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
    Rise_Set_Torque_Output(t_hit_final, t2, t3, t4, t5);
}




// 定义静态变量，记录状态和时间 (这样函数退出后变量还在)
static uint32_t last_toggle_time = 0;
static uint8_t target_state = 0; // 0:去90度, 1:回0度
int g_hit_finished_flag = 0;

void Rise_Hit_Cal()
{
    // 1. 获取当前时间 (单位: 毫秒)
    uint32_t now = HAL_GetTick(); 

    // 2. 判断时间是否到了切换的时候 (例如每 1000ms 切换一次)
    if (now - last_toggle_time > 1000) 
    {
        target_state = !target_state; // 状态反转 (0->1, 1->0)
        last_toggle_time = now;       // 更新时间戳
    }

    // 3. 根据状态设定目标角度
    float target_angle = (target_state == 0) ? 90.0f : 0.0f;

    // 4. 【关键】每一帧都必须调用 PID 计算函数！
    // 只有不停地调用它，电机才能持续获得正确的力矩去接近目标
    Rise_Set_OutputState(Rise_middle);
    Rise_Set_Angle_Output(target_angle, 0.0f, 0.0f, 0.0f, 0.0f);
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
float return_ramp_angle = 0.0f;
float burst_ff = 0.0f;

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
            // Rise_Reset_Hit_Traj();
            g_hit_finished_flag = 0; // 确保标志位干净
            g_auto_state = 4;
        }

        break;
    case 4: // 击打 (Hit)
    {

        // 继续计算归位速度，让它死死锁在 0 点，防止松动
        float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height;
        float hold_speed = -pos_error * LIFT_RETURN_KP;
        LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

        // ==========================================
        // 2. 击打 (Hit) 核心逻辑
        // ==========================================
        float current_pos = Rise->fdb.Hit_pitch_angle;
        

        float pid_target = Rise_Hit_Target_Angle + 100.0f; 

        

        // --- B. 移植过来的【安全刹车逻辑】(重要！) ---
        const float HARD_LIMIT = 270.0f - 10.0f; // 绝对物理限位前一点点
        
        // 如果随挥目标太远，已经撞墙了，必须截断！
        if (pid_target > HARD_LIMIT) {
            pid_target = HARD_LIMIT;
        }
        
        // 如果当前位置已经非常危险，强制把目标设为当前位置或稍微反向，防止 PID 继续推
        if (current_pos > HARD_LIMIT) {
             pid_target = HARD_LIMIT - 5.0f; // 往回拉一点
        }

        // --- C. 计算爆发前馈 (FF) ---
        
        float dist_to_center = fabsf(current_pos - REAL_HIT_ANGLE);
        
        // 在击球区内 (例如 +/- 30度)
        if (dist_to_center < Rise_Hit_Zone_Width)
        {
            // --- 梯形前馈 (Trapezoidal) ---
            // 优势：中间有一段平顶，容许击球点有偏差也能吃满力矩
            
            float scale = 0.0f;
            float ramp_width = 12.0f; // 爬升区宽度 (度)
            
            // 计算平顶区的边缘
            float plateau_edge = Rise_Hit_Zone_Width - ramp_width; 

            if (dist_to_center <= plateau_edge)
            {
                // A. 平顶区 (Plateau): 全功率输出
                // 只要在这个范围内，scale 都是 1.0
                scale = 1.0f; 
            }
            else
            {
                // B. 爬升/下降区 (Ramp): 线性变化
                // dist_from_outer_edge 是距离最外边缘还有多远
                float dist_from_outer_edge = Rise_Hit_Zone_Width - dist_to_center;
                
                // 距离边缘越远(越靠内)，力越大
                scale = dist_from_outer_edge / ramp_width;
            }

            
            
            // 防止计算误差导致负数 (虽然逻辑上不会)
            if (scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;

            burst_ff = Rise_Imp_FF_Hit * scale; 
        }
        else
        {
            burst_ff = 0.0f;
        }
        Rise_Set_Hybrid_FF_Output(pid_target, burst_ff, Rise_Hit_Target_Speed,
                                  Rise_Chop_Front_Target_Speed, 
                                  Rise_Chop_Right_Target_Speed, 
                                  Rise_Chop_Left_Target_Speed, 
                                  hold_speed);
        // Rise_Set_Hybrid_Output(pid_target,
        //                           Rise_Chop_Front_Target_Speed, 
        //                           Rise_Chop_Right_Target_Speed, 
        //                           Rise_Chop_Left_Target_Speed, 
        //                           hold_speed);

        if (current_time - g_auto_start_time >= hit_action_time || g_hit_finished_flag == 1)
        {
            g_hit_finished_flag = 0;
            PID_Clear(&Rise->pid.Hit_Spd_PID);
            PID_Clear(&Rise->pid.Hit_Ang_PID);
            return_ramp_angle = Rise->fdb.Hit_pitch_angle;
            Rise_Set_OutputState(Rise_middle);
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

            float return_step = 0.1f;
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
        g_hit_finished_flag = 0;
        g_case0_entry_count++;
        g_auto_start_time = current_time;
        g_auto_start_height = Rise->fdb.Lift_pitch_angle;
        g_auto_state = 1;
        float hit_start_angle = Rise->fdb.Hit_pitch_angle;
        break;

    case 1: // 预旋转 (Pre-spin)
        // Hit: 锁在返回角 | Chop: 搓球电机转动 | Lift: 0
        Rise_Set_Hybrid_Output(hit_start_angle, Rise_Chop_Front_Target_Speed, 
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

        Rise_Set_Hybrid_Output(hit_start_angle, Rise_Chop_Front_Target_Speed, 
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

        Rise_Set_Hybrid_Output(hit_start_angle, Rise_Chop_Front_Target_Speed, 
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

    if (g_system_homed == 0)
    {
        Rise_Homing_Loop(HIT_INIT_ANGLE);
        return; // 强制返回，不执行后面逻辑
    }

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
        Rise_Auto_Cal();
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
        // Rise_Set_Speed_Output(0, 0, 0, 0, 0);
        break;
    case Rise_Without_Hit:
        Rise_Without_Hit_Cal();
        break;
    case Rise_Hit:
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
