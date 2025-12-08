/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-10-31 18:53:16
 * @LastEditors: WenXin Tan 3086080053@qq.com
 * @LastEditTime: 2025-12-08 21:41:23
 * @FilePath: \MDK-ARM\module_rise.c
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
float Rise_Hit_Target_Angle = 45.0f;   // 击打目标角度
float Rise_Hit_Return_Angle = 0.0f;    // 返回角度
float Rise_Hit_Hold_Time    = 0.5f;    // 保持时间 (s)
float Rise_Hit_Return_Time  = 1.0f;    // 归位等待时间 (s)

// --- 搓球电机 (Chop) 参数 ---
float Rise_Chop_Target_Speed = 100.0f; // 搓球目标转速

// --- 抬升电机 (Lift) 参数 ---
float Rise_Lift_Target_Speed = 10.0f;  // 抬升目标转速

float pre_spin_time = 0.5f; //预旋转时间
float lift_time = 2.0f; //抬升时间
float drop_time = 0.8f; //下落时间
float hit_action_time = 1.0f; //击打动作时间    

float LIFT_RETURN_KP = 0.02f;  // 归位力度 (值越大回得越快，太大会震荡)
float LIFT_MAX_RETURN_SPEED = 10.0f; // 限制最大归位速度，防止太快撞到底

/* =================================================================== */
float torque1, torque2, torque3, torque4, torque5;

Rise_DataTypeDef Rise_Data;
  /**
  * @brief      Get the pointer of Rise control object
  * @param      NULL
  * @retval     Pointer to Rise control object
  */
Rise_DataTypeDef* Rise_GetRisePtr() {
    return &Rise_Data;
}


void Rise_Init() {
  Rise_DataTypeDef *Rise = &Rise_Data;  


    Rise->output_state = Rise_middle;
    Rise->ctrl_mode = Rise_Stop;
    Rise->fdb.Hit_pitch_angle = 0;

    
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Front_Motors , Motor_Enable);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Right_Motors , Motor_Enable);
    Motor_DM_Basic_Output(&Motor_Rise_Chop_Left_Motors , Motor_Enable);

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

 void Rise_Update_Fdb() {
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();




   //零点待标定
    Rise->fdb.Hit_pitch_angle = 0.0f-Motor_Rise_Hit_Motor.encoder.angle;
    Rise->fdb.Hit_pitch_speed = -Motor_Rise_Hit_Motor.encoder.speed;
    Rise->fdb.Chop_front_pitch_angle = 0.0f-Motor_Rise_Chop_Front_Motor.encoder.angle;
    Rise->fdb.Chop_front_pitch_speed = -Motor_Rise_Chop_Front_Motor.encoder.speed;
    Rise->fdb.Chop_right_pitch_angle = 0.0f-Motor_Rise_Chop_Right_Motor.encoder.angle;
    Rise->fdb.Chop_right_pitch_speed = -Motor_Rise_Chop_Right_Motor.encoder.speed;
    Rise->fdb.Chop_left_pitch_angle = 0.0f-Motor_Rise_Chop_Left_Motor.encoder.angle;
    Rise->fdb.Chop_left_pitch_speed = -Motor_Rise_Chop_Left_Motor.encoder.speed;
    Rise->fdb.Lift_pitch_speed = -Motor_Rise_Lift_Motor.encoder.standard_speed;
    float raw_total_angle = Motor_Rise_Lift_Motor.encoder.consequent_angle;
    Rise->fdb.Lift_pitch_angle = raw_total_angle - Rise->lift_zero_offset;


    Motor_Rise_Hit_Motor.watchdog += 1; 
    Motor_Rise_Chop_Front_Motor.watchdog += 1; 
    Motor_Rise_Chop_Right_Motor.watchdog += 1; 
    Motor_Rise_Chop_Left_Motor.watchdog += 1; 
    Motor_Rise_Lift_Motor.watchdog += 1; 

    Rise->update_dt = DWT_GetDeltaT(&Rise->last_update_tick);

 }
uint16_t watchdog2=0;
  /**
  * @brief      Check Rise motors status
  * @param      NULL
  * @retval     NULL
  */
 void Rise_Check(){ 
   Rise_DataTypeDef *Rise = Rise_GetPlatformPtr();
        
    if(Motor_Rise_Hit_Motor.watchdog>20)
    {Rise->error_code = 1;}

    if(Motor_Rise_Chop_Front_Motor.watchdog>20)
                {Rise->error_code = 2;}
                    
    if(Motor_Rise_Chop_Right_Motor.watchdog>20)
                {Rise->error_code = 3;}

    if(Motor_Rise_Chop_Left_Motor.watchdog>20)
                {Rise->error_code = 4;}

    if(Motor_Rise_Lift_Motor.watchdog>20)
                {Rise->error_code = 5;}
                    
    if(fabsf( Motor_Rise_Lift_Motor.encoder.torque)>20.0f)
    {
        watchdog2++;
    }
    else
    {
        watchdog2=0;
    }
    if(watchdog2>20)
    {
        Rise->error_code = 6;
    }
        
}


float torque;
  /**
  * @brief      Set Rise motors torque output
  * @param      torque1: Hit motor torque
  * @param      torque2: Chop front motor torque
  * @param      torque3: Chop right motor torque
  * @param      torque4: Chop left motor torque
  * @param      torque5: Lift motor torque
  * @retval     NULL
  */
 void Rise_Set_Torque_Output(float torque1,float torque2,float torque3,float torque4,float torque5) {
	 
	 torque=torque5;

    Motor_SetMotorOutput(&Motor_Rise_Hit_Motor,torque1) ; 
    Motor_SetMotorOutput(&Motor_Rise_Chop_Front_Motor,torque2) ; 
    Motor_SetMotorOutput(&Motor_Rise_Chop_Right_Motor,torque3);
    Motor_SetMotorOutput(&Motor_Rise_Chop_Left_Motor,torque4) ;
    Motor_SetMotorOutput(&Motor_Rise_Lift_Motor,torque5) ;

 }

 
 void Rise_Set_Angle_Output(float ang1,float ang2,float ang3) {
    Rise_DataTypeDef *Rise = Rise_GetRisePtr(); 
	//float torque1, torque2, torque3, torque4;
    LimitMaxMin(ang1,75.0f,-30.0f);
    LimitMaxMin(ang2,280.0f,-280.0f);
    LimitMaxMin(ang3,360.0f,-360.0f);

    switch (Rise->output_state) {

    case Rise_middle:
        PID_SetPIDRef(&Rise->pid.Hit_Ang_PID, ang1);
        PID_SetPIDFdb(&Rise->pid.Hit_Ang_PID,Rise->fdb.Hit_pitch_angle); 
        PID_CalcPID(&Rise->pid.Hit_Ang_PID, &Rise->pid.Hit_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise ->pid.Hit_Spd_PID,PID_GetPIDOutput(&Rise->pid.Hit_Ang_PID) );
        PID_SetPIDFdb(&Rise ->pid.Hit_Spd_PID, -Motor_Rise_Hit_Motor.encoder.speed);
        PID_CalcPID(&Rise ->pid.Hit_Spd_PID, &Rise ->pid.Hit_Spd_Middle_PIDParam);
        torque1 = -PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID)-0.0f;

        PID_SetPIDRef(&Rise->pid.Chop_Front_Ang_PID, ang2);
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Ang_PID,Rise->fdb.Chop_front_pitch_angle); 
        PID_CalcPID(&Rise->pid.Chop_Front_Ang_PID, &Rise->pid.Chop_Front_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise ->pid.Chop_Front_Spd_PID,PID_GetPIDOutput(&Rise->pid.Chop_Front_Ang_PID) );
        PID_SetPIDFdb(&Rise ->pid.Chop_Front_Spd_PID, -Motor_Rise_Chop_Front_Motor.encoder.speed);
        PID_CalcPID(&Rise ->pid.Chop_Front_Spd_PID, &Rise ->pid.Chop_Front_Spd_Middle_PIDParam);
        torque2 = -PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID)-0.0f;

        PID_SetPIDRef(&Rise->pid.Chop_Right_Ang_PID, ang2);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Ang_PID,Rise->fdb.Chop_right_pitch_angle); 
        PID_CalcPID(&Rise->pid.Chop_Right_Ang_PID, &Rise->pid.Chop_Right_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise ->pid.Chop_Right_Spd_PID,PID_GetPIDOutput(&Rise->pid.Chop_Right_Ang_PID) );
        PID_SetPIDFdb(&Rise ->pid.Chop_Right_Spd_PID, -Motor_Rise_Chop_Right_Motor.encoder.speed);
        PID_CalcPID(&Rise ->pid.Chop_Right_Spd_PID, &Rise ->pid.Chop_Right_Spd_Middle_PIDParam);
        torque3 = -PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID)-0.0f;

        PID_SetPIDRef(&Rise->pid.Lift_Ang_PID, ang2);
        PID_SetPIDFdb(&Rise->pid.Lift_Ang_PID,Rise->fdb.Lift_pitch_angle); 
        PID_CalcPID(&Rise->pid.Lift_Ang_PID, &Rise->pid.Lift_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise ->pid.Lift_Spd_PID,PID_GetPIDOutput(&Rise->pid.Lift_Ang_PID) );
        PID_SetPIDFdb(&Rise ->pid.Lift_Spd_PID, -Motor_Rise_Lift_Motor.encoder.speed);
        PID_CalcPID(&Rise ->pid.Lift_Spd_PID, &Rise ->pid.Lift_Spd_Middle_PIDParam);
        torque4 = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID)-0.0f;

        PID_SetPIDRef(&Rise->pid.Lift_Ang_PID, ang3);
        PID_SetPIDFdb(&Rise->pid.Lift_Ang_PID,Rise->fdb.Lift_pitch_angle); 
        PID_CalcPID(&Rise->pid.Lift_Ang_PID, &Rise->pid.Lift_Ang_Middle_PIDParam);
        PID_SetPIDRef(&Rise ->pid.Lift_Spd_PID,PID_GetPIDOutput(&Rise->pid.Lift_Ang_PID) );
        PID_SetPIDFdb(&Rise ->pid.Lift_Spd_PID, -Motor_Rise_Lift_Motor.encoder.speed);
        PID_CalcPID(&Rise ->pid.Lift_Spd_PID, &Rise ->pid.Lift_Spd_Middle_PIDParam);
        torque5 = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID)-0.0f;
            break;
        default:  
            break;}

  Rise_Set_Torque_Output(torque1,torque2,torque3,torque4,torque5);

 }

     float K_sync = 0.001f; 

     /**
 * @brief 统一的速度控制函数 (包含 PID 计算和交叉耦合)
 * @param hit_speed   击打电机目标速度
 * @param chop_speed  搓球电机(3个)目标速度
 * @param lift_speed  抬升电机目标速度
 */
 
    void Rise_Set_Speed_Output(float hit_speed,float chop_speed,float lift_speed) {
        Rise_DataTypeDef *Rise = Rise_GetRisePtr(); 
        //float torque1, torque2, torque3, torque4;
        LimitMaxMin(hit_speed,500.0f,-500.0f);
        LimitMaxMin(chop_speed,500.0f,-500.0f);
        LimitMaxMin(lift_speed,469.0f,-469.0f);


        switch (Rise->output_state) {

        case Rise_middle:

            PID_SetPIDRef(&Rise ->pid.Hit_Spd_PID,hit_speed);
            PID_SetPIDFdb(&Rise ->pid.Hit_Spd_PID, Rise->fdb.Hit_pitch_speed);
            PID_CalcPID(&Rise ->pid.Hit_Spd_PID, &Rise ->pid.Hit_Spd_Middle_PIDParam);
            torque1 = -PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID);

            //chop的逻辑

            float avg_speed = (Rise->fdb.Chop_front_pitch_speed + Rise->fdb.Chop_right_pitch_speed + Rise->fdb.Chop_left_pitch_speed) / 3.0f;

            float front_sync_err = Rise->fdb.Chop_front_pitch_speed - avg_speed;
            float right_sync_err = Rise->fdb.Chop_right_pitch_speed - avg_speed;
            float left_sync_err  = Rise->fdb.Chop_left_pitch_speed - avg_speed;


            PID_SetPIDRef(&Rise->pid.Chop_Front_Spd_PID, chop_speed);
            // 2. 设置*反馈*速度 (Fdb) 为*电机真实速度*
            PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, Rise->fdb.Chop_front_pitch_speed);
            // 3. 计算PID
            PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
            // 4. 获取力矩
            torque2 = -PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID)- (K_sync * front_sync_err);

            // --- Right Motor (ID: 0x02, Group: 9) ---
            PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, chop_speed);
            PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, Rise->fdb.Chop_right_pitch_speed);
            PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
            torque3 = -PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID)- (K_sync * right_sync_err);

            // --- Left Motor (ID: 0x03, Group: 10) ---
            PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID,chop_speed);
            PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, Rise->fdb.Chop_left_pitch_speed);
            PID_CalcPID(&Rise->pid.Chop_Left_Spd_PID, &Rise->pid.Chop_Left_Spd_Middle_PIDParam);
            torque4 = -PID_GetPIDOutput(&Rise->pid.Chop_Left_Spd_PID)- (K_sync * left_sync_err);

            PID_SetPIDRef(&Rise ->pid.Lift_Spd_PID,lift_speed );
            PID_SetPIDFdb(&Rise ->pid.Lift_Spd_PID, Rise->fdb.Lift_pitch_speed);
            PID_CalcPID(&Rise ->pid.Lift_Spd_PID, &Rise ->pid.Lift_Spd_Middle_PIDParam);
            torque5 = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID);
                break;
            default:  
                break;}

  Rise_Set_Torque_Output(torque1,torque2,torque3,torque4,torque5);

 }

void Rise_Set_ControlMode(uint8_t mode) {
  Rise_DataTypeDef *Rise = Rise_GetRisePtr();
  Rise->ctrl_mode = mode;
}
void Rise_Set_OutputState(uint8_t state) {
  Rise_DataTypeDef *Rise = Rise_GetRisePtr();
  Rise->output_state = state;
}



float t5 = 0;

/**
 * @brief 终极混合控制函数
 * @param hit_angle   击打电机 -> 目标角度 (位置闭环)
 * @param chop_speed  搓球电机 -> 目标速度 (速度闭环 + 交叉耦合)
 * @param lift_speed  抬升电机 -> 目标速度 (速度闭环)
 */
void Rise_Set_Hybrid_Output(float hit_angle, float chop_speed, float lift_speed) 
{
    Rise_DataTypeDef *Rise = Rise_GetRisePtr(); 
    
    // 1. 限幅
    LimitMaxMin(hit_angle, 75.0f, -30.0f); 
    LimitMaxMin(chop_speed, 500.0f, -500.0f);
    LimitMaxMin(lift_speed, 469.0f, -469.0f);

    // 局部变量
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0;

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
        
        t1 = -PID_GetPIDOutput(&Rise->pid.Hit_Spd_PID); // 计算出力矩

        // --- 2. 搓球电机 (Chop): 使用【速度 PID】 + 交叉耦合 ---
        
        // 计算平均速度和误差 (用于同步)
        float avg_speed = (Rise->fdb.Chop_front_pitch_speed + 
                           Rise->fdb.Chop_right_pitch_speed + 
                           Rise->fdb.Chop_left_pitch_speed) / 3.0f;
        float err_f = Rise->fdb.Chop_front_pitch_speed - avg_speed;
        float err_r = Rise->fdb.Chop_right_pitch_speed - avg_speed;
        float err_l = Rise->fdb.Chop_left_pitch_speed  - avg_speed;

        // Front
        PID_SetPIDRef(&Rise->pid.Chop_Front_Spd_PID, chop_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Front_Spd_PID, Rise->fdb.Chop_front_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Front_Spd_PID, &Rise->pid.Chop_Front_Spd_Middle_PIDParam);
        t2 = -PID_GetPIDOutput(&Rise->pid.Chop_Front_Spd_PID) - (Rise_K_Sync * err_f);

        // Right
        PID_SetPIDRef(&Rise->pid.Chop_Right_Spd_PID, chop_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Right_Spd_PID, Rise->fdb.Chop_right_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Right_Spd_PID, &Rise->pid.Chop_Right_Spd_Middle_PIDParam);
        t3 = -PID_GetPIDOutput(&Rise->pid.Chop_Right_Spd_PID) - (Rise_K_Sync * err_r);

        // Left
        PID_SetPIDRef(&Rise->pid.Chop_Left_Spd_PID, chop_speed);
        PID_SetPIDFdb(&Rise->pid.Chop_Left_Spd_PID, Rise->fdb.Chop_left_pitch_speed);
        PID_CalcPID(&Rise->pid.Chop_Left_Spd_PID, &Rise->pid.Chop_Left_Spd_Middle_PIDParam);
        t4 = -PID_GetPIDOutput(&Rise->pid.Chop_Left_Spd_PID) - (Rise_K_Sync * err_l);

        // --- 3. 抬升电机 (Lift): 使用【速度 PID】 ---
        PID_SetPIDRef(&Rise->pid.Lift_Spd_PID, lift_speed);
        PID_SetPIDFdb(&Rise->pid.Lift_Spd_PID, Rise->fdb.Lift_pitch_speed);
        PID_CalcPID(&Rise->pid.Lift_Spd_PID, &Rise->pid.Lift_Spd_Middle_PIDParam);
        t5 = -PID_GetPIDOutput(&Rise->pid.Lift_Spd_PID);
    }
    
    // 统一发送
    Rise_Set_Torque_Output(t1, t2, t3, t4, t5);
}
// 测试触发开关：置 1 开始测试，测试完会自动停在结束状态

void Rise_Test_Hit_Pure(void) {
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
    float current_time = DWT_GetTimeline_s();

    static uint8_t test_state = 0;
    static float start_time = 0.0f;

    switch (test_state) {
        case 0: // [启动]
            start_time = current_time;
            test_state = 1;
            break;

        case 1: // [击打动作]
            // 参数1: Hit目标角度
            // 参数2: Chop速度 -> 设为 0.0f
            // 参数3: Lift速度/力矩 -> 设为 0.0f (电机放松)
            Rise_Set_Angle_Output(Rise_Hit_Target_Angle, 0.0f, 0.0f);

            // 保持 hit_action_time 时间 (例如 0.2秒)
            if (current_time - start_time >= hit_action_time) {
                start_time = current_time; // 重置时间给下一个状态用
                test_state = 2;
            }
            break;

        case 2: // [复位动作]
            // Hit 回到 0 度 (或者 Rise_Hit_Return_Angle)
            // 其他电机依然保持 0
            Rise_Set_Angle_Output(Rise_Hit_Return_Angle, 0.0f, 0.0f);

            // 等待复位完成 (使用你之前定义的 Rise_Hit_Return_Time)
            if (current_time - start_time >= Rise_Hit_Return_Time) {
                test_state = 3;
            }
            break;

        case 3: // [测试结束]
            // 全停
            Rise_Set_Torque_Output(0.0f, 0.0f, 0.0f,0.0f,0.0f);
            
            // 这里可以选择自动把 flag 置 0，这样下次要测得手动再置 1
            // g_test_hit_pure_flag = 0; 
            break;
    }
}

void Rise_Chop_Cal(){
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();     

    float current_hit_target  = Rise->pid.Hit_Spd_PID.ref;  // 读取击打电机的当前目标
    float current_lift_target = Rise->pid.Lift_Spd_PID.ref; // 读取抬升电机的当前目标

    Rise_Set_OutputState(Rise_middle);  
    // Rise_Set_Speed_Output(current_hit_target, Rise_Chop_Target_Speed, current_lift_target);
    Rise_Set_Speed_Output(0.0f, Rise_Chop_Target_Speed, 0.0f);

}


void Rise_Lift_Cal(){
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();     
    // float current_lift_target = Rise->pid.Lift_Spd_PID.ref; // 读取抬升电机的当前目标
    Rise_Set_OutputState(Rise_middle);  
    // Rise_Set_Speed_Output(current_lift_target,Rise_Chop_Target_Speed, Rise_Lift_Target_Speed);
    Rise_Set_Speed_Output(0.0f,0.0f,Rise_Lift_Target_Speed);
    // Rise_Set_Torque_Output(0.0f,0.0f,0.0f,0.0f,torque_test);
}

float current_time;
float current_height;
uint32_t g_case0_entry_count = 0;
static uint8_t g_auto_state = 0; // 初始状态 0: 空闲/启动
float g_auto_start_time = 0.0f;
float  g_auto_start_height = 0.0f;
static uint8_t g_last_auto_mode = Rise_Stop; // 假设默认为 Stop

void Rise_Auto_Cal(){
    Rise_DataTypeDef *Rise = Rise_GetRisePtr(); 
    Rise_Set_OutputState(Rise_middle);
    current_time = DWT_GetTimeline_s();


    switch(g_auto_state){

        case 0: // 启动
            g_case0_entry_count++;  
            g_auto_start_time = current_time;
            g_auto_state = 1;
            g_auto_start_height = Rise->fdb.Lift_pitch_angle; 
            break;

        case 1: // 预旋转 (Pre-spin)
            // Hit: 保持0度 | Chop: 转 | Lift: 0
            Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Target_Speed, 0.0f);

            if (current_time - g_auto_start_time >= pre_spin_time) {
                g_auto_start_time = current_time;
                g_auto_state = 2;
            }
            break;

        case 2: // 抬升 (Lift)
            // Hit: 保持0度 | Chop: 转 | Lift: 转
            Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Target_Speed, Rise_Lift_Target_Speed);

            if(current_time - g_auto_start_time >= lift_time) {
                g_auto_start_time = current_time;
                g_auto_state = 3;
            }
            break;
        
        case 3: // 等待下落 (Drop)
                // 获取当前高度 (假设上方是负数，例如 -7000; 目标是 0)
            current_height = Rise->fdb.Lift_pitch_angle; 
            float return_speed = 0.0f;

            // --- 设定参数 ---
            float slow_down_zone = 500.0f; // 减速区：距离 0 点 500 个单位时开始减速
            float fixed_down_speed = -10.0f; // 匀速下降的速度 (负数代表向下)
            float landing_kp = 0.02f;        // 着陆时的柔和度

            // --- 逻辑判断 ---
            
            if (current_height < g_auto_start_height-slow_down_zone) 
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
                return_speed = (current_height-g_auto_start_height) * landing_kp;
                
                // 增加死区，防止在 0 点抖动
                if (fabs(current_height) < 10.0f) return_speed = 0.0f;
            }

            // 3. 发送指令
            // 注意：这里不需要再用 LimitMaxMin 了，因为我们已经手动控制了速度
            Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, Rise_Chop_Target_Speed, return_speed);

            // 4. 状态转换
            if (current_time - g_auto_start_time >= drop_time) {
                g_auto_start_time = current_time;
                g_auto_state = 4;
            }
         break;
        case 4: // 击打 (Hit)
{
                // 继续计算归位速度，让它死死锁在 0 点，防止松动
                float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height; 
                float hold_speed = pos_error * LIFT_RETURN_KP;
                LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

                // 发送
                Rise_Set_Hybrid_Output(Rise_Hit_Target_Angle, Rise_Chop_Target_Speed, hold_speed);

                if (current_time - g_auto_start_time >= hit_action_time) {
                    g_auto_start_time = current_time;
                    g_auto_state = 5; 
                }
            }
            break;

        case 5: // 复位 (Reset)
            // 动作：Hit 归零 | Chop 停 | Lift 停 (或继续锁零)
            {
                // 保持抬升机构锁在 0 点 (防止复位震动导致托盘掉下来)
                float pos_error = Rise->fdb.Lift_pitch_angle - g_auto_start_height; 
                float hold_speed = pos_error * LIFT_RETURN_KP;
                LimitMaxMin(hold_speed, LIFT_MAX_RETURN_SPEED, -LIFT_MAX_RETURN_SPEED);

                // 发送：Hit 回 0 度
                Rise_Set_Hybrid_Output(Rise_Hit_Return_Angle, 0.0f, hold_speed);
            }

            // ！！！ 关键修改：等待复位完成 ！！！
            // 使用全局变量 Rise_Hit_Return_Time (例如 1.0秒)
            if (current_time - g_auto_start_time >= Rise_Hit_Return_Time) 
            {
                g_auto_state = 6; // 时间到了，才进入“完成态”
            }
            break;
            
        case 6: // 阶段6：完成等待 (DONE)
            // 1. 保持静止 (锁住位置或完全放松，看您需求)
            Rise_Set_Torque_Output(0,0,0,0,0);

            // 2. 没有任何代码让它离开这里
            // 它会一直卡在这里，直到您拨动遥控器开关，触发最上面的 "模式进入重置"
            break;

        default:
            g_auto_state = 0;
            break;
    }
}


void Rise_Control() {
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();

    if (Rise->ctrl_mode != Rise_Auto) {
        g_auto_state = 0; 
    }
    
    switch (Rise->ctrl_mode) {
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
        Rise_Set_Torque_Output(0,0,0,0,0);
        break;
    
		
        default:
            break;
    }

}


void Rise_Output() 
  {
    Rise_DataTypeDef *Rise = Rise_GetRisePtr();
		
//while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
     Motor_SendMotorGroupOutput(Motor_groupHandle[7]) ;
     DWT_Delayus(150);
     Motor_SendMotorGroupOutput(Motor_groupHandle[8]) ;
     DWT_Delayus(150);
     Motor_SendMotorGroupOutput(Motor_groupHandle[9]) ;
     DWT_Delayus(150);
     Motor_SendMotorGroupOutput(Motor_groupHandle[10]) ;
     DWT_Delayus(150);
     Motor_SendMotorGroupOutput(Motor_groupHandle[11]) ;
     

		// last_output_time = DWT_GetTimeline_us();
		 DWT_Delayus(150);
	
  }
