/*  Project      : Polaris
 * 
 *  file         : module_rise.h
 *  Description  : This file contains rise control function
 *  LastEditors  : twx
 *  Date         : 2025-10-31
 *  LastEditTime : 
 */

#ifndef MODULE_RISE_H
#define MODULE_RISE_H


#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_pid.h"

typedef enum
{
	Rise_slow = 1,
	Rise_fast,
	Rise_middle,
	Rise_stop,
}Rise_Output_StateEnum;

typedef enum
{
  Rise_Auto=1,
  Rise_Cuoqiu,
  Rise_Taisheng,
  Rise_Stop,
  Rise_Without_Hit,

}Rise_Ctrl_ModeEnum;

typedef struct
{
  float Hit_pitch_angle,Hit_pitch_speed,Hit_pitch_torque;
  float Chop_front_pitch_angle,Chop_front_pitch_speed ;
  float Chop_right_pitch_angle,Chop_right_pitch_speed;
  float Chop_left_pitch_angle,Chop_left_pitch_speed;
  float Lift_pitch_angle, Lift_pitch_speed,Lift_pitch_torque;


}Rise_FeedbackTypeDef;

typedef struct
{
  float Hit_pitch_torque;
  float Chop_front_pitch_torque;
  float Chop_right_pitch_torque; 
  float Chop_left_pitch_torque;
  float Hit_yaw_torque;
  float Chop_front_yaw_torque;
  float Chop_right_yaw_torque;
  float Chop_left_yaw_torque;
  float Hit_pitch_angle;
  float Chop_front_pitch_angle;
  float Chop_right_pitch_angle; 
  float Chop_left_pitch_angle ;

}Rise_TargetTypeDef;

typedef struct
{
  PID_PIDTypeDef Hit_Ang_PID;
  PID_PIDTypeDef Chop_Front_Ang_PID;
  PID_PIDTypeDef Chop_Right_Ang_PID;
  PID_PIDTypeDef Chop_Left_Ang_PID;
  PID_PIDTypeDef Lift_Ang_PID;

  PID_PIDParamTypeDef Hit_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Hit_Ang_Middle_PIDParam;
  PID_PIDParamTypeDef Hit_Ang_Slow_PIDParam;

  PID_PIDParamTypeDef Chop_Front_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Chop_Front_Ang_Middle_PIDParam;
  PID_PIDParamTypeDef Chop_Front_Ang_Slow_PIDParam;

  PID_PIDParamTypeDef Chop_Right_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Chop_Right_Ang_Middle_PIDParam;
  PID_PIDParamTypeDef Chop_Right_Ang_Slow_PIDParam;

  PID_PIDParamTypeDef Chop_Left_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Chop_Left_Ang_Middle_PIDParam;
  PID_PIDParamTypeDef Chop_Left_Ang_Slow_PIDParam;

  PID_PIDParamTypeDef Lift_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Lift_Ang_Middle_PIDParam;
  PID_PIDParamTypeDef Lift_Ang_Slow_PIDParam;

  PID_PIDTypeDef Hit_Spd_PID;
  PID_PIDTypeDef Chop_Front_Spd_PID;
  PID_PIDTypeDef Chop_Right_Spd_PID;
  PID_PIDTypeDef Chop_Left_Spd_PID;
  PID_PIDTypeDef Lift_Spd_PID;

  PID_PIDParamTypeDef Hit_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Hit_Spd_Middle_PIDParam;
  PID_PIDParamTypeDef Hit_Spd_Slow_PIDParam;

  PID_PIDParamTypeDef Chop_Front_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Chop_Front_Spd_Middle_PIDParam;
  PID_PIDParamTypeDef Chop_Front_Spd_Slow_PIDParam;

  PID_PIDParamTypeDef Chop_Right_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Chop_Right_Spd_Middle_PIDParam;
  PID_PIDParamTypeDef Chop_Right_Spd_Slow_PIDParam; 

  PID_PIDParamTypeDef Chop_Left_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Chop_Left_Spd_Middle_PIDParam;
  PID_PIDParamTypeDef Chop_Left_Spd_Slow_PIDParam;

  PID_PIDParamTypeDef Lift_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Lift_Spd_Middle_PIDParam;
  PID_PIDParamTypeDef Lift_Spd_Slow_PIDParam;

	
}Rise_PIDTypeDef;

typedef struct {
Rise_Output_StateEnum  output_state;
Rise_Ctrl_ModeEnum ctrl_mode;
Rise_FeedbackTypeDef fdb;
Rise_TargetTypeDef tar;
Rise_PIDTypeDef pid;

float update_dt;
uint32_t last_update_tick;
uint8_t error_code;

float lift_zero_offset; // 新增：用于记录上电时的初始角度

} Rise_DataTypeDef;

extern Rise_DataTypeDef Rise_Data;
Rise_DataTypeDef* Rise_GetRisePtr(void);
void Rise_Init();
void Rise_Update_Fdb();
void Rise_Check();
void Rise_Set_Torque_Output(float torque1,float torque2,float torque3,float torque4, float torque5 );
void Rise_Set_Angle_Output(float ang1,float ang2,float ang3,float ang4,float ang5) ;   
void Rise_Set_Speed_Output(float speed1,float speed2,float speed3,float speed4,float speed5);    
void Rise_Set_Hybrid_Output(float hit_angle, float chop_front_speed, float chop_right_speed, float chop_left_speed, float lift_speed) ;   
void Rise_Set_Hybrid_FF_Output(float target_angle,float extra_torque_ff ,float max_speed_limit,float chop_front_speed, float chop_right_speed, float chop_left_speed, float lift_speed) ;                                     
void Rise_Set_ControlMode(uint8_t mode) ;
void Rise_Control(void);
void Rise_Output(void);
void Rise_Chop_Cal();
float Rise_Hit_Control_Variable(float start_angle, float target_angle, float hit_velocity) ;
void Rise_Lift_Cal();
void Rise_Without_Hit_Cal();
#endif

#ifdef __cplusplus
}
#endif
