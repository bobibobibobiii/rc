/*  Project      : Polaris
 * 
 *  file         : module_platform.h
 *  Description  : This file contains Platform control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-10-04 17:53:57
 */

#ifndef MODULE_PLATFORM_H
#define MODULE_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_pid.h"

typedef enum
{
	Platform_slow = 1,
	Platform_fast,
	Platform_middle,
	Platform_stop
}Platform_Output_StateEnum;

typedef enum
{
	Platform_Jiefa = 1,
	Platform_Chuanqiu =2,
	Platform_Dianqiu =3,
  Platform_Initpose =4,	
	Platform_Test =5,
  Platform_Stop =6,
}Platform_Ctrl_ModeEnum;

typedef struct
{
  float front_pitch_angle, front_yaw_angle, left_pitch_angle,left_yaw_angle, right_pitch_angle,right_yaw_angle;

  float plat_x;
  float plat_y;
  float plat_z;
  float plat_pitch;
  float plat_yaw;
  float plat_roll;
}Platform_FeedbackTypeDef;

typedef struct
{
  float plat_x,plat_y,plat_z;
  float plat_pitch,plat_yaw,plat_roll;
  float front_pitch_torque,left_pitch_torque,right_pitch_torque;
  float front_yaw_torque,left_yaw_torque,right_yaw_torque;

}Platform_TargetTypeDef;

typedef struct
{
  PID_PIDTypeDef Pitch1_Ang_PID;
  PID_PIDTypeDef Pitch2_Ang_PID;
  PID_PIDTypeDef Pitch3_Ang_PID;
  PID_PIDParamTypeDef Pitch_Ang_Fast_PIDParam;
	PID_PIDParamTypeDef Pitch_Ang_Middle_PIDParam;
	PID_PIDParamTypeDef Pitch_Ang_Slow_PIDParam;
	
  PID_PIDTypeDef Pitch1_Spd_PID;
  PID_PIDTypeDef Pitch2_Spd_PID;
  PID_PIDTypeDef Pitch3_Spd_PID;
  PID_PIDParamTypeDef Pitch_Spd_Fast_PIDParam;
	PID_PIDParamTypeDef Pitch_Spd_Middle_PIDParam;
	PID_PIDParamTypeDef Pitch_Spd_Slow_PIDParam;

  PID_PIDTypeDef Yaw1_Ang_PID;
  PID_PIDTypeDef Yaw2_Ang_PID;
  PID_PIDTypeDef Yaw3_Ang_PID;
  PID_PIDParamTypeDef Yaw_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Yaw_Ang_Middle_PIDParam;
	PID_PIDParamTypeDef Yaw_Ang_Slow_PIDParam;
	
  PID_PIDTypeDef Yaw1_Spd_PID;
  PID_PIDTypeDef Yaw2_Spd_PID;
  PID_PIDTypeDef Yaw3_Spd_PID;
  PID_PIDParamTypeDef Yaw_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Yaw_Spd_Middle_PIDParam;
	PID_PIDParamTypeDef Yaw_Spd_Slow_PIDParam;
	
}Platform_PIDTypeDef;

typedef struct {
Platform_Output_StateEnum  output_state;
Platform_Ctrl_ModeEnum ctrl_mode;
Platform_FeedbackTypeDef fdb;
Platform_TargetTypeDef tar;
Platform_PIDTypeDef pid;

float update_dt;
uint32_t last_update_tick;
uint8_t error_code;

} Platform_DataTypeDef;

extern Platform_DataTypeDef Platform_Data;
Platform_DataTypeDef* Platform_GetPlatformPtr(void);
void Platform_Init();
void Platform_Update_Fdb();
void Platform_Set_Angle_Output(float ang1,float ang2 ,float ang3 );
void Platform_Set_ControlMode(uint8_t mode) ;
void Platform_Set_OutputState(uint8_t state) ;
void Platform_Set_Target_Pos(float x,float y,float z,float pitch,float yaw,float roll) ;
void Platform_Control(void);
void Platform_Output(void);
void Platform_Cal_3Degree_IK_Output() ;
void Platform_Set_Torque_Output(float torque1,float torque2,float torque3);
void Platform_Dianqiu_Cal(float hit,uint8_t distance);
void Platform_Jiefa_Cal(float hit);
void Platform_Check();
#endif

#ifdef __cplusplus
}
#endif
