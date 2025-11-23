/*
 *  Project      :DeltaPlatform
 * 
 *  file         : module_platform.c
 *  Description  : This file contains DeltaPlatform control function
 *  LastEditors  : lkn
 *  Date         : 2024-7-2
 *  LastEditTime : 2024-7-3
 */
 #include "sys_dwt.h"
 #include "module_communicate.h"
 #include "module_platform.h"
 #include "periph_motor.h"
 #include "sys_const.h"
 #include "alg_math.h"
 #include "cmsis_os.h"
 #include "app_remote.h"
 #include "periph_remote.h"
 
Platform_DataTypeDef Platform_Data;
  /**
  * @brief      Get the pointer of Platform control object
  * @param      NULL
  * @retval     Pointer to Platform control object
  */
Platform_DataTypeDef* Platform_GetPlatformPtr() {
    return &Platform_Data;
}

void Platform_Init() {
  Platform_DataTypeDef *Platform = &Platform_Data;  


    Platform->output_state = Platform_slow;
    Platform->ctrl_mode = Platform_Stop;
    Platform->fdb.front_pitch_angle = 0;
    Platform->fdb.left_pitch_angle = 0;
    Platform->fdb.right_pitch_angle = 0;
  
    Platform->fdb.plat_x = 0;
    Platform->fdb.plat_y = 0;
    Platform->fdb.plat_z = 0;
    Platform->fdb.plat_pitch = 0;
    Platform->fdb.plat_yaw = 0;
    Platform->fdb.plat_roll = 0;
    Platform->tar.plat_x = 0;
    Platform->tar.plat_y = 0;
    Platform->tar.plat_z = 0.15;
    Platform->tar.plat_pitch = 0;
    Platform->tar.plat_yaw = 0;
    Platform->tar.plat_roll = 0;

	PID_InitPIDParam(&Platform->pid.Pitch_Ang_Fast_PIDParam,Const_Platform_Pitch_Ang_Param[0][0][0],Const_Platform_Pitch_Ang_Param[0][0][1],Const_Platform_Pitch_Ang_Param[0][0][2],Const_Platform_Pitch_Ang_Param[0][0][3],Const_Platform_Pitch_Ang_Param[0][0][4],
                                                     Const_Platform_Pitch_Ang_Param[0][1][0],Const_Platform_Pitch_Ang_Param[0][1][1],
                                                     Const_Platform_Pitch_Ang_Param[0][2][0],Const_Platform_Pitch_Ang_Param[0][2][1],
                                                     Const_Platform_Pitch_Ang_Param[0][3][0],Const_Platform_Pitch_Ang_Param[0][3][1],PID_POSITION);
  PID_InitPIDParam(&Platform->pid.Pitch_Spd_Fast_PIDParam,Const_Platform_Pitch_Spd_Param[0][0][0],Const_Platform_Pitch_Spd_Param[0][0][1],Const_Platform_Pitch_Spd_Param[0][0][2],Const_Platform_Pitch_Spd_Param[0][0][3],Const_Platform_Pitch_Spd_Param[0][0][4],
                                                     Const_Platform_Pitch_Spd_Param[0][1][0],Const_Platform_Pitch_Spd_Param[0][1][1],
                                                     Const_Platform_Pitch_Spd_Param[0][2][0],Const_Platform_Pitch_Spd_Param[0][2][1],
                                                     Const_Platform_Pitch_Spd_Param[0][3][0],Const_Platform_Pitch_Spd_Param[0][3][1],PID_POSITION);
													
	PID_InitPIDParam(&Platform->pid.Pitch_Ang_Middle_PIDParam,Const_Platform_Pitch_Ang_Param[1][0][0],Const_Platform_Pitch_Ang_Param[1][0][1],Const_Platform_Pitch_Ang_Param[1][0][2],Const_Platform_Pitch_Ang_Param[1][0][3],Const_Platform_Pitch_Ang_Param[1][0][4],
                                                     Const_Platform_Pitch_Ang_Param[1][1][0],Const_Platform_Pitch_Ang_Param[1][1][1],
                                                     Const_Platform_Pitch_Ang_Param[1][2][0],Const_Platform_Pitch_Ang_Param[1][2][1],
                                                     Const_Platform_Pitch_Ang_Param[1][3][0],Const_Platform_Pitch_Ang_Param[1][3][1],PID_POSITION);
  PID_InitPIDParam(&Platform->pid.Pitch_Spd_Middle_PIDParam,Const_Platform_Pitch_Spd_Param[1][0][0],Const_Platform_Pitch_Spd_Param[1][0][1],Const_Platform_Pitch_Spd_Param[1][0][2],Const_Platform_Pitch_Spd_Param[1][0][3],Const_Platform_Pitch_Spd_Param[1][0][4],
                                                     Const_Platform_Pitch_Spd_Param[1][1][0],Const_Platform_Pitch_Spd_Param[1][1][1],
                                                     Const_Platform_Pitch_Spd_Param[1][2][0],Const_Platform_Pitch_Spd_Param[1][2][1],
                                                     Const_Platform_Pitch_Spd_Param[1][3][0],Const_Platform_Pitch_Spd_Param[1][3][1],PID_POSITION);

   PID_InitPIDParam(&Platform->pid.Pitch_Ang_Slow_PIDParam,Const_Platform_Pitch_Ang_Param[2][0][0],Const_Platform_Pitch_Ang_Param[2][0][1],Const_Platform_Pitch_Ang_Param[2][0][2],Const_Platform_Pitch_Ang_Param[2][0][3],Const_Platform_Pitch_Ang_Param[2][0][4],
                                                      Const_Platform_Pitch_Ang_Param[2][1][0],Const_Platform_Pitch_Ang_Param[2][1][1],
                                                      Const_Platform_Pitch_Ang_Param[2][2][0],Const_Platform_Pitch_Ang_Param[2][2][1],
                                                      Const_Platform_Pitch_Ang_Param[2][3][0],Const_Platform_Pitch_Ang_Param[2][3][1],PID_POSITION);
   PID_InitPIDParam(&Platform->pid.Pitch_Spd_Slow_PIDParam,Const_Platform_Pitch_Spd_Param[2][0][0],Const_Platform_Pitch_Spd_Param[2][0][1],Const_Platform_Pitch_Spd_Param[2][0][2],Const_Platform_Pitch_Spd_Param[2][0][3],Const_Platform_Pitch_Spd_Param[2][0][4],
                                                      Const_Platform_Pitch_Spd_Param[2][1][0],Const_Platform_Pitch_Spd_Param[2][1][1],
                                                      Const_Platform_Pitch_Spd_Param[2][2][0],Const_Platform_Pitch_Spd_Param[2][2][1],
                                                      Const_Platform_Pitch_Spd_Param[2][3][0],Const_Platform_Pitch_Spd_Param[2][3][1],PID_POSITION);

    Platform->update_dt = 0;
    Platform->last_update_tick = DWT_GetTimeline_us();
		Platform->error_code = 0;


}
 void Platform_Update_Fdb() {
   Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

 Platform->fdb.front_pitch_angle = 3.5f-Motor_Pitch_Front_Motor.encoder.angle;
 Platform->fdb.left_pitch_angle  = -5.6f-Motor_Pitch_Left_Motor.encoder.angle;
 Platform->fdb.right_pitch_angle = Motor_Pitch_Right_Motor.encoder.angle-54.0f;
	 
	Motor_Pitch_Front_Motor.watchdog += 1; 
	Motor_Pitch_Left_Motor.watchdog += 1; 
	Motor_Pitch_Right_Motor.watchdog += 1; 

 Platform->update_dt = DWT_GetDeltaT(&Platform->last_update_tick);

 }
 uint16_t watchdog2;
void Platform_Check(){ 
   Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	
 if(Motor_Pitch_Front_Motor.watchdog>20)
   {Platform->error_code = 1;}

 if(Motor_Pitch_Left_Motor.watchdog>20)
		    {Platform->error_code = 2;}
				
 if(Motor_Pitch_Right_Motor.watchdog>20)
		    {Platform->error_code = 3;}
				
 if(fabsf(Motor_Pitch_Left_Motor.encoder.torque + Motor_Pitch_Right_Motor.encoder.torque)>20.0f)
    {watchdog2++;}
 else{watchdog2=0;}
  if(watchdog2>20){
  Platform->error_code = 4;}
	
// if(Platform->state==Platform_3degree && Motor_Pitch_Left_Motor.encoder.torque==0  && Motor_Pitch_Left_Motor.output!=0)
// {//Platform->error_code = 5;
// 	}
// if(Platform->state==Platform_3degree && Motor_Pitch_Right_Motor.encoder.torque==0 && Motor_Pitch_Right_Motor.output!=0)
// {//Platform->error_code = 6;
}

 void Platform_Set_Torque_Output(float torque1,float torque2,float torque3) {

 LimitMaxMin(torque1,23.7f,-23.7f);
 LimitMaxMin(torque2,23.7f,-23.7f);
 LimitMaxMin(torque3,23.7f,-23.7f);
	  
  Motor_SetMotorOutput(&Motor_Pitch_Front_Motor,torque1) ;  
	Motor_SetMotorOutput(&Motor_Pitch_Left_Motor ,torque2) ; 
	Motor_SetMotorOutput(&Motor_Pitch_Right_Motor,torque3) ; 
	 
 }

 void Platform_Set_Angle_Output(float ang1,float ang2 ,float ang3) {
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr(); 
	  float torque1,torque2,torque3,torque4,torque5,torque6;
    LimitMaxMin(ang1,75.0f,-30.0f);
    LimitMaxMin(ang2,75.0f,-30.0f);
    LimitMaxMin(ang3,75.0f,-30.0f);

  
switch (Platform->output_state) {
  case Platform_slow:
    PID_SetPIDRef(&Platform->pid.Pitch1_Ang_PID, ang1);
    PID_SetPIDFdb(&Platform->pid.Pitch1_Ang_PID,Platform->fdb.front_pitch_angle); 
    PID_CalcPID(&Platform->pid.Pitch1_Ang_PID, &Platform->pid.Pitch_Ang_Slow_PIDParam);
    PID_SetPIDRef(&Platform ->pid.Pitch1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch1_Ang_PID) );
    PID_SetPIDFdb(&Platform ->pid.Pitch1_Spd_PID, -Motor_Pitch_Front_Motor.encoder.speed);
    PID_CalcPID(&Platform ->pid.Pitch1_Spd_PID, &Platform ->pid.Pitch_Spd_Slow_PIDParam);
    torque1 = -PID_GetPIDOutput(&Platform->pid.Pitch1_Spd_PID)-1.5f;
  
    PID_SetPIDRef(&Platform->pid.Pitch2_Ang_PID, ang2);
    PID_SetPIDFdb(&Platform->pid.Pitch2_Ang_PID,Platform->fdb.left_pitch_angle);
    PID_CalcPID(&Platform->pid.Pitch2_Ang_PID, &Platform->pid.Pitch_Ang_Slow_PIDParam);
    PID_SetPIDRef(&Platform ->pid.Pitch2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch2_Ang_PID) );
    PID_SetPIDFdb(&Platform ->pid.Pitch2_Spd_PID, -Motor_Pitch_Left_Motor.encoder.speed);
    PID_CalcPID(&Platform ->pid.Pitch2_Spd_PID, &Platform ->pid.Pitch_Spd_Slow_PIDParam);
    torque2 = -PID_GetPIDOutput(&Platform->pid.Pitch2_Spd_PID)-1.5f;
  
    PID_SetPIDRef(&Platform->pid.Pitch3_Ang_PID, ang3);
    PID_SetPIDFdb(&Platform->pid.Pitch3_Ang_PID,Platform->fdb.right_pitch_angle);
    PID_CalcPID(&Platform->pid.Pitch3_Ang_PID, &Platform->pid.Pitch_Ang_Slow_PIDParam);
    PID_SetPIDRef(&Platform ->pid.Pitch3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch3_Ang_PID) );
    PID_SetPIDFdb(&Platform ->pid.Pitch3_Spd_PID, Motor_Pitch_Right_Motor.encoder.speed);
    PID_CalcPID(&Platform ->pid.Pitch3_Spd_PID, &Platform ->pid.Pitch_Spd_Slow_PIDParam);
    torque3 = PID_GetPIDOutput(&Platform->pid.Pitch3_Spd_PID)+1.5f;

        break;

  case Platform_middle:
      PID_SetPIDRef(&Platform->pid.Pitch1_Ang_PID, ang1);
      PID_SetPIDFdb(&Platform->pid.Pitch1_Ang_PID,Platform->fdb.front_pitch_angle); 
      PID_CalcPID(&Platform->pid.Pitch1_Ang_PID, &Platform->pid.Pitch_Ang_Middle_PIDParam);
      PID_SetPIDRef(&Platform ->pid.Pitch1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch1_Ang_PID) );
      PID_SetPIDFdb(&Platform ->pid.Pitch1_Spd_PID, -Motor_Pitch_Front_Motor.encoder.speed);
      PID_CalcPID(&Platform ->pid.Pitch1_Spd_PID, &Platform ->pid.Pitch_Spd_Middle_PIDParam);
      torque1 = -PID_GetPIDOutput(&Platform->pid.Pitch1_Spd_PID)-1.5f;
    
      PID_SetPIDRef(&Platform->pid.Pitch2_Ang_PID, ang2);
      PID_SetPIDFdb(&Platform->pid.Pitch2_Ang_PID,Platform->fdb.left_pitch_angle);
      PID_CalcPID(&Platform->pid.Pitch2_Ang_PID, &Platform->pid.Pitch_Ang_Middle_PIDParam);
      PID_SetPIDRef(&Platform ->pid.Pitch2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch2_Ang_PID) );
      PID_SetPIDFdb(&Platform ->pid.Pitch2_Spd_PID, -Motor_Pitch_Left_Motor.encoder.speed);
      PID_CalcPID(&Platform ->pid.Pitch2_Spd_PID, &Platform ->pid.Pitch_Spd_Middle_PIDParam);
      torque2 = -PID_GetPIDOutput(&Platform->pid.Pitch2_Spd_PID)-1.5f;
    
      PID_SetPIDRef(&Platform->pid.Pitch3_Ang_PID, ang3);
      PID_SetPIDFdb(&Platform->pid.Pitch3_Ang_PID,Platform->fdb.right_pitch_angle);
      PID_CalcPID(&Platform->pid.Pitch3_Ang_PID, &Platform->pid.Pitch_Ang_Middle_PIDParam);
      PID_SetPIDRef(&Platform ->pid.Pitch3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch3_Ang_PID) );
      PID_SetPIDFdb(&Platform ->pid.Pitch3_Spd_PID, Motor_Pitch_Right_Motor.encoder.speed);
      PID_CalcPID(&Platform ->pid.Pitch3_Spd_PID, &Platform ->pid.Pitch_Spd_Middle_PIDParam);
      torque3 = PID_GetPIDOutput(&Platform->pid.Pitch3_Spd_PID)+1.5f;

  
  case Platform_fast:
      PID_SetPIDRef(&Platform->pid.Pitch1_Ang_PID, ang1);
      PID_SetPIDFdb(&Platform->pid.Pitch1_Ang_PID,Platform->fdb.front_pitch_angle); 
      PID_CalcPID(&Platform->pid.Pitch1_Ang_PID, &Platform->pid.Pitch_Ang_Fast_PIDParam);
      PID_SetPIDRef(&Platform ->pid.Pitch1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch1_Ang_PID) );
      PID_SetPIDFdb(&Platform ->pid.Pitch1_Spd_PID, -Motor_Pitch_Front_Motor.encoder.speed);
      PID_CalcPID(&Platform ->pid.Pitch1_Spd_PID, &Platform ->pid.Pitch_Spd_Fast_PIDParam);
      torque1 = -PID_GetPIDOutput(&Platform->pid.Pitch1_Spd_PID)-1.5f;
    
      PID_SetPIDRef(&Platform->pid.Pitch2_Ang_PID, ang2);
      PID_SetPIDFdb(&Platform->pid.Pitch2_Ang_PID,Platform->fdb.left_pitch_angle);
      PID_CalcPID(&Platform->pid.Pitch2_Ang_PID, &Platform->pid.Pitch_Ang_Fast_PIDParam);
      PID_SetPIDRef(&Platform ->pid.Pitch2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch2_Ang_PID) );
      PID_SetPIDFdb(&Platform ->pid.Pitch2_Spd_PID, -Motor_Pitch_Left_Motor.encoder.speed);
      PID_CalcPID(&Platform ->pid.Pitch2_Spd_PID, &Platform ->pid.Pitch_Spd_Fast_PIDParam);
      torque2 = -PID_GetPIDOutput(&Platform->pid.Pitch2_Spd_PID)-1.5f;
    
      PID_SetPIDRef(&Platform->pid.Pitch3_Ang_PID, ang3);
      PID_SetPIDFdb(&Platform->pid.Pitch3_Ang_PID,Platform->fdb.right_pitch_angle);
      PID_CalcPID(&Platform->pid.Pitch3_Ang_PID, &Platform->pid.Pitch_Ang_Fast_PIDParam);
      PID_SetPIDRef(&Platform ->pid.Pitch3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch3_Ang_PID) );
      PID_SetPIDFdb(&Platform ->pid.Pitch3_Spd_PID, Motor_Pitch_Right_Motor.encoder.speed);
      PID_CalcPID(&Platform ->pid.Pitch3_Spd_PID, &Platform ->pid.Pitch_Spd_Fast_PIDParam);
      torque3 = PID_GetPIDOutput(&Platform->pid.Pitch3_Spd_PID)+1.5f;

    
        break;
      default:  
        break;}

  Platform_Set_Torque_Output(torque1,torque2,torque3);

 }
 void Platform_Set_Target_Pos(float x,float y,float z,float pitch,float yaw,float roll) {
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
 
    Platform->tar.plat_x = x;
    Platform->tar.plat_y = y;
    Platform->tar.plat_z = z;
    Platform->tar.plat_pitch = pitch;
    Platform->tar.plat_yaw = yaw;
    Platform->tar.plat_roll = roll;

    LimitMaxMin(Platform->tar.plat_x,0.1f,-0.1f);
    LimitMaxMin(Platform->tar.plat_y,0.1f,-0.1f);
    LimitMaxMin(Platform->tar.plat_z,0.37f,0.10f);
    LimitMaxMin(Platform->tar.plat_pitch,10,-30);
    LimitMaxMin(Platform->tar.plat_yaw,30,-30);
    LimitMaxMin(Platform->tar.plat_roll,10,-10);

 }
 void Platform_Cal_3Degree_IK_Output() {
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

  float z = Platform->tar.plat_z;
  float pi = Platform->tar.plat_pitch;
  float ro = Platform->tar.plat_roll;
  float L = Const_Platform_Big_Arm_Length;
  float l = Const_Platform_Small_Arm_Length;
  float R = Const_Platform_Static_Plat_Radius;
  float r = Const_Platform_Dynamic_Plat_Radius;
  float theta1 ,theta2,theta3;
  float sin_ro, cos_ro, sin_pi, cos_pi; 
	 
  arm_sin_cos_f32(ro, &sin_ro, &cos_ro); 
  arm_sin_cos_f32(pi, &sin_pi, &cos_pi); 

  theta1 = - acosf(((z * z * sin_ro * sin_ro + powf(r *cos_pi - R + z *cos_ro *sin_pi, 2) + L * L - l * l + powf(r *sin_pi - z *cos_pi *cos_ro, 2)) / (2 * L * sqrtf(z * z * sin_ro * sin_ro + powf(r *cos_pi - R + z *cos_ro *sin_pi, 2) + powf(r *sin_pi - z *cos_pi *cos_ro, 2))))) 
           - atanf((r *sin_pi - z *cos_pi *cos_ro) / sqrtf(z * z * sin_ro * sin_ro + powf(r *cos_pi - R + z *cos_ro *sin_pi, 2)));

  theta2 = atanf(((r *sin_pi) / 2 + z *cos_pi *cos_ro + (sqrtf(3) * r *cos_pi * sin_ro) / 2) / sqrtf(powf((sqrtf(3) * R) / 2 + z * sin_ro - (sqrtf(3) * r *cos_ro) / 2, 2) + powf((R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi + (sqrtf(3) * r *sin_pi * sin_ro) / 2), 2))) 
           - acosf((powf((sqrtf(3) * R) / 2 + z * sin_ro - (sqrtf(3) * r *cos_ro) / 2, 2) + powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi +(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro +(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2) +L * L - l * l) /(2 * L * sqrtf(powf((sqrtf(3) * R) / 2 + z * sin_ro -(sqrtf(3) * r *cos_ro) / 2, 2) +powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi +(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro +(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2))));

  theta3 = atanf(((r *sin_pi) / 2 + z *cos_pi *cos_ro - (sqrtf(3) * r *cos_pi * sin_ro) / 2) /sqrtf(powf(z * sin_ro - (sqrtf(3) * R) / 2 + (sqrtf(3) * r *cos_ro) / 2, 2) +      powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi - (sqrtf(3) * r *sin_pi * sin_ro) / 2, 2))) 
           - acosf((powf(z * sin_ro - (sqrtf(3) * R) / 2 +(sqrtf(3) * r *cos_ro) / 2, 2) + powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi -(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro -(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2) +L * L - l * l) /(2 * L * sqrtf(powf(z * sin_ro - (sqrtf(3) * R) / 2 +(sqrtf(3) * r *cos_ro) / 2, 2) +powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi -(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro -(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2))));

  Platform_Set_Angle_Output(rad2deg(theta1),rad2deg(theta2),rad2deg(theta3));
 }
 
  float start_hit_time = 0.0f;
  float start_back_time = 0.0f;
  uint8_t finishhit = 1;
  float l_tar = 0.13f;
void  Platform_Jiefa_Cal(float hit)
{
   Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	if (finishhit == 0 ){
		 if(DWT_GetTimeline_s() - start_hit_time  <= 1.0f) {
			 l_tar +=0.15;
			 LimitMax (l_tar,0.3);
		   Platform_Set_Target_Pos(0,0,l_tar,0,0,0);
       Platform_Set_OutputState(Platform_fast);

		 }else {
			Platform_Set_OutputState(Platform_slow);	
			 Platform_Set_Target_Pos(0,0,0.15f,0,0,0);
		   start_back_time = DWT_GetTimeline_s();	
		  finishhit = 1;	
		 }
  //解释：从 start_hit_time 开始 1 秒内，平台沿 Z 轴上升 (l_tar += 0.15)，做“击发动作”。
  //超过 1 秒后，平台返回原始高度 0.15 m，同时切换慢速状态，并记录返回时间，标记动作完成（finishhit = 1）。
  }	
	else if(finishhit == 1){
		if(DWT_GetTimeline_s() - start_back_time <= 1.5f){
		 Platform_Set_OutputState(Platform_slow);
		}
		else {
		   if((int)hit == 1) {
			   Platform_Set_Target_Pos(0,0,0.15f,0,0,0);
		     Platform_Set_OutputState(Platform_middle);

         start_hit_time = DWT_GetTimeline_s();
         finishhit = 0;
			 }else if((int)hit != 1) {
				 Platform_Set_Target_Pos(0,0,0.15f,0,0,0);
			   Platform_Set_OutputState(Platform_middle);

			 }
		 }
	}	
};
void  Platform_Dianqiu_Cal(float hit, uint8_t distance)
{
   Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	if (finishhit == 0 ){
		 if(DWT_GetTimeline_s() - start_hit_time  <= 0.8f) {
						switch (distance){
									 case Remote_SWITCH_UP://jin
									 l_tar +=0.05;
									 LimitMax (l_tar,0.3f);
									 Platform_Set_Target_Pos(0,0,l_tar,-10.0f,0,0);
									 Platform_Set_OutputState(Platform_middle); 
										break ;
									 case Remote_SWITCH_MIDDLE:
									 l_tar +=0.05;
									 LimitMax (l_tar,0.3f);
									 Platform_Set_Target_Pos(0,0,l_tar,0,0,0);
									 Platform_Set_OutputState(Platform_middle); 
										break ;
									 case Remote_SWITCH_DOWN://yuan
									 l_tar +=0.05;
									 LimitMax (l_tar,0.3f);
									 Platform_Set_Target_Pos(0,0,l_tar,0,0,0);
									 Platform_Set_OutputState(Platform_fast); 
										break ;
								}
			
		 }else {
			Platform_Set_OutputState(Platform_slow);	
			Platform_Set_Target_Pos(0,0,0.15f,0,0,0);
		   start_back_time = DWT_GetTimeline_s();	
		  finishhit = 1;	
		 }
  }	
	else if(finishhit == 1){
		if(DWT_GetTimeline_s() - start_back_time <= 1.0f){
		 Platform_Set_OutputState(Platform_slow);
		}
		else {
		   if((int)hit == 1) {
			   Platform_Set_Target_Pos(0,0,0.15f,0,0,0);
		     Platform_Set_OutputState(Platform_middle);
         start_hit_time = DWT_GetTimeline_s();
         finishhit = 0;
			 }else if((int)hit != 1) {
				 Platform_Set_Target_Pos(0,0,0.15f,0,0,0);
			   Platform_Set_OutputState(Platform_middle);

			 }
		 }
	}	
};
void Platform_Set_ControlMode(uint8_t mode) {
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  Platform->ctrl_mode = mode;
}
void Platform_Set_OutputState(uint8_t state) {
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  Platform->output_state = state;
}
void Platform_Control() {
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    switch (Platform->ctrl_mode) {
		case Platform_Test:
	    Platform_Set_OutputState(Platform_slow);  
      Platform_Cal_3Degree_IK_Output(); 
				    break;			
        case Platform_Dianqiu:
					
  
            break;				
        case Platform_Jiefa:
	
          Platform_Cal_3Degree_IK_Output(); 
            break;				
        case Platform_Chuanqiu:
          Platform_Set_OutputState(Platform_fast); 
          Platform_Cal_3Degree_IK_Output(); 
          break;				
        case Platform_Initpose:
          start_hit_time = 0.0f;
          start_back_time = 0.0f;
          finishhit = 1;
          l_tar = 0.15f;	
          Platform_Set_OutputState(Platform_slow);  
          Platform_Set_Target_Pos(0,0,0.15f,0,0,0);  
          Platform_Cal_3Degree_IK_Output(); 
          break;
        case Platform_Stop:
          Platform_Set_OutputState(Platform_stop);
          Platform_Set_Torque_Output(0,0,0);
          break;				
        default:
          break;
    }
}
 uint8_t output_state;
 uint64_t last_output_time;

void Platform_Output() 
  {
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr(); 
		
//while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
     Motor_SendMotorGroupOutput(Motor_groupHandle[1]) ;
		// last_output_time = DWT_GetTimeline_us();
		 DWT_Delayus(150);
	
//while( (output_state !=2) && (DWT_GetTimeline_us()-last_output_time < 150)){}
		 Motor_SendMotorGroupOutput(Motor_groupHandle[0]) ;
	  // last_output_time = DWT_GetTimeline_us();
		 DWT_Delayus(150);

//while( (output_state !=3) && (DWT_GetTimeline_us()-last_output_time < 150)){}
 		 Motor_SendMotorGroupOutput(Motor_groupHandle[2]) ;
		// last_output_time = DWT_GetTimeline_us();
	
  }

/////////////////////////////////////////////////////////////////////////////////////////
