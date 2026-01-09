#include "sys_const.h"

const float root3 = 1.73205;
const float Initial_Z_Position = 0.12f; // 初始高度位置

const  float Const_DMmotor_P_MIN  = -12.5  ;
const  float Const_DMmotor_P_MAX   =12.5   ;
const  float Const_DMmotor_V_MIN   =-45    ;
const  float Const_DMmotor_V_MAX  = 45     ;
const  float Const_DMmotor_KP_MIN = 0     ; 
const  float Const_DMmotor_KP_MAX  =500    ;
const  float Const_DMmotor_KD_MIN  =0     ; 
const  float Const_DMmotor_KD_MAX  =5     ; 
const  float Const_DMmotor_T_MIN  = -30   ; 
const  float Const_DMmotor_T_MAX   =30  ; 


const float Const_Chassis_Location_Distance_Param[4][5] = 
{{0.04f, 0.0f, 0.0f, 1, 3}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_Chassis_Location_Anglew_Param[4][5] = 
{{0.06f, 0.0f, 0.0f, 1, 3}, {0.1f, -1}, {0, 0}, {-1, -1}};
			
const float Const_Platform_Pitch_Ang_Param[3][4][5] = {    
	{{1.2f, 0.01f, 15.0f, 120.0f, 40.0f},{0.1f, -1}, {0, 0}, {-1, -1}},                   //0fast 1middle 2slow
	{{0.9f, 0.01f, 10.0f, 120.0f, 15.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{0.8f, 0.01f, 8.0f , 8.0f , 10.0f },{0.1f, -1}, {0, 0}, {-1, -1}}
};

const float Const_Platform_Pitch_Spd_Param[3][4][5] = {
	{{0.65f, 0.01f, 0.0f, 18.0f, 24.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{0.8f, 0.01f, 0.0f, 100.0f, 24.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{0.6f , 0.01f, 0.0f, 15.0f, 20.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
};

const float Const_Platform_Yaw_Ang_Param[3][4][5] = {
	{{2.0f, 0.03f, 0.5f, 50.0f, 70.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
    {{2.0f, 0.03f, 0.5f, 50.0f, 70.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{1.8f, 0.03f, 0.5f, 50.0f, 70.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
 };
 
 const float Const_Platform_Yaw_Spd_Param[3][4][5] = {
	{{1.3f, 0.003f, 0.0f, 20.0f, 27.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
    {{1.3f, 0.003f, 0.0f, 20.0f, 27.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{1.1f, 0.002f, 0.0f, 20.0f, 27.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
 };

const float Const_Chassis_Steer_Ang_Param[4][5] = {
	{0.4f, 0.005f, 0.0f, 0.0f, 20.0f}, 
	{0.1f, -1.0f}, {0.0f, 0.0f}, {-1.0f, -1.0f}
};

const float Const_Chassis_Steer_Spd_Param[4][5] = {
	{0.15f, 0.01f, 0.0f, 2.0f, 2.0f}, 
	{0.1f, -1.0f}, {0.0f, 0.0f}, {-1.0f, -1.0f}
};

const float Const_Chassis_Wheel_Spd_Param[4][5] = {
	{0.5f, 0.0f, 0.0f, 0.0f, 3.5f}, 
	{0.15f, -1.0f}, {0.0f, 0.0f}, {-1.0f, -1.0f}
};


// PID Parameters for Serve
const float Const_LeftPosMotorParam[4][5] = {
  {0.25f, 0.0f, 0.0f, 0.0f, 22.0f},
  {0.1f, -1}, {0, 0}, {-1, -1}
};

const float Const_LeftSpdMotorParam[4][5] = {
  {1.5f, 0.0f, 0.0f, 0.0f, 3.5f},
  {0.1f, -1}, {0, 0}, {-1, -1}
};

const float Const_RightPosMotorParam[4][5] = {
  {0.25f, 0.0f, 0.0f, 0.0f, 22.0f},
  {0.1f, -1}, {0, 0}, {-1, -1}
};

const float Const_RightSpdMotorParam[4][5] = {
  {1.5f, 0.0f, 0.0f, 0.0f, 3.5f},
  {0.1f, -1}, {0, 0}, {-1, -1}
};


// PID Parameters for Gimbal
const float Const_YawPosMotorParam[4][5] = {
	{1.3f, 0.1f, 0.0f, 5.0f, 17.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

const float Const_YawSpdMotorParam[4][5] = {
	{0.06f, 0.005f, 0.0f, 0.5f, 1.3f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

const float Const_PitchPosMotorParam[4][5] = {
	{0.7f, 0.0f, 2.0f, 150.0f, 17.0f},//0.8 0.01 0.5 5 17
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

const float Const_PitchSpdMotorParam[4][5] = {
	{0.07f, 0.01f, 0.0f, 2.0f, 1.3f}, //0.07
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

//@twx PID Parameters for Rise

float Const_HitPosMotorParam[2][4][5] = {
    //   Kp      Ki      Kd      I_Limit  Out_Limit
    {{0.3f,  0.001f,  0.0f,   20.0f,   27.0f}, {0.1f, -1}, {0, 0}, {-1, -1}},
	{{0.15f,  0.001f,  1.5f,   20.0f,   27.0f}, {0.1f, -1}, {0, 0}, {-1, -1}}
};


float Const_HitSpdMotorParam[2][4][5] = {
    //   Kp      Ki      Kd      I_Limit  Out_Limit
    {{2.65f,  0.01f,  0.65f,   20.0f,   27.0f}, {0.1f, -1}, {0, 0}, {-1, -1}},
	{{2.0f,  0.001f,  0.0f,   20.0f,   27.0f}, {0.1f, -1}, {0, 0}, {-1, -1}}  
};

float Const_ChopPosMotorParam[4][5] = {
	{0.9f, 0.01f, 10.0f, 280.0f, 24.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

float Const_ChopFrontSpdMotorParam[4][5] = {
	{0.1f, 0.001f, 0.0f, 280.0f, 24.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

float Const_ChopRightSpdMotorParam[4][5] = {
	{0.1f, 0.001f, 0.0f, 280.0f, 24.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

float Const_ChopLeftSpdMotorParam[4][5] = {
	{0.1f, 0.001f, 0.0f, 280.0f, 24.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

float Const_LiftPosMotorParam[4][5] = {
	{0.1f, 0.01f, 0.0f, 0.0f, 15.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};

float Const_LiftSpdMotorParam[4][5] = {
	{0.5f, 0.0f, 0.0f, 2.0f, 10.0f},
	{0.1f, -1}, {0, 0}, {-1, -1}	
};



const float Const_Platform_Big_Arm_Length = 0.2145 ;
const float Const_Platform_Small_Arm_Length = 0.193 ;
const float Const_Platform_Static_Plat_Radius = 0.065 ;
const float Const_Platform_Dynamic_Plat_Radius = 0.2 ;
const float Const_Chassis_Steeringwheel_Radius = 0.333 ;

//��Ϊ��ʱ��
const float Const_DIRECTION_RIGHT_FRONT_MOTOR_INIT_OFFSET =  -116.0f;
const float Const_DIRECTION_LEFT_FRONT_MOTOR_INIT_OFFSET  =   67.0f ;
const float Const_DIRECTION_LEFT_BACK_MOTOR_INIT_OFFSET   =  -54.0f;
const float Const_DIRECTION_RIGHT_BACK_MOTOR_INIT_OFFSET  =  6.0f;

const float Const_DIRECTION_RIGHT_FRONT_MOTOR_INIT_OFFSET_TEST = 64.0f;
const float Const_DIRECTION_LEFT_FRONT_MOTOR_INIT_OFFSET_TEST = 245.0f;
const float Const_DIRECTION_LEFT_BACK_MOTOR_INIT_OFFSET_TEST = 120.0f;
const float Const_DIRECTION_RIGHT_BACK_MOTOR_INIT_OFFSET_TEST = 100.f;

const float Curr2ForceConstant_3508 = 0.3;
const float Roll2SpeedConstant_3508 = 24.48;
const float ReductionRate = 3591 / 187;
const float CtrlCurrent_3508 = 16384;
const float FactCurrent_3508 = 20;
const float Fact2Ctrl_3508 = 16384 / 20;

const float Serve_FeedforwardMoment = 0.0f;

UART_HandleTypeDef* Const_Remote_UART_HANDLER       = &huart1;
UART_HandleTypeDef* Const_Communicate_UART_HANDLER  = &huart7;
UART_HandleTypeDef* Const_Motor_UART_HANDLER        = &huart6;
UART_HandleTypeDef* Const_DT35_UART_HANDLER         = &huart8;
const float Const_Remote_REMOTE_OFFLINE_TIME        = 1.0f;
const float Const_Comm_OFFLINE_TIME                 = 0.5f;

const float Const_Gimbal_Yaw_OFFSET						= -30.0f;
const float Const_Gimbal_Pitch_OFFSET					= 180.0f;
