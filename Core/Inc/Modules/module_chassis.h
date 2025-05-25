#ifndef MODULE_CHASSIS_H
#define MODULE_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_pid.h"

typedef enum
{
	Chassis_Run  = 1,
	Chassis_Lock = 2,
	Chassis_Stop = 3,
}Chassis_StateEnum;

typedef enum
{
	Chassis_Remote = 1,
	Chassis_PC  = 2,
	Chassis_Location = 3,
}Chassis_CtrlModeEnum;

typedef struct
{
	PID_PIDTypeDef Location_Distance_PID;
	PID_PIDTypeDef Location_Anglew_PID;
		
	PID_PIDParamTypeDef  Location_Distance_PIDParam;
	PID_PIDParamTypeDef  Location_Anglew_PIDParam;
	
	PID_PIDTypeDef SteeringWheel_Direction_AnglePID[4];
	PID_PIDParamTypeDef SteeringWheel_Direction_AngleParamPID[4];

	PID_PIDTypeDef SteeringWheel_Direction_SpeedPID[4];
	PID_PIDParamTypeDef SteeringWheel_Direction_SpeedParamPID[4];
	
	PID_PIDTypeDef SteeringWheel_Move_PID[4];
	PID_PIDParamTypeDef SteeringWheel_Move_ParamPID[4];
	
}Chassis_PIDTypeDef;

typedef struct {

	Chassis_StateEnum  Chassis_State;
	Chassis_CtrlModeEnum Chassis_CtrlMode;

	Chassis_PIDTypeDef pid;
	float vx;
	float vy;
	float vw;
	float theta_ref ;
	float theta_now ; 
	float theta_last ;
	float v_move_ref[5];
	float v_direction_ref[5];
	float a_direction_ref[5];                                 
	float update_dt ;
	uint32_t last_update_tick ;
	uint8_t zeropoint_direction;
		
	Motor_MotorGroupTypeDef* DirectionMotors;
	Motor_MotorGroupTypeDef* MoveMotors;
	
	CAN_TxHeaderTypeDef can_header;
	CAN_HandleTypeDef* can_handle;
} Chassis_DataTypeDef;

Chassis_DataTypeDef* Chassis_GetChassisPtr(void);
void Chassis_Control();
void Chassis_CalEachMotorRef();
void Chassis_SteeringWheel_ControlMove();

void Chassis_SteeringWheel_ControlMove_Test();

void Chassis_Trans_ControlData();
void Chassis_Init();
uint16_t float_to_2bytes(float value);
void Chassis_SetControlMode(uint8_t mode);
void Chassis_Set_Speed(uint8_t state,float x,float y,float w,float r);
#endif

#ifdef __cplusplus
}
#endif
