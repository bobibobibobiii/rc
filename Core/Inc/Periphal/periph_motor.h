/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.h
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:44:03
 *  LastEditTime : 2023-10-03 23:45:32
 */
#ifndef MOTOR_PERIPH_H
#define MOTOR_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_can.h"
#include "alg_math.h"
#include "alg_filter.h"

#define MOTOR_GROUP_NUM 12
#define Const_Motor_MOTOR_TX_EXTID  0x01
#define Const_Motor_MOTOR_TX_DLC    8
#define Const_Motor_MOTOR_RX_DLC    8


typedef enum
{
  Motor_Enable = 1,
  Motor_Disable, 
  Motor_SaveInitpos,
  Motor_Clearerr 
} Motor_DMBasicCtrlEnum;

typedef enum {      
    Motor_TYPE_NOT_CONNECTED    = 0,
    Motor_TYPE_Dji3508_origin   = 1,
	Motor_TYPE_Dji3508_xroll    = 2,
	Motor_TYPE_Dji6020          = 3,
    Motor_TYPE_DM4310           = 4,
	Motor_TYPE_DM4340           = 5,
	Motor_TYPE_DM8009           = 6,
	Motor_TYPE_A1               = 7,
	Motor_TYPE_LK5005		    = 8,
    Motor_TYPE_Go1              = 9,
    Motor_TYPE_DMH3510          = 10
} Motor_MotorTypeEnum;

typedef enum {
    Motor_NO_CONNECTED  = 0,
    Motor_Pitch         = 1,
    Motor_Yaw           = 2,
    Motor_DriveWheel    = 3,
    Motor_CourseWheel   = 4,
    Motor_Balance       = 5
} Motor_FuncMotorTypeEnum;

typedef struct
{
   // Motor feedback
    float angle;
    float speed;
  	float current;
    float torque;
    float temp;               // ℃
    uint8_t error_code;
   // Used to calculate continuous angles
    float 	last_angle;
  	float   last_speed;
    int32_t round_count;
	int32_t round_count_reduction;
	float   limit_speed;
    float 	init_offset;
    float 	limited_angle;
    float 	consequent_angle;
	float   accelerate;
    float   standard_speed;
	float   limit_angle;

}Motor_EncoderTypeDef;

typedef struct _motor_type {
    Motor_EncoderTypeDef encoder;
    Motor_MotorTypeEnum type;
    uint8_t is_online;
	uint32_t watchdog;
    float output;
    uint32_t id;
    uint8_t init;
    void (*callback)(struct _motor_type*, uint8_t rxbuff[], uint32_t len);
    Filter_LowPassParamTypeDef fdb_fil_param;
    Filter_LowPassTypeDef fdb_fil;
	uint8_t error_code;

    float update_dt;
    uint32_t last_update_tick;
} Motor_MotorTypeDef;

typedef struct{
float position;
float velocity;
float KP;
float KD;
float torque;
Motor_DMBasicCtrlEnum state;
} Motor_DMMotorTypeDef;

typedef struct {
    uint8_t motor_num;
    Motor_MotorTypeDef* motor_handle[4];
    Motor_MotorTypeEnum type;
    CAN_HandleTypeDef* can_handle;
  	UART_HandleTypeDef* uart_handle;
    CAN_TxHeaderTypeDef can_header;

    Motor_DMMotorTypeDef dmoutput;  
} Motor_MotorGroupTypeDef;



typedef void (*Motor_EncoderCallbackFuncTypeDef)(Motor_MotorTypeDef*, uint8_t[], uint32_t);

extern Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

extern Motor_MotorGroupTypeDef Motor_Pitch_Front_Motors;        // usart * 1
extern Motor_MotorGroupTypeDef Motor_Pitch_Left_Motors;   
extern Motor_MotorGroupTypeDef Motor_Pitch_Right_Motors; 
extern Motor_MotorGroupTypeDef Motor_DirectionMotors;  //can1*4
extern Motor_MotorGroupTypeDef Motor_MoveMotors;       //can2*4
extern Motor_MotorGroupTypeDef Motor_Gimbal_Motors;     // can2 * 2
extern Motor_MotorGroupTypeDef Motor_Serve_Motors; 
//@twx 发球击打板电机
extern Motor_MotorGroupTypeDef Motor_Rise_Hit_Motors;   
//@twx 发球搓球3电机
extern Motor_MotorGroupTypeDef Motor_Rise_Chop_Front_Motors; 
extern Motor_MotorGroupTypeDef Motor_Rise_Chop_Right_Motors; 
extern Motor_MotorGroupTypeDef Motor_Rise_Chop_Left_Motors; 
//@twx发球抬升电机
extern Motor_MotorGroupTypeDef Motor_Rise_Lift_Motors;

extern Motor_MotorTypeDef Motor_Pitch_Front_Motor;
extern Motor_MotorTypeDef Motor_Pitch_Left_Motor;
extern Motor_MotorTypeDef Motor_Pitch_Right_Motor;

extern Motor_MotorTypeDef Motor_LeftFront_DirectionMotor;
extern Motor_MotorTypeDef Motor_RightFront_DirectionMotor;
extern Motor_MotorTypeDef Motor_LeftBack_DirectionMotor;
extern Motor_MotorTypeDef Motor_RightBack_DirectionMotor;
extern Motor_MotorTypeDef Motor_LeftFront_MoveMotor;
extern Motor_MotorTypeDef Motor_RightFront_MoveMotor;
extern Motor_MotorTypeDef Motor_LeftBack_MoveMotor;
extern Motor_MotorTypeDef Motor_RightBack_MoveMotor;

extern Motor_MotorTypeDef Motor_Gimbal_YawMotor;
extern Motor_MotorTypeDef Motor_Gimbal_PitchMotor;
extern Motor_MotorTypeDef Motor_Serve_LeftMotor;
extern Motor_MotorTypeDef Motor_Serve_RightMotor;

//@twx 发球击打板电机
extern Motor_MotorTypeDef Motor_Rise_Hit_Motor;
//@twx 发球搓球3电机
extern Motor_MotorTypeDef Motor_Rise_Chop_Front_Motor;
extern Motor_MotorTypeDef Motor_Rise_Chop_Left_Motor;
extern Motor_MotorTypeDef Motor_Rise_Chop_Right_Motor;
//@twx发球抬升电机
extern Motor_MotorTypeDef Motor_Rise_Lift_Motor;

extern Motor_FuncMotorTypeEnum Motor_ConnectingState;
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, UART_HandleTypeDef* huart, uint32_t stdid, uint8_t rxdata[], uint32_t len);
void Motor_InitAllMotors(void);
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                     Motor_EncoderCallbackFuncTypeDef callback,float offset);
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, UART_HandleTypeDef *uart, uint16_t stdid);
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output);
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor);
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup);
Motor_FuncMotorTypeEnum Motor_GetConnectingMode(void);
void Motor_SendMotorGroupsOutput(void);
uint8_t Motor_IsAnyMotorOffline(void);
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor);

void Motor_DM_Basic_Output(Motor_MotorGroupTypeDef *pgroup , Motor_DMBasicCtrlEnum basic);
void Dji3508_origin_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void Dji3508_xroll_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void Dji6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void DM4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void DM4340_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void DM8009_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void ECA4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void lk5005_encoder_callback(Motor_MotorTypeDef* motor, uint8_t* rxdata, uint32_t len);
void DMH3510_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void Go1_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void A1_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
uint32_t crc32_core(uint32_t* ptr, uint32_t len);
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint32_t len);
void Motor_SendDMMotorTorqueOutput(Motor_MotorGroupTypeDef *pgroup);
void Motor_Set_DM_MIT_Output(Motor_MotorGroupTypeDef* pgroup1, float pos1, float vel1, float tor1, float kp,float kd);
void Motor_DM_Basic_Ctrl(Motor_DMBasicCtrlEnum basic);
void Motor_Send_MIT_Output(Motor_MotorGroupTypeDef *pgroup);

#endif

#ifdef __cplusplus
}
#endif
