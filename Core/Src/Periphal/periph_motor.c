#include "sys_const.h"
#include "periph_motor.h"
#include "stdio.h"
#include "stdlib.h"
#include "alg_filter.h"
#include "lib_buff.h"
#include "cmsis_os.h"
#include "sys_dwt.h"
#include "util_gpio.h"

#define Const_Motor_UART_RX_LEN  128
   
Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM]; // 8 motor groups

Motor_MotorGroupTypeDef Motor_Pitch_Front_Motors;       // usart * 1
Motor_MotorGroupTypeDef Motor_Pitch_Left_Motors;        
Motor_MotorGroupTypeDef Motor_Pitch_Right_Motors; 
Motor_MotorGroupTypeDef Motor_DirectionMotors;  		// can1 * 4
Motor_MotorGroupTypeDef Motor_MoveMotors;       		// can2 * 4
Motor_MotorGroupTypeDef Motor_Gimbal_Motors;    		// can2 * 2
Motor_MotorGroupTypeDef Motor_Serve_Motors;  
//@twx 发球击打板电机
Motor_MotorGroupTypeDef Motor_Rise_Hit_Motors;  
//@twx 发球搓球3电机
Motor_MotorGroupTypeDef Motor_Rise_Chop_Front_Motors; 
Motor_MotorGroupTypeDef Motor_Rise_Chop_Right_Motors; 
Motor_MotorGroupTypeDef Motor_Rise_Chop_Left_Motors; 
//@twx发球抬升电机
Motor_MotorGroupTypeDef Motor_Rise_Lift_Motors;

Motor_MotorTypeDef Motor_Pitch_Front_Motor;
Motor_MotorTypeDef Motor_Pitch_Left_Motor;
Motor_MotorTypeDef Motor_Pitch_Right_Motor;

Motor_MotorTypeDef Motor_LeftFront_DirectionMotor;
Motor_MotorTypeDef Motor_RightFront_DirectionMotor;
Motor_MotorTypeDef Motor_LeftBack_DirectionMotor;
Motor_MotorTypeDef Motor_RightBack_DirectionMotor;

Motor_MotorTypeDef Motor_LeftFront_MoveMotor;
Motor_MotorTypeDef Motor_RightFront_MoveMotor;
Motor_MotorTypeDef Motor_LeftBack_MoveMotor;
Motor_MotorTypeDef Motor_RightBack_MoveMotor;

Motor_MotorTypeDef Motor_Gimbal_YawMotor;
Motor_MotorTypeDef Motor_Gimbal_PitchMotor;

Motor_MotorTypeDef Motor_Serve_LeftMotor;
Motor_MotorTypeDef Motor_Serve_RightMotor;

//@twx 发球击打板电机
Motor_MotorTypeDef Motor_Rise_Hit_Motor;
//@twx 发球搓球3电机
Motor_MotorTypeDef Motor_Rise_Chop_Front_Motor;
Motor_MotorTypeDef Motor_Rise_Chop_Left_Motor;
Motor_MotorTypeDef Motor_Rise_Chop_Right_Motor;
//@twx发球抬升电机
Motor_MotorTypeDef Motor_Rise_Lift_Motor;

uint8_t Motor_Uart6RxData[Const_Motor_UART_RX_LEN];

/**
  * @brief      Motor encoder decoding callback function
  * @param      canid: CAN Handle number
  * @param      stdid: CAN identifier
  * @param      rxdata: CAN rx data buff
  * @retval     NULL
  */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, UART_HandleTypeDef* huart, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    if (phcan != NULL && huart == NULL) {
        for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
            for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
                if ((phcan == Motor_groupHandle[i]->can_handle) 
                      && (stdid == Motor_groupHandle[i]->motor_handle[j]->id) && phcan != NULL) {
                    Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], rxdata, len);
                }
            }
        }
    }
    if (phcan == NULL && huart != NULL) {
        __HAL_DMA_DISABLE(huart->hdmarx);
        int rxdatalen = Const_Motor_UART_RX_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
        for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
            for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
                if ((huart == Motor_groupHandle[i]->uart_handle)&& (rxdatalen == 16))  {    
                   if((Motor_Uart6RxData[2]& 0xf)== Motor_groupHandle[i]->motor_handle[j]->id){							
                        Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], Motor_Uart6RxData , rxdatalen);    }            
                }
            }
        }
        __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Motor_UART_RX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

/**
  * @brief      Initialize all motors
  * @param      NULL
  * @retval     NULL
  */
void Motor_InitAllMotors() {
    Uart_InitUartDMA(&huart6);
    Uart_ReceiveDMA(&huart6, Motor_Uart6RxData, Const_Motor_UART_RX_LEN);

    Motor_groupHandle[0] = &Motor_Pitch_Front_Motors;
    Motor_InitMotorGroup(&Motor_Pitch_Front_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Front_Motor, Motor_TYPE_Go1, 1, 0.1, Go1_encoder_callback,0);
    Motor_Pitch_Front_Motors.motor_handle[0] = &Motor_Pitch_Front_Motor;

    Motor_groupHandle[1] = &Motor_Pitch_Left_Motors;
    Motor_InitMotorGroup(&Motor_Pitch_Left_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Left_Motor, Motor_TYPE_Go1, 2, 0.1, Go1_encoder_callback,0);
    Motor_Pitch_Left_Motors.motor_handle[0] = &Motor_Pitch_Left_Motor;

    Motor_groupHandle[2] = &Motor_Pitch_Right_Motors;
    Motor_InitMotorGroup(&Motor_Pitch_Right_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Right_Motor, Motor_TYPE_Go1, 3, 0.1, Go1_encoder_callback,0);
    Motor_Pitch_Right_Motors.motor_handle[0] = &Motor_Pitch_Right_Motor; 

    Motor_groupHandle[3] = &Motor_DirectionMotors;
    Motor_InitMotorGroup(&Motor_DirectionMotors,Motor_TYPE_Dji6020,4, &hcan2,NULL, 0x1FE);
    Motor_InitMotor( &Motor_RightFront_DirectionMotor, Motor_TYPE_Dji6020, 0x205, 0.1, Dji6020_encoder_callback, Const_DIRECTION_RIGHT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor( &Motor_LeftFront_DirectionMotor , Motor_TYPE_Dji6020, 0x206, 0.1, Dji6020_encoder_callback, Const_DIRECTION_LEFT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor( &Motor_LeftBack_DirectionMotor  , Motor_TYPE_Dji6020, 0x207, 0.1, Dji6020_encoder_callback, Const_DIRECTION_LEFT_BACK_MOTOR_INIT_OFFSET);
    Motor_InitMotor( &Motor_RightBack_DirectionMotor , Motor_TYPE_Dji6020, 0x208, 0.1, Dji6020_encoder_callback, Const_DIRECTION_RIGHT_BACK_MOTOR_INIT_OFFSET);
    Motor_DirectionMotors.motor_handle[0] = &Motor_RightFront_DirectionMotor;
    Motor_DirectionMotors.motor_handle[1] = &Motor_LeftFront_DirectionMotor;
    Motor_DirectionMotors.motor_handle[2] = &Motor_LeftBack_DirectionMotor;
    Motor_DirectionMotors.motor_handle[3] = &Motor_RightBack_DirectionMotor;

    Motor_groupHandle[4] = &Motor_MoveMotors;
    Motor_InitMotorGroup(&Motor_MoveMotors,Motor_TYPE_Dji3508_xroll,4 ,&hcan1, NULL,0x200);
    Motor_InitMotor( &Motor_RightFront_MoveMotor, Motor_TYPE_Dji3508_xroll,  0x201, 0.1,  Dji3508_xroll_encoder_callback,0);
    Motor_InitMotor( &Motor_LeftFront_MoveMotor , Motor_TYPE_Dji3508_xroll,  0x202, 0.1,  Dji3508_xroll_encoder_callback,0);
    Motor_InitMotor( &Motor_LeftBack_MoveMotor  , Motor_TYPE_Dji3508_xroll,  0x203, 0.1,  Dji3508_xroll_encoder_callback,0);
    Motor_InitMotor( &Motor_RightBack_MoveMotor , Motor_TYPE_Dji3508_xroll,  0x204, 0.1,  Dji3508_xroll_encoder_callback,0);
    Motor_MoveMotors.motor_handle[0] = &Motor_RightFront_MoveMotor;
    Motor_MoveMotors.motor_handle[1] = &Motor_LeftFront_MoveMotor;
    Motor_MoveMotors.motor_handle[2] = &Motor_LeftBack_MoveMotor;
    Motor_MoveMotors.motor_handle[3] = &Motor_RightBack_MoveMotor;

    Motor_groupHandle[5] = &Motor_Serve_Motors;
    Motor_InitMotorGroup(&Motor_Serve_Motors ,Motor_TYPE_Dji3508_origin,2, &hcan1  ,NULL,  0x1FF);
    Motor_InitMotor(&Motor_Serve_LeftMotor, Motor_TYPE_Dji3508_origin, 0x205, 0.1, Dji3508_origin_encoder_callback,0);
    Motor_InitMotor(&Motor_Serve_RightMotor, Motor_TYPE_Dji3508_origin, 0x206, 0.1, Dji3508_origin_encoder_callback,0);
    Motor_Serve_Motors.motor_handle[0] = &Motor_Serve_LeftMotor;
    Motor_Serve_Motors.motor_handle[1] = &Motor_Serve_RightMotor;
        
    Motor_groupHandle[6] = &Motor_Gimbal_Motors;
    Motor_InitMotorGroup(&Motor_Gimbal_Motors, Motor_TYPE_Dji6020, 2, &hcan2, NULL, 0x2FE);
    Motor_InitMotor(&Motor_Gimbal_YawMotor, Motor_TYPE_Dji6020, 0x209, 0.1, Dji6020_encoder_callback,0);
    Motor_InitMotor(&Motor_Gimbal_PitchMotor, Motor_TYPE_Dji6020, 0x20A, 0.1, Dji6020_encoder_callback,0);
    Motor_Gimbal_Motors.motor_handle[0] = &Motor_Gimbal_YawMotor;
    Motor_Gimbal_Motors.motor_handle[1] = &Motor_Gimbal_PitchMotor;
    Motor_Gimbal_Motors.motor_handle[2] = NULL;
    Motor_Gimbal_Motors.motor_handle[3] = NULL;

    Motor_groupHandle[7] = &Motor_Rise_Hit_Motors;
    Motor_InitMotorGroup(&Motor_Rise_Hit_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Rise_Hit_Motor, Motor_TYPE_Go1, 4, 0.1, Go1_encoder_callback,0);
    Motor_Rise_Hit_Motors.motor_handle[0] = &Motor_Rise_Hit_Motor;

    Motor_groupHandle[8] = &Motor_Rise_Chop_Front_Motors;
    Motor_InitMotorGroup(&Motor_Rise_Chop_Front_Motors, Motor_TYPE_DMH3510, 1, &hcan1, NULL, 0x01);
    Motor_InitMotor(&Motor_Rise_Chop_Front_Motor, Motor_TYPE_DMH3510, 0x01, 0.1, DMH3510_encoder_callback,0);
    Motor_Rise_Chop_Front_Motors.motor_handle[0] = &Motor_Rise_Chop_Front_Motor;

    Motor_groupHandle[9] = &Motor_Rise_Chop_Right_Motors;
    Motor_InitMotorGroup(&Motor_Rise_Chop_Right_Motors, Motor_TYPE_DMH3510, 1, &hcan1, NULL, 0x02);
    Motor_InitMotor(&Motor_Rise_Chop_Right_Motor, Motor_TYPE_DMH3510, 0x02, 0.1, DMH3510_encoder_callback,0);
    Motor_Rise_Chop_Right_Motors.motor_handle[0] = &Motor_Rise_Chop_Right_Motor;

    Motor_groupHandle[10] = &Motor_Rise_Chop_Left_Motors;
    Motor_InitMotorGroup(&Motor_Rise_Chop_Left_Motors, Motor_TYPE_DMH3510, 1, &hcan1, NULL, 0x03);
    Motor_InitMotor(&Motor_Rise_Chop_Left_Motor, Motor_TYPE_DMH3510, 0x03, 0.1, DMH3510_encoder_callback,0);
    Motor_Rise_Chop_Left_Motors.motor_handle[0] = &Motor_Rise_Chop_Left_Motor;

    Motor_groupHandle[11] = &Motor_Rise_Lift_Motors;
    Motor_InitMotorGroup(&Motor_Rise_Lift_Motors,Motor_TYPE_Dji3508_origin,1,&hcan1, NULL,0x1FF);
    Motor_InitMotor( &Motor_Rise_Lift_Motor, Motor_TYPE_Dji3508_origin,  0x205, 0.1,  Dji3508_origin_encoder_callback,0);
    Motor_Rise_Lift_Motors.motor_handle[0] = &Motor_Rise_Lift_Motor;
    Motor_Rise_Lift_Motors.motor_handle[1] = NULL;
    Motor_Rise_Lift_Motors.motor_handle[2] = NULL;
    Motor_Rise_Lift_Motors.motor_handle[3] = NULL;



}   
/**
  * @brief      Initialize the motor
  * @param      pmotor: Pointer to motor object
  * @param      type: Type of motor (pwm or can)
  * @param      callback: Motor callback function
  * @retval     NULL
  */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                     Motor_EncoderCallbackFuncTypeDef callback, float motor_offset) {
    if (pmotor == NULL) return;
    pmotor->last_update_tick = 0;
    pmotor->type = type;
    pmotor->id = id;			
    pmotor->init = 0;			
    pmotor->is_online = 0;
    pmotor->output = 0;
    pmotor->callback = callback;
    pmotor->encoder.init_offset = motor_offset;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
	
}
/**
  * @brief      Initialization of motor group
  * @param      pgroup: Pointer to motor group
  * @param      type: Type of motor (pwm or can)
  * @param      motor_num: Number of motor group
  * @param      phcan: Pointer of can handle
  * @param      stdid: Motor id
  * @retval     NULL
  */
void  Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, UART_HandleTypeDef *uart, uint16_t stdid) {
    if (pgroup == NULL) return;
   	  pgroup->motor_num = motor_num;
      pgroup->type = type;
	
	if (type == Motor_TYPE_DM8009 || type == Motor_TYPE_DM4310 || type == Motor_TYPE_DM4340 
	  || type == Motor_TYPE_Dji3508_xroll || type == Motor_TYPE_Dji3508_origin || type == Motor_TYPE_LK5005 || type == Motor_TYPE_Dji6020 || type == Motor_TYPE_DMH3510) {
        if (phcan == NULL) return;
        pgroup->can_handle = phcan;
        Can_InitTxHeader(&(pgroup->can_header), stdid, 0, 8);
    }
	
	if (type == Motor_TYPE_A1 || type == Motor_TYPE_Go1) {
        pgroup->uart_handle = uart;
    }
   
    for (int i = 0; i < motor_num; ++i) 
        pgroup->motor_handle[i] = NULL;

}

void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    pmotor->output = output;
}

/**
  * @brief      Get motor output value
  * @param      pmotor: Pointer to motor object
  * @retval     Output value
  */
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
	if (pmotor == NULL) return 0;
	int16_t ret = 0;

	if (pmotor->type == Motor_TYPE_Dji3508_origin || 
			 pmotor->type == Motor_TYPE_Dji3508_xroll ) {
		ret = (int16_t)(pmotor->output / 0.3f  / 20.0f * 16384.0f);
		return ret;
	}
	
	else if (pmotor->type == Motor_TYPE_Dji6020) {
		ret = (int16_t)(pmotor->output / 0.741f / 3.0f * 16384.0f);
		return ret;
	}
	
	else if (pmotor->type == Motor_TYPE_Go1) {
			ret = (int16_t)(pmotor->output / 6.33f * 256.0f);
			  return (uint16_t)ret;
			  }
}

uint32_t Motor_GetLKMotorOutput(Motor_MotorTypeDef* pmotor)
{
	int32_t ret = 0;
	if (pmotor->type == Motor_TYPE_LK5005) {

		ret = (int32_t)((int)pmotor->output);
					return (uint32_t)ret;
        }
	return 0;
}


uint8_t txdata[64];
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup) {

	if (pgroup == NULL) return;
	
	if (pgroup->type == Motor_TYPE_Dji6020 || pgroup->type == Motor_TYPE_Dji3508_origin || 
		pgroup->type == Motor_TYPE_Dji3508_xroll ) {			
	txdata[0] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
	txdata[1] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
	txdata[2] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[1]) >> 8);
	txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[1]); 
	txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[2]) >> 8);
	txdata[5] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[2]);
	txdata[6] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[3]) >> 8);
	txdata[7] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[3]);
	
	Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
    }
	
	else if (pgroup->type == Motor_TYPE_DM4310 || pgroup->type == Motor_TYPE_DM4340 ||pgroup->type == Motor_TYPE_DM8009 || pgroup->type == Motor_TYPE_DMH3510) {
		 uint16_t  pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
		 pos_tmp = float_to_uint(0, Const_DMmotor_P_MIN, Const_DMmotor_P_MAX, 16);
		 kp_tmp = float_to_uint(0, Const_DMmotor_KP_MIN, Const_DMmotor_KP_MAX, 12);
		 kd_tmp = float_to_uint(0, Const_DMmotor_KD_MIN, Const_DMmotor_KD_MAX, 12);
		
		 switch (pgroup->motor_handle[0]->type){
			case Motor_TYPE_DM4310 :
				vel_tmp = float_to_uint(0, -30, 30, 12);
				tor_tmp = float_to_uint(pgroup->motor_handle[0]->output , -10, 10, 12);
				break;
			case Motor_TYPE_DM4340 :
				vel_tmp = float_to_uint(0, -30, 30, 12);
				tor_tmp = float_to_uint(pgroup->motor_handle[0]->output , -28, 28, 12);
				break;
			case Motor_TYPE_DM8009 :
				vel_tmp = float_to_uint(0, -45, 45, 12);
				tor_tmp = float_to_uint(pgroup->motor_handle[0]->output  , -54, 54, 12);
				break;
            case Motor_TYPE_DMH3510 :
				vel_tmp = float_to_uint(0,-280, 280, 12);
				tor_tmp = float_to_uint(pgroup->motor_handle[0]->output  , -12.5, 12.5, 12);
				break;
			default:
				 break;
		 }       
		 txdata[0] = (pos_tmp >> 8);
		 txdata[1] = pos_tmp;
		 txdata[2] = (vel_tmp >> 4);
		 txdata[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
		 txdata[4] = kp_tmp;
		 txdata[5] = (kd_tmp >> 4);
		 txdata[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
		 txdata[7] = tor_tmp;
	 
		 Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
	}
	
	else if (pgroup->type == Motor_TYPE_Go1) {
		memset(txdata, 0, 64);
		txdata[0] = 0XFE;
		txdata[1] = 0XEE;
		txdata[2] =  (0x01<<4) | ((uint8_t)pgroup->motor_handle[0]->id)  ;
		txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
		txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
		uint16_t crc = crc_ccitt(0,(uint8_t*)txdata, 15);
		txdata[15] = (uint8_t)crc;
		txdata[16] = (uint8_t)(crc >> 8);
		GPIO_Set( RS485_TX );
	
		Uart_SendMessageDMA(pgroup->uart_handle, txdata, 17);			      
	}
}

void Motor_Set_DM_MIT_Output(Motor_MotorGroupTypeDef* pgroup1, float pos, float vel, float tor, float kp,float kd)
{   
    pgroup1->dmoutput.position = pos;
    pgroup1->dmoutput.velocity = vel;
    pgroup1->dmoutput.torque = tor;
    pgroup1->dmoutput.KP = kp;
    pgroup1->dmoutput.KD = kd;
}
void Motor_Send_MIT_Output(Motor_MotorGroupTypeDef *pgroup)
{  
    if (pgroup == NULL) return;
    uint16_t  pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint8_t txbuff[8];

    pos_tmp = float_to_uint(pgroup->dmoutput.position, Const_DMmotor_P_MIN, Const_DMmotor_P_MAX, 16);
    kp_tmp = float_to_uint(pgroup->dmoutput.KP, Const_DMmotor_KP_MIN, Const_DMmotor_KP_MAX, 12);
	kd_tmp = float_to_uint(pgroup->dmoutput.KD, Const_DMmotor_KD_MIN, Const_DMmotor_KD_MAX, 12);
	
	switch (pgroup->motor_handle[0]->type){
	case Motor_TYPE_DM4310 :
        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -30, 30, 12);
        tor_tmp = float_to_uint(pgroup->dmoutput.torque, -10, 10, 12);
	    break;
    case Motor_TYPE_DM4340 :
        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -30, 30, 12);
        tor_tmp = float_to_uint(pgroup->dmoutput.torque, -28, 28, 12);
	    break;
    case Motor_TYPE_DM8009 :
        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -45, 45, 12);
        tor_tmp = float_to_uint(pgroup->dmoutput.torque, -54, 54, 12);
	    break;
    case Motor_TYPE_DMH3510 :
        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -280, 280, 12);
        tor_tmp = float_to_uint(pgroup->dmoutput.torque, -12.5, 12.5, 12);
        break;
    default:
        break;
	}
   
    txbuff[0] = (pos_tmp >> 8);
    txbuff[1] = pos_tmp;
    txbuff[2] = (vel_tmp >> 4);
    txbuff[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    txbuff[4] = kp_tmp;
    txbuff[5] = (kd_tmp >> 4);
    txbuff[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    txbuff[7] = tor_tmp;

    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txbuff);
}

uint8_t txbuff[8];
 void Motor_DM_Basic_Output(Motor_MotorGroupTypeDef *pgroup , Motor_DMBasicCtrlEnum basic)
 {
    if (pgroup == NULL) return;
    // uint8_t txbuff[8];
    txbuff[0] = 0xFF;
    txbuff[1] = 0xFF;
    txbuff[2] = 0xFF;
    txbuff[3] = 0xFF;
    txbuff[4] = 0xFF;
    txbuff[5] = 0xFF;
    txbuff[6] = 0xFF;
	switch (basic)
	{
	case Motor_Enable:
	txbuff[7] = 0xFC;
		break;

	case Motor_Disable:
	txbuff[7] = 0xFD;
		break;

	case Motor_SaveInitpos:
    txbuff[7] = 0xFE;
		break;

	case Motor_Clearerr :
    txbuff[7] = 0xFB;
        break;
    }
  	//HAL_Delay(500);
    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txbuff);	
 }

 void DM4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
             
    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -30, 30, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -10, 10, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    pmotor->init = 1; 
     pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
 
    pmotor->type = Motor_TYPE_DM4310;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
} 
 void DM4340_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
             
    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -30, 30, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -28, 28, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    pmotor->init = 1; 
     pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
 
    pmotor->type = Motor_TYPE_DM4340;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
		pmotor->watchdog = 0;
    
} 
 void DM8009_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
             
    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -45, 45, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -54, 54, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    pmotor->init = 1; 
     pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
 
    pmotor->type = Motor_TYPE_DM8009;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
}

void DMH3510_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;

    // 保存上一时刻的原始角度 (用于多圈计数)
    pmotor->encoder.last_angle = pmotor->encoder.angle;

    pmotor->encoder.error_code = rxbuff[0] >> 4; // 获取高4位的错误状态
    uint8_t motor_id = rxbuff[0] & 0x0F; // 获取低4位的ID

    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -280, 280, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5],-12.5, 12.5, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    
    // 多圈计数 
    if (pmotor->init == 0) {
        pmotor->encoder.last_angle = pmotor->encoder.angle; // 首次接收，初始化
        pmotor->encoder.round_count = 0;
        pmotor->init = 1;
    } else {
        // 16位原始数据 (65536) 的溢出判断
        int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;
        if (diff < -180) { // 最小值 -> 最大值 (正转)
            pmotor->encoder.round_count++;
        } else if (diff > 180) { // 最大值 -> 最小值 (反转)
            pmotor->encoder.round_count--;
        }
    }
    // 计算连续角度
    // DM-H3510 减速比为 1:1 [cite: 240]
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360 + pmotor->encoder.angle;


    pmotor->type = Motor_TYPE_DMH3510;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    pmotor->watchdog = 0;
    
}

extern uint64_t output_state;
void Go1_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
	
    if (pmotor == NULL) return;
    if (len != 16) return;
    if ((rxbuff[0] != 0XFD) || (rxbuff[1] != 0XEE)) return; 


    // 2. 解析原始物理数据 (Raw Data)
    // 注意：先不要直接赋值给 pmotor->encoder.angle，否则会覆盖掉去皮逻辑
    
    // 解析角度 (Raw Angle)
    // Go1 数据格式: 4字节 int32, 单位 rad, 需要转换
    // 公式: (raw_int / 16384.0) * 180.0 (转为度) / 减速比 6.33
    int32_t raw_angle_int = (int32_t)((uint32_t)rxbuff[7] | (uint32_t)rxbuff[8] << 8 | (uint32_t)rxbuff[9] << 16 | (uint32_t)rxbuff[10] << 24);
    float raw_angle = (float)raw_angle_int * 180.0f / 16384.0f / 6.33f ;

    // 解析速度 (Speed)
    int16_t raw_speed_int = (int16_t)((uint16_t)rxbuff[5] | (uint16_t)rxbuff[6] << 8);
    float current_speed = (float)raw_speed_int / 128.0f * 3.1415926f/6.33f ; // PI = 3.14...

    // 解析力矩 (Torque)
    int16_t raw_torque_int = (int16_t)((uint16_t)rxbuff[3] | (uint16_t)rxbuff[4] << 8);
    float current_torque = (float)raw_torque_int / 256.0f * 6.33f;

    // 解析其他数据
    float current_temp = (float)((uint32_t)rxbuff[11]);
    uint8_t error_code = rxbuff[12];


    // 3. ！！！ 关键逻辑：上电自动归零 (Tare) ！！！
    if (pmotor->init == 0) {
        // 第一次收到数据，将当前“原始角度”记录为“初始偏移量”
        pmotor->encoder.init_offset = raw_angle;
        
        // 初始化上一时刻角度，防止第一帧计算多圈时出错
        pmotor->encoder.last_angle = 0.0f; 
        pmotor->encoder.round_count = 0;
        
        // 标记已初始化
        pmotor->init = 1;
    }

    // 4. 计算相对角度 (用户看到的角度)
    // 现在的角度 = 原始绝对角度 - 初始偏移量
    // 这样上电那一刻，angle 就会等于 0
    float relative_angle = raw_angle - pmotor->encoder.init_offset;
//  	if(pmotor == &Motor_Pitch_Front_Motor){output_state =2;}
//	  if(pmotor == &Motor_Pitch_Left_Motor ){output_state =3;}
//	  if(pmotor == &Motor_Pitch_Right_Motor){output_state =1;}
    // Assign a value to the previous angle and get the latest angle
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle   = relative_angle;
    pmotor->encoder.speed   = current_speed;
    pmotor->encoder.torque  = current_torque;
    pmotor->encoder.temp    = current_temp;
	pmotor->encoder.error_code = rxbuff[12];
    pmotor->encoder.limited_angle = pmotor->encoder.angle;
    pmotor->encoder.consequent_angle = pmotor->encoder.angle;
    pmotor->type = Motor_TYPE_Go1;
   // pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
		pmotor->watchdog = 0;    
}

uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = (uint32_t)1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++){
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}
const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint32_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}

void Dji3508_origin_encoder_callback(Motor_MotorTypeDef* motor, uint8_t* rxdata, uint32_t len)
{
	if(motor == NULL) return;
	if(len != 8) return;

	motor->encoder.last_speed = motor->encoder.speed;
	motor->encoder.last_angle = motor->init == 1 ? motor->encoder.angle : (rxdata[0] << 8 | rxdata[1]);
	motor->encoder.angle      = (float)((uint16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
	motor->encoder.speed      = (float)((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3]));
	motor->encoder.current    = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
	motor->encoder.temp       = 0;
	motor->encoder.torque     = motor->encoder.current / 16384.0f * 20.0f * 0.3f;
	
	motor->encoder.init_offset = motor->init == 0 ? motor->encoder.angle / 8192.0f / (3591.0f / 187.0f) * 360.0f : motor->encoder.init_offset;
	motor->encoder.round_count_reduction = motor->encoder.round_count / (3591.0f / 187.0f);
	motor->encoder.standard_speed = motor->encoder.speed / 60 * 2 * PI / (3591.0f / 187.0f);
	motor->init = 1;

	motor->encoder.accelerate = (motor->encoder.speed - motor->encoder.last_speed) / motor->update_dt;
	int diff = motor->encoder.angle - motor->encoder.last_angle;

	if(diff < -4096)
	{
			motor->encoder.round_count++;
	}
	else if(diff > 4096)
	{
			motor->encoder.round_count--;
	}

	// Calculate continuous angle
	motor->encoder.consequent_angle = ((float)(motor->encoder.angle / 8192.0f * 360.0f) + (float)(motor->encoder.round_count * 360.0f)) / (3591.0f / 187.0f) - motor->encoder.init_offset;
	
	if(motor->encoder.round_count > 10000)
	{
			motor->encoder.consequent_angle -= 10 * motor->encoder.round_count;
			motor->encoder.round_count = 0;
	}
	else if(motor->encoder.round_count < -10000)
	{
			motor->encoder.consequent_angle += 10 * motor->encoder.round_count;
			motor->encoder.round_count = 0;
	}

	if(motor->encoder.limit_angle < motor->encoder.init_offset - 180 && motor->encoder.init_offset >= 180)
	{
			motor->encoder.limit_angle += 360;
	}
	else if(motor->encoder.limit_angle > motor->encoder.init_offset + 180 && motor->encoder.init_offset < 180)
	{
			motor->encoder.limit_angle -= 360;
	}

	motor->type = Motor_TYPE_Dji3508_origin;
	motor->update_dt = DWT_GetDeltaT(&motor->last_update_tick);
    motor->watchdog = 0;


}

void Dji3508_xroll_encoder_callback(Motor_MotorTypeDef* motor, uint8_t* rxdata, uint32_t len)
{
	if(motor == NULL) return;
	if(len != 8) return;

	motor->encoder.last_speed = motor->encoder.speed;
	motor->encoder.last_angle = motor->init == 1 ? motor->encoder.angle : (rxdata[0] << 8 | rxdata[1]);
	motor->encoder.angle      = (float)((uint16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
	motor->encoder.speed      = (float)((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3]));
	motor->encoder.current    = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
	motor->encoder.temp       = 0;
	motor->encoder.torque     = motor->encoder.current / 16384.0f * 20.0f * 0.3f;
	
	motor->encoder.init_offset = motor->init == 0 ? motor->encoder.angle / 8192.0f / (256.0f / 17.0f) * 360.0f : motor->encoder.init_offset;
	motor->encoder.round_count_reduction = motor->encoder.round_count / (256.0f / 17.0f);
	motor->encoder.standard_speed = motor->encoder.speed / 60 * 2 * PI / (256.0f / 17.0f);
	motor->init = 1;

	motor->encoder.accelerate = (motor->encoder.speed - motor->encoder.last_speed) / motor->update_dt;
	int diff = motor->encoder.angle - motor->encoder.last_angle;

	if(diff < -4096)
	{
			motor->encoder.round_count++;
	}
	else if(diff > 4096)
	{
			motor->encoder.round_count--;
	}

	// Calculate continuous angle
	motor->encoder.consequent_angle = ((float)(motor->encoder.angle / 8192.0f * 360.0f) + (float)(motor->encoder.round_count * 360.0f)) / (256.0f / 17.0f);
	
	if(motor->encoder.round_count > 10000)
	{
			motor->encoder.consequent_angle -= 10 * motor->encoder.round_count;
			motor->encoder.round_count = 0;
	}
	else if(motor->encoder.round_count < -10000)
	{
			motor->encoder.consequent_angle += 10 * motor->encoder.round_count;
			motor->encoder.round_count = 0;
	}

	if(motor->encoder.limit_angle < motor->encoder.init_offset - 180 && motor->encoder.init_offset >= 180)
	{
			motor->encoder.limit_angle += 360;
	}
	else if(motor->encoder.limit_angle > motor->encoder.init_offset + 180 && motor->encoder.init_offset < 180)
	{
			motor->encoder.limit_angle -= 360;
	}

	motor->type = Motor_TYPE_Dji3508_xroll;
	motor->update_dt = DWT_GetDeltaT(&motor->last_update_tick);
    motor->watchdog = 0; 
}

void Dji6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    if (len != 8) return;

    uint16_t reverse_angle = (uint16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]);
    reverse_angle = (reverse_angle == 0) ? 0 : (8192 - reverse_angle);

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : ((float)(reverse_angle) / 8192.0f * 360.0f);
    pmotor->encoder.angle   = (float)(reverse_angle) / 8192.0f * 360.0f;
    pmotor->encoder.speed   = -(float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) * 0.1047f;
    pmotor->encoder.current = -(float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5])) / 16384.0f * 3.0f;
    pmotor->encoder.torque = pmotor->encoder.current * 0.741f;
    pmotor->encoder.temp = (float)(rxbuff[6]); 
    pmotor->init = 1; 

    // Calculate angle difference and number of cycles
    if ((pmotor->encoder.last_angle > 358.0f) && (pmotor->encoder.angle < 2.0f)) 
        pmotor->encoder.round_count++;
    else if ((pmotor->encoder.last_angle < 2.0f) && (pmotor->encoder.angle > 358.0f)) 
        pmotor->encoder.round_count--;
        
    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + (float)pmotor->encoder.angle + pmotor->encoder.init_offset;
    float temp_limited_angle = (float)pmotor->encoder.angle - pmotor->encoder.init_offset;
    while (temp_limited_angle < -180.0f) temp_limited_angle += 360.0f;
    while (temp_limited_angle > 180.0f) temp_limited_angle -= 360.0f;
    pmotor->encoder.limited_angle = temp_limited_angle;
    
    pmotor->type = Motor_TYPE_Dji6020;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick); 
    
}
