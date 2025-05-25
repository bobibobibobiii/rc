#include "module_serve.h"
#include "alg_pid.h"
#include "periph_motor.h"
#include "sys_const.h"

Module_ServeTypeDef ServeData;

void Module_ServeInit(void) {
  Module_ServeTypeDef *serve = Module_ServeGetPtr();

  serve->init = 0;
  serve->is_angle = 0;
  serve->current_angle = 0;
  serve->current_speed = 0;
  serve->target_angle = 0.0f;
  serve->current_PoleTorque = 0;
  serve->target_speed = 0.0f;
  serve->ServeMotors = Motor_groupHandle[3];

  PID_InitPIDParam(&serve->PID_LeftAngleParam, Const_LeftPosMotorParam[0][0],
                   Const_LeftPosMotorParam[0][1], Const_LeftPosMotorParam[0][2],
                   Const_LeftPosMotorParam[0][3], Const_LeftPosMotorParam[0][4],
                   Const_LeftPosMotorParam[1][0], Const_LeftPosMotorParam[1][1],
                   Const_LeftPosMotorParam[2][0], Const_LeftPosMotorParam[2][1],
                   Const_LeftPosMotorParam[3][0], Const_LeftPosMotorParam[3][1],
                   PID_POSITION);
  PID_InitPIDParam(&serve->PID_LeftSpeedParam, Const_LeftSpdMotorParam[0][0],
                   Const_LeftSpdMotorParam[0][1], Const_LeftSpdMotorParam[0][2],
                   Const_LeftSpdMotorParam[0][3], Const_LeftSpdMotorParam[0][4],
                   Const_LeftSpdMotorParam[1][0], Const_LeftSpdMotorParam[1][1],
                   Const_LeftSpdMotorParam[2][0], Const_LeftSpdMotorParam[2][1],
                   Const_LeftSpdMotorParam[3][0], Const_LeftSpdMotorParam[3][1],
                   PID_POSITION);
  PID_InitPIDParam(
      &serve->PID_RightAngleParam, Const_RightPosMotorParam[0][0],
      Const_RightPosMotorParam[0][1], Const_RightPosMotorParam[0][2],
      Const_RightPosMotorParam[0][3], Const_RightPosMotorParam[0][4],
      Const_RightPosMotorParam[1][0], Const_RightPosMotorParam[1][1],
      Const_RightPosMotorParam[2][0], Const_RightPosMotorParam[2][1],
      Const_RightPosMotorParam[3][0], Const_RightPosMotorParam[3][1],
      PID_POSITION);
  PID_InitPIDParam(
      &serve->PID_RightSpeedParam, Const_RightSpdMotorParam[0][0],
      Const_RightSpdMotorParam[0][1], Const_RightSpdMotorParam[0][2],
      Const_RightSpdMotorParam[0][3], Const_RightSpdMotorParam[0][4],
      Const_RightSpdMotorParam[1][0], Const_RightSpdMotorParam[1][1],
      Const_RightSpdMotorParam[2][0], Const_RightSpdMotorParam[2][1],
      Const_RightSpdMotorParam[3][0], Const_RightSpdMotorParam[3][1],
      PID_POSITION);
}

Module_ServeTypeDef *Module_ServeGetPtr(void) { return &ServeData; }

void Module_ServeReady(void) {
  Module_ServeTypeDef *serve = Module_ServeGetPtr();
  serve->target_speed = 2.0f;
	
  if (!Module_ServeIsAngle()) {
    Module_ServeSpdPIDCal();
    Motor_SetMotorOutput(serve->ServeMotors->motor_handle[0],
                         PID_GetPIDOutput(&serve->PID_LeftSpeed) +
                             serve->current_PoleTorque / 2);
    Motor_SetMotorOutput(serve->ServeMotors->motor_handle[1],
                         PID_GetPIDOutput(&serve->PID_RightSpeed) -
                             serve->current_PoleTorque / 2);
  } else {
	serve->target_angle = 150.0f;
    Module_ServeAnglePIDCal();
    Motor_SetMotorOutput(serve->ServeMotors->motor_handle[0],
                         PID_GetPIDOutput(&serve->PID_LeftSpeed) + 
							serve->current_PoleTorque / 2);
    Motor_SetMotorOutput(serve->ServeMotors->motor_handle[1],
                         PID_GetPIDOutput(&serve->PID_RightSpeed) -
                            serve->current_PoleTorque / 2);
	}
	Motor_SendMotorGroupOutput(serve->ServeMotors);
}


void Module_ServeHit(void) {
	Module_ServeTypeDef *serve = Module_ServeGetPtr();
	serve->target_speed = - 27.5f;

	if (!Module_ServeIsAngle()) {
		Module_ServeSpdPIDCal();
		Motor_SetMotorOutput(serve->ServeMotors->motor_handle[0],
							 PID_GetPIDOutput(&serve->PID_LeftSpeed) + serve->current_PoleTorque / 2);
		Motor_SetMotorOutput(serve->ServeMotors->motor_handle[1],
							 PID_GetPIDOutput(&serve->PID_RightSpeed) - serve->current_PoleTorque / 2);

	} 
	else {
		serve->target_speed = 0.0f;
		  
		Module_ServeSpdPIDCal();
		Motor_SetMotorOutput(serve->ServeMotors->motor_handle[0],
						 PID_GetPIDOutput(&serve->PID_LeftSpeed) + serve->current_PoleTorque / 2);
		Motor_SetMotorOutput(serve->ServeMotors->motor_handle[1],
						 PID_GetPIDOutput(&serve->PID_RightSpeed) - serve->current_PoleTorque / 2);
	}
	Motor_SendMotorGroupOutput(serve->ServeMotors);
}

void Module_ServeKeep(void) {
	Module_ServeTypeDef *serve = Module_ServeGetPtr();
	serve->target_speed = 0.0f;
	
	Module_ServeSpdPIDCal();
	Motor_SetMotorOutput(serve->ServeMotors->motor_handle[0],
					 PID_GetPIDOutput(&serve->PID_LeftSpeed) + serve->current_PoleTorque / 2);
	Motor_SetMotorOutput(serve->ServeMotors->motor_handle[1],
					 PID_GetPIDOutput(&serve->PID_RightSpeed) - serve->current_PoleTorque / 2);

	Motor_SendMotorGroupOutput(serve->ServeMotors);
}

/**
* @brief: using for PID Parameter fix 
* @param: none
* @return: none
*/
void Module_ServePIDTest(void) {
	  Module_ServeTypeDef *serve = Module_ServeGetPtr();
	
//	  PID_SetPIDRef(&serve->PID_LeftSpeed, serve->target_speed);
//    PID_SetPIDFdb(
//        &serve->PID_LeftSpeed,
//        Motor_groupHandle[3]->motor_handle[0]->encoder.standard_speed);
//    PID_CalcPID(&serve->PID_LeftSpeed, &serve->PID_LeftSpeedParam);

//    PID_SetPIDRef(&serve->PID_RightSpeed, -serve->target_speed);
//    PID_SetPIDFdb(
//        &serve->PID_RightSpeed,
//        Motor_groupHandle[3]->motor_handle[1]->encoder.standard_speed);
//    PID_CalcPID(&serve->PID_RightSpeed, &serve->PID_RightSpeedParam);

//		Motor_SetMotorOutput(Motor_groupHandle[1]->motor_handle[0],
//												 PID_GetPIDOutput(&serve->PID_LeftSpeed) + serve->current_PoleTorque / 2);
//		Motor_SetMotorOutput(Motor_groupHandle[1]->motor_handle[1],
//												 PID_GetPIDOutput(&serve->PID_RightSpeed) - serve->current_PoleTorque / 2);
//		Motor_SetMotorOutput(Motor_groupHandle[3]->motor_handle[0], PID_GetPIDOutput(&serve->PID_LeftSpeed));
//    Motor_SetMotorOutput(Motor_groupHandle[3]->motor_handle[1], PID_GetPIDOutput(&serve->PID_RightSpeed));
		
		
//		serve->target_angle = 180.0f;
    PID_SetPIDRef(&serve->PID_LeftAngle, serve->target_angle);
    PID_SetPIDFdb(
        &serve->PID_LeftAngle,
        serve->ServeMotors->motor_handle[0]->encoder.consequent_angle);
    PID_CalcPID(&serve->PID_LeftAngle, &serve->PID_LeftAngleParam);

    PID_SetPIDRef(&serve->PID_LeftSpeed, serve->PID_LeftAngle.output);
    PID_SetPIDFdb(
        &serve->PID_LeftSpeed,
        serve->ServeMotors->motor_handle[0]->encoder.standard_speed);
    PID_CalcPID(&serve->PID_LeftSpeed, &serve->PID_LeftSpeedParam);

    PID_SetPIDRef(&serve->PID_RightAngle, -serve->target_angle);
    PID_SetPIDFdb(
        &serve->PID_RightAngle,
        serve->ServeMotors->motor_handle[1]->encoder.consequent_angle);
    PID_CalcPID(&serve->PID_RightAngle, &serve->PID_RightAngleParam);

    PID_SetPIDRef(&serve->PID_RightSpeed, serve->PID_RightAngle.output);
    PID_SetPIDFdb(
        &serve->PID_RightSpeed,
        serve->ServeMotors->motor_handle[1]->encoder.standard_speed);
    PID_CalcPID(&serve->PID_RightSpeed, &serve->PID_RightSpeedParam);

    Motor_SetMotorOutput(serve->ServeMotors->motor_handle[0],
                         PID_GetPIDOutput(&serve->PID_LeftSpeed));
    Motor_SetMotorOutput(serve->ServeMotors->motor_handle[1],
                         PID_GetPIDOutput(&serve->PID_RightSpeed));
												 
	Motor_SendMotorGroupOutput(serve->ServeMotors);
}


uint8_t Module_ServeIsAngle(void) {

  Module_ServeTypeDef *serve = Module_ServeGetPtr();

  if (serve->target_speed < 0) 
  {
    if (serve->current_angle < -180.0f) {
      serve->is_angle = 1;
      return 1;
    }
		else
			return 0;
  } 
  else if (serve->target_speed > 0) 
  {
    if (serve->current_angle > 130.0f) {
      serve->is_angle = 1;
      return 1;
    }
	else
		return 0;
  } 
  else 
  {
    serve->is_angle = 0;
    return 0;
  }
}

void Module_ServeAnglePIDCal(void) {
	Module_ServeTypeDef *serve = Module_ServeGetPtr();
	
	PID_SetPIDRef(&serve->PID_LeftAngle, serve->target_angle);
    PID_SetPIDFdb(
        &serve->PID_LeftAngle,
        serve->ServeMotors->motor_handle[0]->encoder.consequent_angle);
    PID_CalcPID(&serve->PID_LeftAngle, &serve->PID_LeftAngleParam);

    PID_SetPIDRef(&serve->PID_LeftSpeed, serve->PID_LeftAngle.output);
    PID_SetPIDFdb(
        &serve->PID_LeftSpeed,
        serve->ServeMotors->motor_handle[0]->encoder.standard_speed);
    PID_CalcPID(&serve->PID_LeftSpeed, &serve->PID_LeftSpeedParam);

    PID_SetPIDRef(&serve->PID_RightAngle, -serve->target_angle);
    PID_SetPIDFdb(
        &serve->PID_RightAngle,
        serve->ServeMotors->motor_handle[1]->encoder.consequent_angle);
    PID_CalcPID(&serve->PID_RightAngle, &serve->PID_RightAngleParam);

    PID_SetPIDRef(&serve->PID_RightSpeed, serve->PID_RightAngle.output);
    PID_SetPIDFdb(
        &serve->PID_RightSpeed,
        serve->ServeMotors->motor_handle[1]->encoder.standard_speed);
    PID_CalcPID(&serve->PID_RightSpeed, &serve->PID_RightSpeedParam);
	
	
}

void Module_ServeSpdPIDCal(void) {
	Module_ServeTypeDef *serve = Module_ServeGetPtr();
	
	PID_SetPIDRef(&serve->PID_LeftSpeed, serve->target_speed);
    PID_SetPIDFdb(
        &serve->PID_LeftSpeed,
        serve->ServeMotors->motor_handle[0]->encoder.standard_speed);
    PID_CalcPID(&serve->PID_LeftSpeed, &serve->PID_LeftSpeedParam);

    PID_SetPIDRef(&serve->PID_RightSpeed, -serve->target_speed);
    PID_SetPIDFdb(
        &serve->PID_RightSpeed,
        serve->ServeMotors->motor_handle[1]->encoder.standard_speed);
    PID_CalcPID(&serve->PID_RightSpeed, &serve->PID_RightSpeedParam);

}
