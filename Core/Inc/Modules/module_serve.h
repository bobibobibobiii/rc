#ifndef __MODULE_SERVE_H__
#define __MODULE_SERVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "sys_const.h"
#include "alg_pid.h"
#include "main.h"

typedef struct {
    uint8_t init;
    uint8_t is_angle;

    float target_angle;
    float current_angle;
    float target_speed;
    float current_speed;
	float current_PoleTorque;
	
	Motor_MotorGroupTypeDef* ServeMotors;

    PID_PIDTypeDef PID_RightAngle;
    PID_PIDParamTypeDef PID_RightAngleParam;

    PID_PIDTypeDef PID_LeftAngle;
    PID_PIDParamTypeDef PID_LeftAngleParam;

    PID_PIDTypeDef PID_RightSpeed;
    PID_PIDParamTypeDef PID_RightSpeedParam;

    PID_PIDTypeDef PID_LeftSpeed;
    PID_PIDParamTypeDef PID_LeftSpeedParam;

} Module_ServeTypeDef;

void Module_ServeInit(void);
Module_ServeTypeDef* Module_ServeGetPtr(void);
void Module_ServeReady(void);
void Module_ServeHit(void);
void Module_ServeKeep(void);
void Module_ServeAnglePIDCal(void);
void Module_ServeSpdPIDCal(void);
uint8_t Module_ServeIsAngle(void);

#ifdef __cplusplus
}
#endif

#endif
