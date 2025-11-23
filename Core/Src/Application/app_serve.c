#include "app_serve.h"
#include "app_remote.h"
#include "periph_motor.h"
#include "periph_remote.h"
#include "util_can.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "alg_math.h"
#include "cmsis_os.h"
#include "main.h"
#include "math.h"
#include "gpio.h"
#include <stdio.h>

Serve_TypeDef Serve;

void Serve_Task(const void *argument) {
  Serve_TypeDef *serve = Serve_GetPtr();
  for (;;) {
		Serve_ModeUpdate();
		Serve_Control();
		//Module_ServePIDTest();
		osDelay(100);
  }
}

void Serve_Init(void) {
  Serve_TypeDef *serve = Serve_GetPtr();

  serve->mode = Serve_Wait;
  serve->module_serve = Module_ServeGetPtr();
  //Motor_groupHandle[1]->motor_handle[0]->encoder.init_offset = 9.394;
  //Motor_groupHandle[1]->motor_handle[1]->encoder.init_offset = 7.156;
}

Serve_TypeDef *Serve_GetPtr(void) { return &Serve; }

void Serve_ModeUpdate(void) {
  Serve_TypeDef *serve = Serve_GetPtr();
  Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
	
  serve->module_serve->current_speed = Motor_Serve_Motors.motor_handle[0]->encoder.standard_speed;
  serve->module_serve->current_angle = Motor_Serve_Motors.motor_handle[0]->encoder.consequent_angle;
  serve->module_serve->current_PoleTorque = Serve_FeedforwardMoment * sin(DEG_TO_RAD(Motor_Serve_Motors.motor_handle[0]->encoder.consequent_angle));

	if (data->remote.s[0] == Remote_SWITCH_MIDDLE) {
		switch(data->remote.s[1]) {
			case Remote_SWITCH_UP:
				serve->mode = Serve_Hit;
				break;
			case Remote_SWITCH_MIDDLE:
				serve->mode = Serve_Ready;
				break;
			default:
				break;
		}
	}
	else {serve->mode = Serve_Keep;}
}

void Serve_Control(void) {
	Serve_TypeDef *serve = Serve_GetPtr();
	
	switch(serve->mode) {
		case Serve_Hit:
			Module_ServeHit();
			break;
		case Serve_Ready:
			Module_ServeReady();
			break;
		case Serve_Keep:
			Module_ServeKeep();
			break;
		default:
			break;
	}
	
}
