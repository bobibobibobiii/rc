#include "module_chassis.h"
#include "app_chassis.h"
#include "util_can.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "cmsis_os.h"
#include "periph_dt35.h"
#include "alg_math.h" 
#include "periph_motor.h" 

Chassis_DataTypeDef Chassis_Data;

Chassis_DataTypeDef* Chassis_GetChassisPtr() {
    return &Chassis_Data;
}
void Chassis_Init() {
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	
	Chassis->DirectionMotors = Motor_groupHandle[3];
	Chassis->MoveMotors      = Motor_groupHandle[4];
		
	Chassis->Chassis_CtrlMode = Chassis_Remote;    
	Chassis->Chassis_State = Chassis_Stop;
	Chassis->vx = 0;
	Chassis->vy = 0;
	Chassis->vw = 0;
	Chassis->theta_ref = 0;
	Chassis->theta_now = 0; 
	Chassis->theta_last = 0; 

	PID_InitPIDParam(&Chassis->pid.Location_Distance_PIDParam,Const_Chassis_Location_Distance_Param[0][0],Const_Chassis_Location_Distance_Param[0][1],Const_Chassis_Location_Distance_Param[0][2],Const_Chassis_Location_Distance_Param[0][3],Const_Chassis_Location_Distance_Param[0][4],
															  Const_Chassis_Location_Distance_Param[1][0],Const_Chassis_Location_Distance_Param[1][1],
															  Const_Chassis_Location_Distance_Param[2][0],Const_Chassis_Location_Distance_Param[2][1],
															  Const_Chassis_Location_Distance_Param[3][0],Const_Chassis_Location_Distance_Param[3][1],PID_POSITION);
	PID_InitPIDParam(&Chassis->pid.Location_Anglew_PIDParam,Const_Chassis_Location_Anglew_Param[0][0],Const_Chassis_Location_Anglew_Param[0][1],Const_Chassis_Location_Anglew_Param[0][2],Const_Chassis_Location_Anglew_Param[0][3],Const_Chassis_Location_Anglew_Param[0][4],
															Const_Chassis_Location_Anglew_Param[1][0],Const_Chassis_Location_Anglew_Param[1][1],
															Const_Chassis_Location_Anglew_Param[2][0],Const_Chassis_Location_Anglew_Param[2][1],
															Const_Chassis_Location_Anglew_Param[3][0],Const_Chassis_Location_Anglew_Param[3][1],PID_POSITION);

	for(int i = 0; i < 4; i ++)
	{
		// steering angle pid
		PID_InitPIDParam(&Chassis->pid.SteeringWheel_Direction_AngleParamPID[i],Const_Chassis_Steer_Ang_Param[0][0],Const_Chassis_Steer_Ang_Param[0][1],Const_Chassis_Steer_Ang_Param[0][2],Const_Chassis_Steer_Ang_Param[0][3],
											Const_Chassis_Steer_Ang_Param[0][4],Const_Chassis_Steer_Ang_Param[1][0],Const_Chassis_Steer_Ang_Param[1][1],Const_Chassis_Steer_Ang_Param[2][0],Const_Chassis_Steer_Ang_Param[2][1],
											Const_Chassis_Steer_Ang_Param[3][0],Const_Chassis_Steer_Ang_Param[3][1], PID_POSITION);
		
		// steering speed pid
		PID_InitPIDParam(&Chassis->pid.SteeringWheel_Direction_SpeedParamPID[i], Const_Chassis_Steer_Spd_Param[0][0], Const_Chassis_Steer_Spd_Param[0][1], Const_Chassis_Steer_Spd_Param[0][2], Const_Chassis_Steer_Spd_Param[0][3],
											Const_Chassis_Steer_Spd_Param[0][4], Const_Chassis_Steer_Spd_Param[1][0], Const_Chassis_Steer_Spd_Param[1][1], Const_Chassis_Steer_Spd_Param[2][0], Const_Chassis_Steer_Spd_Param[2][1],
											Const_Chassis_Steer_Spd_Param[3][0],Const_Chassis_Steer_Spd_Param[3][1], PID_POSITION);
		
		// wheel speed pid
		PID_InitPIDParam(&Chassis->pid.SteeringWheel_Move_ParamPID[i], Const_Chassis_Wheel_Spd_Param[0][0], Const_Chassis_Wheel_Spd_Param[0][1], Const_Chassis_Wheel_Spd_Param[0][2], Const_Chassis_Wheel_Spd_Param[0][3],
											Const_Chassis_Wheel_Spd_Param[0][4], Const_Chassis_Wheel_Spd_Param[1][0], Const_Chassis_Wheel_Spd_Param[1][1], Const_Chassis_Wheel_Spd_Param[2][0], Const_Chassis_Wheel_Spd_Param[2][1],
											Const_Chassis_Wheel_Spd_Param[3][0],Const_Chassis_Wheel_Spd_Param[3][1], PID_POSITION);
	}		
	
	Chassis->update_dt = 0;
	Chassis->last_update_tick = DWT_GetTimeline_us();
}

void Chassis_Set_Speed(uint8_t state,float vx,float vy,float vw,float r) {  //r 0~100 mean low filter paramter
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	
	Chassis->Chassis_State = state;
	LimitMaxMin(vx,7,-7);
	LimitMaxMin(vy,7,-7);
	LimitMaxMin(vw,7,-7);
	
	Chassis->theta_last = Chassis->theta_now;
	if(vx==0&&vy==0) {}
		
	else if(vx==0 && vy>0) {
		Chassis->theta_ref = PI / 2.0f;
	}
	else if(vx==0 && vy<0) {
		Chassis->theta_ref = 3 * PI / 2.0f;
	}
	else if(vx>0) {
		Chassis->theta_ref = atanf(vy / vx);
	}
	else if(vx<0) {
		Chassis->theta_ref = atanf(vy / vx) + PI;
	}
	int16_t sign = 1 ;
	float temp_theta_error = Chassis->theta_ref -Chassis->theta_now;
	while (temp_theta_error > PI/2.0f)
	{
		temp_theta_error -= PI;
		sign *= -1;
	}
	while (temp_theta_error < -PI/2.0f)
	{
		temp_theta_error += PI;
		sign *= -1;
	} 
	Chassis->theta_now = r / 100.0f * ( temp_theta_error + Chassis->theta_last ) + (100 - r) / 100.0f*(Chassis->theta_last);	     
	Chassis->vx = sign*cos(Chassis->theta_now) *sqrt(vx*vx + vy*vy);
	Chassis->vy = sign*sin(Chassis->theta_now) *sqrt(vx*vx + vy*vy);	
	Chassis->vw = vw;
}
void Chassis_SetControlMode(uint8_t mode) {
	
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	Chassis->Chassis_CtrlMode = mode;
}

void Chassis_SetState(Chassis_StateEnum state) {
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	Chassis->Chassis_State = state;
}

void Chassis_Stepback(float distance,float w_angle)
{
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();	
	float vy,vw;
	DT35_DataTypeDef *DT35 = &DT35_Data; 
	
	PID_SetPIDRef(&Chassis->pid.Location_Distance_PID,distance );
	PID_SetPIDFdb(&Chassis->pid.Location_Distance_PID, DT35->distance );
	
	if(fabsf(distance-DT35->distance) >= 3) {
		PID_CalcPID(&Chassis->pid.Location_Distance_PID, &Chassis->pid.Location_Distance_PIDParam);
		vy = PID_GetPIDOutput(&Chassis->pid.Location_Distance_PID);
	}
	else{vy=0;}

  PID_SetPIDRef(&Chassis->pid.Location_Anglew_PID,  w_angle);
  PID_SetPIDFdb(&Chassis->pid.Location_Anglew_PID, DT35->w_angle);
	if(fabsf(w_angle-DT35->w_angle) >= 1){
  PID_CalcPID(&Chassis->pid.Location_Anglew_PID, &Chassis ->pid.Location_Anglew_PIDParam);
  vw = PID_GetPIDOutput(&Chassis->pid.Location_Anglew_PID);
  }
	else{vw=0;}
	if(DT35->state == 1){
 Chassis_Set_Speed(Chassis_Run ,0,-vy,vw,0);}
	else{Chassis_Set_Speed(Chassis_Run ,0,0,0,0);}
}

void Chassis_Control(){
Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
Chassis->update_dt = DWT_GetDeltaT(&Chassis->last_update_tick);
switch(Chassis->Chassis_CtrlMode){
	case Chassis_Remote: 
		break;
	case Chassis_PC : 
		break;
	case Chassis_Location:
		
		 Chassis_Stepback(163,0);
		break;

}

};
void Chassis_CalEachMotorRef()
{
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	// 1     0
	//
	// 2     3
    float v0x,v0y,v1x,v1y,v2x,v2y,v3x,v3y;

    v0x =Chassis->vx -Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
    v0y =Chassis->vy +Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
			  
    v1x =Chassis->vx -Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
    v1y =Chassis->vy -Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
			  
    v2x =Chassis->vx +Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
    v2y =Chassis->vy -Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
			  
    v3x =Chassis->vx +Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;
    v3y =Chassis->vy +Chassis->vw * Const_Chassis_Steeringwheel_Radius / 1.414;			   	
	
    Chassis->v_move_ref[0] = sqrt(v0x * v0x + v0y * v0y) / 0.06;
    Chassis->v_move_ref[1] = sqrt(v1x * v1x + v1y * v1y) / 0.06;
    Chassis->v_move_ref[2] = sqrt(v2x * v2x + v2y * v2y) / 0.06;
    Chassis->v_move_ref[3] = sqrt(v3x * v3x + v3y * v3y) / 0.06;
		
    if (fabs(Chassis->vx) <= 0.01 && fabs(Chassis->vy) <= 0.01 &&Chassis->vw == 0) {
        switch (Chassis->Chassis_State)
        {
			case Chassis_Lock:
			   Chassis->a_direction_ref[0] = -45.0f;
			   Chassis->a_direction_ref[1] =  45.0f;
			   Chassis->a_direction_ref[2] = -45.0f;
			   Chassis->a_direction_ref[3] =  45.0f;
				break;
			case Chassis_Stop:
				Chassis->a_direction_ref[0] = 0.0f;
				Chassis->a_direction_ref[1] = 0.0f;
				Chassis->a_direction_ref[2] = 0.0f;
				Chassis->a_direction_ref[3] = 0.0f;
			
			case Chassis_Run:
				break;
			default:
				break;
        }
    }
    else {
       Chassis->a_direction_ref[0] = atan2(v0y, v0x) * 57.32f;
       Chassis->a_direction_ref[1] = atan2(v1y, v1x) * 57.32f;
       Chassis->a_direction_ref[2] = atan2(v2y, v2x) * 57.32f;
       Chassis->a_direction_ref[3] = atan2(v3y, v3x) * 57.32f;
	}
}

void Chassis_SteeringWheel_ControlMove()
{
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
    for (int i = 0; i < 4; i++) {
        float temp_DirectionMotor_angle_error = Chassis->a_direction_ref[i] - ( Chassis->DirectionMotors->motor_handle[i]->encoder.limited_angle );
        while (temp_DirectionMotor_angle_error > 90.0f) {
            temp_DirectionMotor_angle_error -= 180.0f;
        }
        while (temp_DirectionMotor_angle_error < -90.0f) {
            temp_DirectionMotor_angle_error += 180.0f;
        }

		float ref_DirectionMotor_angle = Chassis->DirectionMotors->motor_handle[i]->encoder.limited_angle + temp_DirectionMotor_angle_error;
		float ref_MoveMotor_speed = Chassis->v_move_ref[i];
		float diff = fabs(ref_DirectionMotor_angle -  Chassis->a_direction_ref[i]);
     
		if (diff > 180) {			 
            diff = 360 - diff;
        }
        if (diff > 90.0) {
            ref_MoveMotor_speed *= -1;
        }			
				
        PID_SetPIDRef(&Chassis->pid.SteeringWheel_Move_PID[i], ref_MoveMotor_speed);
		//PID_SetPIDRef(&Chassis->pid.SteeringWheel_Move_PID[i], 0);
        PID_SetPIDFdb(&Chassis->pid.SteeringWheel_Move_PID[i], Chassis->MoveMotors->motor_handle[i]->encoder.standard_speed);
        PID_CalcPID(&Chassis->pid.SteeringWheel_Move_PID[i], &Chassis->pid.SteeringWheel_Move_ParamPID[i]);

        PID_SetPIDRef(&Chassis->pid.SteeringWheel_Direction_AnglePID[i], ref_DirectionMotor_angle);
        PID_SetPIDFdb(&Chassis->pid.SteeringWheel_Direction_AnglePID[i], Chassis->DirectionMotors->motor_handle[i]->encoder.limited_angle);
        PID_CalcPID(&Chassis->pid.SteeringWheel_Direction_AnglePID[i], &Chassis->pid.SteeringWheel_Direction_AngleParamPID[i]);

        //PID_SetPIDRef(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], 0);
		PID_SetPIDRef(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], PID_GetPIDOutput(&Chassis->pid.SteeringWheel_Direction_AnglePID[i]));
        PID_SetPIDFdb(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], Chassis->DirectionMotors->motor_handle[i]->encoder.speed);
        PID_CalcPID(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], &Chassis->pid.SteeringWheel_Direction_SpeedParamPID[i]);
    }

    for(int i = 0; i <= 3; i++) {
        Motor_SetMotorOutput(Chassis->DirectionMotors->motor_handle[i], -PID_GetPIDOutput(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i]));
		//Motor_SetMotorOutput(Chassis->DirectionMotors->motor_handle[i], 0);
        Motor_SetMotorOutput(Chassis->MoveMotors->motor_handle[i], PID_GetPIDOutput(&Chassis->pid.SteeringWheel_Move_PID[i]));
		//Motor_SetMotorOutput(Chassis->MoveMotors->motor_handle[i], 0);
	}
	
	//Motor_SendMotorGroupOutput(Chassis->DirectionMotors);
	//Motor_SendMotorGroupOutput(Chassis->MoveMotors);
}


void Chassis_SteeringWheel_ControlMove_Test(void) {
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	
	Chassis->a_direction_ref[0] += Const_DIRECTION_RIGHT_FRONT_MOTOR_INIT_OFFSET_TEST;
    Chassis->a_direction_ref[1] += Const_DIRECTION_LEFT_FRONT_MOTOR_INIT_OFFSET_TEST;
    Chassis->a_direction_ref[2] += Const_DIRECTION_LEFT_BACK_MOTOR_INIT_OFFSET_TEST;
    Chassis->a_direction_ref[3] += Const_DIRECTION_RIGHT_BACK_MOTOR_INIT_OFFSET_TEST;
	
	for (int i = 0; i < 4; i++) {
        float temp_DirectionMotor_angle_error = Chassis->a_direction_ref[i] - ( Chassis->DirectionMotors->motor_handle[i]->encoder.consequent_angle);
        while (temp_DirectionMotor_angle_error > 90.0f) {
            temp_DirectionMotor_angle_error -= 180.0f;
        }
        while (temp_DirectionMotor_angle_error < -90.0f) {
            temp_DirectionMotor_angle_error += 180.0f;
        }

		float ref_DirectionMotor_angle = Chassis->DirectionMotors->motor_handle[i]->encoder.limited_angle + temp_DirectionMotor_angle_error;
		float ref_MoveMotor_speed = Chassis->v_move_ref[i];
		float diff = fabs(ref_DirectionMotor_angle -  Chassis->a_direction_ref[i]);
     
		if (diff > 180) {			 
            diff = 360 - diff;
        }
        if (diff > 90.0) {
            ref_MoveMotor_speed *= -1;
        }			
				
        PID_SetPIDRef(&Chassis->pid.SteeringWheel_Move_PID[i], ref_MoveMotor_speed);
		//PID_SetPIDRef(&Chassis->pid.SteeringWheel_Move_PID[i], 0);
        PID_SetPIDFdb(&Chassis->pid.SteeringWheel_Move_PID[i], Chassis->MoveMotors->motor_handle[i]->encoder.standard_speed);
        PID_CalcPID(&Chassis->pid.SteeringWheel_Move_PID[i], &Chassis->pid.SteeringWheel_Move_ParamPID[i]);

        PID_SetPIDRef(&Chassis->pid.SteeringWheel_Direction_AnglePID[i], ref_DirectionMotor_angle);
        PID_SetPIDFdb(&Chassis->pid.SteeringWheel_Direction_AnglePID[i], Chassis->DirectionMotors->motor_handle[i]->encoder.limited_angle);
        PID_CalcPID(&Chassis->pid.SteeringWheel_Direction_AnglePID[i], &Chassis->pid.SteeringWheel_Direction_AngleParamPID[i]);

        //PID_SetPIDRef(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], 0);
		PID_SetPIDRef(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], PID_GetPIDOutput(&Chassis->pid.SteeringWheel_Direction_AnglePID[i]));
        PID_SetPIDFdb(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], Chassis->DirectionMotors->motor_handle[i]->encoder.speed);
        PID_CalcPID(&Chassis->pid.SteeringWheel_Direction_SpeedPID[i], &Chassis->pid.SteeringWheel_Direction_SpeedParamPID[i]);
    }
	
	
}
