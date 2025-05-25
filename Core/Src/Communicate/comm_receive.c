/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : comm_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-10-04 00:47:42
 */


#include "comm_common.h"
#include "comm_receive.h"

#include "stdlib.h"
#include "math.h"
#include "lib_buff.h"
#include "periph_remote.h"
#include "module_platform.h"
#include "app_gimbal.h"
 #include "module_chassis.h"

static uint32_t _set_Platform_Data_(uint8_t *buff) ;
//static uint32_t _set_Gimbal_Data_(uint8_t *buff) ;
static uint32_t _set_Chassis_Data_(uint8_t *buff);

Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE] = {
	{&_set_Platform_Data_    },
	{&_set_Chassis_Data_     }
};


static uint32_t _set_Platform_Data_(uint8_t *buff) {
	
	Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	if(Platform->ctrl_mode == Platform_Jiefa){
		Platform_Jiefa_Cal(buff2float(buff));
	}
	if(Platform->ctrl_mode == Platform_Dianqiu){
		Platform_Dianqiu_Cal(buff2float(buff));
	}

	return 4;

}


static uint32_t _set_Gimbal_Data_(uint8_t *buff) 
{
	
	Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();
	
	gimbal->Pole_Yaw_ref = buff2float(buff);
	gimbal->Pole_Pitch_ref = buff2float(buff + 4);

	Gimbal_SetPos();
	
	return 8;
}
static uint32_t _set_Chassis_Data_(uint8_t *buff)
{
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
  Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	
 if(Chassis->Chassis_CtrlMode == Chassis_PC){   
        float x =  buff2float(buff);
        float y =  buff2float(buff + 4);
        float w =  buff2float(buff + 8);        
        int state = (int)buff2float(buff + 12);

        LimitMaxMin(x,3.6,-3.6);
        LimitMaxMin(y,3.6,-3.6);
        LimitMaxMin(w,0.6,-0.6);
        if( state == 0 )                                   //speed
        {                                        
     Chassis_Set_Speed(Chassis_Run,x,y,w,0); 
        }  
        else if(state == 1)                                //position
        {
    Chassis_Set_Speed(Chassis_Stop,0,0,0,0); 
        }
				else if(state == 2)                        //position
        {
    Chassis_Set_Speed(Chassis_Lock,0,0,0,0); 
        }
			}
    return 16;        
}