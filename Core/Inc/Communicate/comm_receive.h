/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_receive.h
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:43
 *  LastEditTime : 2023-08-09 00:24:32
 */


#ifndef COMM_RECEIVE_H
#define COMM_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

#define Const_Comm_Receive_BUFF_SIZE 2


typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Comm_ReceiveEntry;

extern Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE];


#endif

#ifdef __cplusplus
}
#endif
