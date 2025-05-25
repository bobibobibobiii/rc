#ifndef __APP_SERVE_H__
#define __APP_SERVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "module_serve.h"
#include "main.h"

typedef enum {
    Serve_Ready = 1,
	Serve_Wait = 2,
    Serve_Hit = 3,
	Serve_Keep = 4
} Serve_Mode;

typedef struct {
    uint8_t init;
    Serve_Mode mode;
	Module_ServeTypeDef *module_serve;
	
} Serve_TypeDef;

void Serve_Init(void);
Serve_TypeDef* Serve_GetPtr(void);
void Serve_ModeUpdate(void);
void Serve_Control(void);


#ifdef __cpluscplus
}
#endif

#endif
