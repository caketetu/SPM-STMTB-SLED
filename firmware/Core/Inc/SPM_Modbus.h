/*
 * Mymodbus.h
 *
 *  Created on: Sep 16, 2025
 *      Author: t-fukumoto
 */

#ifndef __SPM_MODBUS_H_
#define __SPM_MODBUS_H_

#define PLATFORM_STM32C0

//Arduino IDEの時は必要
#ifdef PLATFORM_ESP32
  #include <Arduino.h>
  #include "FS.h"
  #include "SPIFFS.h"
  #define FORMAT_SPIFFS_IF_FAILED true
#endif

#define UNKOWN_FUNC 1
#define INVAILED_ADR 2
#define INVAILED_DATA 3

//Arduino IDEの時は不要
#ifdef PLATFORM_STM32
	#include "stm32f3xx_hal.h"
	//#include "stdio.h"
	//#include "string.h"
	#define	int16_t		signed short
	#define	uint16_t	unsigned short
	#define	uint8_t		unsigned char
	#define uint32_t	unsigned int
#endif

#ifdef PLATFORM_STM32C0
  #include "stm32c0xx_hal.h"
#endif

#define INPUT_REGS_MAX 64
#define HOLDING_REGS_MAX 64
#define COILS_MAX 16

typedef struct{
    int16_t tx_len;
    int16_t rx_len;
    int16_t *tx_adr[8];
    int16_t *rx_adr[8];
}sCycFunc;

extern int16_t *p_input_regs[INPUT_REGS_MAX];
extern uint8_t *p_coils[COILS_MAX/8];
extern int16_t *p_holding_regs[HOLDING_REGS_MAX];
extern sCycFunc cycfunc0;
extern sCycFunc cycfunc1;
extern sCycFunc cycfunc2;
extern sCycFunc cycfunc3;
extern sCycFunc cycfunc4;

void SPM_ModbusSetAddress(uint8_t dev_adr);
int SPM_ModbusTask(uint8_t *rbuf, int rl, uint8_t *s_buf);
void SPM_ModbusInit(void);
int SPM_ModbusParamLoad(void);
int ParamSave(void);
int ParamLoad(void);

#endif /* __SPM_MODBUS_H_ */