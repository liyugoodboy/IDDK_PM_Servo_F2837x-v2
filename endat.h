//----------------------------------------------------------------------------------
//	FILE:			endat.h
//
//	Description:	Contains all the initialization, data declarations and setup
//					for EnDat encoder interface. This file serves are a template for
//					using PM_endat22 Library to interface and incorporates all the encoder
//					and library specific initializations and other important aspects of usage.
//
//	Version: 		1.0
//
//  Target:  		TMS320F28377D,
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 3 Nov 2015  - Example project for PM EnDat22 Library Usage
//----------------------------------------------------------------------------------

#include "PM_endat22_Include.h"

//This parameter defines the type of encoder as
// 22 - for endat 22 encoder
// 21 - for endat 21 encoder
// Tests may fail if this variable is improperly set
#define ENCODER_TYPE	22

#define ENDAT_RUNTIME_FREQ_DIVIDER	6
#define ENDAT_INIT_FREQ_DIVIDER	250
//Endat Initial Clock Frequency = SYSCLK/(4*ENDAT_INIT_FREQ_DIVIDER) - used during initialization (~200KHz)
//Endat Clock Frequency = SYSCLK/(4*ENDAT_FREQ_DIVIDER)	- used during applicaiton (~8 MHz)
//Set ENDAT_FREQ_DIVIDER, ENDAT_INIT_FREQ_DIVIDER accordingly. Only even values greater than 6 are supported.

extern void EnDat_initDelayComp(void);
extern void Endat_setup_GPIO(void);
extern void Endat_config_XBAR(void);
extern uint16_t CheckCRC (uint16_t expectcrc5, uint16_t receivecrc5);
extern void EPWM4_Config(void);
extern void endat22_runCommandSet(void);
extern void endat21_runCommandSet(void);
extern void	endat21_readPosition();
extern void	endat22_readPositionWithAddlData();
extern void endat22_setupAddlData();


extern interrupt void spiRxFifoIsr(void);
extern void EnDat_Init(void);
extern void error();

