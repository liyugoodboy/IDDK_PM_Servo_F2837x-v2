//###########################################################################
//  This software is licensed for use with Texas Instruments C28x
//  family Microcontrollers.  This license was provided to you prior to installing
//  the software.  You may review this license by consulting a copy of
//  the agreement in the doc directory of this library.
// ------------------------------------------------------------------------
//          Copyright (C) 2015 Texas Instruments, Incorporated.
//                          All Rights Reserved.
// ==========================================================================
//
// FILE:   PM_tformat_Include.h
//
// TITLE:  Prototypes and Definitions for the Position Manager Tamagawa-T Library
//
// AUTHOR: Subrahmanya Bharathi Akondy (C2000 Systems Solutions, Houston , TX)
//
// DATE: Jun, 2015
//
//###########################################################################
#ifndef PM_PTO_PULSEGEN_INCLUDE_H_
#define PM_PTO_PULSEGEN_INCLUDE_H_

//Library of TamagawaT functions
#include <stdint.h>
#include "F28x_Project.h"

extern void PM_pto_pulsegen_setupPeriph(void);
extern void PM_pto_pulsegen_reset(void);
extern uint16_t PM_pto_pulsegen_runPulseGen(
		uint32_t PulseLo,
		uint32_t PulseHi,
		uint32_t ptoActivePeriod,
		uint32_t ptoFullPeriod,
		uint32_t ptoInterruptTime,
		uint16_t ptoDirection,
		uint16_t run
		);
extern void PM_pto_pulsegen_startOperation(void);

#endif
