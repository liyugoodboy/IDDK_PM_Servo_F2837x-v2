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
// DATE: Mar, 2017
//
//###########################################################################
#ifndef PM_PTO_QEPDIV_INCLUDE_H_
#define PM_PTO_QEPDIV_INCLUDE_H_

//Library of TamagawaT functions
#include <stdint.h>
#include "F28x_Project.h"

extern void PM_pto_qepdiv_setupPeriph(void);
extern void PM_pto_qepdiv_reset(void);
extern uint16_t PM_pto_qepdiv_config(uint16_t Divider, uint16_t IndexWidth);
extern void PM_pto_qepdiv_startOperation(uint16_t run);

#endif
