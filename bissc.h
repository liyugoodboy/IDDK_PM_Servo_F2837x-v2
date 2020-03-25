//----------------------------------------------------------------------------------
//	FILE:			bissc.h
//
//	Description:	Contains all the initialization, data declarations and setup
//					for BiSS encoder interface. This file serves are a template for
//					using PM_bissc Library to interface and incorporates all the encoder
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
// 4 Nov 2015  - Example project for PM BiSS-C Library Usage
//----------------------------------------------------------------------------------

#include "PM_bissc_Include.h"


//--- Lika HS58S18/I7-P9-RM2 definitions ---
#define	BISSC_FREQ_DIVIDER 25
#define BISS_ENCODER_HAS_CD_INTERFACE	0

#define BISS_POSITION_BITS 	   24
#define BISS_POSITION_CRC_BITS 6
//--- Lika HS58S18/I7-P9-RM2 definitions ---


/*
//--- Kuebler 8.F5863.1426.C423 definitions ---
#define	BISSC_FREQ_DIVIDER 25
#define BISS_ENCODER_HAS_CD_INTERFACE	0

#define BISS_POSITION_BITS 	   26
#define BISS_POSITION_CRC_BITS 6
//--- Kuebler 8.F5863.1426.C423 definitions ---
*/

#define	SPI_FIFO_WIDTH 12

#define BISS_SCD_CRC_SIZEOF_TABLE      256
#define BISS_SCD_CRC_NBITS_POLY1       6
#define BISS_SCD_CRC_POLY1             0x03   //x^6 + x + 1 (inverted output) 1000011

#define BISS_CD_CRC_SIZEOF_TABLE       256
#define BISS_CD_CRC_NBITS_POLY1 	   4
#define BISS_CD_CRC_POLY1              0x03   //x^4 + x + 1 (inverted output) 10011

#define BISS_DATA_CLOCKS 	   BISS_POSITION_BITS+BISS_POSITION_CRC_BITS+4  //4=ST+CDS+E+W

extern uint16_t bissCRCtableSCD[BISS_SCD_CRC_SIZEOF_TABLE];	// Declare CRC table for BiSS-C CRC calculations

#if BISS_ENCODER_HAS_CD_INTERFACE
extern uint16_t bissCRCtableCD[BISS_CD_CRC_SIZEOF_TABLE];
#endif

extern void bissc_init(void);
extern uint16_t bissc_receivePosition(uint16_t positionBits, uint16_t crcBits);
extern uint16_t bissc_doCDTasks(void);
extern interrupt void bissc_spiRxFifoIsr(void);
extern void bissc_setupGPIO(void);
extern void bissc_configXBAR(void);
extern void bissc_configEPWM4(void);
extern void bissc_readPosition(void);





