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
// FILE:   PM_endat22_Include.h
//
// TITLE:  Prototypes and Definitions for the Position Manager Endat22 Library
//
// AUTHOR: Subrahmanya Bharathi Akondy (C2000 Systems Solutions, Houston , TX)
//
// DATE: Nov, 2015
//
//###########################################################################

//Library of EnDat functions
#include <stdint.h>
#include "F28x_Project.h"

#define ENDAT22	1
#define ENDAT21	0

#define ENDAT_CRC_LENGTH			5
#define ENDAT_CRC_POLYNOMIAL		0x0B

#define ENCODER_SEND_POSITION_VALUES	0x7		//0b000111
#define SELECTION_OF_MEMORY_AREA		0xE		//0b001110
#define ENCODER_RECEIVE_PARAMETER		0x1C	//0b011100
#define ENCODER_SEND_PARAMETER			0x23	//0b100011
#define ENCODER_RECEIVE_RESET			0x2A	//0b101010
#define ENCODER_SEND_TEST_VALUES		0x15	//0b010101
#define	ENCODER_RECEIVE_TEST_COMMAND	0x31	//0b110001
#define ENCODER_SEND_POSITION_VALUES_WITH_ADDITIONAL_DATA				0x38	//0b111000
#define ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA	0x9		//0b001001
#define ENCODER_SEND_POSITION_VALUES_AND_RECEIVE_PARAMETER				0x1B	//0b011011
#define ENCODER_SEND_POSITION_VALUES_AND_SEND_PARAMETER					0x24	//0b100100
#define ENCODER_SEND_POSITION_VALUES_AND_RECEIVE_TEST_COMMAND			0x36	//0b110110
#define ENCODER_SEND_POSITION_VALUES_AND_RECEIVE_ERROR_RESET			0x2D	//0b101101
#define ENCODER_RECEIVE_COMMUNICATION_COMMAND							0x12	//0b010010


typedef struct  {                                 // bit descriptions
    uint32_t  position_lo;
    uint32_t  position_hi;
    uint16_t  error1;
    uint16_t  error2;
    uint16_t  data;
    uint16_t  data_crc;
    uint16_t  address;
    uint32_t  additional_data1;
    uint32_t  additional_data2;
    uint32_t  additional_data1_crc;
    uint32_t  additional_data2_crc;
    uint32_t  test_lo;
    uint32_t  test_hi;
    uint32_t  position_clocks;
    volatile struct SPI_REGS *spi;
    uint32_t  delay_comp;
    uint32_t  sdata[16];     // Send data buffer
    uint32_t  rdata[16];     // Receive data buffer
    uint16_t  dataReady;
    uint16_t  fifo_level;

} ENDAT_DATA_STRUCT;

extern ENDAT_DATA_STRUCT endat22Data;

extern uint16_t PM_endat22_setupCommand(uint16_t, uint16_t data1, uint16_t data2, uint16_t nAddData);
extern uint16_t PM_endat22_receiveData(uint16_t, uint16_t nAddData);
extern void PM_endat22_setupPeriph(void);
extern void PM_endat22_setFreq(uint32_t Freq_us);
extern void PM_endat22_startOperation(void);
extern uint16_t PM_endat22_getDelayCompVal(void);


//*****************************************************************************
// CRC related declarations
//*****************************************************************************

#define NBITS_POLY1  5
#define POLY1        0x0B
#define RXLEN        4  //e.g. 100 bytes in the message
#define PARITY       0    //parity 0 means start at the low byte of the first 16-bit address of the input
#define SIZEOF_ENDAT_CRCTABLE  256

//*****************************************************************************
// globals
//*****************************************************************************
extern uint16_t endat22CRCtable[SIZEOF_ENDAT_CRCTABLE];

//*****************************************************************************
// prototypes
//*****************************************************************************
extern void PM_endat22_generateCRCTable(uint16_t nBits, uint16_t polynomial, uint16_t *pTable);
extern uint32_t PM_endat22_getCrcPos(uint32_t total_clocks,uint32_t endat22,uint32_t lowpos,uint32_t highpos, uint32_t error1,uint32_t error2, uint16_t *crc_table);
extern uint32_t PM_endat22_getCrcNorm (uint32_t param8,uint32_t param16, uint16_t *crc_table);
extern uint32_t PM_endat22_getCrcTest(uint32_t lowtest,uint32_t hightest, uint32_t error1, uint16_t *crc_table);
