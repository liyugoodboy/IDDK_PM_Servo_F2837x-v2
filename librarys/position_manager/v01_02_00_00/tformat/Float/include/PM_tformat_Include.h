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
// TITLE:  Prototypes and Definitions for the Position Manager T-format Library
//
// AUTHOR: Subrahmanya Bharathi Akondy (C2000 Systems Solutions, Houston , TX)
//
// DATE: Jun, 2015
//
//###########################################################################

//Library of T-format functions
#include <stdint.h>
#include "F28x_Project.h"

#define TFORMAT_CRC_LENGTH			8
#define TFORMAT_CRC_POLYNOMIAL		0x01

#define DATAID0	0x0			//0b00000
#define DATAID1	0x11		//0b10001
#define DATAID2	0x9			//0b01001
#define DATAID3	0x18		//0b11000
#define DATAID6	0xC			//0b01100
#define DATAIDD	0x17		//0b10111
#define DATAID7	0x1D		//0b11101
#define DATAID8	0x3			//0b00011
#define DATAIDC	0x6			//0b00110


typedef struct  {                                 // bit descriptions
    uint16_t  controlField;
    uint16_t  statusField;
    uint16_t  dataField0;
    uint16_t  dataField1;
    uint16_t  dataField2;
    uint16_t  dataField3;
    uint16_t  dataField4;
    uint16_t  dataField5;
    uint16_t  dataField6;
    uint16_t  dataField7;
    uint16_t  crc;
    uint16_t  eepromAddress;
    uint16_t  eepromWrDtata;
    uint16_t  eepromRdDtata;
    volatile struct SPI_REGS *spi;
    uint32_t  sdata[16];     // Send data buffer
    uint32_t  rdata[16];     // Receive data buffer
    uint16_t  dataReady;
    uint16_t  fifo_level;
    uint32_t  rxPkts[3];
} TFORMAT_DATA_STRUCT;

extern TFORMAT_DATA_STRUCT tformatData;

extern uint16_t PM_tformat_setupCommand(uint16_t dataID, uint16_t eepromAddr, uint16_t eepromData, uint16_t crc);
extern uint16_t PM_tformat_receiveData(uint16_t dataID);
extern void PM_tformat_setupPeriph(void);
extern void PM_tformat_setFreq(uint32_t Freq_us);
extern void PM_tformat_startOperation(void);

//*****************************************************************************
// CRC related declarations
//*****************************************************************************

#define NBITS_POLY1  8
#define POLY1        0x01
#define RXLEN        4  //e.g. 100 bytes in the message
#define PARITY       0    //parity 0 means start at the low byte of the first 16-bit address of the input
#define SIZEOF_TFORMAT_CRCTABLE  256

//*****************************************************************************
// globals
//*****************************************************************************
extern uint16_t tformatCRCtable[SIZEOF_TFORMAT_CRCTABLE];

//*****************************************************************************
// prototypes
//*****************************************************************************
extern void PM_tformat_generateCRCTable(uint16_t nBits, uint16_t polynomial, uint16_t *pTable);
extern uint16_t PM_tformat_getCRC(uint16_t input_crc_accum, uint16_t nBitsData,  uint16_t nBitsPoly, uint16_t * msg, uint16_t *crc_table, uint16_t rxLen);
