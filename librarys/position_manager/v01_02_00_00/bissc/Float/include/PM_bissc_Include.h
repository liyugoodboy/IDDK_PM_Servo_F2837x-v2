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
// FILE:   PM_bissc_Include.h
//
// TITLE:  Prototypes and Definitions for the Position Manager BiSS-C Library
//
// AUTHOR: Steven Brett Larimore (C2000 Systems Solutions, Houston, TX)
//
// DATE: Aug, 2015
//
//###########################################################################


#include <stdint.h>
#include "F28x_Project.h"


typedef struct  {                                 // bit descriptions
	uint16_t  cd_status;	// 0 - start new cd; 1 - cd transfer ongoing; 2 - cd rx'd; 3 - cd parsed & complete
	int16_t  remaining_cd_bits;
	uint16_t  cd_bits_to_send;

	uint32_t  cdm;

	uint32_t  cds_stream;
	uint16_t  cds_raw;      // cds without the crc

	uint16_t  cd_register_xfer_address;
	uint16_t  cd_register_xfer_rxdata;
	uint16_t  cd_register_xfer_txdata;
	uint16_t  cd_register_xfer_is_write;

	uint64_t  scd_raw;      //scd without the crc
	uint16_t  scd_crc;
	uint64_t  position;
    uint16_t  scd_error;
    uint16_t  scd_warning;
    uint16_t  crc_incorrect_count;

    uint16_t  dataReady;

    uint32_t sdata[16];     // Send data buffer
    uint32_t rdata[16];     // Receive data buffer
    uint16_t fifo_level;

    uint16_t xfer_address_withCTS;

    volatile struct SPI_REGS *spi;

} BISSC_DATA_STRUCT;

extern BISSC_DATA_STRUCT bissc_data_struct;

extern void PM_bissc_setFreq(uint32_t Freq_us);
extern void PM_bissc_startOperation(void);
extern void PM_bissc_setCDBit(uint32_t);
extern void PM_bissc_setupPeriph(void);
extern uint64_t PM_bissc_getBits (uint16_t len, uint16_t bitsParsed, uint16_t charBits);
extern void PM_bissc_generateCRCTable(uint16_t nBits, uint16_t polynomial, uint16_t *pTable);
extern uint16_t PM_bissc_getCRC(uint16_t input_crc_accum, uint16_t nBitsData,  uint16_t nBitsPoly, uint16_t * msg, uint16_t *crc_table, uint16_t rxLen);
extern void PM_bissc_setupNewSCDTransfer(uint16_t nDataClks, uint16_t spi_fifo_width);
