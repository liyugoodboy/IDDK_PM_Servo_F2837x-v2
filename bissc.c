//----------------------------------------------------------------------------------
//	FILE:			bissc.c
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

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "bissc.h"


uint16_t bissCRCtableSCD[BISS_SCD_CRC_SIZEOF_TABLE];	// Declare CRC table for BiSS CRC calculations

#if BISS_ENCODER_HAS_CD_INTERFACE
uint16_t bissCRCtableCD[BISS_CD_CRC_SIZEOF_TABLE];
#endif

BISSC_DATA_STRUCT bissc_data_struct;


void bissc_init(void)
{
	EALLOW;
	CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
	EDIS;

	//Configure EPWM4 to drive default values on GPIO6 and GPIO7
	bissc_configEPWM4();

	// Generate CRC tables for BiSS-C as defined in bissc.h
	PM_bissc_generateCRCTable(BISS_SCD_CRC_NBITS_POLY1, BISS_SCD_CRC_POLY1, bissCRCtableSCD);
	#if BISS_ENCODER_HAS_CD_INTERFACE
	PM_bissc_generateCRCTable(BISS_CD_CRC_NBITS_POLY1, BISS_CD_CRC_POLY1, bissCRCtableCD);
	#endif

	//GPIO and XBAR configuration for BiSS operation
	bissc_setupGPIO();
	bissc_configXBAR();

	bissc_data_struct.spi = &SpibRegs;
	PM_bissc_setupPeriph();


	EALLOW;
	PieVectTable.SPIB_RX_INT = &bissc_spiRxFifoIsr;
	EDIS;

	// Enable interrupts required for this example
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER6.bit.INTx3 = 1;     // Enable PIE Group 6, INT 9 (SPIB RX interrupt)

	IER |= M_INT6;
	EINT;


	// Biss-C TxEn to be driven low for IDDK
	GpioDataRegs.GPBDAT.bit.GPIO34 = 0;

	// Power up bissC 5v supply through GPIO32 (enable clock transciever)
	GpioDataRegs.GPBDAT.bit.GPIO32 = 1;

	DELAY_US(1000000L); 	//Delay 1s


	// BiSS Clock frequency will be 200/(4*n) MHz - for a device running @ 200MHz
	//PM_bissc_setFreq(200);		// 250kHz
	//PM_bissc_setFreq(50);		// 1MHz
	PM_bissc_setFreq(BISSC_FREQ_DIVIDER);			// set BISSC_FREQ_DIVIDER to 25 for 2MHz

	DELAY_US(2000L);

	bissc_data_struct.dataReady = 0;
	bissc_data_struct.cd_status = 3;
	bissc_data_struct.cd_register_xfer_is_write = 0;
	bissc_data_struct.cd_register_xfer_rxdata = 0xFFFF;
	bissc_data_struct.cd_register_xfer_txdata = 0xFFFF;
	bissc_data_struct.crc_incorrect_count = 0;
}

uint16_t bissc_receivePosition(uint16_t positionBits, uint16_t crcBits)
{
	uint16_t scdBitsParsed = 1;  //start with 1, we don't need to parse the start bit
	uint16_t crcResult = 0xFFFF;
	uint16_t mask = 0xFFFF;
	uint16_t receiveSuccessful = 0xFFFF;
	uint16_t numWordsPosCrc = 0;

	if(bissc_data_struct.cd_status == 1)
	{
		//if CD stream is active, receive CDS bits
		bissc_data_struct.cds_stream = bissc_data_struct.cds_stream << 1;
		bissc_data_struct.cds_stream |= PM_bissc_getBits(1, scdBitsParsed, SPI_FIFO_WIDTH);
	}
	scdBitsParsed++;  //we've now parsed the CDS bit (or didn't need to)

	//CRC is run on position + E + W, parse all these bits as one
	bissc_data_struct.scd_raw = PM_bissc_getBits(positionBits+2, scdBitsParsed, SPI_FIFO_WIDTH);
	scdBitsParsed = scdBitsParsed + positionBits + 2;  //positionBits + E + W

	bissc_data_struct.scd_crc = PM_bissc_getBits(crcBits, scdBitsParsed, SPI_FIFO_WIDTH);

	//change the amount of words the CRC will run based on the amount of bits
	numWordsPosCrc = ((positionBits+2)/8)+1;

	crcResult =  PM_bissc_getCRC(0, positionBits+2, BISS_SCD_CRC_NBITS_POLY1, (uint16_t *)&bissc_data_struct.scd_raw, bissCRCtableSCD, numWordsPosCrc);

	mask = (1 << crcBits) - 1;

	if(crcResult == ((~bissc_data_struct.scd_crc) & mask))
	{
		//crc was correct, so lets populate the following
		bissc_data_struct.scd_error = (~bissc_data_struct.scd_raw) & 0x00000001;
		bissc_data_struct.scd_warning = ((~bissc_data_struct.scd_raw) & 0x00000002) >> 1;
		bissc_data_struct.position = bissc_data_struct.scd_raw >> 2;
		receiveSuccessful = 1;
	}
	else
	{
		receiveSuccessful = 0;
		bissc_data_struct.crc_incorrect_count++;
	}

	return(receiveSuccessful);
}

#if BISS_ENCODER_HAS_CD_INTERFACE
uint16_t bissc_doCDTasks(void)
{
	uint16_t taskSuccessful = 0xFFFF;

	//the below code only utilizes the CD's register communication - control communication option (CTS=0 commands)
	// are not implemented

	//new transmission - ready it for sending
	switch(bissc_data_struct.cd_status)
	{
		uint32_t crcSelf;
		uint32_t rxdRWS;

		// new cdm request - fill cdm buffer
		case 0:

			//CRC is generated based on CTS + register
			bissc_data_struct.xfer_address_withCTS = 1024 + bissc_data_struct.cd_register_xfer_address;  //CTS = 1;
			crcSelf =  PM_bissc_getCRC(0, 11, BISS_CD_CRC_NBITS_POLY1, (uint16_t *)&bissc_data_struct.xfer_address_withCTS, bissCRCtableCD, 2);

			//cdm = ST(1) + CTS(1) + ID(3) + Addr(7) CRC(4) + R(1) + W(1) + S(1) + TxData(8) + TxDataCRC(4) + STOP(1)
			//for address read of 0x7E, cdm = 1100 0111 1110 (CRC[4]) 1010 0000 0000 0000

			//fill CDM
			bissc_data_struct.cdm = ((((uint32_t)bissc_data_struct.cd_register_xfer_address) & 0xFF) << 16) << 4;
			bissc_data_struct.cdm = bissc_data_struct.cdm + ((~crcSelf & 0xF)<<16);

			if(bissc_data_struct.cd_register_xfer_is_write == 0)
			{
				//read
				bissc_data_struct.cdm = bissc_data_struct.cdm + 0xC000A000;
			}
			else
			{
				//if write, another CRC is required - one for the txdata
				crcSelf = PM_bissc_getCRC(0, 11, BISS_CD_CRC_NBITS_POLY1, (uint16_t *)&bissc_data_struct.cd_register_xfer_txdata, bissCRCtableCD, 2);

				bissc_data_struct.cdm = bissc_data_struct.cdm + 0xC0006000;
				bissc_data_struct.cdm = bissc_data_struct.cdm + (bissc_data_struct.cd_register_xfer_txdata<<5) + ((~crcSelf & 0xF)<<1);
			}

			bissc_data_struct.cd_bits_to_send = 32;
			bissc_data_struct.remaining_cd_bits = 32;
			bissc_data_struct.cd_status = 1;
			bissc_data_struct.cd_register_xfer_rxdata = 0xFFFF;

			taskSuccessful = 1;

			break;

		//update cdm bit for next SCD
		case 1:

			if(bissc_data_struct.remaining_cd_bits > -1)
			{
				uint32_t temp32;

				//if >= 16, seperate into two shift instructions
				if(bissc_data_struct.remaining_cd_bits >= 16)
				{
					temp32 = bissc_data_struct.cdm >> 16;
					temp32 = (~(temp32 >> (bissc_data_struct.remaining_cd_bits - 16))) & 0x1;
					PM_bissc_setCDBit(temp32);
				}
				else
				{
					temp32 = (~(bissc_data_struct.cdm >> (bissc_data_struct.remaining_cd_bits))) & 0x1;
					PM_bissc_setCDBit(temp32);
				}

				bissc_data_struct.remaining_cd_bits--;
			}
			else
			{
				bissc_data_struct.cd_status = 2;
			}

			taskSuccessful = 1;

			break;

		//biss CD transaction complete - parse it
		case 2:

			// rxdRWS = the Read, Write and Stop bits received from the encoder
			rxdRWS = ((bissc_data_struct.cds_stream>>12) & 0x00000007);

			// 0x3 = Write command was sent, 0x5 = Read command was sent
			if(rxdRWS == 0x3 || rxdRWS == 0x5)
			{
				uint16_t crc_rxd;

				crc_rxd = bissc_data_struct.cds_stream & 0xF;
				bissc_data_struct.cds_raw = (bissc_data_struct.cds_stream >> 4) & 0xFF;

				//crc is done on 8bit data
				crcSelf = 0xF & (~PM_bissc_getCRC(0, 11, BISS_CD_CRC_NBITS_POLY1, (uint16_t *)&bissc_data_struct.cds_raw, bissCRCtableCD, 2));

				if(crcSelf == crc_rxd)
				{
					bissc_data_struct.cd_register_xfer_rxdata = bissc_data_struct.cds_raw;

					if(rxdRWS == 0x5)
					{
						taskSuccessful = 1;
					}
					else if(bissc_data_struct.cd_register_xfer_txdata == bissc_data_struct.cd_register_xfer_rxdata)
					{
						//if write command was sent and was successful, received data should match transmitted data
						taskSuccessful = 1;
					}
					else
					{
						// transmit happened and encoder didn't write properly
						//error
						taskSuccessful = 0;
					}
				}
				else
				{
					taskSuccessful = 0;
				}
			}
			else
			{
				taskSuccessful = 0;
			}

			bissc_data_struct.cd_status = 3;
			break;

		// wait here until a new cd request is desired
		case 3:
			taskSuccessful = 1;
			break;

		default:
			//error
			break;
	}

	return(taskSuccessful);
}
#endif


interrupt void bissc_spiRxFifoIsr(void)
{
	uint16_t i;

	for (i=0;i<=bissc_data_struct.fifo_level;i++){bissc_data_struct.rdata[i]= bissc_data_struct.spi->SPIRXBUF;}
	bissc_data_struct.spi->SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
	bissc_data_struct.spi->SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack

    bissc_data_struct.dataReady=1;
}


void bissc_setupGPIO(void) {

	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; // Configure GPIO6 as bissC Clk master
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1; // Configure GPIO7 as SPI Clk slave

	GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = 1;
	GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 1;
	GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 1;
	GpioCtrlRegs.GPAGMUX2.bit.GPIO27 = 1;

	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2; // Configure GPIO24 as SPISIMOB
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2; // Configure GPIO25 as SPISOMIB
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2; // Configure GPIO26 as SPICLKB
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 2; // Configure GPIO27 as SPISTEB

	GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3; // Asynch input GPIO24 (SPISIMOB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3; // Asynch input GPIO25 (SPISOMIB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3; // Asynch input GPIO26 (SPICLKB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3; // Asynch input GPIO27 (SPISTEB)

	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; // Configure GPIO34 as bissC TxEN - drive low

	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1; // Configure GPIO32 as bissC Pwr Ctl

	EDIS;
}


void bissc_configXBAR(void)
{
	EALLOW;

	//Choose SPISIMO GPIO as INPUTXBAR Input1 - SPISIMOB in this case
	InputXbarRegs.INPUT1SELECT = 24;	//  INPUTXBAR TRIP1 -> GPIO24

	EDIS;
}

void bissc_configEPWM4(void)
{
// Set the PWMA and B high as default values of bissC clk.
	EALLOW;
	EPwm4Regs.TZCTL.bit.TZA = 1;
	EPwm4Regs.TZCTL.bit.TZB = 1;
	EPwm4Regs.TZFRC.bit.OST = 1;
	EDIS;
}


void bissc_readPosition(void)
{
	// BISS_DATA_CLOCKS
	//  for Lika HS58S18/I7-P9-RM2 it is 34 ==> 32 (pos+E+W+posCRC) + CDS + Start
	//  for Kuebler 8.F5863.1426.C423 it is 36 ==> 34 (pos+E+W+posCRC) + CDS + Start
	PM_bissc_setupNewSCDTransfer(BISS_DATA_CLOCKS, SPI_FIFO_WIDTH);

	#if BISS_ENCODER_HAS_CD_INTERFACE
	if(bissc_doCDTasks() == 0)
	{
		ESTOP0; //most likely the BiSS CD CRC did not match
	}
	#endif

	PM_bissc_startOperation();

	while (bissc_data_struct.dataReady != 1) {}

	// fill and receive SCD data (including position) and putting it into the BiSS data structure
	if(bissc_receivePosition(BISS_POSITION_BITS, BISS_POSITION_CRC_BITS) == 0)
	{
		ESTOP0; //BiSS SCD CRC did not match
	}
}
