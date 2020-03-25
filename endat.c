//----------------------------------------------------------------------------------
//	FILE:			endat.c
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
// 4 Nov 2015  - Example project for PM EnDAT Library Usage
//----------------------------------------------------------------------------------

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "endat.h"

uint16_t endat22CRCtable[SIZEOF_ENDAT_CRCTABLE];	// Declare CRC table for EnDat CRC calculations

ENDAT_DATA_STRUCT endat22Data;		//PM EnDat22 data structure

uint16_t crc5_result;	//variable for calculated crc result checking
uint16_t retval1;		//used for function return val storage and checks


// Function to initialize EnDat operation
void EnDat_Init(void) {

	EALLOW;
//Enable clocks to PWM1/2/3/4
	CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
	EDIS;

//Configure EPWM4 to drive default values on GPIO6 and GPIO7
	EPWM4_Config();

// Generate table for EnDat Polynomial defied as POLY1
 	PM_endat22_generateCRCTable(NBITS_POLY1, POLY1, endat22CRCtable);
//GPIO configuration for ENDat operation
 	Endat_setup_GPIO();
//XBAR configuration for ENDat operation
	Endat_config_XBAR();

	endat22Data.spi = &SpibRegs;
	PM_endat22_setupPeriph();

	EALLOW;
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
	PieVectTable.SPIB_RX_INT = &spiRxFifoIsr;
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER6.bit.INTx3 = 1;     // Enable PIE Group 6, INT 9
	IER = 0x20;                            // Enable CPU INT6
	EINT;
	EDIS;

	// Power up EnDat 5v supply through GPIO32
	GpioDataRegs.GPBDAT.bit.GPIO32 = 1;
	DELAY_US(10000L); 	//Delay 10us
	//EncCLK high for 100ms
	EALLOW;
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;  //set as GPIO output
	GpioDataRegs.GPASET.bit.GPIO6 = 1; //set GPIO output to be high once GPIO6 is turned to be a GPIO
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
	DELAY_US(100000L);
	//EncCLK low for ~200ns (>125ns)
	GpioDataRegs.GPADAT.bit.GPIO6 = 0;
	//EncCLK high for 425ms (>381ms)
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
	DELAY_US(425000L);

	//Ensure that EncData is now low
	if (GpioDataRegs.GPADAT.bit.GPIO24 == 1) {
		    ESTOP0;
	}
	DELAY_US(1000L); 	//Delay 1s

	PM_endat22_setFreq(ENDAT_INIT_FREQ_DIVIDER);

	DELAY_US(1000L);

//Encoder Receive Reset - Command ERR
	retval1 = PM_endat22_setupCommand (ENCODER_RECEIVE_RESET, 0xAA, 0x2222, 0); //data1=any; data2=any
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_RECEIVE_RESET, 0);
	DELAY_US(1000000L); 	//Delay 1s

	retval1 = PM_endat22_setupCommand (ENCODER_RECEIVE_RESET, 0xAA, 0x2222, 0); //data1=any; data2=any
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_RECEIVE_RESET, 0);

	DELAY_US(2000L);

//Select memory area to read out encoder manufacturer parameters.
//MRS Code A1 - Select Parameters of Encoder Manufacturer
	 retval1 = PM_endat22_setupCommand (SELECTION_OF_MEMORY_AREA, 0xA1, 0x5555, 0);  //data1=MRS; data2=any
	 PM_endat22_startOperation();
	 while (endat22Data.dataReady != 1) {}
	 //CHECK CRC!
	 retval1 = PM_endat22_receiveData(SELECTION_OF_MEMORY_AREA, 0);
	 crc5_result =  PM_endat22_getCrcNorm(endat22Data.address, endat22Data.data, endat22CRCtable);
	 if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
		     ESTOP0;
	 }

	 DELAY_US(200L); 	//Delay 200us

//Request Encoder to send the number of clock pulses needed to shift out Encoder Position Values
//Reading Address 0xD Returns no.of clock pulses needed to shift out position data
	 retval1 = PM_endat22_setupCommand (ENCODER_SEND_PARAMETER, 0xD, 0xAAAA, 0);	//data1=address; data2=any
	 PM_endat22_startOperation();
	 while (endat22Data.dataReady != 1) {}
	 //CHECK CRC!
	 retval1 = PM_endat22_receiveData(ENCODER_SEND_PARAMETER, 0);
	 crc5_result =  PM_endat22_getCrcNorm(endat22Data.address, endat22Data.data, endat22CRCtable);

	 if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
		     ESTOP0;
	 }

// No.of clocks for position data needs to be set before attempting any position data access
	 endat22Data.position_clocks = endat22Data.data & 0xFF;

	 DELAY_US(200L);
}

void Endat_setup_GPIO(void) {

	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; // Configure GPIO6 as EnDat Clk master
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

	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 1; // Configure GPIO34 as EnDat TxEN

	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1; // Configure GPIO32 as EnDat Pwr Ctl

	EDIS;
}

void Endat_config_XBAR(void)
{
	EALLOW;

//Choose SPISIMO GPIO as InputXbar Input1 - SPISIMO (B) on GPIO24 in this case
	InputXbarRegs.INPUT1SELECT = 24;	// GPTRIP XBAR TRIP1 -> GPIO24
	EDIS;
}

//Template for performing delay compensation
//This function should only be called while operating at low frequencies (~200KHz) only.
void EnDat_initDelayComp(void) {
	uint16_t delay1, delay2;

//Transmit Send position values command to the encoder
//Delay is internally measured during this command operation
	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES, 0, 0, 0);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES, 0);

	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT21, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);
	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	{
		    ESTOP0;
	}
	 DELAY_US(200L); 	//Delay 200us
//Delay is internally measured during the above operation
//Read the measured delay and store in a variable - n * 2 * SYSCLKs
	delay1 = PM_endat22_getDelayCompVal();

//Redo the same procedure again
	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES, 0, 0, 0);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES, 0);

	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT21, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	     ESTOP0;
	 }
	 DELAY_US(200L); 	//Delay 200us

 //Delay is internally measured during the above operation
 //Read the measured delay and store in a variable - n * 2 * SYSCLKs
	 delay2 = PM_endat22_getDelayCompVal();

//Average value of the measured delays has to be updated into endat22Data.delay_comp
// for delay compensation to take effect
	endat22Data.delay_comp = (delay1+delay2) >> 1;
}


void error(void) {
	asm("     ESTOP0");
	//Test failed!! Stop!
	for (;;);
}

interrupt void spiRxFifoIsr(void)
{
	uint16_t i;
    for (i=0;i<=endat22Data.fifo_level;i++){endat22Data.rdata[i]= endat22Data.spi->SPIRXBUF;}
	endat22Data.spi->SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    endat22Data.spi->SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
    endat22Data.dataReady=1;
}


void EPWM4_Config(void) {
// Set the PWMA and B high as default values of ENDAT clk.
	EALLOW;
	EPwm4Regs.TZCTL.bit.TZA = 1;
	EPwm4Regs.TZCTL.bit.TZB = 1;
	EPwm4Regs.TZFRC.bit.OST = 1;
	EDIS;
}

uint16_t CheckCRC (uint16_t expectcrc5, uint16_t receivecrc5) {
	if (expectcrc5 == receivecrc5) {
		return 1;
	}
	else {
		return 0;
	}
}



void endat22_setupAddlData(void)
{
// EnDat22 Command reading position values and performing selection of memory area
// 0xA1 MRS code for selection of encoder manufacturer parameters
// Additional data enabled at this time is 0

	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA, 0xA1, 0, 0);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA, 0);

	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT22, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	 ESTOP0;
	 }

	 DELAY_US(200L); 	//Delay 200us

// EnDat22 Command reading position values and sending a parameter to encoder
// Send 0xD parameter address - will select no. of clock pulses to shift out position value
// Additional data enabled at this time is 0

	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES_AND_SEND_PARAMETER, 0xD, 0, 0);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES_AND_SEND_PARAMETER, 0);

	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT22, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	 ESTOP0;
	 }

	 DELAY_US(200L); 	//Delay 200us

// Enable Additional data 1 - to read out no. of  clock pulses to shift out position value
// MRS code 0x45	- Acknowledge memory content LSB

	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA, 0x45, 0, 0);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA, 0);

	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT22, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	 ESTOP0;
	 }

	 DELAY_US(200L); 	//Delay 200us

 // Enable Additional data 2 - to read out Operating status and error sources
 // MRS code 0x59	- operating status on additional data 2
 // Additional data enabled before this command is 1, after this command 2
	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA, 0x59, 0, 1);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES_AND_SELECTION_OF_THE_MEMORY_AREA, 1);

	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT22, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	 ESTOP0;
	 }

	 DELAY_US(2000L); 	//Delay 200us

}

void endat22_readPositionWithAddlData(void)
{
// Read out position value with 2 additional data previously enabled
// Additional data enabled at this time is 2

	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES_WITH_ADDITIONAL_DATA, 0, 0, 2);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES_WITH_ADDITIONAL_DATA, 2);

// CRC check for Position Data
	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT22, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	 ESTOP0;
	 }

// CRC check for Additional Data 1
	crc5_result =  PM_endat22_getCrcNorm((endat22Data.additional_data1 >> 16), endat22Data.additional_data1, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.additional_data1_crc))
	 {
	 ESTOP0;
	 }

// CRC check for Additional Data 2
	crc5_result =  PM_endat22_getCrcNorm((endat22Data.additional_data2 >> 16), endat22Data.additional_data2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.additional_data2_crc))
	 {
	 ESTOP0;
	 }

	 DELAY_US(200L); 	//Delay 200us
}

void endat21_readPosition(void)
{
// Read position value in EnDat21 mode - no additional data
	retval1 = PM_endat22_setupCommand (ENCODER_SEND_POSITION_VALUES, 0, 0, 0);
	PM_endat22_startOperation();
	while (endat22Data.dataReady != 1) {}
	retval1 = PM_endat22_receiveData(ENCODER_SEND_POSITION_VALUES, 0);

// CRC check for Position Data
	crc5_result =  PM_endat22_getCrcPos(endat22Data.position_clocks, ENDAT21, endat22Data.position_lo,
					endat22Data.position_hi, endat22Data.error1, endat22Data.error2, endat22CRCtable);

	if (!CheckCRC(crc5_result,endat22Data.data_crc))
	 {
	     ESTOP0;
	 }
}
//***************************************************************************
// End of file
// **************************************************************************
