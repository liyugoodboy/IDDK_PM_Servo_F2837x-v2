/* PM_sincos_lib.h: sincos library header file */

#ifndef _C_PM_SINCOS_LIB_H
#define _C_PM_SINCOS_LIB_H

typedef unsigned int Uint16;
typedef unsigned long Uint32;

/* include files */
#include "IQmathLib.h"					// IQ math functions
#include "F2837xD_eqep.h"				// QEP register header file

/* clock periods in SYSCLK cycles - used for default mode transition formulae */
#define	UTO_PERIOD	625000				// period of unit time-out counter
#define INT_PERIOD	10000				// interrupt period

/* default QEP speed for mode tranistions - note: pre-processor truncates integers */
#define	DEFAULT_MODE01_TRANS (unsigned int)(UTO_PERIOD / INT_PERIOD / 6)	// max quadrants per UTO event to perform calibration
#define DEFAULT_MODE12_TRANS (unsigned int)(UTO_PERIOD / INT_PERIOD / 3)	// max quadrants per UTO event for s/w quadrant detection

/* sincos library definitions */
#define LINECOUNT 2048					// encoder counts per mechanical revolution
#define ADC_RESOLUTION 12				// ADC resolution in bits
#define	AVG_FILTER_SHIFT 2				// averaging filter length (power of 2)
#define AVG_FILTER_LENGTH (1 << AVG_FILTER_SHIFT)
#define CAL_DELAY_CYCLES 20				// number of complete sincos cycles to capture calibration data

/* limits & error bounds */
#define COS_GAIN_CLAMP_HI 1.05			// upper bound on cosine relative gain (float)
#define COS_GAIN_CLAMP_LO 0.95			// lower bound on cosine relative gain (float)
#define QCOUNT_ERROR_TOL 1.1			// QEP/software quadrature edge count matching: error tolerance (float)
#define SC_MIN_RANGE 100				// minimum signal range for LOS detection (int)

/* ADC max reading */
#if ADC_RESOLUTION == 12
#define ADC_MAX_RDG 2047
#elif ADC_RESOLUTION == 16
#define ADC_MAX_RDG 65535
#endif


/* SinCos transducer struct */
typedef volatile struct {
	int initFlg;				// [1] initialisation & first pass-through complete
	int calFlg;					// [2] first calibration complete
	unsigned int sindata;		// [3] ch0 - raw data
	unsigned int cosdata;		// [4] ch1 - raw data
	unsigned int ch0offs;		// [5] ch0 offset as integer
	unsigned int ch1offs;		// [6] ch1 offset as integer
	_iq15 sinoffs;				// [7] ch0 offset as IQ
	_iq15 cosoffs;				// [8] ch1 offset as IQ
	_iq15 singain;				// [9] ch0 gain (fixed at 1)
	_iq15 cosgain;				// [10] ch1 gain (relative to singain)
	_iq15 sincorr;				// [11] ch0 after static correction
	_iq15 coscorr;				// [12] ch1 after static correction
	_iq15 qcount;				// [13] quadrature count from angle
	_iq15 qmaxpos;				// [14] maximum position count
	int qdir;					// [15] rotation direction: 0 = CW, 1 = CCW.
	int qcflg;					// [16] calibration update ready flag
	long qepcnt;				// [17] raw QEP count
	long qepspd;				// [18] capture counter from QEP
	int mode;					// [19] operating mode (0 = normal, 1 = no cal., 2 = coarse angle only)
	long mchg01;				// [20] speed transition between modes 0 <-> 1
	long mchg12;				// [21] speed transition between modes 1 <-> 2
	_iq15 itheta;				// [22] encoder angle in fixed-point
	float ftheta;				// [23] encoder angle in float-point
	int status;					// [24] sincos status & errors
	volatile struct EQEP_REGS *qep;	// [25] pointer to QEP register structure
} SINCOS;


/* library functions */
extern int PM_sincos_calcAngle(SINCOS *p);			// calculate shaft angle
extern int PM_sincos_updateCalData(SINCOS *p);		// reload gain & offset correction data
extern void PM_sincos_initLib(SINCOS *p);			// library initialisation
extern void PM_sincos_reset(SINCOS *p);				// reset sincos calculation

#endif	// _C_PM_SINCOS_LIB_H

/* end of file */
