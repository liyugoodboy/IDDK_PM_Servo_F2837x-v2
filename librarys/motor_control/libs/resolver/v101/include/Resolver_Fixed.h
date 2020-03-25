//----------------------------------------------------------------------------------
//	FILE:			Resolver_Fixed.h
//
//	Description:	shared variables and constants
//
//   Revision/ Version: See Resolver-Main.c
//----------------------------------------------------------------------------------
#ifndef Resolver_Fixed_H
#define Resolver_Fixed_H

#ifdef __cplusplus
extern "C" {
#endif

  #define   GLOBAL_Q       20
  #include "IQmathLib.h"
  
// For Portability, User Is Recommended To Use Following Data Type Size
// Definitions For 16-bit and 32-Bit Signed/Unsigned Integers:
//
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int             int16;
typedef long            int32;
typedef unsigned int    Uint16;
typedef unsigned long   Uint32;
typedef float           float32;
typedef long double     float64;
#endif


// ****************************************************************************
// Macros to convert numbers between floating point and IQ formats
#define  IQn2float(X, Qn)   ((float)X * (1/(1<<Qn)))
#define  float2IQn(f, Qn)   (f * (float)((long)1<<Qn))

// The following macro helps to count the CPU clock in between a RESTART_CLOCK
//    and READ_CLOCK
#define	READ_CLOCK_C28(X)   EALLOW;                                    \
					        EPwm3Regs.TBCTL.bit.CTRMODE = TB_FREEZE;   \
                            X = EPwm3Regs.TBCTR;                       \
                            EDIS;

#define RESTART_CLOCK_C28   EALLOW;                                    \
					        EPwm3Regs.TBCTL.bit.CTRMODE = TB_FREEZE;   \
                            EPwm3Regs.TBCTR = 0;                       \
                            EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; \
                            EDIS;

// ****************************************************************************
// ============================================
// Structure of variables between CPU and CLA
// ============================================
// Input variables
typedef struct  {
	// variables to set up basic functions
    Uint16 firLag,       // lag between sine index and FIR index
           TUNING,       // select-1 / deselect-0 TUNING function
           TABLE_LENGTH; // set up FIR filter length

    int32  offsetS,       //  dc offset of sine fbk analog channel   (A0)
       	   offsetC,       //  dc offset of cosine fbk analog channel   (B0)
           SAMPLING_TIME, // loop decimation sampling time (carrier cycle time)
           testAngle;     // test Angle used for tuning the PI coefficients

	// control loop parameters (can be replaced with MACROs)
	int32  errorWfT,   // error filter constant, internally computed based on filter coefficients
		   picon_K0,   // PI controller constant, internally computed based on PI coefficients
		   picon_K1,   // PI controller constant, internally computed based on PI coefficients
		   rpsMax;     // max motor speed in eqvt elec freq
} RESOLVER_INPUT;

//--------------------------------------------------------------------------------
// Output variables
typedef struct  {
	// variables for outputting results in float
    int32     angleRaw,      // raw angle estimate from arctan
              angleObs,      // observer angle estimate w/o FIR delay compensation
              angleOut,      // final angle estimate after FIR delay compensation
              rpsObs,        // shaft speed estimate by the observer
              errorNew,      // new angle error estimated by the observer
              sinFIRout,     // FIR bandpass filter output of sine feedback signal
              cosFIRout,     // FIR bandpass filter output of cosine feedback signal
              resMag;        // resolver fbk mag

    // variables used within library
    int32     sin_input,    // sine input from resolver
              cos_input;    // cosine input from resolver
    Uint16    sineIndex;    // index to element within sine table

	// variables for cycle counting the execution times
	Uint16 uxCycleCount,      // counter to store cycle count of selected code - debug purpose
	       uxCycleCount1;     // counter to store cycle count of selected code - debug purpose
} RESOLVER_OUTPUT;

//--------------------------------------------------------------------------------
// constant data
//typedef struct  {
//	int32 *sineTable;   // pointer to SINE_TABLE
//	int32 *firCoeff;    // pointer to FIR_COEFF_TABLE
//} RESOLVER_DATA;
//
//extern RESOLVER_DATA rslvrData;

// ============================================
// Functions defined in library
// ============================================
extern void init_resolver_Fixed ( void );
extern Uint16 reslvr_algo_Fixed ( void );


// ============================================
// variables used between CPU and CLA
// ============================================
extern int32 *sineTable;            // pointer to SINE_TABLE
extern int32 *firCoeff;             // pointer to FIR_COEFF_TABLE
extern RESOLVER_INPUT rslvrIn;      // variables from main to lib
extern RESOLVER_OUTPUT rslvrOut;    // variables from lib to main


// ****************************************************************************

#ifdef __cplusplus
}
#endif /* extern "C" */


#endif  // end of SHARED_C28_CPU_H definition
