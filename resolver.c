//----------------------------------------------------------------------------------
//	FILE:			Resolver.c
//
//	Description:
//
//	Version: 		1.0
//
//  Target:  		Floating point C2000 MCUs
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2012
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------

#include "IDDK_PM_Servo_F2837x-Settings.h"
#include "Resolver_Float.h"


// ****************************************************************************
// Functions, Macros and Variables for RESOLVER interface
// ****************************************************************************
void baseParamsInit(void);
void derivParamsCal(void);
void resolver_PostProcess(void);

#define  BASE            (SINE_CARRIER/2)
#define  SINE900         (BASE*4.0/7.5)         /* generate 4.55V sine ref @7.5V offset*/
#define  SINE225         (int)(SINE900*0.383)
#define  SINE450         (int)(SINE900*0.707)
#define  SINE675         (int)(SINE900*0.924)

#define DAC_BASE			2048
#define DAC_SINE900		    900   /*  1100  */
#define DAC_SINE225			DAC_SINE900*0.383
#define DAC_SINE450			DAC_SINE900*0.707
#define DAC_SINE675			DAC_SINE900*0.924

int32 SINE_TABLE[DELAY_LENGTH] = {
	DAC_BASE,             DAC_BASE + DAC_SINE225,   DAC_BASE + DAC_SINE450,   DAC_BASE + DAC_SINE675,
	DAC_BASE + DAC_SINE900,   DAC_BASE + DAC_SINE675,   DAC_BASE + DAC_SINE450,   DAC_BASE + DAC_SINE225,
	DAC_BASE,             DAC_BASE - DAC_SINE225,   DAC_BASE - DAC_SINE450,   DAC_BASE - DAC_SINE675,
	DAC_BASE - DAC_SINE900,   DAC_BASE - DAC_SINE675,   DAC_BASE - DAC_SINE450,   DAC_BASE - DAC_SINE225,
};

#if FIR32
  float FIR_COEFF[33] = {
                  0,   0.002575182589,   0.005207811017,   0.005215597805,
   5.943273511e-017,  -0.01166561246,   -0.02868790552,   -0.04744737595,
     -0.06249880791,  -0.06803549826,   -0.05969916657,   -0.03616938367,
   1.150078466e-015,   0.042620074,      0.08318161219,    0.1129118577,
       0.1250027567,   0.1129118577,     0.08318161219,    0.042620074,
   1.150078466e-015,  -0.03616938367,   -0.05969916657,   -0.06803549826,
     -0.06249880791,  -0.04744737595,   -0.02868790552,   -0.01166561246,
   5.943273511e-017,   0.005215597805,   0.005207811017,   0.002575182589,
                  0
};
#else
  float FIR_COEFF[DELAY_LENGTH+1] = {
   -0.08303630352,  -0.08532184362, -0.07218718529, -0.04381770268,
   -0.004064163659,  0.04017074406,  0.08038958907,  0.1084507555,
    0.1185115501,    0.1084507555,   0.08038958907,  0.04017074406,
   -0.004064163659, -0.04381770268, -0.07218718529, -0.08532184362,
   -0.08303630352
};
#endif

// input to resolver loop
RESOLVER_INPUT rslvrIn;

// output from resolver loop
RESOLVER_OUTPUT rslvrOut;

// constant data to resolver
int32 *sineTable = SINE_TABLE;
float *firCoeff  = FIR_COEFF;

// analog input offset estimation variables
float offsetFc,    // offset filter corner freq (Hz)
      offsetWfT;   // offset filter constant, internally computed based on filter coefficients

// control loop parameters (can be replaced with MACROs)
float  errorFc,    // error filter corner freq (in rad/sec)
       rslvrKp,    // pi controller P gain
       piconFz;    // PI controller zero

// Resolver test variables
int32  testCntr = 0,         // local counter for ISR tick
	   testCntMax = 1000;    // max count value

float testAngleMax = 0.8;   // max test angle for ref gen

// variables for resolver data analysis
int32 resetAll = 0,                 // reset all error flags
      resMagMax = _IQ(-2.0),        // resolver magnitude max value
      resMagMin = _IQ(2.0),         // resolver magnitude min value
      DOS_ERROR = 0,                // degradation of signal (DOS) - error flag
      LOS_ERROR = 0,                // loss of signal (LOS) - error flag
      PLL_ERROR = 0,                // PLL loop error flag
      dos,                          // degradation of signal (DOS)
      dosLimit = _IQ(0.5),          // Limit value of DOS
      losLimit = _IQ(0.8),          // Limit value of LOS
      skipLosCnt  = 0,              // cnt value for los validation
      skipLosPrd  = 100,            // Prd value for los validation
      angleErrMax = _IQ(0.4);       // Limit value of PLL loop error

//***********************************************************************

// ****************************************************************************
// ****************************************************************************
// RESOLVER FUNCTIONS
// ****************************************************************************
// ****************************************************************************
void baseParamsInit(void) {
	// initialize all parameters
	rslvrIn.SAMPLING_TIME = 1.0 / (RESOLVER_EXC_FREQUENCY*1000);
	rslvrIn.TABLE_LENGTH  =  DELAY_LENGTH;
	rslvrIn.TUNING        = TUNING_SEL;
	rslvrIn.FIR32         = FIR32_SEL;
	rslvrIn.testAngle     = 0.0;
	rslvrIn.rpsMax        = 200.0;    // max motor speed in freq
	rslvrIn.firLag        = 3;        // fbk sine and cosine peaks occur when firLag = 3

	offsetFc = 500.0 / TWO_PI;        // offset filter corner frequency (Hz)
	errorFc  = 1000.0;                // error filter corner frequency (Hz)
	piconFz  = 200.0;                 // pi controller - ZERO frequency
	rslvrKp  = 5000.0;                      // pi controller - prop gain

	return;
}

void derivParamsCal(void) {
	float  kiTsBy2;

	rslvrIn.errorWfT  =  errorFc  * TWO_PI * rslvrIn.SAMPLING_TIME;

	kiTsBy2   =  rslvrKp * (piconFz * TWO_PI) * rslvrIn.SAMPLING_TIME / 2.0;
	rslvrIn.picon_K0  =  rslvrKp + kiTsBy2;
	rslvrIn.picon_K1  = -rslvrKp + kiTsBy2;

	return;
}

// ******************************************************************************
// RESOLVER post process functions
// - Qualify the resolver data signals in regular operation mode
// ******************************************************************************
void resolver_PostProcess(void)
{
    /*+++++++++++++++++++++++++++++++++++++
     *    Resolver signal quality tests
     *+++++++++++++++++++++++++++++++++++++
     */
    if (rslvrOut.resMag20 > resMagMax)
  	  resMagMax = rslvrOut.resMag20;
    else if (rslvrOut.resMag20 < resMagMin)
  	  resMagMin = rslvrOut.resMag20;

    // Degradation Of Signal (DOS) identification
    dos = resMagMax - resMagMin;
    if (dos > dosLimit)
  	  DOS_ERROR = 1;

    // Loss Of Signal (LOS) identification
    if (skipLosCnt < skipLosPrd)
      skipLosCnt++;
    else if (rslvrOut.resMag20 < losLimit)
  	  LOS_ERROR = 1;

    // Phase Lock Loop (PLL) Error identification
    // abs(error angle mag) = 0 <---> _IQ(0.5)
    if (abs(rslvrOut.errorNew20) > angleErrMax)
      PLL_ERROR = 1;

    // reseting all error flags
    if (resetAll == 1)
    {
  	  DOS_ERROR  = 0;
  	  LOS_ERROR  = 0;
  	  PLL_ERROR  = 0;
  	  resetAll   = 0;
  	  skipLosCnt = 0;
  	  resMagMax  = _IQ(-2.0);
  	  resMagMin  = _IQ(2.0);
    }

    return;
}

// ******************************************************************************
// RESOLVER Interrupt Service Routine
// - Generate sine wave exc by updating the DAC
// - Read sine and cosine feedback after after offset compensation
// - Compute angle using module from resolver.lib
// ******************************************************************************
interrupt void ResolverISR ( void )
{
    // *** Set up next sampling instant and update excitation sine wave ***
	rslvrOut.sineIndex = (rslvrOut.sineIndex-1) & (rslvrIn.TABLE_LENGTH-1);  // DELAY_LENGTH must be 2^n
	DacaRegs.DACVALS.bit.DACVALS = sineTable[rslvrOut.sineIndex];;

    // *** Sample sine and cosine feedbacks from ADC ***

	rslvrOut.sin_input = (((float)R_SIN_PPB*ADC_PU_PPB_SCALE_FACTOR));
	rslvrOut.cos_input = (((float)R_COS_PPB*ADC_PU_PPB_SCALE_FACTOR));

	if(resolver_algo_Float())
	{
		// Tune the resolver coefficients in TUNING mode
		// In this mode, angle derived from sin/ cos is bypassed and test angle is used instead
	    if (rslvrIn.TUNING)
	    {
	    	if (++testCntr >= testCntMax)
	    	{
	    		derivParamsCal();  // derive newer params based on inputs
	    		if (rslvrIn.testAngle > 0.15)
	    			rslvrIn.testAngle = 0.0;
	    	    else
	    		    rslvrIn.testAngle = testAngleMax;
	    	    testCntr = 0;
	        }
	    }

	    // Perform post processing function here
	    else
	    	resolver_PostProcess();
	}
    
    //clear EPWM1 INT and ack PIE INT
	AdccRegs.ADCINTFLGCLR.bit.ADCINT1=1;
	PieCtrlRegs.PIEACK.all=PIEACK_GROUP1;
	
	return;
}

//***************************************************************************
// End of file
// **************************************************************************
