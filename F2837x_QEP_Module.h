//----------------------------------------------------------------------------------
//	FILE:			F2837x_QEP_module.h
//
//	Description:	Contains QEP macros
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
//  Date	   | Description / Status
//----------------------------------------------------------------------------------
//  4 Nov 2015 - QEP macros
//----------------------------------------------------------------------------------

#ifndef __F2837X_QEP_H__
#define __F2837X_QEP_H__

#include "f2837xbmsk.h"

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Decode Control Register
------------------------------------------------------------------------------*/
#define QDECCTL_INIT_STATE     ( XCR_X2 + QSRC_QUAD_MODE )

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Control Register
------------------------------------------------------------------------------*/
#define QEPCTL_INIT_STATE      ( QEP_EMULATION_FREE + \
                                 PCRM_INDEX + \
                                 IEI_RISING + \
                                 IEL_RISING + \
                                 QPEN_ENABLE + \
                                 QCLM_TIME_OUT + \
                                 UTE_ENABLE )  

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Position-Compare Control Register
------------------------------------------------------------------------------*/
#define QPOSCTL_INIT_STATE      PCE_DISABLE 

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Capture Control Register
------------------------------------------------------------------------------*/
#define QCAPCTL_INIT_STATE     ( UPPS_X32 + \
                                 CCPS_X128 + \
                                 CEN_ENABLE )

/*-----------------------------------------------------------------------------
Define the structure of the QEP (Quadrature Encoder) Driver Object 
-----------------------------------------------------------------------------*/
typedef struct {float32 ElecTheta;        // Output: Motor Electrical angle (Q24)
                float32 MechTheta;        // Output: Motor Mechanical Angle (Q24)
                Uint16 DirectionQep;    // Output: Motor rotation direction (Q0)
                Uint16 QepPeriod;       // Output: Capture period of QEP signal in number of EQEP capture timer (QCTMR) period  (Q0)
                Uint32 QepCountIndex;   // Variable: Encoder counter index (Q0) 
                 int32 RawTheta;        // Variable: Raw angle from EQEP Postiion counter (Q0)
                float32 MechScaler;      // Parameter: 0.9999/total count (Q30)
                Uint16 LineEncoder;     // Parameter: Number of line encoder (Q0) 
                Uint16 PolePairs;       // Parameter: Number of pole pairs (Q0) 
                int32 CalibratedAngle; // Parameter: Raw angular offset between encoder index and phase a (Q0)
                Uint16 IndexSyncFlag;   // Output: Index sync status (Q0) 
                }  QEP;

/*-----------------------------------------------------------------------------
Default initializer for the QEP Object.
-----------------------------------------------------------------------------*/
#define QEP_DEFAULTS { 0x0,0x0,0x0,0x0,0x0,0x0,0x00020000,0x0,2,0,0x0}   

/*-----------------------------------------------------------------------------
	QEP Init and QEP Update Macro Definitions                                 
-----------------------------------------------------------------------------*/

#define QEP_INIT_MACRO(m,v)																	\
																								\
     (*eQEP[m]).QDECCTL.all = QDECCTL_INIT_STATE;												\
     (*eQEP[m]).QEPCTL.all = QEPCTL_INIT_STATE;													\
     (*eQEP[m]).QPOSCTL.all = QPOSCTL_INIT_STATE;												\
     (*eQEP[m]).QUPRD = 600000;		        	/* Unit Timer for 100Hz*/						\
     (*eQEP[m]).QCAPCTL.all = QCAPCTL_INIT_STATE;												\
     (*eQEP[m]).QPOSMAX = 4*v.LineEncoder;														\
																								

#define QEP_MACRO(m,v)																		\
																							\
/* Check the rotational direction */														\
v.DirectionQep = (*eQEP[m]).QEPSTS.bit.QDF;													\
																							\
/* Check the position counter for EQEP1 */													\
v.RawTheta = (*eQEP[m]).QPOSCNT + v.CalibratedAngle;										\
																							\
if (v.RawTheta < 0)																			\
v.RawTheta = v.RawTheta + (*eQEP[m]).QPOSMAX;												\
else if (v.RawTheta > (*eQEP[m]).QPOSMAX)													\
v.RawTheta = v.RawTheta - (*eQEP[m]).QPOSMAX;												\
																							\
/* Compute the mechanical angle */															\
v.MechTheta= v.MechScaler*v.RawTheta;														\
/* Compute the electrical angle  */															\
v.ElecTheta = (v.PolePairs*v.MechTheta) -floor(v.PolePairs*v.MechTheta); /* Q24 = Q0*Q24 */ \
																							\
/* Check an index occurrence*/																\
if ((*eQEP[m]).QFLG.bit.IEL == 1)    														\
{  																							\
v.IndexSyncFlag = 0x00F0;																	\
v.QepCountIndex = (*eQEP[m]).QPOSILAT;														\
(*eQEP[m]).QCLR.bit.IEL = 1;	/* Clear interrupt flag */									\
}																							\
																							\
/* Check unit Time out-event for speed calculation: */										\
/* Unit Timer is configured for 100Hz in INIT function*/									\
if((*eQEP[m]).QFLG.bit.UTO == 1)															\
{																							\
/***** Low Speed Calculation   ****/														\
if(((*eQEP[m]).QEPSTS.bit.COEF || (*eQEP[m]).QEPSTS.bit.CDEF))								\
{	/* Capture Counter overflowed, hence do no compute speed*/								\
(*eQEP[m]).QEPSTS.all = 0x000C;																\
}																							\
else if((*eQEP[m]).QCPRDLAT!=0xffff)														\
/* Compute lowspeed using capture counter value*/											\
v.QepPeriod = (*eQEP[m]).QCPRDLAT;															\
    }
#endif // __F2837X_QEP_H__




