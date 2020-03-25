//----------------------------------------------------------------------------------
//	FILE:			encoder-integration.h
//
//	Description:	Encoder declarations
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
// 4 Nov 2015  - Encoder declarations
//----------------------------------------------------------------------------------


#include "endat.h"
#include "bissc.h"

/*-----------------------------------------------------------------------------
Define the structure of the Encoder Driver Object
-----------------------------------------------------------------------------*/
typedef struct {
                    _iq ElecTheta;       // Output: Motor Electrical angle (Q24)
                    _iq MechTheta;       // Output: Motor Mechanical Angle (Q24)
                    _iq RawTheta;        // Variable: Raw position data from resolver (Q0)
                    _iq Speed;           // Variable: Speed data from resolver (Q4)
                    _iq InitTheta;       // Parameter: Raw angular offset between encoder index and phase a (Q0)
                    _iq MechScaler;      // Parameter: 0.9999/total count (Q30)
                    _iq StepsPerTurn;    // Parameter: Number of discrete positions (Q0)
                 Uint16 PolePairs;       // Parameter: Number of pole pairs (Q0)

                }  ABS_ENCODER;


/*-----------------------------------------------------------------------------
Default initializer for the Encoder Object.
-----------------------------------------------------------------------------*/
#define ABSENC_DEFAULTS {                                                   \
                               0x0,            /*  ElecTheta    - (Q24)  */   \
                               0x0,            /*  MechTheta    - (Q24)  */   \
                               0x0,            /*  RawTheta     - (Q0)   */   \
                               0x0,            /*  Speed        - (Q4)   */   \
                               0x0,            /*  InitTheta    - (Q0)   */   \
                               0x00020000,     /*  MechScaler   - (Q30)  */   \
                               0x0,            /*  StepsPerTurn - (Q0)   */   \
                               2,              /*  PolePairs    - (Q0)   */   \
       }
