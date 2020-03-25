//----------------------------------------------------------------------------------
//	文件:    IDDK_PM_Servo_F2837x-Settings.h
//
//	描述:	 用户配置
//
//	版本:    1.0
//
//  目标:  	TMS320F28377D,
//
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//  修改记录:
//----------------------------------------------------------------------------------
//  时间	  | 描述 / 状态
//----------------------------------------------------------------------------------
// 2015.11.4  - 用户设置
//----------------------------------------------------------------------------------

#ifndef PROJ_INCLUDE_H
#define PROJ_INCLUDE_H

/*-------------------------------------------------------------------------------
包含项目特定的包含文件
-------------------------------------------------------------------------------*/
// 将数学类型定义为float（1）
#define   MATH_TYPE      1

#include "IQmathLib.h"
#include "F28x_Project.h"

#include "smopos.h"       		// Include header for the SMOPOS object 
#include "smopos_const.h"       // Include header for the SMOPOS object
#include "park.h"       		// Include header for the PARK object 
#include "ipark.h"       		// Include header for the IPARK object 
#include "pi.h"       			// Include header for the PIDREG3 object 
#include "clarke.h"         	// Include header for the CLARKE object 
#include "svgen.h"		       	// Include header for the SVGENDQ object 
#include "rampgen.h"        	// Include header for the RAMPGEN object 
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object 
#include "volt_calc.h"      	// Include header for the PHASEVOLTAGE object 
#include "speed_est.h"          // Include header for the SPEED_ESTIMATION object 
#include "speed_fr.h"			// Include header for the SPEED_MEAS_QEP object
#include "resolver.h"
#include "pid_grando.h"
#include "pid_reg3.h"
#include <math.h>

#include "DLOG_4CH_F.h"

#include "F2837xD_sdfm_drivers.h"
#include "Resolver_Float.h"

#include "F2837x_QEP_Module.h"
#include "encoder-integration.h"

/*------------------------------------------------------------------------------
GND配置列表 - COLD or HOT
------------------------------------------------------------------------------*/
#define COLD  1      	  // control GND is COLD
#define HOT   2           // control GND is HOT

/*------------------------------------------------------------------------------
将CGND配置设置为可用选项之一。
------------------------------------------------------------------------------*/
#define  CGND  COLD

/*------------------------------------------------------------------------------
以下是“构建级别”选项的列表。
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// 模块检出（请勿连接电机）
#define LEVEL2  2           // 验证ADC， park/clarke转换，校准失调和速度测量
#define LEVEL3  3           // 验证闭合电流（转矩）回路及其PI
#define LEVEL4  4           // 验证速度环和速度PID
#define LEVEL5  5           // 验证位置环

/*------------------------------------------------------------------------------
设置系统构建等级
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL2

/*------------------------------------------------------------------------------
以下是电流传感器选项的列表
------------------------------------------------------------------------------*/
#define SHUNT_CURRENT_SENSE       1
#define LEM_CURRENT_SENSE         2
#define SD_CURRENT_SENSE          3

/*------------------------------------------------------------------------------
设置电流传感器类型。
------------------------------------------------------------------------------*/
#define CURRENT_SENSE LEM_CURRENT_SENSE

/*------------------------------------------------------------------------------
以下是位置编码器选项的列表
------------------------------------------------------------------------------*/
#define QEP_POS_ENCODER           1
#define RESOLVER_POS_ENCODER      2    //解析器位置传感器
#define BISS_POS_ENCODER          3
#define ENDAT_POS_ENCODER         4
#define SINCOS_POS_ENCODER        5
/*------------------------------------------------------------------------------
此行将POSITION_ENCODER设置为用户从上面的选择值
------------------------------------------------------------------------------*/
#define POSITION_ENCODER  QEP_POS_ENCODER

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------

//定义常数值
#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

//定义常数PI值
#define PI 3.14159265358979

//定义系统频率（MHz）
#if (DSP2803x_DEVICE_H == 1)
#define SYSTEM_FREQUENCY   60
#elif (DSP280x_DEVICE_H == 1)
#define SYSTEM_FREQUENCY   100
#elif (F28_2837xD == 1)
#define SYSTEM_FREQUENCY   200
#endif

// 基于系统时钟的计时器定义
#define		mSec0_5		0.5*SYSTEM_FREQUENCY*1000		// 0.5 mS
#define		mSec1		1*SYSTEM_FREQUENCY*1000		    // 1.0 mS
#define		mSec2		2.0*SYSTEM_FREQUENCY*1000		// 2.0 mS
#define		mSec5		5*SYSTEM_FREQUENCY*1000		    // 5.0 mS
#define		mSec7_5		7.5*SYSTEM_FREQUENCY*1000		// 7.5 mS
#define		mSec10		10*SYSTEM_FREQUENCY*1000		// 10 mS
#define		mSec20		20*SYSTEM_FREQUENCY*1000		// 20 mS
#define		mSec50		50*SYSTEM_FREQUENCY*1000		// 50 mS
#define		mSec100		100*SYSTEM_FREQUENCY*1000		// 100 mS
#define		mSec500		500*SYSTEM_FREQUENCY*1000	    // 500 mS
#define		mSec1000	1000*SYSTEM_FREQUENCY*1000	    // 1000 mS

// 定义ISR频率（kHz）
#define ISR_FREQUENCY        10
#define INV_PWM_TICKS        ((SYSTEM_FREQUENCY/2.0)/ISR_FREQUENCY)*1000
#define INV_PWM_TBPRD        INV_PWM_TICKS/2
#define INV_PWM_HALF_TBPRD   INV_PWM_TICKS/4

// 解析器相关的定义
#define RESOLVER_EXC_SAMPLE_FREQUENCY   160   // KHz
#define RESOLVER_EXC_FREQUENCY           10   // KHz
#define RESOLVER_PWM_TICKS       ((SYSTEM_FREQUENCY/2.0)/RESOLVER_EXC_SAMPLE_FREQUENCY)*1000

#define  TWO_PI                  (2*PI)
#define  DELAY_LENGTH            16

#define  FIR32_SEL    0   /* 1 - 实施33次FIR，在2个周期内采样
                             0 - 实施17次FIR，在1个周期内采样   */

#define  TUNING_SEL   0   /* 1 - 绕过atan值-调整PI系数
                             0 - 使用atan值*/

// 分辨率规格
#define RESOLVER_STEPS_PER_TURN         4096 //旋转变压器的离散步数/转数
#define RESOLVER_STEPS_PER_POLEPAIR    (RESOLVER_STEPS_PER_TURN/(POLES/2))

#define  POS_KI_LOW_SPD   0.8// _IQ(0.0001)
#define  POS_KI_MED_SPD   1.0// _IQ(0.001)
#define  POS_KI_HI_SPD    1.2// _IQ(0.01)


/*------------------------------------------------------------------------------
将电动机参数设置为一个可用的参数
------------------------------------------------------------------------------*/

// 定义电动机参数（Estun伺服电动机）
#define RS 		2.35		    	    // 定子电阻(ohm)
#define RR   			               	// 转子电阻(ohm)
#define LS   	0.0065					// 定子电感(H)
#define LR   			  				// 转子电感(H)
#define LM   			   				// 励磁电感(H)
#define POLES  	8						// 极数


// 定义基本数量
#define BASE_VOLTAGE          236.14   // 基本峰值相电压（伏特），Vdc / sqrt（3）
#define BASE_SHUNT_CURRENT    9.95     // 基本峰值相电流（安培），最大值 可测量的峰值电流。
#define BASE_LEM_CURRENT      12.0     //  ----- do -----
#define BASE_TORQUE     		       // 基本扭矩（N.m）
#define BASE_FLUX       			   // 基础磁链（volt.sec/rad）
#define BASE_FREQ      	      200      // 基本电气频率（Hz）


/*-----------------------------------------------------------------------------
 * Sigma Delta过滤器模块-框架设置
 * ----------------------------------------------------------------------------
 */
#define  SDFM_TICKS    5
#define  OSR_RATE      OSR_128

/*------------------------------------------------------------------------------
电流传感器比例
------------------------------------------------------------------------------*/
// LEM    1.0pu current ==> 12.0A -> 2048 counts ==> 8A -> 1365
// SHUNT  1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
#define LEM(A)        2048*A/BASE_LEM_CURRENT
#define SHUNT(A)      2048*A/BASE_SHUNT_CURRENT

//ADC配置
//选择ADC分辨率的定义
#define RESOLUTION_12BIT           0 //12位分辨率
#define RESOLUTION_16BIT           1 //16位分辨率(并非所有都支持)

//选择ADC信号模式的定义
#define SIGNAL_SINGLE              0 //单端通道转换（仅12位模式）
#define SIGNAL_DIFFERENTIAL        1 //差分对通道转换

#define ADC_PU_SCALE_FACTOR        0.000244140625      //1/2^12
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250      //1/2^11
#define SD_PU_SCALE_FACTOR         0.000030517578125   //1/2^15

#define REFERENCE_VDAC             0
#define REFERENCE_VREF             1

//ADC相关定义
#define IFB_SV          AdcaResultRegs.ADCRESULT0
#define IFB_SW          AdcbResultRegs.ADCRESULT0
#define IFB_SV_PPB      ((signed int)AdcaResultRegs.ADCPPB1RESULT.all)
#define IFB_SW_PPB      ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)

#define R_SIN           AdcdResultRegs.ADCRESULT0
#define R_COS           AdccResultRegs.ADCRESULT0
#define R_SIN_PPB       ((signed int)AdcdResultRegs.ADCPPB1RESULT.all)
#define R_COS_PPB       ((signed int)AdccResultRegs.ADCPPB1RESULT.all)

#define IFB_LEMV        AdcaResultRegs.ADCRESULT1
#define IFB_LEMW        AdcbResultRegs.ADCRESULT1
#define IFB_LEMV_PPB    ((signed int)AdcaResultRegs.ADCPPB2RESULT.all)
#define IFB_LEMW_PPB    ((signed int)AdcbResultRegs.ADCPPB2RESULT.all)

// ************************************************************************
//比例系数:将所有电流反馈恢复到正常比例，并基于分流器进行测量
//      带分流,  1.0pu current  == 9.945A
//        LEM,  1.0pu current  == 12A
//       SDFM,  0.8906pu current  == 12.5A
// ************************************************************************
#define  LEM_TO_SHUNT    1.206637   // (12.0/9.945)
#define  SDFM_TO_SHUNT   1.41131    // (12.5/0.8906)/9.945

/*****************************************************************************
 * ***************************************************************************
 */
#endif
