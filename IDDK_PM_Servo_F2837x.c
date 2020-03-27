/* ============================================================================
系统名称:    	IDDK PMSM伺服控制

文件名称:	  	IDDK_PM_Servo_F2837x.C

目标:			F2837x控制卡的修订版1.3

母板 :           工业驱动器开发套件-IDDK-R2.2.1

作者:			C2000系统实验室，2015年11月6日

描述:	                 项目使用以下ISR：
				1. ADC INT触发的解析器ISR，这被定义为高优先级中断，并以160Khz运行
				2. 逆变器ISR速率为10Khz，由EPWM11触发，之所以选择EPWM11，是因为SD读
				数EPWM11为SDFM模块提供了周期性的同步复位，每次复位后3个OSR之后SDFM数
				据准备就绪，需要立即读取。
				3. SPI ISR->用于EnDAT和BiSS位置编码器接口

===========================================================================  */
//------------------------------------------------------------------------------
//  修改记录:
//  v1.0 - 初始发布
//  v2.0 - 添加EnDAT绝对编码器接口库和代码
//       - 添加BiSS-C编码器接口库和代码
//       - 过电流保护算法中的错误修复：
//         - 添加了新变量“ curLimit”，如果需要在10A时进行保护，则用户可以将其设置为实
//           际值设置为10.0
//       - COLD控制GND配置的默认代码设置
//           （只能在HOT配置中感测SHUNT电流和电压）
//       - 设备支持库从v150更改为v170
//         - TrigRegs更改为SyncSocRegs
//
//----------------------------------------------------------------------------------
//  日期	  | 描述 / 状态
//----------------------------------------------------------------------------------
// 2015年11月4日-PM BiSS-C库使用示例项目
//----------------------------------------------------------------------------------


/**********************************************************************************
 * 外设功能分配:
   EPWMs
		- EPWM1, EPWM2, EPWM3 ---> 逆变器PWMs相A, B, C
		- EPWM5  ---> Sigma Delta(调制器)时钟
		- EPWM6  ---> 解析器(位置传感器)反馈采样@ 160KHz
//XXX EPWM11使用 EPWM8替代
		- EPWM11 ---> 电机控制PWM同步SDFM滤波器窗口
		- EPWM4  ---> 如果EnDAT / BiSS界面处于活动状态，则不适用于用户

	SPIs
		- SPIB  ---> 如果EnDAT / BiSS界面处于活动状态，则不适用于用户

	模数转换通道：
	  ADC A4/C+  ---> Ifb-SV     --->V相电流
	  ADC B4/C+  ---> Ifb-SW     --->W相电流
	  ADC A2/C+  ---> LEM V      --->LEM传感器V相电流
	  ADC B2/C+  ---> LEM W      --->LEM传感器W相电流
//xxx ADC D1 使用 ADC C5替代
	  ADC D1     ---> R_SIN      --->解析器(位置传感器)正弦反馈
	  ADC C1     ---> R_COS      --->解析器(位置传感器)余弦反馈
	  ADC C3     ---> Vfb-U      --->U相电压
	  ADC A3     ---> Vfb-V      --->V相电压
	  ADC B3     ---> Vfb-W      --->W相电压
	  ADC B0     ---> Vfb-Bus    --->母线电压

	 模拟信号但未采样
	  ADC C2/C+  ---> Ifb-SU
	  ADC A5     --->
	  ADC C0/C+  ---> SC-A2
	  ADC D0/C+  ---> SC-B2
	  ADC D2/C+  ---> SC-R

	  DAC-A  ---> 解析器电流激励
	  DAC-B  ---> 通用指示器
	  DAC-C  ---> 通用指示器

 ********************************************************************************
 */
//包括主要功能中使用的头文件
//定义浮点数学，然后包含IQmath库

#include "IDDK_PM_Servo_F2837x-Settings.h"

#ifdef _FLASH
#pragma CODE_SECTION(MotorControlISR,"ramfuncs");
#pragma CODE_SECTION(ResolverISR,"ramfuncs");
#endif
/*
 * INTERRUPT编译指示使您可以直接使用C代码处理中断。在C语言中，参数func是函数的名称。
 * C语言中的编译指示语法为：
 * #pragma INTERRUPT（func）;

 * 除了_c_int00（为C程序的系统复位中断保留的名称）外，该中断的名称（func参数）不需要
 * 符合命名约定。在FPU上，有两种中断-高优先级中断（HPI）和低优先级中断（LPI）。
 * 高优先级中断使用快速上下文保存，并且不能嵌套。低优先级中断的行为类似于正常的C28x中断，
 * 可以嵌套。
 * 可以通过使用第二个可选参数的中断编译指示来指定中断的类型。
 * 编译指示的C语法为：
 * #pragma INTERRUPT（func，{HPI | LPI}）;

 * 要指定HPI中断，请使用HPI。要指定LPI中断，请使用LPI。在FPU上，如果没有中断优先级假定
 * 使用指定的LPI。用interrupt关键字指定的中断也默认为LPI。
 */
#pragma INTERRUPT (ResolverISR, HPI)
#pragma INTERRUPT (MotorControlISR, LPI)

//******************************函数声明****************************************
interrupt void MotorControlISR(void);
interrupt void ResolverISR(void);
void DeviceInit();
void MemCopy();
void InitFlash();
void HVDMC_Protection(void);
void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
void PWM_1ch_UpCnt_CNF(int16 n, Uint16 period);
void ConfigureADC(void);

_iq refPosGen(_iq out);
_iq ramper(_iq in, _iq out, _iq rampDelta);

// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

// ****************************************************************************
// 全局变量
// ****************************************************************************
// ADC静态校准
int *adc_cal;

//xxx 修改为删除&EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs
// 用于间接访问所有EPWM模块
volatile struct EPWM_REGS *ePWM[] = {
		&EPwm1Regs,
		&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
		&EPwm7Regs, &EPwm8Regs};

//xxx 修改为删除sdfm2Regs
//用于间接访问所有SDFM(数字滤波)模块
volatile struct SDFM_REGS *SDFM[2] = {0, &Sdfm1Regs};

//用于间接访问eQEP模块
volatile struct EQEP_REGS *eQEP[] = { &EQep1Regs,&EQep1Regs,&EQep2Regs,};

int16	VTimer0[4];			// 虚拟计时器CPU定时器0(A事件)
int16	VTimer1[4]; 		// 虚拟计时器CPU定时器1(B事件)
int16	VTimer2[4]; 		// 虚拟计时器CPU定时器2(C事件)
int16	SerialCommsTimer;

// 默认ADC初始化
int ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int	TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

//*************************** 用户变量 ****************************************
// 该系统中使用的全局变量
//****************************************************************************

// ****************************************************************************
// 电流测量变量
// ****************************************************************************
// 运行失调校准例程以校准运算放大器上的任何失调
_iq offset_Rsin,         // 解析器正弦fbk通道中的偏移@无激励
	offset_Rcos,         // 解析器cos fbk通道中的偏移@无激励
#if (CNGD == HOT)
	offset_shntV,        // 分流电流V fbk通道中的偏移@ 0A
    offset_shntW,        // 分流电流W fbk通道中的偏移@ 0A
	offset_shntU,        // 分流电流U fbk通道中的偏移@ 0A
#endif
    offset_lemV,         // LEM当前V fbk通道中的偏移@ 0A
	offset_lemW;         // LEM当前W fbk通道中的偏移@ 0A

int16 OffsetCalCounter;

//SDFM量程等级-附加工作范围
Uint16  HLT = 0x7FFF,
        LLT = 0x0;

volatile float offset_SDFM1;  // SDFM电流V fbk通道中的偏移@ 0A
volatile float offset_SDFM2;  // SDFM电流W fbk通道中的偏移@ 0A

_iq K1 = _IQ(0.998),		  // 偏移滤波器系数 K1: 0.05/(T+0.05);
    K2 = _IQ(0.001999);	      // 偏移滤波器系数 K2: T/(T+0.05);

//电流传感器采集的电流值
typedef struct {
	float As;      // A相
	float Bs;      // B相
	float Cs;      // C相
} CURRENT_SENSOR;

//不同电流传感器采集的电流值
CURRENT_SENSOR current_sensor[3];

//电流限制值
float curLimit = 8.0;

// 用于过电流保护的CMPSS参数
Uint16  clkPrescale = 20,
		sampwin     = 30,
		thresh      = 18,
		LEM_curHi   = LEM(8.0),
		LEM_curLo   = LEM(8.0),
		SHUNT_curHi = SHUNT(8.0),
		SHUNT_curLo = SHUNT(8.0);

// ****************************************************************************
// 标志变量
// ****************************************************************************
volatile Uint16 EnableFlag = FALSE; //系统使能标志

Uint32 IsrTicker = 0;

Uint16 BackTicker = 0;
Uint16 lsw = 0;
Uint16 TripFlagDMC = 0;				//PWM跳闸状态
Uint16 clearTripFlagDMC = 0;
Uint16 RunMotor = 0;

int    EnableResolverISR;
int    Run_Delay;
int    LedCnt1=0;

Uint16 SpeedLoopPrescaler = 10;      // 速度环标量
Uint16 SpeedLoopCount = 1;           // 速度环计数器

// ****************************************************************************
// 磁场定向控制的变量
// ****************************************************************************
float32 T = 0.001/ISR_FREQUENCY;    // 延迟时间（秒），请参阅parameter.h
_iq  VdTesting = _IQ(0.0);			// Vd 参考 (pu)
_iq  VqTesting = _IQ(0.10);			// Vq 参考 (pu)
_iq  IdRef     = _IQ(0.0);			// Id 参考 (pu)
_iq  IqRef     = _IQ(0.0);			// Iq 参考 (pu)
_iq  SpeedRef  = _IQ(0.0);           //闭环测试使用

// park/clarke变换对象实例
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;

// 实例PI（D）调节器可调节d和q轴电流，速度和位置
PIDREG3         pid_pos = PIDREG3_DEFAULTS;
PI_CONTROLLER   pi_pos  = PI_CONTROLLER_DEFAULTS;
PID_CONTROLLER	pid_spd = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PI_CONTROLLER   pi_id   = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER   pi_iq   = PI_CONTROLLER_DEFAULTS;

//PID_CONTROLLER pid_pos = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
//PI_CONTROLLER  pi_spd = PI_CONTROLLER_DEFAULTS;

// 实例PWM驱动器实例
//PWMGEN pwm1 = PWMGEN_DEFAULTS;

// 空间矢量PWM调制器实例。 该调制器根据d和q参考帧输入生成a，b和c相位
SVGEN svgen1 = SVGEN_DEFAULTS;

// 设置一个斜坡控制器以平滑地斜坡化频率
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS; // 速度使用

// 实例一个斜坡生成器来模拟电机角度
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//	实例相电压计算
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;

// 实例化基于编码器位置的速度计算器
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// 实例QEP接口驱动程序
QEP qep1 = QEP_DEFAULTS;

// 实例化解析器接口驱动程序
volatile RESOLVER resolver1 = RESOLVER_DEFAULTS;
volatile ABS_ENCODER endat1 = ABSENC_DEFAULTS;
volatile ABS_ENCODER biss1 = ABSENC_DEFAULTS;

// 位置传感器套件的变量
_iq  posEncElecTheta[6];
_iq  posEncMechTheta[6];

_iq  cntr=0;
_iq	 alignCnt = 20000;
_iq  IdRef_start = _IQ(0.1);
_iq	 IdRef_run   = _IQ(0.0);

// 用于位置参考生成和控制的变量
// =========================================================
_iq   posArray[8] = { _IQ(1.5), _IQ(-1.5), _IQ(2.5), _IQ(-2.5) };
_iq	  cntr1=0;
_iq	  posSlewRate = _IQ(0.001);

int16 ptrMax = 2;
int16 ptr1=0;

// ****************************************************************************
// 从resolver.c引用的外部函数和变量
// ****************************************************************************
extern void baseParamsInit(void);
extern void derivParamsCal(void);

extern RESOLVER_OUTPUT rslvrOut;

// ****************************************************************************
// 数据观测模块的变量
// ****************************************************************************
float DBUFF_4CH1[200],
      DBUFF_4CH2[200],
      DBUFF_4CH3[200],
      DBUFF_4CH4[200],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;

// 创建数据观测模块实例
DLOG_4CH_F dlog_4ch1;

//*******************************************************************************

// ******************************************************************************
// 电流传感器组件
// - 从逆变器底脚电流分流器读取电机电流
// - 从连接到电动机V和W相的LEM磁通门电流传感器读取电机电流
// - 使用隔离的Sigma Delta滤波器模块从连接到电动机的串联支路读取电动机电流
// ******************************************************************************
#if BUILDLEVEL != LEVEL1
inline void currentSensorSuite()
{
	volatile int16 temp;  // 临时变量，用于避免警告消息

#if (CNGD == HOT)
	current_sensor[SHUNT_CURRENT_SENSE-1].As = (float)IFB_SV_PPB* ADC_PU_PPB_SCALE_FACTOR;
	current_sensor[SHUNT_CURRENT_SENSE-1].Bs = (float)IFB_SW_PPB* ADC_PU_PPB_SCALE_FACTOR;
	current_sensor[SHUNT_CURRENT_SENSE-1].Cs = -current_sensor[SHUNT_CURRENT_SENSE-1].As
			                                   -current_sensor[SHUNT_CURRENT_SENSE-1].Bs;
#endif

	current_sensor[LEM_CURRENT_SENSE-1].As   = (float)IFB_LEMV_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor[LEM_CURRENT_SENSE-1].Bs   = (float)IFB_LEMW_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor[LEM_CURRENT_SENSE-1].Cs   = -current_sensor[LEM_CURRENT_SENSE-1].As
			                                   -current_sensor[LEM_CURRENT_SENSE-1].Bs;

	current_sensor[SD_CURRENT_SENSE-1].As    = ((temp=SDFM1_READ_FILTER1_DATA_16BIT)*SD_PU_SCALE_FACTOR -
			                                    offset_SDFM1) * SDFM_TO_SHUNT;
	current_sensor[SD_CURRENT_SENSE-1].Bs    = ((temp=SDFM1_READ_FILTER2_DATA_16BIT)*SD_PU_SCALE_FACTOR -
			                                    offset_SDFM2) * SDFM_TO_SHUNT;
	current_sensor[SD_CURRENT_SENSE-1].Cs    = -current_sensor[SD_CURRENT_SENSE-1].As
			                                   -current_sensor[SD_CURRENT_SENSE-1].Bs;

	return;
}
#endif

// ******************************************************************************
// 位置编码器组件
// - 读取QEP
// - 解析器解码（可在resolver.lib中使用核心算法）
// - 角度标准化为0到0.99999（1.0）的范围
// ******************************************************************************
void posEncoderSuite(void)
{
// ----------------------------------
// lsw = 0 ---> 对准程序
// ----------------------------------
#if (POSITION_ENCODER == QEP_POS_ENCODER)
	if (lsw == 0)
	{
		// 对准期间，将当前轴位置指定为初始位置
		EQep1Regs.QPOSCNT = 0;
		EQep1Regs.QCLR.bit.IEL = 1;  // 重置QEP初始位置
	}

// ******************************************************************************
//    检测校准角度并调用QEP模块
// ******************************************************************************
	// 一旦找到QEP索引脉冲，则转到lsw = 2
	if(lsw==1)
	{
		if (EQep1Regs.QFLG.bit.IEL == 1)
		{
			qep1.CalibratedAngle=EQep1Regs.QPOSILAT;
//			EQep1Regs.QPOSINIT = EQep1Regs.QPOSILAT;
//			EQep1Regs.QEPCTL.bit.IEI = IEI_RISING;
			lsw=2;
		}   // 将锁住的位置保持在第一个索引处
	}

	if (lsw!=0){
		QEP_MACRO(1,qep1);
	}

	// 如有必要，请反转位置传感器-相应地注释/取消注释
	// 按原样定位:
	//posEncElecTheta[QEP_POS_ENCODER] = qep1.ElecTheta;
	//posEncMechTheta[QEP_POS_ENCODER] = qep1.MechTheta;

	// 位置感应反转:
	posEncElecTheta[QEP_POS_ENCODER] = 1.0 - qep1.ElecTheta;
	posEncMechTheta[QEP_POS_ENCODER] = 1.0 - qep1.MechTheta;

// ******************************************************************************
//  读取解析器数据，获取位置和速度反馈
// ******************************************************************************
#elif (POSITION_ENCODER == RESOLVER_POS_ENCODER)
	// 对准期间，将当前轴位置指定为初始位置
	if (lsw == 0)
	{
		resolver1.InitTheta = resolver1.RawTheta;
	}

	resolver1.Speed    = rslvrOut.angleObs;
	resolver1.RawTheta = (rslvrOut.angleObs*0.5)+0.5;
	//RESOLVER_MACRO_F(resolver1)
	resolver1.MechTheta   = resolver1.RawTheta - resolver1.InitTheta;
	if(resolver1.MechTheta < 0)
		resolver1.MechTheta = resolver1.MechTheta + 1.0;
	else if (resolver1.MechTheta > 1.0)
		resolver1.MechTheta = resolver1.MechTheta - 1.0;

    // 计算Q24中的电角度
	resolver1.ElecTheta  = _IQfrac(resolver1.PolePairs * resolver1.MechTheta);

	// 如有必要，请反转位置传感器-相应地注释/取消注释
	// 按原样定位:
	posEncElecTheta[RESOLVER_POS_ENCODER] = resolver1.ElecTheta;
	posEncMechTheta[RESOLVER_POS_ENCODER] = resolver1.MechTheta;

	// 位置感应反转:
	//posEncElecTheta[RESOLVER_POS_ENCODER] = 1.0 - resolver1.ElecTheta;
	//posEncMechTheta[RESOLVER_POS_ENCODER] = 1.0 - resolver1.MechTheta;

#endif

#if POSITION_ENCODER==ENDAT_POS_ENCODER
//在EnDat21模式下读取位置数据。在endat.c中定义的功能
	if(endat22Data.dataReady == 1)
	{
	    endat21_readPosition();
	}

	endat1.RawTheta = (1.0-((float)endat22Data.position_lo)/33554432.0);; // 除以2 ^ 25：EnDat编码器25位

	// 对准期间，将当前轴位置指定为初始位置
	if (lsw == 0)
	{
		endat1.InitTheta = endat1.RawTheta;
	}

	endat1.MechTheta   = endat1.RawTheta - endat1.InitTheta;
	if(endat1.MechTheta < 0)
		endat1.MechTheta = endat1.MechTheta + 1.0;
	else if (endat1.MechTheta > 1.0)
		endat1.MechTheta = endat1.MechTheta - 1.0;

    // 计算Q24中的电角度
	endat1.ElecTheta  = _IQfrac(endat1.PolePairs * endat1.MechTheta);

	// 如有必要，请反转位置传感器-相应地注释/取消注释
	// 按原样定位:
//	posEncElecTheta[ENDAT_POS_ENCODER] = endat1.ElecTheta;
//	posEncMechTheta[ENDAT_POS_ENCODER] = endat1.MechTheta;

	// 位置感应反转:
	posEncElecTheta[ENDAT_POS_ENCODER] = 1.0 - endat1.ElecTheta;
	posEncMechTheta[ENDAT_POS_ENCODER] = 1.0 - endat1.MechTheta;

#endif
#if POSITION_ENCODER==BISS_POS_ENCODER
//在EnDat21模式下读取位置数据。在endat.c中定义的功能
	if(bissc_data_struct.dataReady == 1)
	{
				bissc_readPosition();
	}

	biss1.RawTheta = (1.0-((float)bissc_data_struct.position)/262144.0);; // 除以2 ^ 25：EnDat编码器25位

	// 对准期间，将当前轴位置指定为初始位置
	if (lsw == 0)
	{
		biss1.InitTheta = biss1.RawTheta;
	}

	biss1.MechTheta   = biss1.RawTheta - biss1.InitTheta;
	if(biss1.MechTheta < 0)
		biss1.MechTheta = biss1.MechTheta + 1.0;
	else if (biss1.MechTheta > 1.0)
		biss1.MechTheta = biss1.MechTheta - 1.0;

    // 计算Q24中的电角度
	biss1.ElecTheta  = _IQfrac(biss1.PolePairs * biss1.MechTheta);

	// 如有必要，请反转位置传感器-相应地注释/取消注释
	// 按原样定位:
	posEncElecTheta[BISS_POS_ENCODER] = biss1.ElecTheta;
	posEncMechTheta[BISS_POS_ENCODER] = biss1.MechTheta;

	// 位置感应反转:
//	posEncElecTheta[BISS_POS_ENCODER] = 1.0 - biss1.ElecTheta;
//	posEncMechTheta[BISS_POS_ENCODER] = 1.0 - biss1.MechTheta;

#endif

	return;
}

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

void main(void)
{

	volatile int16 temp;

	//初始化系统时钟:
	InitSysCtrl();

	// 等待启用标志设置
	while (EnableFlag == FALSE)
	{
	  BackTicker++;
	}

    //禁用CPU中断
	DINT;

    //初始化PIE管理器
	InitPieCtrl();

	//清除中断标志
	IER = 0x0000;
	IFR = 0x0000;

    //初始化中断向量表
	InitPieVectTable();

	//定时同步进行后台循环
	CpuTimer0Regs.PRD.all =  10000;		// A 任务50us
	CpuTimer1Regs.PRD.all =  20000;		// B 任务100us
	CpuTimer2Regs.PRD.all =  30000;	    // C 任务150us

//  任务状态机初始化
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;


// ****************************************************************************
// ****************************************************************************
//TODO PWM 配置
// ****************************************************************************
// ****************************************************************************

    // 初始化PWM模块
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	// ******************************************
	// 逆变器PWM配置
	// ******************************************
	/* 默认情况下，PWM时钟被2分频
	 * ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1
	 * 死区需要为2.0us => 10ns * 200 = 2us
	 */

//	PWM_1ch_UpDwnCnt_CNF(1,INV_PWM_TICKS,200);
//	PWM_1ch_UpDwnCnt_CNF(2,INV_PWM_TICKS,200);
//	PWM_1ch_UpDwnCnt_CNF(3,INV_PWM_TICKS,200);
    PWM_1ch_UpDwnCnt_CNF(1,10000,200);
    PWM_1ch_UpDwnCnt_CNF(2,10000,200);
    PWM_1ch_UpDwnCnt_CNF(3,10000,200);
	// **********************************************
	// Sigma Delta时钟设置-pwm5
	// *********************************************
	/* 为SD时钟配置PWM5A，即20Mhz
	 * 20 Mhz => 50ns => 50ns/10
	 */
	PWM_1ch_UpCnt_CNF(5,SDFM_TICKS);
	EPwm5Regs.CMPA.bit.CMPA=EPwm5Regs.TBPRD>>1;

	// **********************************************
	// 解析器的载波时钟-pwm6
	// *********************************************
	// 旋转变压器的PWM6时基
	PWM_1ch_UpDwnCnt_CNF(6,RESOLVER_PWM_TICKS,160);

//xxx EPWM11 修改为EPWM8
	// ********************************************************************
	//PWM11用于将SD滤波器窗口与电机控制PWM同步
	// ********************************************************************
	PWM_1ch_UpCnt_CNF(8,INV_PWM_TICKS);

	//将2和3配置为从属
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;//同步输出选择：EPWMxSYNC
	EPwm2Regs.TBCTL.bit.PHSEN    = TB_ENABLE;//允许加载相位寄存器值
	EPwm2Regs.TBPHS.bit.TBPHS    = 2;//相位寄存器值
	EPwm2Regs.TBCTL.bit.PHSDIR   = TB_UP;//相位方向：同步事件后递增计数

	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;//同步输出选择：EPWMxSYNC
	EPwm3Regs.TBCTL.bit.PHSEN    = TB_ENABLE;//允许加载相位寄存器值
	EPwm3Regs.TBPHS.bit.TBPHS    = 2;//相位寄存器值
	EPwm3Regs.TBCTL.bit.PHSDIR   = TB_UP;//相位方向：同步事件后递增计数

	SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0; 	 //EPwm1SyncOut

	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm6Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm6Regs.TBPHS.bit.TBPHS    = 2;
	EPwm6Regs.TBCTL.bit.PHSDIR   = TB_UP;

//XXX EPWM10 X修改为EPWM7
	SyncSocRegs.SYNCSELECT.bit.EPWM7SYNCIN = 0;  //EPwm1Sync Out
	EPwm7Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;
//XXX EPWM11配置修改为EPWM8
	EPwm8Regs.TBCTL.bit.PHSEN  = TB_ENABLE;
	EPwm8Regs.TBPHS.bit.TBPHS  = 2;
	EPwm8Regs.TBCTL.bit.PHSDIR = TB_UP;

	EPwm8Regs.CMPC = EPwm8Regs.TBPRD - SDFM_TICKS*(OSR_RATE+1)*3/2;
	EPwm8Regs.CMPA.bit.CMPA = (SDFM_TICKS*(OSR_RATE+1)*3/2) + 500; // 500 is arbitrary
	EPwm8Regs.CMPD = 0;

//xxx PWM GPIO配置
	// ***********************************
	// Set up GPIOs for PWM functions
	// **************************************
extern InitEPwm1Gpio();
extern InitEPwm2Gpio();
extern InitEPwm3Gpio();
extern InitEPwm4Gpio();
extern InitEPwm5Gpio();
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
	InitEPwm5Gpio();

	EDIS;

// ****************************************************
// Initialize DATALOG module
// ****************************************************
	DLOG_4CH_F_init(&dlog_4ch1);
	dlog_4ch1.input_ptr1 = &DlogCh1;	//data value
	dlog_4ch1.input_ptr2 = &DlogCh2;
	dlog_4ch1.input_ptr3 = &DlogCh3;
	dlog_4ch1.input_ptr4 = &DlogCh4;
	dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
	dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
	dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
	dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
	dlog_4ch1.size = 200;
	dlog_4ch1.pre_scalar = 5;
	dlog_4ch1.trig_value = 0.01;
	dlog_4ch1.status = 2;



// ****************************************************************************
// ****************************************************************************
//TODO ADC配置
// ****************************************************************************
// ****************************************************************************
    //初始化ADC模块并上电
	ConfigureADC();

	//选择要转换的通道触发方式和转换结束标志
	//电流测量有多个选项
	EALLOW;

	// 采样的模拟信号：
	// Ifb-SV  ADC A4/C+
	// Ifb-SW  ADC B4/C+
	// LEM V   ADC A2/C+
	// LEM W   ADC B2/C+
	// R_SIN   ADC D1       //ADC位置修改到C4上
	// R_COS   ADC C1
	// Vfb-U   ADC C3
	// Vfb-V   ADC A3
	// Vfb-W   ADC B3
	// Vfb-Bus ADC B0

	// 引入模拟信号但未采样：
	// Ifb-SU  ADC C2/C+ (& A5 not used)
	// SC-A2   ADC C0/C+
	// SC-B2   ADC D0/C+
	// SC-R    ADC D2/C+
	// Bus Volt ADC B0


#if (CGND == HOT)
	// 并联电流传感器 (SV) @ A4
	// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // ADC_A SOC0 连接引脚A4
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB与SOC0相关联
	AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR
	// 并联电流传感器 (SW) @ B4
	// ********************************
	AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // ADC_B SOC0 连接引脚B4
	AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB与SOC0相关联
	AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR
#endif
	// 位置解析器Fbk - cos信号@ C1
	// ********************************
	AdccRegs.ADCSOC0CTL.bit.CHSEL     = 1;    // SOC0连接引脚C1
	AdccRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // 触发ePWM6 SOCA/C
	AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB与SOC0相关联
	AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR
	//xxx 位置解析器Fbk - sin信号 @ D1-->C4
	// ********************************
	AdccRegs.ADCSOC5CTL.bit.CHSEL     = 4;    // SOC5 连接引脚 C4
	AdccRegs.ADCSOC5CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdccRegs.ADCSOC5CTL.bit.TRIGSEL   = 15;   // 触发ePWM6 SOCA/C
	AdccRegs.ADCPPB4CONFIG.bit.CONFIG = 5;    // PPB4与SOC5相关联
	AdccRegs.ADCPPB4OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR

	// LEM电流传感器 LEM-V @ A2
	// ********************************
	AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 2;    // SOC1连接到引脚A2
	AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB与SOC1相关联
	AdcaRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR

	// LEM电流传感器 LEM-W @ B2
	// ********************************
	AdcbRegs.ADCSOC1CTL.bit.CHSEL     = 2;    // SOC1连接到引脚B2
	AdcbRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdcbRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB与SOC1相关联
	AdcbRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR

	// 相电压 Vfb-V @ A3
	// ***************************
	AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // ADC_A SOC2连接到引脚A3
	AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB与SOC2相关联
	AdcaRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR

	// 相电压 Vfb-W @ B3
	// ***************************
	AdcbRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // ADC_B SOC2连接到引脚B3
	AdcbRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdcbRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB与SOC2相关联
	AdcbRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR

	// 相电压 Vfb-U @ C3
	// ****************************
	AdccRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // ADC_C SOC2连接到引脚C3
	AdccRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // 采样窗口时间
	AdccRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // 触发ePWM1 SOCA/C
	AdccRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB与SOC2相关联
	AdccRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // 写入零，等待运行偏移ISR

	// Bus Voltage Feedback at B0 (未使用)
	// **************************************
	AdcbRegs.ADCSOC3CTL.bit.CHSEL    = 0;     // ADC_B SOC3连接到引脚B0
	AdcbRegs.ADCSOC3CTL.bit.ACQPS    = 30;    // 采样窗口时间
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;     // 触发ePWM1 SOCA/C

	// ******************************************************
	// 所有ADC的静态模拟调整 (A, B, C and D)
	// *******************************************************
	adc_cal=(int*)0x0000743F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000074BF;
	*adc_cal=0x7000;
	adc_cal=(int*)0x0000753F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000075BF;
	*adc_cal=0x7000;

	// 设置从EPWM到ADC的链接
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // 选择SOCA在ctr = 0时触发
	EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // 在第1个事件产生脉冲
	EPwm1Regs.ETSEL.bit.SOCAEN  = 1;          // 使能SOCA脉冲

	EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // 选择SOCA在ctr = 0时触发
	EPwm6Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // 在第1个事件产生脉冲
	EPwm6Regs.ETSEL.bit.SOCAEN  = 1;          // 使能SOCA脉冲

//XXX EPWM11 修改为EPWM8
	EPwm8Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // 中断周期CMPA事件
	EPwm8Regs.ETSEL.bit.INTEN  = 1;              // 使能中断
	EPwm8Regs.ETPS.bit.INTPRD  = ET_1ST;         // 触发中断在每个事件

	// SETUP DACS
	DacaRegs.DACCTL.bit.DACREFSEL = REFERENCE_VREF;
	DacaRegs.DACCTL.bit.LOADMODE  = 1;      //仅在同步信号上启用值更改
	//使能DAC输出
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacaRegs.DACCTL.bit.SYNCSEL    = 5;     //5表示从pwm 6同步
	DacaRegs.DACVALS.bit.DACVALS   = 1024;

	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//使能DAC输出
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacbRegs.DACVALS.bit.DACVALS   = 1024;

//XXX 删除DAC_C
//	DaccRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//使能DAC输出
//	DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
//	DaccRegs.DACVALS.bit.DACVALS   = 1024;
	EDIS;

// ****************************************************************************
// ****************************************************************************
//TODO 	Sigma Delta 初始化
// ****************************************************************************
// ****************************************************************************
    // 设置GPIO用于SD电流测量
	GPIO_SetupPinOptions(48, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(48,0,7);

	GPIO_SetupPinOptions(49, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(49,0,7);

	GPIO_SetupPinOptions(50, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(50,0,7);

	GPIO_SetupPinOptions(51, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(51,0,7);

	// 设置GPIO用于SD电流测量
	GPIO_SetupPinOptions(52, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(52,0,7);

	GPIO_SetupPinOptions(53, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(53,0,7);

	/*******************************************************/
	/* 输入控制模块 */
	/*******************************************************/
	//配置输入控制模式：调制器时钟速率=调制器数据速率
	Sdfm_configureInputCtrl(1,FILTER1,MODE_0);
	Sdfm_configureInputCtrl(1,FILTER2,MODE_0);
	Sdfm_configureInputCtrl(1,FILTER3,MODE_0);

	/*******************************************************/
	/* 比较器模块 */
	/*******************************************************/
	//比较器HLT和LLT
	//配置比较器模块的比较器滤波器类型和比较器的OSR值，
	//高电平阈值，低电平阈值
	Sdfm_configureComparator(1, FILTER1, SINC3, OSR_32, HLT, LLT);
	Sdfm_configureComparator(1, FILTER2, SINC3, OSR_32, HLT, LLT);
	Sdfm_configureComparator(1, FILTER3, SINC3, OSR_32, HLT, LLT);

	/*******************************************************/
	/* 过滤器模块 */
	/*******************************************************/
	//配置数据过滤器模块的过滤器类型，OSR值并启用/禁用数据过滤器
	//从TRM的表中使用Sinc3为OSR 128选择16位数据表示
	//使用Sinc 3为OSR 128表示的最大值是+/- 2097152，即2 ^ 21
	//以16位格式表示，其中第一位是符号移位6位
	Sdfm_configureData_filter(1, FILTER1, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);
	Sdfm_configureData_filter(1, FILTER2, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);
	Sdfm_configureData_filter(1, FILTER3, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);

	//PWM11.CMPC、PWM11.CMPD、PWM12.CMPC和PWM12.CMPD信号无法同步过滤器。 在此示例中未使用此选项。
    Sdfm_configureExternalreset(1,FILTER_1_EXT_RESET_ENABLE, FILTER_2_EXT_RESET_ENABLE, FILTER_3_EXT_RESET_ENABLE, FILTER_4_EXT_RESET_ENABLE);

    // 使能SDFM模块1
    Sdfm_enableMFE(1);

// ****************************************************************************
// ****************************************************************************
// 初始化QEP模块
// ****************************************************************************
// ****************************************************************************
    // 设置QEP模块引脚
	GPIO_SetupPinOptions(20, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(20,0,1);

	GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(21,0,1);

	GPIO_SetupPinOptions(22, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(22,0,1);

	GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(23,0,1);

// ****************************************************************************
// ****************************************************************************
// 使用GPIO 42和GPIO43
// ****************************************************************************
// ****************************************************************************
	//指示LED
	GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(34,0,0);

	GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(42,0,0);

	GPIO_SetupPinOptions(43, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(43,0,0);

// ****************************************************************************
// ****************************************************************************
// TODO 参数初始化
// ****************************************************************************
// ****************************************************************************

	// 初始化QEP参数
    qep1.LineEncoder = 2500; // 这些是QEP编码器中的插槽数
    qep1.MechScaler  = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs   = POLES/2;
    qep1.CalibratedAngle = 0;
    QEP_INIT_MACRO(1,qep1)
    EQep1Regs.QEPCTL.bit.IEI = 0;

    // 初始化RESOLVER参数
	resolver1.StepsPerTurn = RESOLVER_STEPS_PER_TURN;
	resolver1.MechScaler   =  1.0;
	resolver1.PolePairs    = POLES/2;

	// 初始化BiSS-C参数
	biss1.PolePairs    = POLES/2;
	// 初始化EnDat22参数
	endat1.PolePairs    = POLES/2;

	baseParamsInit();                    // 初始化所有参数
	derivParamsCal();                    // 设置微分循环参数
	init_resolver_Float();

    // 初始化速度模块以通过QEP / RESOLVER进行速度计算
    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));      // 低通截止频率
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 120*(BASE_FREQ/POLES);

    // 初始化RAMPGEN模块
    rg1.StepAngleMax = _IQ(BASE_FREQ*T);

    // 初始化PI模块（位置环）
	pi_pos.Kp = _IQ(1.0);            //_IQ(10.0);
	pi_pos.Ki = _IQ(0.001);          //_IQ(T*SpeedLoopPrescaler/0.3);
	pi_pos.Umax = _IQ(1.0);
	pi_pos.Umin = _IQ(-1.0);

    // 初始化PID模块的位置（eval的替代选项）
    pid_pos.Ref = 0;
    pid_pos.Fdb = 0;
    pid_pos.OutMin = _IQ(-0.5);
    pid_pos.OutMax = _IQ(0.5);
    pid_pos.Out = 0;

    pid_pos.Kp = 1.0;
    pid_pos.Ki = 0;
    pid_pos.Kd = 0;
    pid_pos.Kc = 0.9;

    pid_pos.Up1 = 0;
	pid_pos.Up  = 0;
	pid_pos.Ui  = 0;
	pid_pos.Ud  = 0;
	pid_pos.SatErr    = 0;
	pid_pos.OutPreSat = 0;

	// endat操作的初始化例程-在endat.c中定义
	//配置外围设备并启用所需模块的时钟
	//根据需要配置GPIO和XBar，以实现EnDat操作
	//以endat数据结构设置SPI外设并启用中断
#if POSITION_ENCODER==ENDAT_POS_ENCODER
	EnDat_Init();
	//执行电缆传播延迟计算。
	//这对于较长的电缆长度和更高的EnDat Clock频率是必需的
	//在endat.c中定义的函数

	EnDat_initDelayComp();

	//切换到高频 - 8.3MHz	(=200/4*ENDAT_RUNTIME_FREQ_DIVIDER)
	PM_endat22_setFreq(ENDAT_RUNTIME_FREQ_DIVIDER);
	DELAY_US(800L); 	//Delay 800us
	endat22Data.dataReady = 1;

#endif

#if POSITION_ENCODER==BISS_POS_ENCODER
	bissc_init();
	DELAY_US(800L); 	//Delay 800us
	bissc_data_struct.dataReady = 1;

#endif

	// 初始化PID模块（速度环）
#if (BUILDLEVEL==LEVEL5) || (BUILDLEVEL == LEVEL6)
//	pid_spd.param.Kp=_IQ(2.5);
//	pid_spd.param.Ki=_IQ(0.0001);
//	pid_spd.param.Kd=_IQ(0.0);
//	pid_spd.param.Kr=_IQ(1.0);
//	pid_spd.param.Umax=_IQ(0.9);
//	pid_spd.param.Umin=_IQ(-0.9);
	pid_spd.param.Kp=_IQ(1.0);
	pid_spd.param.Ki=_IQ(0.001);
	pid_spd.param.Kd=_IQ(0.0);
	pid_spd.param.Kr=_IQ(1.0);
	pid_spd.param.Umax=_IQ(0.95);
	pid_spd.param.Umin=_IQ(-0.95);
#else
	pid_spd.param.Kp   = _IQ(1.0);
	pid_spd.param.Ki   = _IQ(0.001);
	pid_spd.param.Kd   = _IQ(0.0);
	pid_spd.param.Kr   = _IQ(1.0);
	pid_spd.param.Umax = _IQ(0.95);
	pid_spd.param.Umin = _IQ(-0.95);
#endif

	// 初始化PI模块（Id）
	pi_id.Kp   = _IQ(1.0);//_IQ(3.0);
	pi_id.Ki   = _IQ(T/0.04);//0.0075);
	pi_id.Umax = _IQ(0.5);
	pi_id.Umin = _IQ(-0.5);

	// 初始化PI模块（Iq）
	pi_iq.Kp   = _IQ(1.0);//_IQ(4.0);
	pi_iq.Ki   = _IQ(T/0.04);//_IQ(0.015);
	pi_iq.Umax = _IQ(0.8);
	pi_iq.Umin = _IQ(-0.8);

	// 为Speed和Iq循环设置模拟参考
	SpeedRef = 0.05;

#if BUILDLEVEL == LEVEL3
	IqRef    = 0.0;
#else
	IqRef = _IQ(0.05);
#endif

	// 初始FLAGS
	RunMotor = 0;
	LedCnt1  = 0;
	EnableResolverISR = 1;

#if (BUILDLEVEL==LEVEL2)
	Run_Delay = 10;
#else
	Run_Delay = 100;
#endif

	/*注意，d-q PI输出的矢量和应小于1.0，这是SVGEN的最大占空比。 另一个占空比限制因素是通过分流电阻
	的电流检测，这取决于硬件/软件的实现方式。 根据应用要求，可将3,2或单个并联电阻用于电流波形重建。 并
	联电阻的数量越多，占空比越高，直流总线的利用率越高。 如项目手册中所述，用户应在开环测试
	（即pi_id.Umax，pi_iq.Umax和Umins）中仔细调整PI饱和度水平。 违反此程序会产生失真的电流波
	形和不稳定的闭环操作，这可能会损坏逆变器。*/

// ****************************************************************************
// ****************************************************************************
// 调用HVDMC保护函数
// ****************************************************************************
// ****************************************************************************
	HVDMC_Protection();

// TODO
// ****************************************************************************
// ****************************************************************************
// 反馈偏移校准程序
// ****************************************************************************
// ****************************************************************************
	EALLOW;
	  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	offset_Rsin  = 0;
	offset_Rcos  = 0;
#if (CNGD == HOT)
	offset_shntV = 0;
	offset_shntW = 0;
	offset_shntU = 0;
#endif
	offset_lemW  = 0;
	offset_lemV  = 0;
	offset_SDFM1 = 0;
	offset_SDFM2 = 0;

	for (OffsetCalCounter=0; OffsetCalCounter<20000; )
	{
//XXX EPWM11修改为EPWM8
		if(EPwm8Regs.ETFLG.bit.INT==1)
		{
			if(OffsetCalCounter>1000)
			{
				offset_SDFM1 = K1*offset_SDFM1 + K2*(temp=SDFM1_READ_FILTER1_DATA_16BIT)*SD_PU_SCALE_FACTOR;
				offset_SDFM2 = K1*offset_SDFM2 + K2*(temp=SDFM1_READ_FILTER2_DATA_16BIT)*SD_PU_SCALE_FACTOR;
#if (CNGD == HOT)
				offset_shntV = K1*offset_shntV + K2*(IFB_SV)*ADC_PU_SCALE_FACTOR; 			//Phase A offset
				offset_shntW = K1*offset_shntW + K2*(IFB_SW)*ADC_PU_SCALE_FACTOR; 			//Phase B offset
#endif
				offset_lemV  = K1*offset_lemV + K2*(IFB_LEMV)*ADC_PU_SCALE_FACTOR;
				offset_lemW  = K1*offset_lemW + K2*(IFB_LEMW)*ADC_PU_SCALE_FACTOR;

				offset_Rsin  = K1*offset_Rsin + K2*R_SIN*ADC_PU_SCALE_FACTOR;
				offset_Rcos  = K1*offset_Rcos + K2*R_COS*ADC_PU_SCALE_FACTOR;
			}
//XXX EPWM11修改为EPWM8
			EPwm8Regs.ETCLR.bit.INT=1;
			OffsetCalCounter++;
		}
	}

	// ********************************************
	// 具有标识值的初始化偏移量
	// ********************************************
	EALLOW;
#if (CNGD == HOT)
	AdcaRegs.ADCPPB1OFFREF = (offset_shntV*4096.0);     // 设置分流电流  Iu 偏移
	AdcbRegs.ADCPPB1OFFREF = (offset_shntW*4096.0);     // 设置分流电流  Iv 偏移
#endif
	AdccRegs.ADCPPB1OFFREF = (offset_Rcos*4096.0);      // 设置解析器 cos 偏移
//xxx 解析器 sin 偏移 adc ppb选择
	AdccRegs.ADCPPB4OFFREF = (offset_Rsin*4096.0);      // 设置解析器 sin 偏移

	AdcaRegs.ADCPPB2OFFREF = (offset_lemV*4096.0);      // 设置 LEM Iv 偏移
	AdcbRegs.ADCPPB2OFFREF = (offset_lemW*4096.0);      // 设置 LEM Iw 偏移

	EDIS;

// ****************************************************************************
// ****************************************************************************
//TODO 中断设置
// ****************************************************************************
// ****************************************************************************
	EALLOW;
	// ADC-C的SOC0的 EOC用于触发解析器中断
	AdccRegs.ADCINTSEL1N2.bit.INT1SEL  = 0;
	AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdccRegs.ADCINTSEL1N2.bit.INT1E    = 1;

//XXX EPWM11修改为EPWM8
	//PWM11 INT用于触发电机控制ISR
	EPwm8Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // 中断在CMPA事件
	EPwm8Regs.ETSEL.bit.INTEN  = 1;              // 使能中断
	EPwm8Regs.ETPS.bit.INTPRD  = ET_1ST;         // 中断发生在每个事件

	//配置解析器中断向量:ADC_C1--解析器sin信号
	PieVectTable.ADCC1_INT = &ResolverISR;

//XXX EPWM11修改为EPWM8
	//配置电机控制中断向量:EPWM11
	PieVectTable.EPWM8_INT = &MotorControlISR;
//XXX EPWM11修改为EPWM8
    //使能PWM8 PIE中断3.8
	PieCtrlRegs.PIEIER3.bit.INTx8 = 1;

#if POSITION_ENCODER == RESOLVER_POS_ENCODER
	//使能ADC_C1 PIE中断1.3
	PieCtrlRegs.PIEIER1.bit.INTx3  = 1;
#endif
//XXX EPWM11修改为EPWM8
    //清除中断标志
	EPwm8Regs.ETCLR.bit.INT=1;

	IER |= M_INT3; // 使能组3中断
	IER |= M_INT1; // 使能组1中断
	EINT;          // 使能全局中断
	ERTM;
	EDIS;

// ***************************************************************************
	//初始化完成
	//IDLE任务循环
// ***************************************************************************
	for(;;)
	{
		//状态机入口和出口点
		//===========================================================
		(*Alpha_State_Ptr)();	//在各任务之间跳转(A0,B0,...)
		//===========================================================
	}
} //END MAIN CODE

/******************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 */

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// A任务的循环速率同步器
	if(CpuTimer0Regs.TCR.bit.TIF == 1)
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	//清除标志

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// 执行A任务中某个子任务(A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// 虚拟计时器0
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// 跳转到B任务
}

void B0(void)
{
	// B任务的循环速率同步器
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;				//清除标志

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// 执行B任务中某个子任务(B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// 虚拟计时器1
	}

	Alpha_State_Ptr = &C0;		// 跳转到C任务
}

void C0(void)
{
	// C任务的循环速率同步器
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;				//清除标志

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// 执行C任务中某个子任务(C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			// 虚拟计时器2
	}

	Alpha_State_Ptr = &A0;	// 跳转到A任务
}


//=================================================================================
//	A - TASKS (每50us执行一次)
//=================================================================================

// 设置CMPSS的OCP限制和数字滤波器参数
void  CMPSS_DIG_FILTER(volatile struct CMPSS_REGS *v, Uint16 curHi, Uint16 curLo)
{
	// 比较器参考
	v->DACHVALS.bit.DACVAL = curHi;   // 正最大电流限制
	v->DACLVALS.bit.DACVAL = curLo;   // 负最大电流限制

	// 数字滤波器设置-高侧
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale;    // 设置样本之间的时间，最大值：1023
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;        // 窗口中的样本数量，最大值：31
	v->CTRIPHFILCTL.bit.THRESH         = thresh;         // 推荐: thresh > sampwin/2

	// 数字滤波器设置-低侧
	v->CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale;    // 设置样本之间的时间，最大值：1023
	v->CTRIPLFILCTL.bit.SAMPWIN        = sampwin;        // 窗口中的样本数量，最大值：31
	v->CTRIPLFILCTL.bit.THRESH         = thresh;         // 推荐: thresh > sampwin/2

	return;
}

//--------------------------------------------------------
void A1(void)
//--------------------------------------------------------
{
	// *******************************************************
	// 电流限制设置/调试环境中的调整
	// *******************************************************
	EALLOW;
	  LEM_curHi = 2048 + LEM(curLimit);
	  LEM_curLo = 2048 - LEM(curLimit);
	  SHUNT_curHi = 2048 + SHUNT(curLimit);
	  SHUNT_curLo = 2048 - SHUNT(curLimit);

	  CMPSS_DIG_FILTER(&Cmpss1Regs, LEM_curHi, LEM_curLo);      // LEM - V
	  CMPSS_DIG_FILTER(&Cmpss3Regs, LEM_curHi, LEM_curLo);      // LEM - W
#if (CGND == HOT)
	  CMPSS_DIG_FILTER(&Cmpss2Regs, SHUNT_curHi, SHUNT_curLo);  // SHUNT - V
	  CMPSS_DIG_FILTER(&Cmpss6Regs, SHUNT_curHi, SHUNT_curLo);  // SHUNT - U
#endif
	EDIS;

	//检查由于过电流导致的PWM跳闸
	if (EPwm1Regs.TZFLG.bit.OST ||
		EPwm2Regs.TZFLG.bit.OST ||
		EPwm3Regs.TZFLG.bit.OST
	   )
	{
		//如果设置了EPwm的OST，则将所有三个的OST强制为DISABLE逆变器
		EALLOW;
		  EPwm1Regs.TZFRC.bit.OST = 1;
		  EPwm2Regs.TZFRC.bit.OST = 1;
		  EPwm3Regs.TZFRC.bit.OST = 1;
		EDIS;
	    TripFlagDMC = 1;      // Trip on DMC (halt and IPM fault trip )
	    RunMotor = 0;
	}

	//如果收到清除的cmd，请重置PWM跳闸
	if (clearTripFlagDMC)
	{
		GpioDataRegs.GPBDAT.bit.GPIO41 = 0;  //清除宏M6中的ocp锁存器
		TripFlagDMC = 0;
		clearTripFlagDMC = 0;
		GpioDataRegs.GPBDAT.bit.GPIO41 = 1;

		DELAY_US(1L);
		EALLOW;
		  // 清除 OST 标志
		  EPwm1Regs.TZCLR.bit.OST = 1;
		  EPwm2Regs.TZCLR.bit.OST = 1;
		  EPwm3Regs.TZCLR.bit.OST = 1;

		  // 清除 DCAEVT1 标志
		  EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
		  EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
		  EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;

		  // 清除HLATCH
		  Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = 1;
		  Cmpss3Regs.COMPSTSCLR.bit.HLATCHCLR = 1;
		  Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1;
		  Cmpss6Regs.COMPSTSCLR.bit.HLATCHCLR = 1;

		  // 清除LLATCH
		  Cmpss1Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		  Cmpss3Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		  Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		  Cmpss6Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		EDIS;
	}

	//-------------------
	//下次CpuTimer0定时值到达，将转到A2
	A_Task_Ptr = &A2;
	//-------------------
}

//-----------------------------------------------------------------
void A2(void)
//-----------------------------------------------------------------
{

	//-------------------
    //下次CpuTimer0定时值到达，将转到A3
	A_Task_Ptr = &A3;
	//-------------------
}

//-----------------------------------------
void A3(void)
//-----------------------------------------
{

	//-----------------
    //下次CpuTimer0定时值到达，将转到A1
	A_Task_Ptr = &A1;
	//-----------------
}



//=================================================================================
//	B - TASKS (每100us执行一次)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void)
//----------------------------------------
{

	//-----------------
	//下次CpuTimer1定时值到达，将转到B2
	B_Task_Ptr = &B2;
	//-----------------
}

//----------------------------------------
void B2(void)
//----------------------------------------
{

	//-----------------
    //下次CpuTimer1定时值到达，将转到B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void)
//----------------------------------------
{

	//-----------------
    //下次CpuTimer1定时值到达，将转到B1
	B_Task_Ptr = &B1;
	//-----------------
}


//=================================================================================
//	C - TASKS (每150us执行一次)
//=================================================================================

//--------------------------------- USER ------------------------------------------

//----------------------------------------
void C1(void) 	// 翻转 GPIO-34
//----------------------------------------
{

	if(LedCnt1==0)
	{
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // LED闪烁
		LedCnt1 = 200;
	}
	else
	{
        LedCnt1--;
	}


	//-----------------
	//下次CpuTimer2定时值到达，将转到C2
	C_Task_Ptr = &C2;
	//-----------------

}

//----------------------------------------
void C2(void)
//----------------------------------------
{

	//-----------------
    //下次CpuTimer2定时值到达，将转到C3
	C_Task_Ptr = &C3;
	//-----------------
}


//-----------------------------------------
void C3(void)
//-----------------------------------------
{

	//-----------------
    //下次CpuTimer2定时值到达，将转到C1
	C_Task_Ptr = &C1;
	//-----------------
}

// ****************************************************************************
// ****************************************************************************
//TODO 电机控制ISR
// ****************************************************************************
// ****************************************************************************
interrupt void MotorControlISR(void)
{

	EINT;

	//中断计数器
    IsrTicker++;


// =============================== LEVEL 1 ======================================
//    检查目标独立模块，占空比波形和PWM更新，在此级别保持电机断开连接
// ==============================================================================

//TODO 构建等级1
#if (BUILDLEVEL == LEVEL1)

// ------------------------------------------------------------------------------
//  连接RMP模块的输入并调用斜坡控制宏
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  连接RAMP GEN模块的输入并调用斜坡发生器宏
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  连接INV_PARK模块的输入和调用INV_PARK
//	三角函数有两个选项：
//  IQ sin/cos查找表以Q30格式提供512个离散的sin和cos点。
//  IQsin / cos PU函数对查询表中的数据进行插值，从而获得更高的分辨率。
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

    park1.Angle  = rg1.Out;
	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);

	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  连接SVGEN_DQ模块的输入，并调用空间矢量 gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  计算占空比并写入CMPA寄存器
// ------------------------------------------------------------------------------
 	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//  连接DATALOG模块
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = svgen1.Ta;
	DlogCh3 = svgen1.Tb;
	DlogCh4 = svgen1.Tc;

//------------------------------------------------------------------------------
// 变量显示在 DACs B 和 C
//------------------------------------------------------------------------------
	if (rslvrIn.TUNING)
	{
		DacbRegs.DACVALS.bit.DACVALS = rslvrOut.angleRaw*4096;
//xxx 删除DAC_C数据显示器模块
//		DaccRegs.DACVALS.bit.DACVALS = rslvrOut.angleOut*4096;
	}
	else
	{
		DacbRegs.DACVALS.bit.DACVALS = (svgen1.Tb*0.5+0.5)*4096;
//xxx 删除DAC_C数据显示器模块
//		DaccRegs.DACVALS.bit.DACVALS = (svgen1.Tc*0.5+0.5)*4096;
	}

#endif // (BUILDLEVEL==LEVEL1)

// =============================== LEVEL 2 ======================================
//	  Level 2 verifies
//	     - all current sense schems
//         - analog-to-digital conversion (shunt and LEM)
//         - SDFM function
//       - Current Limit Settings for over current protection
//       - clarke/park transformations (CLARKE/PARK)
//       - Position sensor interface
//         - speed estimation
// ==============================================================================

//TODO 构建等级2
#if (BUILDLEVEL==LEVEL2)

	// ------------------------------------------------------------------------------
	// Alignment Routine: this routine aligns the motor to zero electrical angle
	// and in case of QEP also finds the index location and initializes the angle
	// w.r.t. the index location
	// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

#if POSITION_ENCODER==QEP_POS_ENCODER
		lsw = 1;   // for QEP, spin the motor to find the index pulse
#else
		lsw  = 2;  // for absolute encoders no need for lsw=1
#endif
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
	if(lsw==0)rc1.TargetValue = 0;
	else rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha  = clarke1.Alpha;
	park1.Beta   = clarke1.Beta;
	park1.Angle  = rg1.Out;
	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = svgen1.Ta;
	DlogCh3 = clarke1.As;
	DlogCh4 = clarke1.Bs;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
	DaccRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;


#endif // (BUILDLEVEL==LEVEL2)


// =============================== LEVEL 3 ======================================
//	Level 3 verifies the dq-axis current regulation performed by PID and speed
//	measurement modules
//  lsw=0: lock the rotor of the motor
//  lsw=1: close the current loop
//  NOTE:- Iq loop is closed using internal ramp angle as position feedback.
//	       Therefore, motor speed does not race high with lighter load. User's
//         wanting to use actual rotor angle should ensure that the test value
//         for Iq reference will not race the motor to high speeds. In other
//         words, to use the actual angle, a loaded motor is needed.
// ==============================================================================

//TODO 构建等级3
#if (BUILDLEVEL==LEVEL3)

	// ------------------------------------------------------------------------------
	// Alignment Routine: this routine aligns the motor to zero electrical angle
	// and in case of QEP also finds the index location and initializes the angle
	// w.r.t. the index location
	// ------------------------------------------------------------------------------
	if(!RunMotor)
	{
		lsw = 0;
		pi_id.ui = 0;
		pi_iq.ui = 0;
	}
	else if (lsw == 0)
	{
		// alignment current
		IdRef = IdRef_start;  //IQ(0.1);

		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

		// set up an alignment and hold time for shaft to settle down
		if (pi_id.Ref >= IdRef)
		{
			if (cntr < alignCnt)
				cntr++;
			else
			{
#if POSITION_ENCODER==QEP_POS_ENCODER
				lsw = 1;      // for QEP, spin the motor to find the index pulse
				IqRef = _IQ(0.05);
#else
				lsw = 2;   // for absolute encoders no need for lsw=1
#endif
				cntr  = 0;
				IdRef = IdRef_run;
			}
		}
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;

	if(lsw==0) park1.Angle = 0;
	else if (lsw==1)  park1.Angle = rg1.Out; // this state exists only for QEP
	else park1.Angle = rg1.Out; // posEncElecTheta[POSITION_ENCODER];

	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
//	if(lsw==0) 	pi_iq.Ref = 0;
//	else		pi_iq.Ref= IqRef;
	pi_iq.Ref =  IqRef * (lsw==0 ? 0 : 1);
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
//	if(lsw==0)	pi_id.Ref = IdRef;
//	else		pi_id.Ref = IdRef;
	pi_id.Ref = ramper(IdRef, pi_id.Ref, _IQ(0.00001));
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;


// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
 	DlogCh1 = posEncElecTheta[POSITION_ENCODER];
 	DlogCh2 = rg1.Out;     // current_sensor[SHUNT_CURRENT_SENSE-1].As;
 	DlogCh3 = clarke1.As;  // current_sensor[LEM_CURRENT_SENSE-1].As;
 	DlogCh4 = clarke1.Bs;  // current_sensor[SD_CURRENT_SENSE-1].As;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
 	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
 	DaccRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;

#endif // (BUILDLEVEL==LEVEL3)


// =============================== LEVEL 4 ======================================
//	  Level 4 verifies the speed regulator performed by PID module.
//	  The system speed loop is closed by using the measured speed as feedback
//  lsw=0: lock the rotor of the motor
//  lsw=1: - needed only with QEP encoders until first index pulse
//         - Loops shown for lsw=2 are closed in this stage
//  lsw=2: close speed loop and current loops Id, Iq
// ==============================================================================

//TODO 构建等级4
#if (BUILDLEVEL==LEVEL4)

// ------------------------------------------------------------------------------
// Alignment Routine: this routine aligns the motor to zero electrical angle
// and in case of QEP also finds the index location and initializes the angle
// w.r.t. the index location
// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// alignment current
		IdRef = IdRef_start;  //IQ(0.1);

		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

		// set up an alignment and hold time for shaft to settle down
		if (pi_id.Ref >= IdRef)
		{
			if (cntr < alignCnt)
				cntr++;
			else
			{
				cntr  = 0;
				IdRef = IdRef_run;
#if POSITION_ENCODER==QEP_POS_ENCODER
				lsw = 1;       // for QEP, spin the motor to find the index pulse
				IqRef = _IQ(0.05);
#else
				lsw = 2;    // for absolute encoders no need for lsw=1
#endif
			}
		}
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;

	if(lsw==0) park1.Angle = 0;
//	else if (lsw==1)  park1.Angle =  rg1.Out; // this state exists only for QEP
	else park1.Angle = posEncElecTheta[POSITION_ENCODER];

	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);

	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID speed controller macro
// ------------------------------------------------------------------------------
   if (SpeedLoopCount==SpeedLoopPrescaler)
     {
		pid_spd.term.Ref = rc1.SetpointValue;  //SpeedRef;
		pid_spd.term.Fbk = speed1.Speed;
		PID_MACRO(pid_spd);

		SpeedLoopCount = 1;
     }
	else SpeedLoopCount++;

	if(lsw==0 || lsw==1)
	{
		pid_spd.data.d1 = 0; pid_spd.data.d2 = 0; pid_spd.data.i1 = 0;
		pid_spd.data.ud = 0; pid_spd.data.ui = 0; pid_spd.data.up = 0;
	}

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
	if(lsw==0)
	{
		pi_iq.Ref = 0;
		pi_iq.ui  = 0;
	}
    else if(lsw==1) pi_iq.Ref = IqRef;
    else pi_iq.Ref = pid_spd.term.Out;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
	pi_id.Ref=ramper(IdRef, pi_id.Ref, _IQ(0.00001));
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	ipark1.Ds = pi_id.Out;
	ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
	ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
	svgen1.Ualpha = ipark1.Alpha;
	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = posEncElecTheta[POSITION_ENCODER];
	DlogCh2 = svgen1.Ta;
	DlogCh3 = clarke1.As;
	DlogCh4 = clarke1.Bs;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
 	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
 	DaccRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;

#endif // (BUILDLEVEL==LEVEL4)


// =============================== LEVEL 5 ======================================
//  Level 5 verifies the position control
//  Position references generated locally
//  lsw=0: lock the rotor of the motor
//  lsw=1: - needed only with QEP encoders until first index pulse
//         - Loops shown for lsw=2 are closed in this stage
//  lsw=2: close all loops, position, speed and currents Id, Iq
// ==============================================================================

//TODO 构建等级5
#if (BUILDLEVEL==LEVEL5)

	// ------------------------------------------------------------------------------
	//  Connect inputs of the RMP module and call the ramp control macro
	// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// alignment curretnt
		IdRef = IdRef_start;  //IQ(0.1);

		// for restarting from (RunMotor = 0)
		rc1.TargetValue = rc1.SetpointValue = 0;

		// set up an alignment and hold time for shaft to settle down
		if (pi_id.Ref >= IdRef)
		{
			if (cntr < alignCnt)
				cntr++;
			else
			{
				cntr  = 0;
				IdRef = IdRef_run;
#if POSITION_ENCODER==QEP_POS_ENCODER
				// for QEP because it is an incremental encoder spin the motor to find the index pulse
				lsw=1;
				IqRef= _IQ(0.05); // make the motor spin to get an index pulse
#else
				// for absolute encoders no need for lsw=1
				lsw = 2;
#endif
			}
		}
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//  Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;

	if(lsw==0) park1.Angle = 0;
//	else if (lsw==1)  park1.Angle =  rg1.Out; // this state exists only for QEP
	else park1.Angle = posEncElecTheta[POSITION_ENCODER];

	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID speed controller macro
// ------------------------------------------------------------------------------
	if (++SpeedLoopCount >= SpeedLoopPrescaler)
	{
		SpeedLoopCount=0;
		if (lsw == 0)
		{
			rc1.SetpointValue = 0;  // position = 0 deg
			pid_spd.data.d1 = 0; pid_spd.data.d2 = 0; pid_spd.data.i1 = 0;
			pid_spd.data.ud = 0; pid_spd.data.ui = 0; pid_spd.data.up = 0;
			pi_pos.ui = 0;
		}
		else
		{

		// ========== reference position setting =========
		// --------------------------------------------------------------------
		//  Connect inputs of RAMP GEN module and call ramp generator macro
		// --------------------------------------------------------------------
		rg1.Freq = SpeedRef;
		RG_MACRO(rg1)

		// choose between one of  two position commands - watch Ref and Fbk
		// Position command read from a table
		   rc1.TargetValue = refPosGen(rc1.TargetValue);
		// Position command generated as integral of SpeedRef
//		   rc1.TargetValue = rg1.Out;

		rc1.SetpointValue = _IQfrac(rc1.TargetValue);
		// Rolling in angle within 0 to 1pu
		if (rc1.SetpointValue < 0)
			rc1.SetpointValue += _IQ(1.0);

		pi_pos.Ref = rc1.SetpointValue;
		pi_pos.Fbk = posEncMechTheta[POSITION_ENCODER];
		PI_POS_MACRO(pi_pos);

		// speed PI regulator
		pid_spd.term.Ref = pi_pos.Out;     //	pid_spd.term.Ref = rc1.SetpointValue;
		pid_spd.term.Fbk = speed1.Speed;
		PID_MACRO(pid_spd);                //	pi_spd.Out = pi_pos.Out;
		}
	}

// ------------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
	if(lsw==0)
	{
		pi_iq.Ref = 0;
		pi_iq.ui  = 0;
	}
//	else if(lsw==1) pi_iq.Ref = IqRef;
	else pi_iq.Ref =  pid_spd.term.Out;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq);

// ------------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID ID controller macro
// ------------------------------------------------------------------------------
	pi_id.Ref = ramper(IdRef, pi_id.Ref, _IQ(0.00001));  //ramprate = 1pu/s
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id);

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	ipark1.Qs = pi_iq.Out;
	ipark1.Ds = pi_id.Out;
	ipark1.Sine   = park1.Sine;
	ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1);

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
	svgen1.Ualpha = ipark1.Alpha;
	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  计算占空比写入CMPA寄存器
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    连接变量到DATALOG模块
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = park1.Ds;
	DlogCh3 = park1.Qs;
	DlogCh4 = park1.Beta;

//------------------------------------------------------------------------------
// 变量输出到DACs B和C通道
//------------------------------------------------------------------------------
	DacbRegs.DACVALS.bit.DACVALS = pi_pos.Ref*4096;
	DaccRegs.DACVALS.bit.DACVALS = pi_pos.Fbk*4096;

#endif // (BUILDLEVEL==LEVEL5)


// ------------------------------------------------------------------------------
//    调用DATALOG模块更新函数
// ------------------------------------------------------------------------------
	DLOG_4CH_F_FUNC(&dlog_4ch1);

//XXX 修改为清除PWM8中断标志
	//清除PWM11中断标志
    EPwm8Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all   = PIEACK_GROUP3;

}// MainISR Ends Here


// ****************************************************************************
// ****************************************************************************
//TODO  DMC Protection Against Over Current Protection
// ****************************************************************************
// ****************************************************************************

//定义DACH参考选择
#define REFERENCE_VDDA     0

//定义COMPH输入选择
#define NEGIN_DAC          0
#define NEGIN_PIN          1

//定义CTRIPH/CTRIPOUTH输出选择
#define CTRIP_ASYNCH       0
#define CTRIP_SYNCH        1
#define CTRIP_FILTER       2
#define CTRIP_LATCH        3

void cmpssConfig(volatile struct CMPSS_REGS *v, int16 Hi, int16 Lo)
{
	//设置COMPCTL寄存器
	v->COMPCTL.bit.COMPDACE    = 1;             // 使能CMPSS
	v->COMPCTL.bit.COMPLSOURCE = NEGIN_DAC;     // 来自DAC的COMP-L的NEG信号
	v->COMPCTL.bit.COMPHSOURCE = NEGIN_DAC;     // 来自DAC的COMP-H的NEG信号
	v->COMPCTL.bit.COMPHINV    = 0;             // COMP-H输出未反相
	v->COMPCTL.bit.COMPLINV    = 1;             // COMP-L输出反相
	v->COMPCTL.bit.ASYNCHEN    = 0;             // 禁用同步COMP-H输出
	v->COMPCTL.bit.ASYNCLEN    = 0;             // 禁用同步COMP-L输出
	v->COMPCTL.bit.CTRIPHSEL    = CTRIP_FILTER; // Dig filter output ==> CTRIPH
	v->COMPCTL.bit.CTRIPOUTHSEL = CTRIP_FILTER; // Dig filter output ==> CTRIPOUTH
	v->COMPCTL.bit.CTRIPLSEL    = CTRIP_FILTER; // Dig filter output ==> CTRIPL
	v->COMPCTL.bit.CTRIPOUTLSEL = CTRIP_FILTER; // Dig filter output ==> CTRIPOUTL

	//设置COMPHYSCTL寄存器
	v->COMPHYSCTL.bit.COMPHYS   = 2; // COMP hysteresis set to 2x typical value

	// 设置COMPDACCTL寄存器
	v->COMPDACCTL.bit.SELREF    = 0; // VDDA is REF for CMPSS DACs
	v->COMPDACCTL.bit.SWLOADSEL = 0; // DAC updated on sysclock
	v->COMPDACCTL.bit.DACSOURCE = 0; // Ramp bypassed

	// Load DACs - High and Low
	v->DACHVALS.bit.DACVAL = Hi;     // Set DAC-H to allowed MAX +ve current
	v->DACLVALS.bit.DACVAL = Lo;     // Set DAC-L to allowed MAX -ve current

	// 数字滤波器设置-高侧
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale; // set time between samples, max : 1023
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;     // # of samples in window, max : 31
	v->CTRIPHFILCTL.bit.THRESH         = thresh;      // recommended : thresh > sampwin/2
	v->CTRIPHFILCTL.bit.FILINIT        = 1;           // Init samples to filter input value

	//数字滤波器设置-低侧
	v->CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale; // set time between samples, max : 1023
	v->CTRIPLFILCTL.bit.SAMPWIN        = sampwin;     // # of samples in window, max : 31
	v->CTRIPLFILCTL.bit.THRESH         = thresh;      // recommended : thresh > sampwin/2
	v->CTRIPLFILCTL.bit.FILINIT        = 1;           // Init samples to filter input value

	// 清除状态寄存器以获取锁存的比较器事件
	v->COMPSTSCLR.bit.HLATCHCLR = 1;
	v->COMPSTSCLR.bit.LLATCHCLR = 1;

	return;
}

/**********************************************************************
 * 函数名称： HVDMC_Protection
 * 函数说明：高压数字电动机控制保护
 * 注解：
 *    HVDMC  High Voltage Digital Motor Control 高压数字电动机
 *********************************************************************/
void HVDMC_Protection(void)
{
	EALLOW;

	// 配置用于跳闸机制的GPIO

    // GPIO输入用于读取LEM过电流宏模块的状态（低电平有效），如果需要，GPIO40可以基于此跳闸PWM
	// 配置为输入
	GpioCtrlRegs.GPBPUD.bit.GPIO40  = 1; // 禁止上拉
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; // 选择GPIO作为普通引脚使用
	GpioCtrlRegs.GPBDIR.bit.GPIO40  = 0; // 设置为输入
	GpioCtrlRegs.GPBINV.bit.GPIO40  = 1; // 反转输入，以使反转后0为好而1为坏

	InputXbarRegs.INPUT2SELECT = 40;     // 选择GPIO40作为INPUTXBAR2

	// 清除故障输出引脚GPIO41（低电平有效）
	// 配置为输出
	GpioCtrlRegs.GPBPUD.bit.GPIO41  = 1; // 禁止上拉
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // 选择GPIO作为普通引脚使用
	GpioCtrlRegs.GPBDIR.bit.GPIO41  = 1; // 设置为输出
	GpioDataRegs.GPBSET.bit.GPIO41  = 1; // 设置默认值为高电平

	// 使用GPIO58强制IPM(功率模块)关断（Trip）（高电平有效）
	// 配置为输出
	GpioCtrlRegs.GPBPUD.bit.GPIO58   = 1; // 禁止上拉
	GpioCtrlRegs.GPBMUX2.bit.GPIO58  = 0; // 选择GPIO作为普通引脚使用
	GpioCtrlRegs.GPBDIR.bit.GPIO58   = 1; // 设置为输出
	GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1; // 设置默认值为低电平

	// LEM当前相V（ADC A2，COMP1）和W（ADC B2，COMP3），高低比较事件跳闸
	LEM_curHi = 2048 + LEM(curLimit);
	LEM_curLo = 2048 - LEM(curLimit);
	cmpssConfig(&Cmpss1Regs, LEM_curHi, LEM_curLo);  //启用CMPSS1-LEM V
	cmpssConfig(&Cmpss3Regs, LEM_curHi, LEM_curLo);  //启用CMPSS3-LEM W

#if (CNGD == HOT)
	// 分流电流相位V（ADC A4，COMP2）和W（ADC C2，COMP6），高低比较事件跳闸
	SHUNT_curHi = 2048 + SHUNT(curLimit);
	SHUNT_curLo = 2048 - SHUNT(curLimit);
	cmpssConfig(&Cmpss2Regs, SHUNT_curHi, SHUNT_curLo);  //使能CMPSS2
	cmpssConfig(&Cmpss6Regs, SHUNT_curHi, SHUNT_curLo);  //使能CMPSS6
#endif

	// 将TRIP4配置为比较器1和3的高跳和低跳
	// 首先清除所有内容
	EPwmXbarRegs.TRIP4MUX0TO15CFG.all  = 0x0000;
	EPwmXbarRegs.TRIP4MUX16TO31CFG.all = 0x0000;
	// 为CMPSS1H和1L的命令输入启用多路复用器，即为Mux0的.1多路复用器
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0  = 1;
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4  = 1;
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2  = 1;
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX10 = 1;
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3  = 1;

	// 首先禁用所有多路复用器
	EPwmXbarRegs.TRIP4MUXENABLE.all = 0x0000;
	// 使能Mux 0或Mux 4生成TRIP4
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX10 = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1  = 1;

//	EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip4是DCAHCOMPSEL的输入
//	EPwm1Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
//	EPwm1Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
//	EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
//	EPwm1Regs.TZSEL.bit.DCAEVT1         = 1;
//
//	EPwm2Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip4是DCAHCOMPSEL的输入
//	EPwm2Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
//	EPwm2Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
//	EPwm2Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
//	EPwm2Regs.TZSEL.bit.DCAEVT1         = 1;
//
//	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip4是DCAHCOMPSEL的输入
//	EPwm3Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
//	EPwm3Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
//	EPwm3Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
//	EPwm3Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm1Regs.TZSEL.bit.CBC6 = 0x1; //启用TZ6作为此ePWM模块的CBC跳闸源
	EPwm2Regs.TZSEL.bit.CBC6 = 0x1; //启用TZ6作为此ePWM模块的CBC跳闸源
	EPwm3Regs.TZSEL.bit.CBC6 = 0x1; //启用TZ6作为此ePWM模块的CBC跳闸源

	//当发生OST/CBC事件时：
	//TZA事件可以强制控制EPWMxA
	//TZB事件可以强制控制EPWMxB

	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWM1A输出变为低
	EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWM1B输出变为低

	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWM2A输出变为低
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWM2B输出变为低

	EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWM3A输出变为低
	EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWM3B输出变为低

	//启用数字比较输出A事件1作为此ePWM模块的触发源
	EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;
	//TZ中断使能
	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;

	EDIS;

//************************** End of Prot. Conf. ***************************//
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM配置
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db) {
	EALLOW;

	//时基子模块寄存器
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; //设置立即加载
	(*ePWM[n]).TBPRD = period / 2; // PWM频率= 1 / 周期
	(*ePWM[n]).TBPHS.bit.TBPHS = 0; //设置相位寄存器值
	(*ePWM[n]).TBCTR = 0; //清除时基寄存器值
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;//计数器模式：向上向下计数模式

	//设置分频值为sysclk/2
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;//高速时基时钟预分频值
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;//时基时钟分频值

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;//加载相位寄存器值：关闭
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; //同步信号选择：时基寄存器等于0

	//计数器比较子模块寄存器
	(*ePWM[n]).CMPA.bit.CMPA = 0; // 最初设定为0％
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;//比较器A工作模式：影子寄存器
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//比较器A加载方式：时基寄存器值=0

	//动作限定子模块寄存器
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;//增模式：清除输出
	(*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;//减模式：设置输出

	//有源互补PWM-设置死区
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;//死区输入模式：EPWMA为源
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;//死区输出模式：DBM完全使能
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;//极性选择：有效高互补模式EPWMB被反转
//xxx 修改寄存器共用体
	(*ePWM[n]).DBRED.bit.DBRED = db;//死区发生器上升沿延迟高分辨率镜像寄存器
	(*ePWM[n]).DBFED.bit.DBFED = db;//死区发生器下降沿计数寄存器
	EDIS;
}

void PWM_1ch_UpCnt_CNF(int16 n, Uint16 period) {
	EALLOW;
	// 时基子模块寄存器
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // 设置立即加载模式
	(*ePWM[n]).TBPRD = period-1; // PWM 频率 = 1 / 周期
	(*ePWM[n]).TBPHS.bit.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UP;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // /同步信号选择：时基寄存器等于0

	// 计数器比较子模块寄存器
	(*ePWM[n]).CMPA.bit.CMPA        = 0; // 最初设定为0％
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// 动作限定子模块寄存器
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;
	(*ePWM[n]).AQCTLA.bit.ZRO = AQ_SET;

	// 有源互补PWM-设置死区
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	//xxx 修改寄存器共用体
	(*ePWM[n]).DBRED.bit.DBRED = 0;
	(*ePWM[n]).DBFED.bit.DBFED = 0;
	EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO ADC配置
// ****************************************************************************
// ****************************************************************************
void ConfigureADC(void)
{
	Uint16 i;

	EALLOW;

	//配置ADC-A
	//设置模块时钟分频系数:sysclk/4
	AdcaRegs.ADCCTL2.bit.PRESCALE   = 6;
	//xxx 设置分辨率：12位
//	AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	//xxx 设置信号输入方式：单端输入
//	AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//设置ADC中断脉冲的位置:在转换完成后
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//ADC-A上电
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//配置ADC-B
	AdcbRegs.ADCCTL2.bit.PRESCALE   = 6;
	//xxx 设置分辨率：12位
//	AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	//xxx 设置信号输入方式：单端输入
//	AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//设置ADC中断脉冲的位置:在转换完成后
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//ADC-B上电
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//配置ADC-C
	//设置模块时钟分频系数:sysclk/4
	AdccRegs.ADCCTL2.bit.PRESCALE   = 6;
	//xxx 设置分辨率：12位
//	AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	//xxx 设置信号输入方式：单端输入
//	AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//设置ADC中断脉冲的位置:在转换完成后
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//ADC-C上电
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

//xxx	//配置ADC-D
//    //设置模块时钟分频系数:sysclk/4
//	AdcdRegs.ADCCTL2.bit.PRESCALE   = 6;
//	//设置分辨率：12位
//	AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
//	//设置信号输入方式：单端输入
//	AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
//
//	//设置ADC中断脉冲的位置:在转换完成后
//	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
//
//	//ADC-D上电
//	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//延时1ms等待ADC上电完成
	for(i = 0; i < 1000; i++){
		asm("   RPT#255 || NOP");
	}

	EDIS;
}

// ****************************************************************************
// ****************************************************************************
// 位置闭环实用功能
// ****************************************************************************
// ****************************************************************************

// 可编程斜坡
_iq ramper(_iq in, _iq out, _iq rampDelta) {
	_iq err;

	err = in - out;
	if (err > rampDelta)
		return(out + rampDelta);
  	else if (err < -rampDelta)
  		return(out - rampDelta);
    else
    	return(in);
}

// 用于速度参考的斜坡控制器（当前不使用）
_iq ramper_speed(_iq in, _iq out, _iq rampDelta) {
	_iq err;

	err = in - out;
	if (err > rampDelta)
	{
		if((out+rampDelta)>1.0)
			return(1.0);
		else
			return (out+rampDelta);
	}
  	else if (err < -rampDelta)
  	{
  		if(out-rampDelta<=0.0)
  			return(0.0);
  		else
  			return(out - rampDelta);
  	}
    else
    	return(in);
}

// 参考位置发生器，用于位置环
_iq refPosGen(_iq out)
{
	_iq in = posArray[ptr1];

	out = ramper(in, out, posSlewRate);

	if (in == out)
	if (++cntr1 > 1000)
	{
		cntr1 = 0;
		if (++ptr1 >= ptrMax)
			ptr1 = 0;
	}
	return (out);
}

/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */
