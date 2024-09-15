/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_timer.ch
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#ifndef __CH56x_TIMER_H__
#define __CH56x_TIMER_H__

#ifdef __cplusplus
 extern "C" {
#endif
	

/**
  * @brief  Pulse Width Modulation Effective Output Words
  */
typedef enum
{
	PWM_Times_1 = 0,						// PWM effective output repeats 1 times
	PWM_Times_4 = 1,						// PWM effective output repeats 4 times
	PWM_Times_8 = 2,						// PWM effective output repeats 8 times
	PWM_Times_16 = 3,						// PWM effective output repeats 16 times
}PWM_RepeatTsTypeDef;


/**
  * @brief  Input Capture Edge Mode
  */
typedef enum
{
	CAP_NULL = 0,							// not capture
	Edge_To_Edge = 1,						// between any edge
	FallEdge_To_FallEdge = 2,				// falling edge to falling edge
	RiseEdge_To_RiseEdge = 3,				// rising edge to rising edge
}CapModeTypeDef;


/**
  * @brief  Direct access memory loop mode
  */
typedef enum
{
	Mode_Single = 0,					// single mode
	Mode_LOOP = 1,						// cycle mode
}DMAModeTypeDef;


/**
  * @brief  PWM output polarity
  */
typedef enum
{
	high_on_low = 0,						// Default low level, high level is active
	low_on_high = 1,						// Default high level, low level active
}PWM_PolarTypeDef;


/****************** TMR0 */
// Timing and counting
void TMR0_TimerInit( UINT32 t );									/* Timing function initialization */
void TMR0_EXTSignalCounterInit( UINT32 c );							/* External signal counting function initialization */
#define  TMR0_GetCurrentCount()		R32_TMR0_COUNT	 				/* Get the current count value, 67108864 */

// Pulse Width Modulation Function
#define TMR0_PWMCycleCfg( cyc )	    (R32_TMR0_CNT_END=cyc)			/* PWM0 channel output waveform period configuration, maximum 67108864 */
void TMR0_PWMInit( PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts );	/* PWM0 output initialization */
#define TMR0_PWMActDataWidth( d )   (R32_TMR0_FIFO = d)				/* PWM0 effective data pulse width, maximum 67108864 */

// Catch pulse width
#define TMR0_CAPTimeoutCfg( cyc )   (R32_TMR0_CNT_END=cyc)			/* CAP0 capture level timeout configuration, maximum 33554432 */
void TMR0_CapInit( CapModeTypeDef cap );							/* External signal capture function initialization */
#define TMR0_CAPGetData()			R32_TMR0_FIFO					/* Get pulse data */
#define TMR0_CAPDataCounter()		R8_TMR0_FIFO_COUNT				/* Get the number of currently captured data */

#define TMR0_Disable()				(R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN)		/* Close TMR0 */
// refer to TMR0 interrupt bit define
#define	TMR0_ITCfg(s,f)				((s)?(R8_TMR0_INTER_EN|=f):(R8_TMR0_INTER_EN&=~f))		/* TMR0 corresponding interrupt bit on and off */
// refer to TMR0 interrupt bit define
#define TMR0_ClearITFlag(f)         (R8_TMR0_INT_FLAG = f)			/* Clear interrupt flag */
#define TMR0_GetITFlag(f)           (R8_TMR0_INT_FLAG&f)			/* Query interrupt flag status */


/****************** TMR1 */
// Timing and counting
void TMR1_TimerInit( UINT32 t );									/* Timing function initialization */
void TMR1_EXTSignalCounterInit( UINT32 c );							/* External signal counting function initialization */
#define  TMR1_GetCurrentCount()		R32_TMR1_COUNT	 				/* Get the current count value, 67108864 */

// Pulse Width Modulation Function
#define TMR1_PWMCycleCfg( cyc )	    (R32_TMR1_CNT_END=cyc)			/* PWM1 channel output waveform period configuration, maximum 67108864 */
void TMR1_PWMInit( PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts );	/* PWM1 output initialization */
#define TMR1_PWMActDataWidth( d )   (R32_TMR1_FIFO = d)				/* PWM1 effective data pulse width, maximum 67108864 */

// Catch pulse width
#define TMR1_CAPTimeoutCfg( cyc )   (R32_TMR1_CNT_END=cyc)			/* CAP1 capture level timeout configuration, maximum 33554432 */
void TMR1_CapInit( CapModeTypeDef cap );							/* External signal capture function initialization */
#define TMR1_CAPGetData()			R32_TMR1_FIFO					/* Get pulse data */
#define TMR1_CAPDataCounter()		R8_TMR1_FIFO_COUNT				/* Get the number of currently captured data */

void TMR1_DMACfg( UINT8 s, UINT16 startAddr, UINT16 endAddr, DMAModeTypeDef m );    /* DMA configuration */

#define TMR1_Disable()				(R8_TMR1_CTRL_MOD &= ~RB_TMR_COUNT_EN)		/* Close TMR1 */
// refer to TMR1 interrupt bit define
#define	TMR1_ITCfg(s,f)				((s)?(R8_TMR1_INTER_EN|=f):(R8_TMR1_INTER_EN&=~f))		/* TMR1 corresponding interrupt bit on and off */
// refer to TMR1 interrupt bit define
#define TMR1_ClearITFlag(f)         (R8_TMR1_INT_FLAG = f)			/* Clear interrupt flag */
#define TMR1_GetITFlag(f)           (R8_TMR1_INT_FLAG&f)			/* Query interrupt flag status */


/****************** TMR2 */
// Timing and counting
void TMR2_TimerInit( UINT32 t );									/* Timing function initialization */
void TMR2_EXTSignalCounterInit( UINT32 c );							/* External signal counting function initialization */
#define  TMR2_GetCurrentCount()		R32_TMR2_COUNT	 				/* Get the current count value, 67108864 */

// Pulse Width Modulation Function
#define TMR2_PWMCycleCfg( cyc )	    (R32_TMR2_CNT_END=cyc)			/* PWM2 channel output waveform period configuration, maximum 67108864 */
void TMR2_PWMInit( PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts );	/* PWM2 output initialization */
#define TMR2_PWMActDataWidth( d )   (R32_TMR2_FIFO = d)				/* PWM2 effective data pulse width, maximum 67108864 */

// Catch pulse width
#define TMR2_CAPTimeoutCfg( cyc )   (R32_TMR2_CNT_END=cyc)			/* CAP2 capture level timeout configuration, maximum 33554432 */
void TMR2_CapInit( CapModeTypeDef cap );							/* External signal capture function initialization */
#define TMR2_CAPGetData()			R32_TMR2_FIFO					/* Get pulse data */
#define TMR2_CAPDataCounter()		R8_TMR2_FIFO_COUNT				/* Get the number of currently captured data */

void TMR2_DMACfg( UINT8 s, UINT16 startAddr, UINT16 endAddr, DMAModeTypeDef m );    /* DMA configuration */

#define TMR2_Disable()				(R8_TMR2_CTRL_MOD &= ~RB_TMR_COUNT_EN)		/* Close TMR2 */
// refer to TMR2 interrupt bit define
#define	TMR2_ITCfg(s,f)				((s)?(R8_TMR2_INTER_EN|=f):(R8_TMR2_INTER_EN&=~f))		/* TMR2 corresponding interrupt bit on and off */
// refer to TMR2 interrupt bit define
#define TMR2_ClearITFlag(f)         (R8_TMR2_INT_FLAG = f)			/* Clear interrupt flag */
#define TMR2_GetITFlag(f)           (R8_TMR2_INT_FLAG & f)			/* Query interrupt flag status */



#ifdef __cplusplus
 }
#endif

#endif // __CH56x_TIMER_H__





