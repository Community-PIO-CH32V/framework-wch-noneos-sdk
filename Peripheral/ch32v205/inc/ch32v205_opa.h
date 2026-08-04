/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v00X_opa.h
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/12/15
 * Description        : This file contains all the functions prototypes for the
 *                      OPA firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32V205_OPA_H
#define __CH32V205_OPA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v205.h"

/* OPA_member_enumeration */
typedef enum
{
    OPA1 = 0,
    OPA2
} OPA_Num_TypeDef;

/* OPA1_POLL_Mode_enumeration */
typedef enum
{
    OPA_POLL_Mode_NoSingle = 0,
    OPA_POLL_Mode_Single,
} OPA1_POLL_Mode_TypeDef;

/* OPA1_SampleSource_enumeration */
typedef enum
{
    OPA_SampleSource_OPA1 = 0,
    OPA_SampleSource_CMP1,
} OPA1_SampleSource_TypeDef;

/* OPA1_NMI_IE_enumeration */
typedef enum
{
    NMI_IE_OFF = 0,
    NMI_IE_ON,
} OPA1_NMI_IE_TypeDef;

/* OPA1_CNT_IE_enumeration */
typedef enum
{
    CNT_IE_OFF = 0,
    CNT_IE_ON,
} OPA1_CNT_IE_TypeDef;

/* OPA1_OUT_IE_enumeration */
typedef enum
{
    OUT_IE_OFF= 0,
    OUT_IE_ON,
} OPA1_OUT_IE_TypeDef;

/* OPA1_RST_EN_enumeration */
typedef enum
{
    RST_OPA_OFF = 0,
    RST_OPA_ON,
} OPA1_RST_EN_TypeDef;

/* OPA1_POLL_CH3_definition */
typedef enum
{
    OPA_POLL_CH3_PB5 = 0,
    OPA_POLL_CH3_PB0,
    OPA_POLL_CH3_PB12,
    OPA_POLL_CH3_FLOATING
} OPA1_POLL_CH3_TypeDef;

/* OPA1_POLL_CH2_definition */
typedef enum
{
    OPA_POLL_CH2_PB5 = 0,
    OPA_POLL_CH2_PB0,
    OPA_POLL_CH2_PB12,
    OPA_POLL_CH2_FLOATING
} OPA1_POLL_CH2_TypeDef;

/* OPA1_POLL_CH1_definition */
typedef enum
{
    OPA_POLL_CH1_PB5 = 0,
    OPA_POLL_CH1_PB0,
    OPA_POLL_CH1_PB12,
    OPA_POLL_CH1_FLOATING
} OPA1_POLL_CH1_TypeDef;

/* OPA1_POLL_SEL_enumeration */
typedef enum
{
    OPA_POLL_SEL_SOFT = 0,
    OPA_POLL_SEL_TIM1_CH4,
    OPA_POLL_SEL_TIM2_CH4,
    OPA_POLL_SEL_TIM3_CH4,
    OPA_POLL_SEL_TIM4_CH4
} OPA1_POLL_SEL_TypeDef;

/* OPA1_PSEL_POLL_NUM_enumeration */
typedef enum
{
    CHP_POLL_NUM_1 = 0,
    CHP_POLL_NUM_2,
    CHP_POLL_NUM_3
} OPA1_PSEL_POLL_NUM_TypeDef;

/* OPA1_PSEL_POLL_enumeration */
typedef enum
{
    CHP_OPA_POLL_OFF = 0,
    CHP_OPA_POLL_ON,
} OPA1_PSEL_POLL_TypeDef;

/* OPA1_ChopFrequency_enumeration */
typedef enum
{
    OPA_ChopFrequency_1MHz = 0,
    OPA_ChopFrequency_500KHz,
    OPA_ChopFrequency_250KHz,
    OPA_ChopFrequency_125Hz
} OPA1_ChopFrequency_TypeDef;

/* OPA_HIGH_SPEED_enumeration */
typedef enum
{
    HS_OFF = 0,
    HS_ON
} OPA_HIGH_SPEED_TypeDef;

/* OPA_PGADIF_enumeration */
typedef enum
{
    PGADIF_GND = 0,
    PGADIF_IO //OPA1-PA13, OPA2-PB15
} OPA_PGADIF_TypeDef;

/* OPA_FB_enumeration */
typedef enum
{
    FB_OFF = 0,
    FB_ON
} OPA_FB_TypeDef;

/* OPA_PGA_SEL_enumeration */
typedef enum
{
    CHN_PGA_4xIN = 0,
    CHN_PGA_8xIN,
    CHN_PGA_16xIN,
    CHN_PGA_32xIN,
} OPA_PGA_SEL_TypeDef;

/* OPA_NSEL_enumeration */
typedef enum
{
    CHN0 = 0, //OPA1-PA6, OPA2-PA5
    CHN1,     //OPA1-PB11, OPA2-PB10
} OPA_NSEL_TypeDef;

/* OPA_PSEL_enumeration */
typedef enum
{
    CHP0 = 0, //OPA1-PB5, OPA2-PA7
    CHP1,     //OPA1-PB0, OPA2-PB14
    CHP2,     //OPA1-PB12
    CHP3      //OPA1-OFF
} OPA_PSEL_TypeDef;

/* OPA_out_channel_enumeration */
typedef enum
{
    OUT_IO_OUT0 = 0, //OPA1-PB1, OPA2-PA4
    OUT_IO_OUT1,     //OPA1-PA3, OPA2-PA2
    OUT_OFF
} OPA_Mode_TypeDef;

/* OPA Init Structure definition */
typedef struct
{
    uint32_t                   SETUP_Time;/* Specifies the ADC conversion interval for the OPA1 
                                          This parameter must range from 0 to 0xFF */
    OPA1_POLL_Mode_TypeDef     POLL_Mode; /* Specifies the poll mode of OPA1 */
    OPA1_SampleSource_TypeDef  SampleSource; /* Specifies the sample source of OPA1 */
    OPA1_NMI_IE_TypeDef        NMI_IE;   /* Specifies the out NMI interrupt of OPA1 */
    OPA1_CNT_IE_TypeDef        CNT_IE;   /* Specifies the out interrupt polling interval end for the OPA1 */
    OPA1_OUT_IE_TypeDef        OUT_IE;   /* Specifies the out interrupt of OPA1 */
    OPA1_POLL_CH3_TypeDef      POLL_CH3; /* Specifies the poll channel 3 of OPA1 */
    OPA1_POLL_CH2_TypeDef      POLL_CH2; /* Specifies the poll channel 2 of OPA1 */
    OPA1_POLL_CH1_TypeDef      POLL_CH1; /* Specifies the poll channel 1 of OPA1 */
    OPA1_POLL_SEL_TypeDef      POLL_SEL; /* specifies Poll Trigger Event of OPA1 */
    OPA1_PSEL_POLL_NUM_TypeDef POLL_NUM; /* Specifies the number of OPA poll */
    OPA1_PSEL_POLL_TypeDef     PSEL_POLL;/* Specifies the positive channel poll of OPA */
    OPA1_ChopFrequency_TypeDef Chop_Frequency; /* Specifies the chop frequency of OPA */
    OPA_HIGH_SPEED_TypeDef     OPA_HS;   /* specifies high speed mode enable of OPA */
    OPA_PGADIF_TypeDef         PGADIF;   /* OPA with NSEL1 for use as a PGA */
    OPA_FB_TypeDef             FB;       /* Specifies the internal feedback resistor of OPA */
    OPA_PGA_SEL_TypeDef        PGA_SEL;  /* Specifies the PGA select of OPA */
    OPA_NSEL_TypeDef           NSEL;     /* Specifies the negative channel of OPA */
    OPA_PSEL_TypeDef           PSEL;     /* Specifies the positive channel of OPA */
    OPA_Mode_TypeDef           Mode;     /* Specifies the mode of OPA */
} OPA_InitTypeDef;

/* CMP_member_enumeration */
typedef enum
{
    CMP1 = 0,
    CMP2
} CMP_Num_TypeDef;

/* CMP_HYEN_enumeration */
typedef enum
{
    CMP_HYS_Mode0,
    CMP_HYS_Mode1,
    CMP_HYS_Mode2,
    CMP_HYS_Mode3
} CMP_HYS_TypeDef;

/* CMP_PSEL_enumeration */
typedef enum
{
    CMP_CHP0 = 0, //CMP1-PA2, CMP2-PB8
    CMP_CHP1,     //CMP1-PB2, CMP2-PA8
    CMP_CHP2,     //CMP1-OPA1_OUT, CMP2-OPA2_OUT
    CMP_CHP_OFF   //CMP1-OFF, CMP2-OFF
} CMP_PSEL_TypeDef;

/* CMP_NSEL_enumeration */
typedef enum
{
    CMP_CHN0 = 0, //CMP1-PB1, CMP2-PA9
    CMP_CHN1,     //CMP1-PB3, CMP2-PB11
    CMP_CHN2,     //CMP1-DAC_OUT, CMP2-DAC_OUT
    CMP_CHN_OFF   //CMP1-OFF, CMP2-OFF
} CMP_NSEL_TypeDef;

/* CMP_out_mode_enumeration */
typedef enum
{
    OUT_IO0 = 0,      //CMP1-PB12, CMP2-PA12
    OUT_IO_TIM_Mode1, //CMP1-TIM1_CH4, CMP2-TIM3_CH4
    OUT_IO_TIM_Mode2, //CMP1-TIM2_CH4, CMP2-TIM2_CH3
    OUT_IO_TIM_Mode3  //CMP1-TIM4_CH4, CMP2-TIM1_CH3
} CMP_OutMode_TypeDef;

/* CMP1 Init Structure definition */
typedef struct
{
    CMP_HYS_TypeDef       HYS;               /* Selection of Hysteresis voltage */
    CMP_PSEL_TypeDef      PSEL;              /* Specifies the positive channel of CMP */
    CMP_NSEL_TypeDef      NSEL;              /* Specifies the negative channel of CMP */
    CMP_OutMode_TypeDef   CMP_Out_Mode;      /* Specifies the out mode of CMP */
} CMP_InitTypeDef;

/* TIM1_brake_source_definition */
#define TIM1_Brake_Source_IO                ((uint32_t)0x00000000)
#define TIM1_Brake_Source_CMP1              ((uint32_t)0x00000008)
#define TIM1_Brake_Source_CMP2              ((uint32_t)0x00040008)
#define TIM1_Brake_Source_OPA1              ((uint32_t)0x00000010)
#define TIM1_Brake_Source_OPA2              ((uint32_t)0x00000018)

/* CMP_WakeUp_IO_mode_definition */
#define CMP_WakeUp_OFF                      ((uint32_t)0x00000000)
#define CMP_WakeUp_CMP1                     ((uint32_t)0x00400000)
#define CMP_WakeUp_CMP2                     ((uint32_t)0x00800000)
#define CMP_WakeUp_CMP1_CMP2                ((uint32_t)0x00C00000)

/* OPA_OUT_POLL_FLAG */
#define OPA_FLAG_OUT_POLL_CH_1              ((uint32_t)0x00000001)
#define OPA_FLAG_OUT_POLL_CH_2              ((uint32_t)0x00000002)
#define OPA_FLAG_OUT_POLL_CH_3              ((uint32_t)0x00000004)
#define OPA_FLAG_OUT_POLLIntervalEnd        ((uint32_t)0x00000010)

/* OPA_OUT_POLL_IT */
#define OPA_IT_OUT_POLL_CH_1                ((uint32_t)0x00010100)
#define OPA_IT_OUT_POLL_CH_2                ((uint32_t)0x00020100)
#define OPA_IT_OUT_POLL_CH_3                ((uint32_t)0x00040100)
#define OPA_IT_OUT_POLLIntervalEnd          ((uint32_t)0x00100200)


void OPA_Unlock(void);
void OPA_Lock(void);
void OPA_CMP_Unlock(void);
void OPA_CMP_Lock(void);
void OPA_Init(OPA_Num_TypeDef OPA_NUM, OPA_InitTypeDef *OPA_InitStruct);
void OPA_StructInit(OPA_Num_TypeDef OPA_NUM, OPA_InitTypeDef *OPA_InitStruct);
void OPA_CMP_Init(CMP_Num_TypeDef CMP_NUM, CMP_InitTypeDef *CMP_InitStruct);
void OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct);
void OPA_Cmd(OPA_Num_TypeDef OPA_NUM, FunctionalState NewState);
void OPA_CMP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
void OPA1_SystemReset_Cmd(FunctionalState NewState);
void OPA_CMP_SystemReset_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
void OPA_CMP_FILT_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
void OPA_CMP_FILT_LEN_Config(CMP_Num_TypeDef CMP_NUM, uint8_t FILT_Len);
void OPA_CMP_TIM1_BKINConfig(uint32_t Brake_Source);
void OPA_CMP_DAC_Cmd(FunctionalState NewState);
void OPA_CMP_DAC_DataConfig(uint8_t Data);
void OPA_CMP_WakeUp_ModeConfig(uint32_t CMP_WakeUP_Mode);
void OPA1_SoftwareStartPollCmd(FunctionalState NewState);
void OPA_CMP_LP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
FlagStatus OPA1_GetFlagStatus(uint32_t OPA_FLAG);
void OPA1_ClearFlag(uint32_t OPA_FLAG);
FlagStatus OPA1_GetITStatus(uint32_t OPA_IT);
void OPA1_ClearITPendingBit(uint32_t OPA_IT);

#ifdef __cplusplus
}
#endif

#endif
