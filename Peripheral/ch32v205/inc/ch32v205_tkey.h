/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v205_tkey.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/05/20
 * Description        : This file contains all the functions prototypes for the
 *                      TKEY firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32V205_TKEY_H
#define __CH32V205_TKEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v205.h"

/* TKEY Charge Transfer Mode Init structure definition */
typedef struct
{
    uint32_t TKEY_SwitchTime; /* Configures the Tkey Switching time of the switch.
                                 This parameter can be a value of @ref TKEY_SwitchTime */

    uint32_t TKEY_ChargeTransferCycles; /* Configures the Tkey the number of Charge Transfer cycles.
                                 This parameter must range from 1 to 1024 */  

    uint32_t TKEY_T2_T4; /* Configures the Tkey capacitor charge time(T2) and Charge Transfer time(T4).
                                 This parameter can be a value of @ref TKEY_T2_T4 */                           

} TKEY_ChargeTransferModeInitTypeDef;

/* TKEY Current Source Charge Mode Init structure definition */
typedef struct
{
    uint32_t TKEY_SwitchTime; /* Configures the Tkey Switching time of the switch.
                                 This parameter can be a value of @ref TKEY_SwitchTime */

    uint32_t TKEY_ChargeTime; /* Configures the Tkey Charging time of the switch.
                                 This parameter must range from 0 to 0x7FF */      

    uint32_t TKEY_DischargeTime; /* Configures the Tkey Discharging time of the switch.
                                 This parameter must range from 0 to 0x7FF */       

} TKEY_CurrentSourceChargeModeInitTypeDef;

/* TKEY_SwitchTime */
#define TKEY_SwitchTime_1HCLK           ((uint32_t)0x00000000)
#define TKEY_SwitchTime_2HCLK           ((uint32_t)0x00002000)
#define TKEY_SwitchTime_4HCLK           ((uint32_t)0x00004000)
#define TKEY_SwitchTime_8HCLK           ((uint32_t)0x00006000)

/* TKEY_T2_T4 */
#define TKEY_T2_16HCLK_T4_128HCLK       ((uint32_t)0x00000000)
#define TKEY_T2_16HCLK_T4_192HCLK       ((uint32_t)0x00000001)
#define TKEY_T2_32HCLK_T4_128HCLK       ((uint32_t)0x00000002)
#define TKEY_T2_32HCLK_T4_192HCLK       ((uint32_t)0x00000003)
#define TKEY_T2_32HCLK_T4_256HCLK       ((uint32_t)0x00000004)
#define TKEY_T2_32HCLK_T4_320HCLK       ((uint32_t)0x00000005)
#define TKEY_T2_48HCLK_T4_320HCLK       ((uint32_t)0x00000006)
#define TKEY_T2_48HCLK_T4_384HCLK       ((uint32_t)0x00000007)

/* TKEY_ChargeCurrentMode */
#define TKEY_ChargeCurrentMode0         ((uint32_t)0x00000000)
#define TKEY_ChargeCurrentMode1         ((uint32_t)0x00000002)

/* TKEY_WorkMode */
#define TKEY_WorkMode_ChargeTransfer         ((uint32_t)0x00000000)
#define TKEY_WorkMode_CurrentSourceCharge    ((uint32_t)0x00000004)

/* TKEY_CtrlChannelMode */
#define TKEY_CtrlChannelMode_ADC        ((uint32_t)0x00000000)
#define TKEY_CtrlChannelMode_TKEY       ((uint32_t)0x00000008)

/* TKEY_channels */
#define TKEY_Channel_0                  ((uint8_t)0x00)
#define TKEY_Channel_1                  ((uint8_t)0x01)
#define TKEY_Channel_2                  ((uint8_t)0x02)
#define TKEY_Channel_3                  ((uint8_t)0x03)
#define TKEY_Channel_4                  ((uint8_t)0x04)
#define TKEY_Channel_5                  ((uint8_t)0x05)
#define TKEY_Channel_6                  ((uint8_t)0x06)
#define TKEY_Channel_7                  ((uint8_t)0x07)
#define TKEY_Channel_8                  ((uint8_t)0x08)
#define TKEY_Channel_9                  ((uint8_t)0x09)
#define TKEY_Channel_10                 ((uint8_t)0x0A)
#define TKEY_Channel_11                 ((uint8_t)0x0B)
#define TKEY_Channel_12                 ((uint8_t)0x0C)
#define TKEY_Channel_13                 ((uint8_t)0x0D)
#define TKEY_Channel_14                 ((uint8_t)0x0E)
#define TKEY_Channel_15                 ((uint8_t)0x0F)
#define TKEY_Channel_None               ((uint8_t)0x80)

/* TKEY_interrupts_definition */
#define TKEY_IT_SCAN                    ((uint32_t)0x00000400)

/* TKEY_flags_definition */
#define TKEY_FLAG_SCAN                  ((uint32_t)0x00000400)


void TKEY_ChargeTransferMode_Init(TKEY_TypeDef *TKEYx, TKEY_ChargeTransferModeInitTypeDef *TKEY_ChargeTransferMode_InitStruct);
void TKEY_ChargeTransferMode_StructInit(TKEY_ChargeTransferModeInitTypeDef *TKEY_ChargeTransferMode_InitStruct);
void TKEY_CurrentSourceChargeMode_Init(TKEY_TypeDef *TKEYx, TKEY_CurrentSourceChargeModeInitTypeDef *TKEY_CurrentSourceChargeMode_InitStruct);
void TKEY_CurrentSourceChargeMode_Init_StructInit(TKEY_CurrentSourceChargeModeInitTypeDef *TKEY_CurrentSourceChargeMode_InitStruct);
void TKEY_Cmd(TKEY_TypeDef *TKEYx, FunctionalState NewState);
void TKEY_ScanCmd(TKEY_TypeDef *TKEYx, FunctionalState NewState);
void TKEY_SoftwareStartCmd(TKEY_TypeDef *TKEYx, FunctionalState NewState);
void TKEY_ChargeCurrentModeConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_ChargeCurrentMode);
void TKEY_WorkModeConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_WorkMode);
void TKEY_CtrlChannelModeConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_CtrlChannelMode);
void TKEY_ScanChannelNumConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_ScanChannelNum);
void TKey_ChannelxMulShieldCmd(TKEY_TypeDef *TKEYx, uint8_t TKEY_Channel, FunctionalState NewState);
void TKey_MulShieldCmd(TKEY_TypeDef *TKEYx, FunctionalState NewState);
void TKEY_ITConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_IT, FunctionalState NewState);
void TKEY_ChargeTransferChannelConfig(TKEY_TypeDef *TKEYx, uint8_t TKEY_Channel_S, uint8_t TKEY_Channel_X);
void TKEY_CurrentSourceChargeChannelConfig(TKEY_TypeDef *TKEYx, uint8_t TKEY_Channel_S);
FlagStatus TKEY_GetFlagStatus(TKEY_TypeDef *TKEYx, uint32_t TKEY_FLAG);
void TKEY_ClearFlag(TKEY_TypeDef *TKEYx, uint32_t TKEY_FLAG);
ITStatus TKEY_GetITStatus(TKEY_TypeDef *TKEYx, uint32_t TKEY_IT);
void TKEY_ClearITPendingBit(TKEY_TypeDef *TKEYx, uint32_t TKEY_IT);


#ifdef __cplusplus
}
#endif

#endif
