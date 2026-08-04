/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v205_tkey.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/05/20
 * Description        : This file provides all the TKEY firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32v205_tkey.h"
#include "ch32v205_rcc.h"


/*********************************************************************
 * @fn      TKEY_ChargeTransferMode_Init
 *
 * @brief   Initializes the TKEYx peripheral according to the specified
 *        parameters in the TKEY_ChargeTransferMode_InitStruct.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_ChargeTransferMode_InitStruct - pointer to an TKEY_ChargeTransferModeInitTypeDef structure that
 *        contains the configuration information for the specified TKEY
 *        peripheral.
 *
 * @return  none
 */
void TKEY_ChargeTransferMode_Init(TKEY_TypeDef *TKEYx, TKEY_ChargeTransferModeInitTypeDef *TKEY_ChargeTransferMode_InitStruct)
{
    TKEYx->TRANS_CFG = ((((uint32_t)TKEY_ChargeTransferMode_InitStruct->TKEY_ChargeTransferCycles - 1) << 3) |
                    TKEY_ChargeTransferMode_InitStruct->TKEY_SwitchTime | 
                    TKEY_ChargeTransferMode_InitStruct->TKEY_T2_T4);
}

/*********************************************************************
 * @fn      TKEY_ChargeTransferMode_StructInit
 *
 * @brief   Fills each TKEY_ChargeTransferMode_InitStruct member with its default value.
 *
 * @param   TKEY_ChargeTransferMode_InitStruct - pointer to an TKEY_ChargeTransferModeInitTypeDef structure that
 *        contains the configuration information for the specified TKEY
 *        peripheral.
 *
 * @return  none
 */
void TKEY_ChargeTransferMode_StructInit(TKEY_ChargeTransferModeInitTypeDef *TKEY_ChargeTransferMode_InitStruct)
{
    TKEY_ChargeTransferMode_InitStruct->TKEY_ChargeTransferCycles = 1;
    TKEY_ChargeTransferMode_InitStruct->TKEY_SwitchTime = TKEY_SwitchTime_1HCLK;
    TKEY_ChargeTransferMode_InitStruct->TKEY_T2_T4 = TKEY_T2_16HCLK_T4_128HCLK;
}

/*********************************************************************
 * @fn      TKEY_CurrentSourceChargeMode_Init
 *
 * @brief   Initializes the TKEYx peripheral according to the specified
 *        parameters in the TKEY_CurrentSourceChargeMode_InitStruct.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_CurrentSourceChargeMode_InitStruct - pointer to an TKEY_CurrentSourceChargeModeInitTypeDef structure that
 *        contains the configuration information for the specified TKEY
 *        peripheral.
 *
 * @return  none
 */
void TKEY_CurrentSourceChargeMode_Init(TKEY_TypeDef *TKEYx, TKEY_CurrentSourceChargeModeInitTypeDef *TKEY_CurrentSourceChargeMode_InitStruct)
{
    TKEYx->CDCC = ((((uint32_t)TKEY_CurrentSourceChargeMode_InitStruct->TKEY_ChargeTime) << 16) |
                    (uint32_t)TKEY_CurrentSourceChargeMode_InitStruct->TKEY_DischargeTime);

    TKEYx->TRANS_CFG &= ~(3 << 13);
    TKEYx->TRANS_CFG |= TKEY_CurrentSourceChargeMode_InitStruct->TKEY_SwitchTime;
}

/*********************************************************************
 * @fn      TKEY_CurrentSourceChargeMode_Init_StructInit
 *
 * @brief   Fills each TKEY_CurrentSourceChargeMode_InitStruct member with its default value.
 *
 * @param   TKEY_CurrentSourceChargeMode_InitStruct - pointer to an TKEY_CurrentSourceChargeModeInitTypeDef structure that
 *        contains the configuration information for the specified TKEY
 *        peripheral.
 *
 * @return  none
 */
void TKEY_CurrentSourceChargeMode_Init_StructInit(TKEY_CurrentSourceChargeModeInitTypeDef *TKEY_CurrentSourceChargeMode_InitStruct)
{
    TKEY_CurrentSourceChargeMode_InitStruct->TKEY_SwitchTime = TKEY_SwitchTime_1HCLK;
    TKEY_CurrentSourceChargeMode_InitStruct->TKEY_ChargeTime = 0;
    TKEY_CurrentSourceChargeMode_InitStruct->TKEY_DischargeTime = 0;
}

/*********************************************************************
 * @fn      TKEY_Cmd
 *
 * @brief   Enables or disables the specified TKEY peripheral.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TKEY_Cmd(TKEY_TypeDef *TKEYx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        TKEYx->CTLR |= (1 << 0);
    }
    else
    {
        TKEYx->CTLR &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      TKEY_ScanCmd
 *
 * @brief   Enables or disables the specified TKEY scan peripheral.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TKEY_ScanCmd(TKEY_TypeDef *TKEYx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        TKEYx->CTLR |= (1 << 11);
    }
    else
    {
        TKEYx->CTLR &= ~(1 << 11);
    }
}

/*********************************************************************
 * @fn      TKEY_SoftwareStartCmd
 *
 * @brief   Enables or disables the selected TKEY software start conversion.
 *
 * @param   ADCx - where x can be 1 to select the TKEY peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TKEY_SoftwareStartCmd(TKEY_TypeDef *TKEYx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        TKEYx->CTLR |= (1 << 8);
    }
    else
    {
        TKEYx->CTLR &= ~(1 << 8);
    }
}

/*********************************************************************
 * @fn      TKEY_ChargeCurrentModeConfig
 *
 * @brief   Configures the charge current mode for the selected TKEY.
 *
 * @param   TKEYx - where x can be 1 to select the KEY peripheral.
 *          TKEY_ChargeCurrentMode - Charge current mode.
 *            TKEY_ChargeCurrentMode0 - Charge current mode 0 (35uA).
 *            TKEY_ChargeCurrentMode1 - Charge current mode 1 ((35/2)uA).
 *
 * @return  none
 */
void TKEY_ChargeCurrentModeConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_ChargeCurrentMode)
{
    TKEYx->CTLR &= ~(1 << 1);
    TKEYx->CTLR |= TKEY_ChargeCurrentMode;
}

/*********************************************************************
 * @fn      TKEY_WorkModeConfig
 *
 * @brief   Configures the work mode for the selected TKEY.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_WorkMode - work mode.
 *            TKEY_WorkMode_ChargeTransfer - Charge transfer work mode.
 *            TKEY_WorkMode_CurrentSourceCharge - Current source charge work mode.
 *
 * @return  none
 */
void TKEY_WorkModeConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_WorkMode)
{
    TKEYx->CTLR &= ~(1 << 2);
    TKEYx->CTLR |= TKEY_WorkMode;
}

/*********************************************************************
 * @fn      TKEY_CtrlChannelModeConfig
 *
 * @brief   Configures the control channel mode for the selected TKEY.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_CtrlChannelMode - Control channel mode.
 *            TKEY_CtrlChannelMode_ADC - ADC control channel mode.
 *            TKEY_CtrlChannelMode_TKEY - TKEY control channel mode.
 *
 * @return  none
 */
void TKEY_CtrlChannelModeConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_CtrlChannelMode)
{
    TKEYx->CTLR &= ~(1 << 3);
    TKEYx->CTLR |= TKEY_CtrlChannelMode;
}

/*********************************************************************
 * @fn      TKEY_ScanChannelNumConfig
 *
 * @brief   Configures the scan channel number for the TKEY.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_ScanChannelNum - scan channel number.
 *            This parameter must range from 1 to 16.
 *
 * @return  none
 */
void TKEY_ScanChannelNumConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_ScanChannelNum)
{
    TKEYx->CTLR &= ~(0xF << 12);
    TKEYx->CTLR |= ((TKEY_ScanChannelNum - 1) << 12);
}

/*********************************************************************
 * @fn      TKey_ChannelxMulShieldCmd
 *
 * @brief   Enables or disables TKey Multiplex shielding of the selected TKEY channel.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_Channel - the TKEY channel to configure.
 *            TKEY_Channel_x - TKEY Channel x selected.
 *              x must from 0 to 15.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TKey_ChannelxMulShieldCmd(TKEY_TypeDef *TKEYx, uint8_t TKEY_Channel, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        TKEYx->DRV_CFG |= ((1 << 16) << TKEY_Channel);
    }
    else
    {
        TKEYx->DRV_CFG &= ~((1 << 16) << TKEY_Channel);
    }
}

/*********************************************************************
 * @fn      TKey_MulShieldCmd
 *
 * @brief   Enables or disables the TKey Multiplex shielding.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TKey_MulShieldCmd(TKEY_TypeDef *TKEYx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        TKEYx->DRV_CFG |= (1 << 0);
    }
    else
    {
        TKEYx->DRV_CFG &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      TKEY_ITConfig
 *
 * @brief   Enables or disables the specified TKEY interrupts.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_IT - specifies the TKEY interrupt sources to be enabled or disabled.
 *            TKEY_IT_SCAN - Scan interrupt mask.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TKEY_ITConfig(TKEY_TypeDef *TKEYx, uint32_t TKEY_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        TKEYx->CTLR |= TKEY_IT;
    }
    else
    {
        TKEYx->CTLR &= ~TKEY_IT;
    }
}

/*********************************************************************
 * @fn      TKEY_ChargeTransferChannelConfig
 *
 * @brief   Configures the charge transfer channel for the selected TKEY.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_Channel_S - Charge transfer channel.
 *            TKEY_Channel_x - TKEY Channel x selected.
 *              x must from 0 to 15.
 *            TKEY_Channel_None - No channel selected
 *          TKEY_Channel_X - External capacitor channel.
 *            TKEY_Channel_x - TKEY Channel x selected.
 *              x must from 0 to 15. 
 *
 * @return  none
 */
void TKEY_ChargeTransferChannelConfig(TKEY_TypeDef *TKEYx, uint8_t TKEY_Channel_S, uint8_t TKEY_Channel_X)
{
    if(TKEY_Channel_S == TKEY_Channel_None)
    {
        TKEYx->CAP_CFG = (uint32_t)TKEY_Channel_X;
    }
    else
    {
        TKEYx->CAP_CFG = (uint32_t)TKEY_Channel_X | (uint32_t)(1 << (TKEY_Channel_S + 4)); 
    }
    
}

/*********************************************************************
 * @fn      TKEY_CurrentSourceChargeChannelConfig
 *
 * @brief   Configures the current source charge channel for the selected TKEY.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_Channel_S - Charge transfer channel.
 *            TKEY_Channel_x - TKEY Channel x selected.
 *              x must from 0 to 15.
 *            TKEY_Channel_None - No channel selected
 *
 * @return  none
 */
void TKEY_CurrentSourceChargeChannelConfig(TKEY_TypeDef *TKEYx, uint8_t TKEY_Channel_S)
{
    TKEYx->CAP_CFG &= ~(0xFF << 4);

    if(TKEY_Channel_S != TKEY_Channel_None)
    {
        TKEYx->CAP_CFG = (uint32_t)(1 << (TKEY_Channel_S + 4)); 
    }      
}

/*********************************************************************
 * @fn      TKEY_GetFlagStatus
 *
 * @brief   Checks whether the specified TKEY flag is set or not.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_FLAG - specifies the flag to check.
 *            TKEY_FLAG_SCAN - Scan flag.
 *
 * @return  FlagStatus: SET or RESET.
 */
FlagStatus TKEY_GetFlagStatus(TKEY_TypeDef *TKEYx, uint32_t TKEY_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((TKEYx->SR & (TKEY_FLAG >> 10)) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      TKEY_ClearFlag
 *
 * @brief   Clears the TKEYx's pending flags.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_FLAG - specifies the flag to clear.
 *            TKEY_FLAG_SCAN - Scan flag.
 *
 * @return  none
 */
void TKEY_ClearFlag(TKEY_TypeDef *TKEYx, uint32_t TKEY_FLAG)
{
    TKEYx->SR = ~(uint32_t)(TKEY_FLAG >> 10);
}

/*********************************************************************
 * @fn      TKEY_GetITStatus
 *
 * @brief   Checks whether the specified TKEY interrupt has occurred or not.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_IT - specifies the interrupt pending bit to check.
 *            TKEY_IT_SCAN - Scan interrupt mask.
 *
 * @return  FlagStatus: SET or RESET.
 */
ITStatus TKEY_GetITStatus(TKEY_TypeDef *TKEYx, uint32_t TKEY_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;

    enablestatus = (TKEYx->CTLR & TKEY_IT);

    if(((TKEYx->SR & (TKEY_IT >> 10)) != (uint32_t)RESET) && enablestatus)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      TKEY_ClearITPendingBit
 *
 * @brief   Clears the TKEYx's interrupt pending bits.
 *
 * @param   TKEYx - where x can be 1 to select the TKEY peripheral.
 *          TKEY_IT - specifies the TKEY interrupt pending bit to clear.
 *            TKEY_IT_SCAN - Scan interrupt mask.
 *
 * @return  none
 */
void TKEY_ClearITPendingBit(TKEY_TypeDef *TKEYx, uint32_t TKEY_IT)
{
    TKEYx->SR = ~(uint32_t)(TKEY_IT >> 10);
}
