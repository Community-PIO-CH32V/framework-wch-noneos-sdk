/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v205_opa.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/20
 * Description        : This file provides all the OPA firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32v205_opa.h"


/* FLASH Keys */
#define OPA_KEY1                                ((uint32_t)0x45670123)
#define OPA_KEY2                                ((uint32_t)0xCDEF89AB)

/* TIM1 Brake Source mask definition */
#define TIM1_Brake_Source_Config_MASK           ((uint32_t)0xFFFFFFE7)

/********************************************************************************
 * @fn      OPA_Unlock
 *
 * @brief   Unlocks the OPA Controller.
 *
 * @return  none
 */
void OPA_Unlock(void)
{
    OPA->OPA_KEY = OPA_KEY1;
    OPA->OPA_KEY = OPA_KEY2;
}

/********************************************************************************
 * @fn      OPA_Lock
 *
 * @brief   Locks the OPA Controller.
 *
 * @return  none
 */
void OPA_Lock(void)
{
    OPA->OPA_CFGR1 |= (1<<31);
}

/********************************************************************************
 * @fn      OPA_CMP_Unlock
 *
 * @brief   Unlocks the CMP Controller.
 *
 * @return  none
 */
void OPA_CMP_Unlock(void)
{
    OPA->CMP_KEY = OPA_KEY1;
    OPA->CMP_KEY = OPA_KEY2;
}

/********************************************************************************
 * @fn      OPA_CMP_Lock
 *
 * @brief   Locks the CMP Controller.
 *
 * @return  none
 */
void OPA_CMP_Lock(void)
{
    OPA->CMP_CTLR |= (1<<31);
}

/*********************************************************************
 * @fn      OPA_Init
 *
 * @brief   Initializes the OPA peripheral according to the specified
 *        parameters in the OPA_InitStruct.
 *
 * @param   OPA_NUM - Select OPA
 *          OPA_InitStruct - pointer to a OPA_InitTypeDef structure
 *
 * @return  none
 */
void OPA_Init(OPA_Num_TypeDef OPA_NUM, OPA_InitTypeDef *OPA_InitStruct)
{
    uint32_t tmp0 = 0, tmp1 = 0;

    if(OPA_NUM == OPA1)
    {
        tmp0 = OPA->OPA_CFGR1;
        tmp1 = OPA->OPA_CTLR1;
        tmp0 &= 0x4F00F8FF;
        tmp1 &= 0xC04020C1;

        tmp0 |= ((uint32_t)OPA_InitStruct->SETUP_Time << 16) | 
                ((uint32_t)OPA_InitStruct->POLL_Mode << 29) |
                ((uint32_t)OPA_InitStruct->SampleSource << 28) |
                ((uint32_t)OPA_InitStruct->NMI_IE << 10) |
                ((uint32_t)OPA_InitStruct->CNT_IE << 9) |
                ((uint32_t)OPA_InitStruct->OUT_IE << 8);

        tmp1 |= ((uint32_t)OPA_InitStruct->POLL_CH3 << 28) | 
                ((uint32_t)OPA_InitStruct->POLL_CH2 << 26) | 
                ((uint32_t)OPA_InitStruct->POLL_CH1 << 24) | 
                ((uint32_t)OPA_InitStruct->POLL_SEL << 19) | 
                ((uint32_t)OPA_InitStruct->POLL_NUM << 17) | 
                ((uint32_t)OPA_InitStruct->PSEL_POLL << 16) | 
                ((uint32_t)OPA_InitStruct->Chop_Frequency << 14) | 
                ((uint32_t)OPA_InitStruct->OPA_HS << 12) | 
                ((uint32_t)OPA_InitStruct->PGADIF << 11) | 
                ((uint32_t)OPA_InitStruct->FB << 10) | 
                ((uint32_t)OPA_InitStruct->PGA_SEL << 8) | 
                ((uint32_t)OPA_InitStruct->NSEL << 5) | 
                ((uint32_t)OPA_InitStruct->PSEL << 3) | 
                ((uint32_t)OPA_InitStruct->Mode << 1);

        OPA->OPA_CFGR1 = tmp0;
        OPA->OPA_CTLR1 = tmp1;
    }
    else if(OPA_NUM == OPA2)
    {
        tmp1 = OPA->OPA_CTLR2;
        tmp1 &= 0xFFFFE0C1;
      
        tmp1 |= ((uint32_t)OPA_InitStruct->OPA_HS << 12) | 
                ((uint32_t)OPA_InitStruct->PGADIF << 11) | 
                ((uint32_t)OPA_InitStruct->FB << 10) | 
                ((uint32_t)OPA_InitStruct->PGA_SEL << 8) | 
                ((uint32_t)OPA_InitStruct->NSEL << 5) | 
                ((uint32_t)OPA_InitStruct->PSEL << 3) | 
                ((uint32_t)OPA_InitStruct->Mode << 1);

        OPA->OPA_CTLR2 = tmp1;
    }
}

/*********************************************************************
 * @fn      OPA_StructInit
 *
 * @brief   Fills each OPA_StructInit member with its reset value.
 *
 * @param   OPA_NUM - Select OPA
 *          OPA_StructInit - pointer to a OPA_InitTypeDef structure
 *
 * @return  none
 */
void OPA_StructInit(OPA_Num_TypeDef OPA_NUM, OPA_InitTypeDef *OPA_InitStruct)
{
    if(OPA_NUM == OPA1)
    {
        OPA_InitStruct->SETUP_Time = 0x00;
        OPA_InitStruct->POLL_Mode = OPA_POLL_Mode_NoSingle;
        OPA_InitStruct->SampleSource = OPA_SampleSource_OPA1;
        OPA_InitStruct->NMI_IE = NMI_IE_OFF;
        OPA_InitStruct->CNT_IE = CNT_IE_OFF;
        OPA_InitStruct->OUT_IE = OUT_IE_OFF;
        OPA_InitStruct->POLL_CH3 = OPA_POLL_CH3_PB5;
        OPA_InitStruct->POLL_CH2 = OPA_POLL_CH2_PB5;
        OPA_InitStruct->POLL_CH1 = OPA_POLL_CH1_PB5;
        OPA_InitStruct->POLL_SEL = OPA_POLL_SEL_SOFT;
        OPA_InitStruct->POLL_NUM = CHP_POLL_NUM_1;
        OPA_InitStruct->PSEL_POLL = CHP_OPA_POLL_OFF;
        OPA_InitStruct->Chop_Frequency = OPA_ChopFrequency_1MHz;
        OPA_InitStruct->PSEL = CHP3;
    }
    else if(OPA_NUM == OPA2)
    {
        OPA_InitStruct->PSEL = CHP1;
    }

    OPA_InitStruct->OPA_HS = HS_OFF;
    OPA_InitStruct->PGADIF = PGADIF_GND;
    OPA_InitStruct->FB = FB_OFF;
    OPA_InitStruct->PGA_SEL = CHN_PGA_32xIN;
    OPA_InitStruct->NSEL = CHN0;    
    OPA_InitStruct->Mode = OUT_OFF;
}

/*********************************************************************
 * @fn      OPA_CMP_Init
 *
 * @brief   Initializes the CMP peripheral according to the specified
 *        parameters in the CMP_InitTypeDef.
 *
 * @param   CMP_NUM - Select CMP
 *          CMP_InitStruct - pointer to a CMP_InitTypeDef structure
 *
 * @return  none
 */
void OPA_CMP_Init(CMP_Num_TypeDef CMP_NUM, CMP_InitTypeDef *CMP_InitStruct)
{
    uint32_t tmp0 = 0;

    tmp0 = OPA->CMP_CTLR;

    if(CMP_NUM == CMP1)
    {
        tmp0 &= 0xCFFFFF01;
        tmp0 |= ((uint32_t)CMP_InitStruct->CMP_Out_Mode << 1) |
                ((uint32_t)CMP_InitStruct->NSEL << 3) |
                ((uint32_t)CMP_InitStruct->PSEL << 5) |
                ((((uint32_t)CMP_InitStruct->HYS & 0x01) << 7) | (((uint32_t)CMP_InitStruct->HYS & 0x02) << 29));
        OPA->CMP_CTLR = tmp0;
    }
    else  if(CMP_NUM == CMP2)
    {
        tmp0 &= 0xAFFF01FF;
        tmp0 |= ((uint32_t)CMP_InitStruct->CMP_Out_Mode << 9) |
                ((uint32_t)CMP_InitStruct->NSEL << 11) |
                ((uint32_t)CMP_InitStruct->PSEL << 13) |
                ((((uint32_t)CMP_InitStruct->HYS & 0x01) << 15) | (((uint32_t)CMP_InitStruct->HYS & 0x02) << 30));;
        OPA->CMP_CTLR = tmp0;
    }
}

/*********************************************************************
 * @fn      OPA_CMP_StructInit
 *
 * @brief   Fills each OPA_CMP_StructInit member with its reset value.
 *
 * @param   CMP_NUM - Select CMP
 *          CMP_StructInit - pointer to a OPA_CMP_StructInit structure
 *
 * @return  none
 */
void OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct)
{
    CMP_InitStruct->CMP_Out_Mode = OUT_IO0;
    CMP_InitStruct->NSEL = CMP_CHN_OFF;
    CMP_InitStruct->PSEL = CMP_CHP_OFF;
    CMP_InitStruct->HYS = CMP_HYS_Mode0;
}

/*********************************************************************
 * @fn      OPA_Cmd
 *
 * @brief   Enables or disables the specified OPA peripheral.
 *
 * @param   OPA_NUM - Select OPA
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_Cmd(OPA_Num_TypeDef OPA_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        if(OPA_NUM == OPA1)
        {
            OPA->OPA_CTLR1 |= (uint32_t)(1 << 0);
        }
        else if(OPA_NUM == OPA2)
        {
            OPA->OPA_CTLR2 |= (uint32_t)(1 << 0);
        } 
    }
    else
    {
        if(OPA_NUM == OPA1)
        {
            OPA->OPA_CTLR1 &= ~(uint32_t)(1 << 0);
        }
        else if(OPA_NUM == OPA2)
        {
            OPA->OPA_CTLR2 &= ~(uint32_t)(1 << 0);
        } 
    }
}

/*********************************************************************
 * @fn      OPA_CMP_Cmd
 *
 * @brief   Enables or disables the specified CMP peripheral.
 *
 * @param   CMP_NUM - Select CMP
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        OPA->CMP_CTLR |= (uint32_t)(1 << (CMP_NUM <<3));
    }
    else
    {
        OPA->CMP_CTLR &= ~(uint32_t)(1 << (CMP_NUM <<3));
    }
}

/*********************************************************************
 * @fn      OPA1_SystemReset_Cmd
 *
 * @brief   Enables or disables system reset the specified OPA1 peripheral.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA1_SystemReset_Cmd(FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        OPA->OPA_CFGR1 |= (uint32_t)(1 << 5);
    }
    else
    {
        OPA->OPA_CFGR1 &= ~(uint32_t)(1 << 5);
    }
}

/*********************************************************************
 * @fn      OPA_CMP_SystemReset_Cmd
 *
 * @brief   Enables or disables system reset the specified CMP peripheral.
 *
 * @param   CMP_NUM - Select CMP
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_SystemReset_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        OPA->CMP_CTLR |= (uint32_t)(1 << (CMP_NUM + 16));
    }
    else
    {
        OPA->CMP_CTLR &= ~(uint32_t)(1 << (CMP_NUM + 16));
    }
}

/*********************************************************************
 * @fn      OPA_CMP_FILT_Cmd
 *
 * @brief   Enables or disables digital filtering the specified CMP peripheral.
 *
 * @param   CMP_NUM - Select CMP
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_FILT_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        if(CMP_NUM == CMP1)
        {
            OPA->CMP_FILT |= (uint32_t)(1 << 0);
        }
        else if(CMP_NUM == CMP2)
        {
            OPA->CMP_FILT |= (uint32_t)(1 << 8);
        }       
    }
    else
    {
        if(CMP_NUM == CMP1)
        {
            OPA->CMP_FILT &= ~(uint32_t)(1 << 0);
        }
        else if(CMP_NUM == CMP2)
        {
            OPA->CMP_FILT &= ~(uint32_t)(1 << 8);
        } 
    }
}

/*********************************************************************
 * @fn      OPA_CMP_FILT_LEN_Config
 *
 * @brief   Configures the length of digital filtering for the CMP.
 *
 * @param   CMP_NUM - Select CMP
 *          FILT_Len - The length of digital filtering.
 *            This parameter must range from 0 to 0x1F¡£
 *
 * @return  none
 */
void OPA_CMP_FILT_LEN_Config(CMP_Num_TypeDef CMP_NUM, uint8_t FILT_Len)
{
    if(CMP_NUM == CMP1)
    {
        OPA->CMP_FILT &= ~(uint32_t)(0x1F << 1);
        OPA->CMP_FILT |= ((uint32_t)FILT_Len << 1);
    }
    else if(CMP_NUM == CMP2)
    {
        OPA->CMP_FILT &= ~(uint32_t)(0x1F << 9);
        OPA->CMP_FILT |= ((uint32_t)FILT_Len << 9);
    } 
}

/*********************************************************************
 * @fn      OPA_CMP_TIM1_BKINConfig
 *
 * @brief   Specifies the TIM1 Brake source.
 *
 * @param   Brake_Source - specifies the brake source to clear.
 *            TIM1_Brake_Source_IO - choose GPIO to be the TIM1 brake source
 *            TIM1_Brake_Source_CMP1 - choose CMP1 to be the TIM1 brake source
 *            TIM1_Brake_Source_CMP2 - choose CMP2 to be the TIM1 brake source
 *            TIM1_Brake_Source_OPA1 - choose OPA1 to be the TIM1 brake source
 *            TIM1_Brake_Source_OPA2 - choose OPA2 to be the TIM1 brake source
 *
 * @return  none
 */
void OPA_CMP_TIM1_BKINConfig(uint32_t Brake_Source)
{
    OPA->CMP_CTLR &= ~(1 << 18);
    OPA->OPA_CFGR1 &= TIM1_Brake_Source_Config_MASK;
    if(Brake_Source == TIM1_Brake_Source_CMP2)
    {
        OPA->CMP_CTLR |= (1 << 18);
        OPA->OPA_CFGR1 |= (1 << 3); 
    }
    else 
    {
        OPA->OPA_CFGR1 |= Brake_Source;   
    }
}

/*********************************************************************
 * @fn      OPA1_SoftwareStartPollCmd
 *
 * @brief   Enables or disables the selected OPA1 software start POLL.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA1_SoftwareStartPollCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        OPA->OPA_CFGR1 |= (1 << 23);
    }
    else
    {
        OPA->OPA_CFGR1 &= ~(1 << 23);
    }
}

/*********************************************************************
 * @fn      OPA_CMP_DAC_Cmd
 *
 * @brief   Enables or disables the selected DAC function.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_DAC_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        OPA->CMP_CTLR |= (1 << 24);
    }
    else
    {
        OPA->CMP_CTLR &= ~(1 << 24);
    }
}

/*********************************************************************
 * @fn      OPA_CMP_DAC_DataConfig
 *
 * @brief   Configurate the DAC data.
 *
 * @param   Data - DAC data..
 *            This parameter must range from 0 to 0xF.
 *
 * @return  none
 */
void OPA_CMP_DAC_DataConfig(uint8_t Data)
{
    OPA->CMP_CTLR &= ~(0xF << 25);
    OPA->CMP_CTLR |= ((uint32_t)Data << 25);
}

/*********************************************************************
 * @fn      OPA_CMP_WakeUp_ModeConfig
 *
 * @brief   Configures the CMP wake up Mode.
 *
 * @param   CMP_WakeUP_Mode -  Specifies wake up of the CMP.
 *            CMP_WakeUp_OFF - wake up off.
 *            CMP_WakeUp_CMP1 - CMP1 out wake up.
 *            CMP_WakeUp_CMP2 - CMP2 out wake up.
 *            CMP_WakeUp_CMP1_CMP2 - CMP1 and CMP2 out wake up.
 *
 * @return  none
 */
void OPA_CMP_WakeUp_ModeConfig(uint32_t CMP_WakeUP_Mode)
{
    OPA->CMP_CTLR &= ~(3 << 22);
    OPA->CMP_CTLR |= CMP_WakeUP_Mode;
}

/*********************************************************************
 * @fn      OPA_CMP_LP_Cmd
 *
 * @brief   Enables or disables the CMP enter low power mode.
 *
 * @param   CMP_NUM - Select CMP
 *          NewState - new state of the CMP enter low power mode
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void OPA_CMP_LP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    if(NewState)
    {
        OPA->CMP_CTLR |= (1 << (CMP_NUM + 20));
    }
    else
    {
        OPA->CMP_CTLR &= ~(1 << (CMP_NUM + 20));
    }
}

/*********************************************************************
 * @fn      OPA1_GetFlagStatus
 *
 * @brief   Checks whether the OPA flag is set or not.
 *
 * @param   OPA_FLAG - specifies the OPA flag to check.
 *            OPA_FLAG_OUT_POLL_CH_1 - the poll channel 1 of OPA1
 *            OPA_FLAG_OUT_POLL_CH_2 - the poll channel 2 of OPA1
 *            OPA_FLAG_OUT_POLL_CH_3 - the poll channel 3 of OPA1
 *            OPA_FLAG_OUT_POLLIntervalEnd - the poll poll interval end of OPA1
 *
 * @return  FlagStatus: SET or RESET.
 */
FlagStatus OPA1_GetFlagStatus(uint32_t OPA_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((OPA->OPCM_SR & OPA_FLAG) != (uint32_t)RESET)
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
 * @fn      OPA1_ClearFlag
 *
 * @brief   Clears the OPA flag.
 *
 * @param   OPA_FLAG - specifies the OPA flag to clear.
 *            OPA_FLAG_OUT_POLL_CH_1 - the poll channel 1 of OPA1
 *            OPA_FLAG_OUT_POLL_CH_2 - the poll channel 2 of OPA1
 *            OPA_FLAG_OUT_POLL_CH_3 - the poll channel 3 of OPA1
 *            OPA_FLAG_OUT_POLLIntervalEnd - the poll poll interval end of OPA1
 *
 * @return  none
 */
void OPA1_ClearFlag(uint32_t OPA_FLAG)
{
    OPA->OPCM_SR |= (uint32_t)OPA_FLAG;
}

/*********************************************************************
 * @fn      OPA1_GetITStatus
 *
 * @brief   Checks whether the specified OPA1 interrupt has occurred or not.
 *
 * @param   OPA_FLAG - specifies the OPA flag to check.
 *            OPA_IT_OUT_POLL_CH_1 - the poll channel 1 of OPA1
 *            OPA_IT_OUT_POLL_CH_2 - the poll channel 2 of OPA1
 *            OPA_IT_OUT_POLL_CH_3 - the poll channel 3 of OPA1
 *            OPA_IT_OUT_POLLIntervalEnd - the poll poll interval end of OPA1
 *
 * @return  FlagStatus: SET or RESET.
 */
FlagStatus OPA1_GetITStatus(uint32_t OPA_IT)
{
    FlagStatus bitstatus = RESET;
    uint32_t enablestatus = 0;

    enablestatus = (OPA->OPA_CFGR1 & (OPA_IT & 0xFFFF));

    if(((OPA->OPCM_SR & (OPA_IT >> 16)) != (uint32_t)RESET) && enablestatus)
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
 * @fn      OPA1_ClearITPendingBit
 *
 * @brief   Clears the OPA1 flag.
 *
 * @param   OPA_IT - specifies the OPA1 flag to clear.
 *            OPA_IT_OUT_POLL_CH_1 - the poll channel 1 of OPA1
 *            OPA_IT_OUT_POLL_CH_2 - the poll channel 2 of OPA1
 *            OPA_IT_OUT_POLL_CH_3 - the poll channel 3 of OPA1
 *            OPA_IT_OUT_POLLIntervalEnd - the poll poll interval end of OPA1
 *
 * @return  none
 */
void OPA1_ClearITPendingBit(uint32_t OPA_IT)
{
    OPA->OPCM_SR &= (uint32_t)~(OPA_IT >> 16);
}

