/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v205_gpio.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/05/20
 * Description        : This file contains all the functions prototypes for the
 *                      GPIO firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32V205_GPIO_H
#define __CH32V205_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v205.h"

/* Output Maximum frequency selection */
typedef enum
{
    GPIO_Speed_Medium = 1,
    GPIO_Speed_Low,
    GPIO_Speed_High
} GPIOSpeed_TypeDef;

/* Configuration Mode enumeration */
typedef enum
{
    GPIO_Mode_AIN = 0x0,
    GPIO_Mode_IN_FLOATING = 0x04,
    GPIO_Mode_IPD = 0x28,
    GPIO_Mode_IPU = 0x48,
    GPIO_Mode_Out_OD = 0x14,
    GPIO_Mode_Out_PP = 0x10,
    GPIO_Mode_AF_OD = 0x1C,
    GPIO_Mode_AF_PP = 0x18
} GPIOMode_TypeDef;

/* GPIO Init structure definition */
typedef struct
{
    uint16_t GPIO_Pin; /* Specifies the GPIO pins to be configured.
                          This parameter can be any value of @ref GPIO_pins_define */

    GPIOSpeed_TypeDef GPIO_Speed; /* Specifies the speed for the selected pins.
                                     This parameter can be a value of @ref GPIOSpeed_TypeDef */

    GPIOMode_TypeDef GPIO_Mode; /* Specifies the operating mode for the selected pins.
                                   This parameter can be a value of @ref GPIOMode_TypeDef */
} GPIO_InitTypeDef;

/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
    Bit_RESET = 0,
    Bit_SET
} BitAction;

/* GPIO_pins_define */
#define GPIO_Pin_0                      ((uint16_t)0x0001) /* Pin 0 selected */
#define GPIO_Pin_1                      ((uint16_t)0x0002) /* Pin 1 selected */
#define GPIO_Pin_2                      ((uint16_t)0x0004) /* Pin 2 selected */
#define GPIO_Pin_3                      ((uint16_t)0x0008) /* Pin 3 selected */
#define GPIO_Pin_4                      ((uint16_t)0x0010) /* Pin 4 selected */
#define GPIO_Pin_5                      ((uint16_t)0x0020) /* Pin 5 selected */
#define GPIO_Pin_6                      ((uint16_t)0x0040) /* Pin 6 selected */
#define GPIO_Pin_7                      ((uint16_t)0x0080) /* Pin 7 selected */
#define GPIO_Pin_8                      ((uint16_t)0x0100) /* Pin 8 selected */
#define GPIO_Pin_9                      ((uint16_t)0x0200) /* Pin 9 selected */
#define GPIO_Pin_10                     ((uint16_t)0x0400) /* Pin 10 selected */
#define GPIO_Pin_11                     ((uint16_t)0x0800) /* Pin 11 selected */
#define GPIO_Pin_12                     ((uint16_t)0x1000) /* Pin 12 selected */
#define GPIO_Pin_13                     ((uint16_t)0x2000) /* Pin 13 selected */
#define GPIO_Pin_14                     ((uint16_t)0x4000) /* Pin 14 selected */
#define GPIO_Pin_15                     ((uint16_t)0x8000) /* Pin 15 selected */
#define GPIO_Pin_All                    ((uint16_t)0xFFFF) /* All pins selected */

/* GPIO_Remap_define */
#define GPIO_Remap_PD01                 ((uint32_t)0x00008000) /* PD01 Alternate Function mapping */
#define GPIO_Remap_SWJ_Disable          ((uint32_t)0x00300400) /* GPIO_Remap_SWJ_Disable - Full SDI Disabled (SDI) */
#define GPIO_Remap_TIM1_LSI_Calibration ((uint32_t)0x00208000) /* TIM1 CH1 as LSI Calibration Function mapping */

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA            ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB            ((uint8_t)0x01)
#define GPIO_PortSourceGPIOC            ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD            ((uint8_t)0x03)
#define GPIO_PortSourceGPIOE            ((uint8_t)0x04)

/* GPIO_Pin_sources */
#define GPIO_PinSource0                 ((uint8_t)0x00)
#define GPIO_PinSource1                 ((uint8_t)0x01)
#define GPIO_PinSource2                 ((uint8_t)0x02)
#define GPIO_PinSource3                 ((uint8_t)0x03)
#define GPIO_PinSource4                 ((uint8_t)0x04)
#define GPIO_PinSource5                 ((uint8_t)0x05)
#define GPIO_PinSource6                 ((uint8_t)0x06)
#define GPIO_PinSource7                 ((uint8_t)0x07)
#define GPIO_PinSource8                 ((uint8_t)0x08)
#define GPIO_PinSource9                 ((uint8_t)0x09)
#define GPIO_PinSource10                ((uint8_t)0x0A)
#define GPIO_PinSource11                ((uint8_t)0x0B)
#define GPIO_PinSource12                ((uint8_t)0x0C)
#define GPIO_PinSource13                ((uint8_t)0x0D)
#define GPIO_PinSource14                ((uint8_t)0x0E)
#define GPIO_PinSource15                ((uint8_t)0x0F)

/* GPIO_AF_Define */
#define GPIO_AF0                        ((uint8_t)0x00)
#define GPIO_AF1                        ((uint8_t)0x01)
#define GPIO_AF2                        ((uint8_t)0x02)
#define GPIO_AF3                        ((uint8_t)0x03)
#define GPIO_AF4                        ((uint8_t)0x04)
#define GPIO_AF5                        ((uint8_t)0x05)
#define GPIO_AF6                        ((uint8_t)0x06)
#define GPIO_AF7                        ((uint8_t)0x07)

/* GPIO_Remap_define */
#define GPIO_Remap_USBPD_IN_HVT         ((uint32_t)0x00000200) 
#define GPIO_Remap_UDP_BC_VSRC          ((uint32_t)0x00010000)
#define GPIO_Remap_UDM_BC_VSRC          ((uint32_t)0x00020000) 
#define GPIO_Remap_UDP_BC_CMPE          ((uint32_t)0x00040000) 
#define GPIO_Remap_UDM_BC_CMPE          ((uint32_t)0x00080000) 
#define GPIO_Remap_UDP_BC_CMPO          ((uint32_t)0x00100000) 
#define GPIO_Remap_UDM_BC_CMPO          ((uint32_t)0x00200000) 



void     GPIO_DeInit(GPIO_TypeDef *GPIOx);
void     GPIO_AFIODeInit(void);
void     GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);
void     GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct);
uint8_t  GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
uint8_t  GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void     GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void     GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);
void     GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void     GPIO_EventOutputCmd(FunctionalState NewState);
void     GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void     GPIO_PinAFConfig(GPIO_TypeDef *GPIOx, uint8_t GPIO_PinSource, uint8_t GPIO_AF);
void     GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void     GPIO_PinUSB_PD_BC_Config(uint32_t GPIO_Remap, FunctionalState NewState);
void     GPIO_IPD_Unused(void);

#ifdef __cplusplus
}
#endif

#endif
