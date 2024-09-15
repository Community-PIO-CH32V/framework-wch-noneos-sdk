/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_gpio.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#ifndef __CH56x_GPIO_H__
#define __CH56x_GPIO_H__

#ifdef __cplusplus
 extern "C" {
#endif


#define GPIO_Pin_0                 (0x00000001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 (0x00000002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 (0x00000004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 (0x00000008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 (0x00000010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 (0x00000020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 (0x00000040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 (0x00000080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 (0x00000100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 (0x00000200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                (0x00000400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                (0x00000800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                (0x00001000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                (0x00002000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                (0x00004000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                (0x00008000)  /*!< Pin 15 selected */
#define GPIO_Pin_16                (0x00010000)  /*!< Pin 16 selected */
#define GPIO_Pin_17                (0x00020000)  /*!< Pin 17 selected */
#define GPIO_Pin_18                (0x00040000)  /*!< Pin 18 selected */
#define GPIO_Pin_19                (0x00080000)  /*!< Pin 19 selected */
#define GPIO_Pin_20                (0x00100000)  /*!< Pin 20 selected */
#define GPIO_Pin_21                (0x00200000)  /*!< Pin 21 selected */
#define GPIO_Pin_22                (0x00400000)  /*!< Pin 22 selected */
#define GPIO_Pin_23                (0x00800000)  /*!< Pin 23 selected */
#define GPIO_Pin_24                (0x01000000)  /*!< PinB23 selected */
#define GPIO_Pin_All               (0xFFFFFFFF)  /*!< All pins selected */
 
 
/**
  * @brief  GPIO mode structure configuration
  */
typedef enum
{
	GPIO_ModeIN_Floating,			//floating input
	GPIO_ModeIN_PU_NSMT,
	GPIO_ModeIN_PD_NSMT,
	GPIO_ModeIN_PU_SMT,
	GPIO_ModeIN_PD_SMT,
	GPIO_Slowascent_PP_8mA,
	GPIO_Slowascent_PP_16mA,
	GPIO_Highspeed_PP_8mA,
	GPIO_Highspeed_PP_16mA,
	GPIO_ModeOut_OP_8mA,
	GPIO_ModeOut_OP_16mA,
}GPIOModeTypeDef;


/**
  * @brief  GPIO interrupt structure configuration
  */
typedef enum
{
	GPIO_ITMode_LowLevel,			//Low level trigger
	GPIO_ITMode_HighLevel,			//High level trigger
	GPIO_ITMode_FallEdge,			//Falling edge trigger
	GPIO_ITMode_RiseEdge,			//Rising edge trigger

}GPIOITModeTpDef;


/**
  * @brief  GPIO MCO structure configuration
  */
typedef enum
{
	MCO_125 = 0,
	MCO_25 = 4,
	MCO_2d5 = 0xC,
}MCOMode;

void GPIOA_ModeCfg( UINT32 pin, GPIOModeTypeDef mode );				/* GPIOA port pin mode configuration */
void GPIOB_ModeCfg( UINT32 pin, GPIOModeTypeDef mode );				/* GPIOB port pin mode configuration */
#define	GPIOA_ResetBits( pin )			(R32_PA_CLR |= pin)			/* GPIOA port pin output set low */
#define	GPIOA_SetBits( pin )			(R32_PA_OUT |= pin)			/* GPIOA port pin output set high */
#define	GPIOB_ResetBits( pin )			(R32_PB_CLR |= pin)			/* GPIOB port pin output set low */
#define	GPIOB_SetBits( pin )			(R32_PB_OUT |= pin)			/* GPIOB port pin output set high */	 
#define	GPIOA_InverseBits( pin )		(R32_PA_OUT ^= pin)			/* GPIOA port pin output level flip */
#define	GPIOB_InverseBits( pin )		(R32_PB_OUT ^= pin)			/* GPIOB port pin output level flip */
#define	GPIOA_ReadPort()				(R32_PA_PIN)				/* The 32-bit data returned by the GPIOA port, the lower 16 bits are valid */
#define	GPIOB_ReadPort()				(R32_PB_PIN)				/* The 32-bit data returned by the GPIOB port, the lower 24 bits are valid */
#define	GPIOA_ReadPortPin( pin )		(R32_PA_PIN&pin)			/* GPIOA port pin status, 0-pin low level, (!0)-pin high level */
#define	GPIOB_ReadPortPin( pin )		(R32_PB_PIN&pin)			/* GPIOB port pin status, 0-pin low level, (!0)-pin high level */

void GPIOA_ITModeCfg( UINT32 pin, GPIOITModeTpDef mode );			/* GPIOA pin interrupt mode configuration */
void GPIOB_ITModeCfg( UINT32 pin, GPIOITModeTpDef mode );			/* GPIOB pin interrupt mode configuration */
#define	GPIOA_ReadITFlagPort()			(R8_GPIO_INT_FLAG)				/* Read GPIOA port interrupt flag status */
#define	GPIOB_ReadITFlagPort()			(R8_GPIO_INT_FLAG)				/* Read GPIOB port interrupt flag status */

/*************************************Read Interrupt Bit Flag************************************/
#define	GPIOA_2_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x01)		    /* Read GPIOA port pin interrupt flag status */
#define	GPIOA_3_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x02)
#define	GPIOA_4_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x04)

#define	GPIOB_3_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x08)		    /* Read GPIOB port pin interrupt flag status */
#define	GPIOB_4_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x10)
#define	GPIOB_11_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x20)
#define	GPIOB_12_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x40)
#define	GPIOB_15_ReadITFlagBit(  )		(R8_GPIO_INT_FLAG & 0x80)


/*************************************Clear Interrupt Bit Flag************************************/
#define	GPIOA_2_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x01)		/* Clear the GPIOA port pin interrupt flag status */
#define	GPIOA_3_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x02)
#define	GPIOA_4_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x04)

#define	GPIOB_3_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x08)		/* Clear the GPIOB port pin interrupt flag status */
#define	GPIOB_4_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x10)
#define	GPIOB_11_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x20)
#define	GPIOB_12_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x40)
#define	GPIOB_15_ClearITFlagBit(  )		(R8_GPIO_INT_FLAG = 0x80)



void GPIOPinRemap( UINT8 s, UINT16 perph );				/* Peripheral Function Pin Mapping */
void GPIOMco( UINT8 s, UINT16 freq );                     /* MCO function */



#ifdef __cplusplus
}
#endif

#endif  // __CH56x_GPIO_H__
