/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32x035_conf.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : Library configuration file.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_CONF_H
#define __CH32X035_CONF_H

#include "ch32x035_adc.h"
#include "ch32x035_awu.h"
#include "ch32x035_dbgmcu.h"
#include "ch32x035_dma.h"
#include "ch32x035_exti.h"
#include "ch32x035_flash.h"
#include "ch32x035_gpio.h"
#include "ch32x035_i2c.h"
#include "ch32x035_iwdg.h"
#include "ch32x035_pwr.h"
#include "ch32x035_rcc.h"
#include "ch32x035_spi.h"
#include "ch32x035_tim.h"
#include "ch32x035_usart.h"
#include "ch32x035_wwdg.h"
//#include "ch32x035_it.h"
#include "ch32x035_misc.h"
// Including USBPD causes a whole slew of errors within LiteOS compilation,
// so disable that for now.
#if !defined(__PIO_BUILD_HARMONY_LITEOS__)
#include "ch32x035_usbpd.h"
#endif

/* RT Thread expects the interrupt header file from the project to be included */
/* Support both our unified name and the old name */
#if defined(__PIO_BUILD_RT_THREAD__)
#if __has_include("ch32v_it.h")
# include "ch32v_it.h"
#elif __has_include("ch32x035_it.h")
# include "ch32x035_it.h"
#endif
#endif

// Fixes ch32x05_pwr.c compilation by declaring these functions
#include "debug.h"

#endif


	
	
	
