/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32l103_conf.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/07/08
 * Description        : Library configuration file.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32L103_CONF_H
#define __CH32L103_CONF_H

#include "ch32l103_adc.h"
#include "ch32l103_bkp.h"
#include "ch32l103_can.h"
#include "ch32l103_crc.h"
#include "ch32l103_dbgmcu.h"
#include "ch32l103_dma.h"
#include "ch32l103_exti.h"
#include "ch32l103_flash.h"
#include "ch32l103_gpio.h"
#include "ch32l103_i2c.h"
#include "ch32l103_iwdg.h"
#include "ch32l103_pwr.h"
#include "ch32l103_rcc.h"
#include "ch32l103_rtc.h"
#include "ch32l103_spi.h"
#include "ch32l103_tim.h"
#include "ch32l103_usart.h"
#include "ch32l103_wwdg.h"
//#include "ch32l103_it.h"
#include "ch32l103_misc.h"
#include "ch32l103_lptim.h"
#include "ch32l103_opa.h"

#if __has_include("debug.h")
# include "debug.h"
#endif

/* RT Thread expects the interrupt header file from the project to be included */
/* Support both our unified name and the old name */
#if defined(__PIO_BUILD_RT_THREAD__)
#if __has_include("ch32v_it.h")
# include "ch32v_it.h"
#elif __has_include("ch32l103_it.h")
# include "ch32l103_it.h"
#endif
#endif

#endif
