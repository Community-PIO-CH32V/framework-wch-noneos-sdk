/********************************** (C) COPYRIGHT *******************************
* File Name          : startup_ch32v30x_D8C.s
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/04/06
* Description        : CH32V307x-CH32V305x vector table for eclipse toolchain.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

	.section	.init,"ax",@progbits
	.global	_start
	.align	1
_start:
	j	handle_reset

    .section    .vector,"ax",@progbits
    .align  1
_vector_base:
    .option norvc;
    .word   _start
    .word   0
    .word   NMI_Handler                /* NMI */
    .word   HardFault_Handler          /* Hard Fault */
    .word   0
    .word   Ecall_M_Mode_Handler       /* Ecall M Mode */
    .word   0
    .word   0
    .word   Ecall_U_Mode_Handler       /* Ecall U Mode */
    .word   Break_Point_Handler        /* Break Point */
    .word   0
    .word   0
    .word   SysTick_Handler            /* SysTick */
    .word   0
    .word   SW_Handler                 /* SW */
    .word   0
    /* External Interrupts */
    .word   WWDG_IRQHandler            /* Window Watchdog */
    .word   PVD_IRQHandler             /* PVD through EXTI Line detect */
    .word   TAMPER_IRQHandler          /* TAMPER */
    .word   RTC_IRQHandler             /* RTC */
    .word   FLASH_IRQHandler           /* Flash */
    .word   RCC_IRQHandler             /* RCC */
    .word   EXTI0_IRQHandler           /* EXTI Line 0 */
    .word   EXTI1_IRQHandler           /* EXTI Line 1 */
    .word   EXTI2_IRQHandler           /* EXTI Line 2 */
    .word   EXTI3_IRQHandler           /* EXTI Line 3 */
    .word   EXTI4_IRQHandler           /* EXTI Line 4 */
    .word   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
    .word   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
    .word   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
    .word   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
    .word   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
    .word   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
    .word   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
    .word   ADC1_2_IRQHandler          /* ADC1_2 */
    .word   USB_HP_CAN1_TX_IRQHandler  /* USB HP and CAN1 TX */
    .word   USB_LP_CAN1_RX0_IRQHandler /* USB LP and CAN1RX0 */
    .word   CAN1_RX1_IRQHandler        /* CAN1 RX1 */
    .word   CAN1_SCE_IRQHandler        /* CAN1 SCE */
    .word   EXTI9_5_IRQHandler         /* EXTI Line 9..5 */
    .word   TIM1_BRK_IRQHandler        /* TIM1 Break */
    .word   TIM1_UP_IRQHandler         /* TIM1 Update */
    .word   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
    .word   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
    .word   TIM2_IRQHandler            /* TIM2 */
    .word   TIM3_IRQHandler            /* TIM3 */
    .word   TIM4_IRQHandler            /* TIM4 */
    .word   I2C1_EV_IRQHandler         /* I2C1 Event */
    .word   I2C1_ER_IRQHandler         /* I2C1 Error */
    .word   I2C2_EV_IRQHandler         /* I2C2 Event */
    .word   I2C2_ER_IRQHandler         /* I2C2 Error */
    .word   SPI1_IRQHandler            /* SPI1 */
    .word   SPI2_IRQHandler            /* SPI2 */
    .word   USART1_IRQHandler          /* USART1 */
    .word   USART2_IRQHandler          /* USART2 */
    .word   USART3_IRQHandler          /* USART3 */
    .word   EXTI15_10_IRQHandler       /* EXTI Line 15..10 */
    .word   RTCAlarm_IRQHandler        /* RTC Alarm through EXTI Line */
    .word   USBWakeUp_IRQHandler       /* USB Wakeup from suspend */
    .word   TIM8_BRK_IRQHandler        /* TIM8 Break */
    .word   TIM8_UP_IRQHandler         /* TIM8 Update */
    .word   TIM8_TRG_COM_IRQHandler    /* TIM8 Trigger and Commutation */
    .word   TIM8_CC_IRQHandler         /* TIM8 Capture Compare */
    .word   RNG_IRQHandler             /* RNG */
    .word   0
    .word   SDIO_IRQHandler            /* SDIO */
    .word   TIM5_IRQHandler            /* TIM5 */
    .word   SPI3_IRQHandler            /* SPI3 */
    .word   UART4_IRQHandler           /* UART4 */
    .word   UART5_IRQHandler           /* UART5 */
    .word   TIM6_IRQHandler            /* TIM6 */
    .word   TIM7_IRQHandler            /* TIM7 */
    .word   DMA2_Channel1_IRQHandler   /* DMA2 Channel 1 */
    .word   DMA2_Channel2_IRQHandler   /* DMA2 Channel 2 */
    .word   DMA2_Channel3_IRQHandler   /* DMA2 Channel 3 */
    .word   DMA2_Channel4_IRQHandler   /* DMA2 Channel 4 */
    .word   DMA2_Channel5_IRQHandler   /* DMA2 Channel 5 */
    .word   ETH_IRQHandler             /* ETH */
    .word   ETH_WKUP_IRQHandler        /* ETH WakeUp */
    .word   CAN2_TX_IRQHandler         /* CAN2 TX */
    .word   CAN2_RX0_IRQHandler        /* CAN2 RX0 */
    .word   CAN2_RX1_IRQHandler        /* CAN2 RX1 */
    .word   CAN2_SCE_IRQHandler        /* CAN2 SCE */
    .word   USBFS_IRQHandler           /* USBFS */
    .word   USBHSWakeup_IRQHandler     /* USBHS Wakeup */
    .word   USBHS_IRQHandler           /* USBHS */
    .word   DVP_IRQHandler             /* DVP */
    .word   UART6_IRQHandler           /* UART6 */
    .word   UART7_IRQHandler           /* UART7 */
    .word   UART8_IRQHandler           /* UART8 */
    .word   TIM9_BRK_IRQHandler        /* TIM9 Break */
    .word   TIM9_UP_IRQHandler         /* TIM9 Update */
    .word   TIM9_TRG_COM_IRQHandler    /* TIM9 Trigger and Commutation */
    .word   TIM9_CC_IRQHandler         /* TIM9 Capture Compare */
    .word   TIM10_BRK_IRQHandler       /* TIM10 Break */
    .word   TIM10_UP_IRQHandler        /* TIM10 Update */
    .word   TIM10_TRG_COM_IRQHandler   /* TIM10 Trigger and Commutation */
    .word   TIM10_CC_IRQHandler        /* TIM10 Capture Compare */
    .word   DMA2_Channel6_IRQHandler   /* DMA2 Channel 6 */
    .word   DMA2_Channel7_IRQHandler   /* DMA2 Channel 7 */
    .word   DMA2_Channel8_IRQHandler   /* DMA2 Channel 8 */
    .word   DMA2_Channel9_IRQHandler   /* DMA2 Channel 9 */
    .word   DMA2_Channel10_IRQHandler  /* DMA2 Channel 10 */
    .word   DMA2_Channel11_IRQHandler  /* DMA2 Channel 11 */

    .option rvc;
    .section    .text.vector_handler, "ax", @progbits
    .weak   NMI_Handler                /* NMI */
    .weak   HardFault_Handler          /* Hard Fault */
    .weak   Ecall_M_Mode_Handler       /* Ecall M Mode */
    .weak   Ecall_U_Mode_Handler       /* Ecall U Mode */
    .weak   Break_Point_Handler        /* Break Point */
    .weak   SysTick_Handler            /* SysTick */
    .weak   SW_Handler                 /* SW */
    .weak   WWDG_IRQHandler            /* Window Watchdog */
    .weak   PVD_IRQHandler             /* PVD through EXTI Line detect */
    .weak   TAMPER_IRQHandler          /* TAMPER */
    .weak   RTC_IRQHandler             /* RTC */
    .weak   FLASH_IRQHandler           /* Flash */
    .weak   RCC_IRQHandler             /* RCC */
    .weak   EXTI0_IRQHandler           /* EXTI Line 0 */
    .weak   EXTI1_IRQHandler           /* EXTI Line 1 */
    .weak   EXTI2_IRQHandler           /* EXTI Line 2 */
    .weak   EXTI3_IRQHandler           /* EXTI Line 3 */
    .weak   EXTI4_IRQHandler           /* EXTI Line 4 */
    .weak   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
    .weak   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
    .weak   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
    .weak   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
    .weak   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
    .weak   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
    .weak   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
    .weak   ADC1_2_IRQHandler          /* ADC1_2 */
    .weak   USB_HP_CAN1_TX_IRQHandler  /* USB HP and CAN1 TX */
    .weak   USB_LP_CAN1_RX0_IRQHandler /* USB LP and CAN1RX0 */
    .weak   CAN1_RX1_IRQHandler        /* CAN1 RX1 */
    .weak   CAN1_SCE_IRQHandler        /* CAN1 SCE */
    .weak   EXTI9_5_IRQHandler         /* EXTI Line 9..5 */
    .weak   TIM1_BRK_IRQHandler        /* TIM1 Break */
    .weak   TIM1_UP_IRQHandler         /* TIM1 Update */
    .weak   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
    .weak   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
    .weak   TIM2_IRQHandler            /* TIM2 */
    .weak   TIM3_IRQHandler            /* TIM3 */
    .weak   TIM4_IRQHandler            /* TIM4 */
    .weak   I2C1_EV_IRQHandler         /* I2C1 Event */
    .weak   I2C1_ER_IRQHandler         /* I2C1 Error */
    .weak   I2C2_EV_IRQHandler         /* I2C2 Event */
    .weak   I2C2_ER_IRQHandler         /* I2C2 Error */
    .weak   SPI1_IRQHandler            /* SPI1 */
    .weak   SPI2_IRQHandler            /* SPI2 */
    .weak   USART1_IRQHandler          /* USART1 */
    .weak   USART2_IRQHandler          /* USART2 */
    .weak   USART3_IRQHandler          /* USART3 */
    .weak   EXTI15_10_IRQHandler       /* EXTI Line 15..10 */
    .weak   RTCAlarm_IRQHandler        /* RTC Alarm through EXTI Line */
    .weak   USBWakeUp_IRQHandler       /* USB Wakeup from suspend */
    .weak   TIM8_BRK_IRQHandler        /* TIM8 Break */
    .weak   TIM8_UP_IRQHandler         /* TIM8 Update */
    .weak   TIM8_TRG_COM_IRQHandler    /* TIM8 Trigger and Commutation */
    .weak   TIM8_CC_IRQHandler         /* TIM8 Capture Compare */
    .weak   RNG_IRQHandler             /* RNG */
    .weak   SDIO_IRQHandler            /* SDIO */
    .weak   TIM5_IRQHandler            /* TIM5 */
    .weak   SPI3_IRQHandler            /* SPI3 */
    .weak   UART4_IRQHandler           /* UART4 */
    .weak   UART5_IRQHandler           /* UART5 */
    .weak   TIM6_IRQHandler            /* TIM6 */
    .weak   TIM7_IRQHandler            /* TIM7 */
    .weak   DMA2_Channel1_IRQHandler   /* DMA2 Channel 1 */
    .weak   DMA2_Channel2_IRQHandler   /* DMA2 Channel 2 */
    .weak   DMA2_Channel3_IRQHandler   /* DMA2 Channel 3 */
    .weak   DMA2_Channel4_IRQHandler   /* DMA2 Channel 4 */
    .weak   DMA2_Channel5_IRQHandler   /* DMA2 Channel 5 */
    .weak   ETH_IRQHandler             /* ETH */
    .weak   ETH_WKUP_IRQHandler        /* ETH WakeUp */
    .weak   CAN2_TX_IRQHandler         /* CAN2 TX */
    .weak   CAN2_RX0_IRQHandler        /* CAN2 RX0 */
    .weak   CAN2_RX1_IRQHandler        /* CAN2 RX1 */
    .weak   CAN2_SCE_IRQHandler        /* CAN2 SCE */
    .weak   USBFS_IRQHandler           /* USBFS */
    .weak   USBHSWakeup_IRQHandler     /* USBHS Wakeup */
    .weak   USBHS_IRQHandler           /* USBHS */
    .weak   DVP_IRQHandler             /* DVP */
    .weak   UART6_IRQHandler           /* UART6 */
    .weak   UART7_IRQHandler           /* UART7 */
    .weak   UART8_IRQHandler           /* UART8 */
    .weak   TIM9_BRK_IRQHandler        /* TIM9 Break */
    .weak   TIM9_UP_IRQHandler         /* TIM9 Update */
    .weak   TIM9_TRG_COM_IRQHandler    /* TIM9 Trigger and Commutation */
    .weak   TIM9_CC_IRQHandler         /* TIM9 Capture Compare */
    .weak   TIM10_BRK_IRQHandler       /* TIM10 Break */
    .weak   TIM10_UP_IRQHandler        /* TIM10 Update */
    .weak   TIM10_TRG_COM_IRQHandler   /* TIM10 Trigger and Commutation */
    .weak   TIM10_CC_IRQHandler        /* TIM10 Capture Compare */
    .weak   DMA2_Channel6_IRQHandler   /* DMA2 Channel 6 */
    .weak   DMA2_Channel7_IRQHandler   /* DMA2 Channel 7 */
    .weak   DMA2_Channel8_IRQHandler   /* DMA2 Channel 8 */
    .weak   DMA2_Channel9_IRQHandler   /* DMA2 Channel 9 */
    .weak   DMA2_Channel10_IRQHandler  /* DMA2 Channel 10 */
    .weak   DMA2_Channel11_IRQHandler  /* DMA2 Channel 11 */

NMI_Handler:
HardFault_Handler:
Ecall_M_Mode_Handler:
Ecall_U_Mode_Handler:
Break_Point_Handler:
SysTick_Handler:
SW_Handler:
WWDG_IRQHandler:
PVD_IRQHandler:
TAMPER_IRQHandler:
RTC_IRQHandler:
FLASH_IRQHandler:
RCC_IRQHandler:
EXTI0_IRQHandler:
EXTI1_IRQHandler:
EXTI2_IRQHandler:
EXTI3_IRQHandler:
EXTI4_IRQHandler:
DMA1_Channel1_IRQHandler:
DMA1_Channel2_IRQHandler:
DMA1_Channel3_IRQHandler:
DMA1_Channel4_IRQHandler:
DMA1_Channel5_IRQHandler:
DMA1_Channel6_IRQHandler:
DMA1_Channel7_IRQHandler:
ADC1_2_IRQHandler:
USB_HP_CAN1_TX_IRQHandler:
USB_LP_CAN1_RX0_IRQHandler:
CAN1_RX1_IRQHandler:
CAN1_SCE_IRQHandler:
EXTI9_5_IRQHandler:
TIM1_BRK_IRQHandler:
TIM1_UP_IRQHandler:
TIM1_TRG_COM_IRQHandler:
TIM1_CC_IRQHandler:
TIM2_IRQHandler:
TIM3_IRQHandler:
TIM4_IRQHandler:
I2C1_EV_IRQHandler:
I2C1_ER_IRQHandler:
I2C2_EV_IRQHandler:
I2C2_ER_IRQHandler:
SPI1_IRQHandler:
SPI2_IRQHandler:
USART1_IRQHandler:
USART2_IRQHandler:
USART3_IRQHandler:
EXTI15_10_IRQHandler:
RTCAlarm_IRQHandler:
USBWakeUp_IRQHandler:
TIM8_BRK_IRQHandler:
TIM8_UP_IRQHandler:
TIM8_TRG_COM_IRQHandler:
TIM8_CC_IRQHandler:
RNG_IRQHandler:
SDIO_IRQHandler:
TIM5_IRQHandler:
SPI3_IRQHandler:
UART4_IRQHandler:
UART5_IRQHandler:
TIM6_IRQHandler:
TIM7_IRQHandler:
DMA2_Channel1_IRQHandler:
DMA2_Channel2_IRQHandler:
DMA2_Channel3_IRQHandler:
DMA2_Channel4_IRQHandler:
DMA2_Channel5_IRQHandler:
ETH_IRQHandler:
ETH_WKUP_IRQHandler:
CAN2_TX_IRQHandler:
CAN2_RX0_IRQHandler:
CAN2_RX1_IRQHandler:
CAN2_SCE_IRQHandler:
USBFS_IRQHandler:
USBHSWakeup_IRQHandler:
USBHS_IRQHandler:
DVP_IRQHandler:
UART6_IRQHandler:
UART7_IRQHandler:
UART8_IRQHandler:
TIM9_BRK_IRQHandler:
TIM9_UP_IRQHandler:
TIM9_TRG_COM_IRQHandler:
TIM9_CC_IRQHandler:
TIM10_BRK_IRQHandler:
TIM10_UP_IRQHandler:
TIM10_TRG_COM_IRQHandler:
TIM10_CC_IRQHandler:
DMA2_Channel6_IRQHandler:
DMA2_Channel7_IRQHandler:
DMA2_Channel8_IRQHandler:
DMA2_Channel9_IRQHandler:
DMA2_Channel10_IRQHandler:
DMA2_Channel11_IRQHandler:
1:
	j 1b

	.section	.text.handle_reset,"ax",@progbits
	.weak	handle_reset
	.align	1
handle_reset:
.option push 
.option	norelax 
	la gp, __global_pointer$
.option	pop 

	la sp, _eusrstack 

/* Load data section from flash to RAM */
	la a0, _data_lma
	la a1, _data_vma
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:
    /* Clear bss section */
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 2f
1:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, 1b
2:
    li t0, 0x1f
    csrw 0xbc0, t0

    #if defined(__PIO_BUILD_FREERTOS__) || defined(__PIO_BUILD_HARMONY_LITEOS__) || defined(__PIO_BUILD_RT_THREAD__) || defined(__PIO_BUILD_TENCENT_OS__)
    /* Enable nested and hardware stack */
    li t0, 0x1f
    csrw 0x804, t0

    /* Enable floating point and interrupt */
    /* Generate the value 0x7880, which are the MPIE, MPP and FS bits to set in mstatus. */
    li t0, 0x7800
    csrw mstatus, t0
    #else /* regular */
    li t0, 0x0b
    csrw 0x804, t0

    /* Enable floating point and interrupt */
    li t0, 0x6088           
    csrw mstatus, t0
    #endif

     la t0, _vector_base
    ori t0, t0, 3           
    csrw mtvec, t0

    #if defined(__PIO_CPP_SUPPORT__)
    /* register fini (destructor array) call at exit if wanted (bloats up RAM+Flash) */
    #if defined(__PIO_CPP_CALL_FINI__)
    la a0,__libc_fini_array
    call atexit
    #endif
    /* call into C++ constructors now */
    call __libc_init_array
    #endif

    jal  SystemInit
    #if defined(__PIO_BUILD_RT_THREAD__)
    /* RT-Thread has special entry point, it will call our main() later */
    la t0, entry
    #else
    la t0, main
    #endif
    csrw mepc, t0
    mret


