;/********************************** (C) COPYRIGHT *******************************
;* File Name          : startup_ch32v205.s
;* Author             : WCH
;* Version            : V1.0.1
;* Date               : 2025/10/21
;* Description        : Startup File 
;*********************************************************************************
;* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
;* Attention: This software (modified or not) and binary are used for 
;* microcontroller manufactured by Nanjing Qinheng Microelectronics.
;*******************************************************************************/

	.section	.init,"ax",@progbits
	.global	_start
	.align	2
_start:

    .option norvc;
    j       handle_reset
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
    .word   ADC_IRQHandler             /* ADC */
    .word   USB_HP_CAN1_TX_IRQHandler  /* USB HP and CAN1 TX */
    .word   USB_LP_CAN1_RX0_IRQHandler /* USB LP and CAN1RX0 */
    .word   CAN1_RX1_IRQHandler        /* CAN RX1*/
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
    .word   QSPI1_IRQHandler           /* QSPI1 */
    .word   USBFS_IRQHandler           /* USBFS Break */
    .word   USBFSWakeUp_IRQHandler     /* USBFS Wake up from suspend */
    .word   USART4_IRQHandler          /* USART4 */
    .word   DMA1_Channel8_IRQHandler   /* DMA1 Channel8 */
    .word   PIOC_IRQHandler            /* PIOC */
    .word   OPA_IRQHandler             /* OPA */
    .word   USBPD_IRQHandler           /* USBPD */
    .word   0
    .word   USBPDWakeUp_IRQHandler     /* USBPD Wake Up */
    .word   CMPWakeUp_IRQHandler       /* CMP Wake Up */
    .word   USBHS_IRQHandler           /* USBHS */
    .word   LVDS_IRQHandler            /* LVDS */
    .word   USBHSWakeup_IRQHandler     /* USBHS Wake Up */   
    .word   LVDSWakeup_IRQHandler      /* LVDS Wake Up */  
    .word   USART5_IRQHandler          /* USART5 */            
    .word   USART6_IRQHandler          /* USART64 */
    .word   USART7_IRQHandler          /* USART7 */      
    .word   USART8_IRQHandler          /* USART8 */
    .word   DMA2_Channel1_IRQHandler   /* DMA2 Channel 1 */      
    .word   DMA2_Channel2_IRQHandler   /* DMA2 Channel 2 */               
    .word   DMA2_Channel3_IRQHandler   /* DMA2 Channel 3 */               
    .word   DMA2_Channel4_IRQHandler   /* DMA2 Channel 4 */               
    .word   DMA2_Channel5_IRQHandler   /* DMA2 Channel 5 */              
    .word   DMA2_Channel6_IRQHandler   /* DMA2 Channel 6 */             
    .word   DMA2_Channel7_IRQHandler   /* DMA2 Channel 7 */              
    .word   DMA2_Channel8_IRQHandler   /* DMA2 Channel 8 */           

    .option rvc;
    .section    .text.vector_handler, "ax", @progbits
    .weak   NMI_Handler                
    .weak   HardFault_Handler          
    .weak   Ecall_M_Mode_Handler       
    .weak   Ecall_U_Mode_Handler       
    .weak   Break_Point_Handler        
    .weak   SysTick_Handler           
    .weak   SW_Handler                
    .weak   WWDG_IRQHandler           
    .weak   PVD_IRQHandler             
    .weak   TAMPER_IRQHandler        
    .weak   RTC_IRQHandler            
    .weak   FLASH_IRQHandler           
    .weak   RCC_IRQHandler             
    .weak   EXTI0_IRQHandler          
    .weak   EXTI1_IRQHandler           
    .weak   EXTI2_IRQHandler          
    .weak   EXTI3_IRQHandler          
    .weak   EXTI4_IRQHandler           
    .weak   DMA1_Channel1_IRQHandler   
    .weak   DMA1_Channel2_IRQHandler   
    .weak   DMA1_Channel3_IRQHandler   
    .weak   DMA1_Channel4_IRQHandler   
    .weak   DMA1_Channel5_IRQHandler   
    .weak   DMA1_Channel6_IRQHandler   
    .weak   DMA1_Channel7_IRQHandler  
    .weak   ADC_IRQHandler            
    .weak   USB_HP_CAN1_TX_IRQHandler  
    .weak   USB_LP_CAN1_RX0_IRQHandler 
    .weak   CAN1_RX1_IRQHandler
    .weak   CAN1_SCE_IRQHandler       
    .weak   EXTI9_5_IRQHandler         
    .weak   TIM1_BRK_IRQHandler        
    .weak   TIM1_UP_IRQHandler         
    .weak   TIM1_TRG_COM_IRQHandler    
    .weak   TIM1_CC_IRQHandler        
    .weak   TIM2_IRQHandler          
    .weak   TIM3_IRQHandler          
    .weak   TIM4_IRQHandler           
    .weak   I2C1_EV_IRQHandler         
    .weak   I2C1_ER_IRQHandler        
    .weak   I2C2_EV_IRQHandler         
    .weak   I2C2_ER_IRQHandler        
    .weak   SPI1_IRQHandler           
    .weak   SPI2_IRQHandler          
    .weak   USART1_IRQHandler         
    .weak   USART2_IRQHandler         
    .weak   USART3_IRQHandler         
    .weak   EXTI15_10_IRQHandler       
    .weak   RTCAlarm_IRQHandler   
    .weak   QSPI1_IRQHandler    
    .weak   USBFS_IRQHandler          
    .weak   USBFSWakeUp_IRQHandler     
    .weak   USART4_IRQHandler          
    .weak   DMA1_Channel8_IRQHandler  
    .weak   PIOC_IRQHandler                
    .weak   OPA_IRQHandler           
    .weak   USBPD_IRQHandler          
    .weak   USBPDWakeUp_IRQHandler     
    .weak   CMPWakeUp_IRQHandler    
    .weak   USBHS_IRQHandler   
    .weak   LVDS_IRQHandler  
    .weak   USBHSWakeup_IRQHandler 
    .weak   LVDSWakeup_IRQHandler
    .weak   USART5_IRQHandler               
    .weak   USART6_IRQHandler  
    .weak   USART7_IRQHandler               
    .weak   USART8_IRQHandler  
    .weak   DMA2_Channel1_IRQHandler         
    .weak   DMA2_Channel2_IRQHandler         
    .weak   DMA2_Channel3_IRQHandler         
    .weak   DMA2_Channel4_IRQHandler         
    .weak   DMA2_Channel5_IRQHandler        
    .weak   DMA2_Channel6_IRQHandler       
    .weak   DMA2_Channel7_IRQHandler        
    .weak   DMA2_Channel8_IRQHandler

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
ADC_IRQHandler:
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
QSPI1_IRQHandler:
USBFS_IRQHandler:
USBFSWakeUp_IRQHandler:
USART4_IRQHandler:
DMA1_Channel8_IRQHandler:
PIOC_IRQHandler: 
OPA_IRQHandler:
USBPD_IRQHandler:
USBPDWakeUp_IRQHandler:
CMPWakeUp_IRQHandler:
USBHS_IRQHandler:  
LVDS_IRQHandler:
USBHSWakeup_IRQHandler:  
LVDSWakeup_IRQHandler:  
USART5_IRQHandler:               
USART6_IRQHandler:   
USART7_IRQHandler:               
USART8_IRQHandler:  
DMA2_Channel1_IRQHandler:         
DMA2_Channel2_IRQHandler:         
DMA2_Channel3_IRQHandler:         
DMA2_Channel4_IRQHandler:         
DMA2_Channel5_IRQHandler:        
DMA2_Channel6_IRQHandler:       
DMA2_Channel7_IRQHandler:        
DMA2_Channel8_IRQHandler: 
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
/* Enabale MIE mapping and prefetch */
    li t0, 0x21
    csrw 0xbc0, t0
/* Enable interrupt nesting and hardware stack */
	li t0, 0x7
	csrw 0x804, t0
	li t0, 0x1
	csrw 0xbc1, t0
/* Enable global interrupt and configure privileged mode */
  	li t0, 0x88
  	csrw mstatus, t0
/* Configure entry address mode */
	la t0, _start
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
	la t0, main
	csrw mepc, t0
	mret


