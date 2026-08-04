/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32v205.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/05/20
 * Description        : CH32V205 Device Peripheral Access Layer System Source File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v205.h"

/* 
* Uncomment the line corresponding to the desired System clock (SYSCLK) frequency (after 
* reset the HSI is used as SYSCLK source).
* If none of the define below is enabled, the HSI is used as System clock source. 
*/
// #define SYSCLK_FREQ_HSE    HSE_VALUE
// #define SYSCLK_FREQ_192MHz_HSE  192000000
// #define SYSCLK_FREQ_160MHz_HSE  160000000
// #define SYSCLK_FREQ_120MHz_HSE  120000000
// #define SYSCLK_FREQ_80MHz_HSE  80000000
// #define SYSCLK_FREQ_40MHz_HSE  40000000
// #define SYSCLK_FREQ_HSI    HSI_VALUE
// #define SYSCLK_FREQ_192MHz_HSI  192000000
// #define SYSCLK_FREQ_160MHz_HSI  160000000
// #define SYSCLK_FREQ_120MHz_HSI  120000000
// #define SYSCLK_FREQ_80MHz_HSI  80000000
// #define SYSCLK_FREQ_40MHz_HSI  40000000
// #define SYSCLK_FREQ_HSI_LP   HSI_LP_VALUE                      /* Baud rate support less than 62.5Kbps when using UART */

/* Clock Definitions */
#ifdef SYSCLK_FREQ_HSE
uint32_t SystemCoreClock         = SYSCLK_FREQ_HSE;              /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_192MHz_HSE
uint32_t SystemCoreClock         = SYSCLK_FREQ_192MHz_HSE;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_160MHz_HSE
uint32_t SystemCoreClock         = SYSCLK_FREQ_160MHz_HSE;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_120MHz_HSE
uint32_t SystemCoreClock         = SYSCLK_FREQ_120MHz_HSE;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_80MHz_HSE
uint32_t SystemCoreClock         = SYSCLK_FREQ_80MHz_HSE;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_40MHz_HSE
uint32_t SystemCoreClock         = SYSCLK_FREQ_40MHz_HSE;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_192MHz_HSI
uint32_t SystemCoreClock         = SYSCLK_FREQ_192MHz_HSI;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_160MHz_HSI
uint32_t SystemCoreClock         = SYSCLK_FREQ_160MHz_HSI;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_120MHz_HSI
uint32_t SystemCoreClock         = SYSCLK_FREQ_120MHz_HSI;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_80MHz_HSI
uint32_t SystemCoreClock         = SYSCLK_FREQ_80MHz_HSI;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_40MHz_HSI
uint32_t SystemCoreClock         = SYSCLK_FREQ_40MHz_HSI;        /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_HSI_LP
uint32_t SystemCoreClock         = SYSCLK_FREQ_HSI_LP;           /* System Clock Frequency (Core Clock) */
#else
uint32_t SystemCoreClock         = HSI_VALUE;                    /* System Clock Frequency (Core Clock) */

#endif

__I uint8_t HBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};


/* system_private_function_proto_types */
static void SetSysClock(void);

#ifdef SYSCLK_FREQ_HSE
static void SetSysClockToHSE( void );
#elif defined SYSCLK_FREQ_192MHz_HSE
static void SetSysClockTo192_HSE( void );
#elif defined SYSCLK_FREQ_160MHz_HSE
static void SetSysClockTo160_HSE( void );
#elif defined SYSCLK_FREQ_120MHz_HSE
static void SetSysClockTo120_HSE( void );
#elif defined SYSCLK_FREQ_80MHz_HSE
static void SetSysClockTo80_HSE( void );
#elif defined SYSCLK_FREQ_40MHz_HSE
static void SetSysClockTo40_HSE( void );
#elif defined SYSCLK_FREQ_192MHz_HSI
static void SetSysClockTo192_HSI( void );
#elif defined SYSCLK_FREQ_160MHz_HSI
static void SetSysClockTo160_HSI( void );
#elif defined SYSCLK_FREQ_120MHz_HSI
static void SetSysClockTo120_HSI( void );
#elif defined SYSCLK_FREQ_80MHz_HSI
static void SetSysClockTo80_HSI( void );
#elif defined SYSCLK_FREQ_40MHz_HSI
static void SetSysClockTo40_HSI( void );
#elif defined SYSCLK_FREQ_HSI_LP
static void SetSysClockToHSI_LP( void );

#endif

/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   Setup the microcontroller system Initialize the Embedded Flash Interface,
 *        the PLL and update the SystemCoreClock variable.
 *
 * @return  none
 */
void SystemInit (void)
{
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_4 | FLASH_PEEN);
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0x08FF0000;
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x00)
  {
  }
  RCC->CTLR &= (uint32_t)0xFEF6FFFB;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFF00FFFF;
  RCC->INTR = 0x009F0000;
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_0 | FLASH_PEEN);
  SetSysClock();
}

/*********************************************************************
 * @fn      SystemCoreClockUpdate
 *
 * @brief   Update SystemCoreClock variable according to Clock Register Values.
 *
 * @return  none
 */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0;

  tmp = RCC->CFGR0 & RCC_SWS;

  switch (tmp)
  {
    case 0x00:
        if(RCC->CTLR & (1<<2))
        {
            SystemCoreClock = HSI_LP_VALUE;
        }
        else 
        {
            SystemCoreClock = HSI_VALUE;
        }
      break;
    case 0x04:
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:
      pllmull = RCC->CFGR0 & RCC_PLLMULL;
      pllsource = RCC->CFGR0 & RCC_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;

      if (pllsource == 0x00)
      {
          if(EXTEN->CTLR0 & EXTEN_PLL_HSI_PRE){
              SystemCoreClock = HSI_VALUE * pllmull;
          }
          else{
              SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
          }
      }
      else
      {
          if ((RCC->CFGR0 & RCC_PLLXTPRE) != (uint32_t)RESET)
          {
              SystemCoreClock = (HSE_VALUE >> 1) * pllmull;
          }
          else
          {
              SystemCoreClock = HSE_VALUE * pllmull;
          }
      }

      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }

  tmp = HBPrescTable[((RCC->CFGR0 & RCC_HPRE) >> 4)];
  SystemCoreClock >>= tmp;
}

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClock(void)
{
  GPIO_IPD_Unused();

#ifdef SYSCLK_FREQ_HSE
    SetSysClockToHSE();
#elif defined SYSCLK_FREQ_192MHz_HSE
    SetSysClockTo192_HSE();
#elif defined SYSCLK_FREQ_160MHz_HSE
    SetSysClockTo160_HSE();
#elif defined SYSCLK_FREQ_120MHz_HSE
    SetSysClockTo120_HSE();
#elif defined SYSCLK_FREQ_80MHz_HSE
    SetSysClockTo80_HSE();
#elif defined SYSCLK_FREQ_40MHz_HSE
    SetSysClockTo40_HSE();
#elif defined SYSCLK_FREQ_192MHz_HSI
    SetSysClockTo192_HSI();
#elif defined SYSCLK_FREQ_160MHz_HSI
    SetSysClockTo160_HSI();
#elif defined SYSCLK_FREQ_120MHz_HSI
    SetSysClockTo120_HSI();
#elif defined SYSCLK_FREQ_80MHz_HSI
    SetSysClockTo80_HSI();
#elif defined SYSCLK_FREQ_40MHz_HSI
    SetSysClockTo40_HSI();
#elif defined SYSCLK_FREQ_HSI_LP
    SetSysClockToHSI_LP();

#endif
 
 /* If none of the define above is enabled, the HSI is used as System clock
  * source (default after reset) 
  */
}


#ifdef SYSCLK_FREQ_HSE

/*********************************************************************
 * @fn      SetSysClockToHSE
 *
 * @brief   Sets HSE as System clock source and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockToHSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* Flash 0 wait state */
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_0 | FLASH_PEEN);
   
  RCC->CTLR |= ((uint32_t)RCC_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;      
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;
    
    /* Select HSE as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_HSE;    

    /* Wait till HSE is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x04)
    {
    }
  }
  else
  { 
    /* If HSE fails to start-up, the application will have wrong clock
     * configuration. User can add here some code to deal with this error 
     */
  }  
}

#elif defined SYSCLK_FREQ_192MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo192_HSE
 *
 * @brief   Sets System clock frequency to 192MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo192_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Flash 4 wait state */
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_4 | FLASH_PEEN);

  RCC->CTLR |= ((uint32_t)RCC_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSE * 24 = 192 MHz (HSE=8MHZ) */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL24);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  {
    /* If HSE fails to start-up, the application will have wrong clock
    * configuration. User can add here some code to deal with this error
    */
  }
}

#elif defined SYSCLK_FREQ_160MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo160_HSE
 *
 * @brief   Sets System clock frequency to 160MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo160_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Flash 3 wait state */
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_3 | FLASH_PEEN);

  RCC->CTLR |= ((uint32_t)RCC_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSE * 20 = 160 MHz (HSE=8MHZ) */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL20);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  {

    /* If HSE fails to start-up, the application will have wrong clock
    * configuration. User can add here some code to deal with this error
    */
  }
}

#elif defined SYSCLK_FREQ_120MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo120_HSE
 *
 * @brief   Sets System clock frequency to 120MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo120_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Flash 2 wait state */
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_2 | FLASH_PEEN);

  RCC->CTLR |= ((uint32_t)RCC_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSE * 15 = 120 MHz (HSE=8MHZ) */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL15);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  {

    /* If HSE fails to start-up, the application will have wrong clock
    * configuration. User can add here some code to deal with this error
    */
  }
}

#elif defined SYSCLK_FREQ_80MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo80_HSE
 *
 * @brief   Sets System clock frequency to 80MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo80_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Flash 1 wait state */
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_1 | FLASH_PEEN);

  RCC->CTLR |= ((uint32_t)RCC_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSE * 10 = 80 MHz (HSE=8MHZ) */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL10);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  {

    /* If HSE fails to start-up, the application will have wrong clock
    * configuration. User can add here some code to deal with this error
    */
  }
}

#elif defined SYSCLK_FREQ_40MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo40_HSE
 *
 * @brief   Sets System clock frequency to 40MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo40_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Flash 0 wait state */
  FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_0 | FLASH_PEEN);

  RCC->CTLR |= ((uint32_t)RCC_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSE * 5 = 40 MHz (HSE=8MHZ) */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | RCC_PLLMULL5);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  {

    /* If HSE fails to start-up, the application will have wrong clock
    * configuration. User can add here some code to deal with this error
    */
  }
}

#elif defined SYSCLK_FREQ_192MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo192_HSI
 *
 * @brief   Sets System clock frequency to 192MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo192_HSI(void)
{
    /* Flash 4 wait state */
    FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_4 | FLASH_PEEN);

    EXTEN->CTLR0 |= EXTEN_PLL_HSI_PRE;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSI * 24 = 192 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL24);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_FREQ_160MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo160_HSI
 *
 * @brief   Sets System clock frequency to 160MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo160_HSI(void)
{
    /* Flash 4 wait state */
    FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_3 | FLASH_PEEN);

    EXTEN->CTLR0 |= EXTEN_PLL_HSI_PRE;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSI * 20 = 160 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL20);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_FREQ_120MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo120_HSI
 *
 * @brief   Sets System clock frequency to 120MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo120_HSI(void)
{
    /* Flash 2 wait state */
    FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_2 | FLASH_PEEN);

    EXTEN->CTLR0 |= EXTEN_PLL_HSI_PRE;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSI * 15 = 120 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL15);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_FREQ_80MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo80_HSI
 *
 * @brief   Sets System clock frequency to 80MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo80_HSI(void)
{
    /* Flash 1 wait state */
    FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_1 | FLASH_PEEN);

    EXTEN->CTLR0 |= EXTEN_PLL_HSI_PRE;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSI * 10 = 80 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL10);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_FREQ_40MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo40_HSI
 *
 * @brief   Sets System clock frequency to 40MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo40_HSI(void)
{
    /* Flash 0 wait state */
    FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_0 | FLASH_PEEN);

    EXTEN->CTLR0 |= EXTEN_PLL_HSI_PRE;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: PLLCLK = HSI * 5 = 40 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL5);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_FREQ_HSI_LP

/*********************************************************************
 * @fn      SetSysClockToHSI_LP
 *
 * @brief   Sets HSI as System clock source of Low power and configure HCLK, PCLK2 and PCLK1 prescalers.
 *          In this mode, HSI is 1MHz and Baud rate support less than 62.5Kbps when using UART.
 *
 * @return  none
 */
static void SetSysClockToHSI_LP(void)
{
    /* Flash 0 wait state */
    FLASH->ACTLR = (uint32_t)(FLASH_ACTLR_LATENCY_0 | FLASH_PEEN);

    RCC_HSI_LP_Cmd(ENABLE);

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;
    /* Low power mode HSI  */
    RCC_AdjustHSICalibrationValue((*(uint8_t*)HSI_LP_TRIM_BASE)&0x1F);
}

#endif
