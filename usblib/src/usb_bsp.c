/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   This file is responsible to offer board support package and is 
  *          configurable by user.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS. 
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx_exti.h"

#include "usb_bsp.h"
#include "usbd_conf.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "usbd_cdc_core.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/

/** @defgroup USB_BSP
* @brief This file is responsible to offer board support package
* @{
*/ 

/** @defgroup USB_BSP_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_BSP_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 





/** @defgroup USB_BSP_Private_Macros
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBH_BSP_Private_Variables
* @{
*/ 

/**
* @}
*/ 

/** @defgroup USBH_BSP_Private_FunctionPrototypes
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USB_BSP_Private_Functions
* @{
*/ 


/**
* @brief  USB_OTG_BSP_Init
*         Initilizes BSP configurations
* @param  None
* @retval None
*/

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
#ifdef USE_STM3210C_EVAL

  RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE) ;

#else // USE_STM32F4_DISCOVERY
  GPIO_InitTypeDef GPIO_InitStructure;   
 #ifdef USE_USB_OTG_FS 
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);  
  
  /* Configure SOF VBUS ID DM DP Pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8  | 
    GPIO_Pin_9  | 
      GPIO_Pin_11 | 
        GPIO_Pin_12;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_OTG_FS) ;
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_OTG_FS) ;
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_OTG_FS) ;
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG_FS) ;
  
  /* this for ID line debug */

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_OTG_FS) ;
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;

 #else // USE_USB_OTG_HS 

  #ifdef USE_ULPI_PHY // ULPI
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | 
                         RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOH | 
                           RCC_AHB1Periph_GPIOI, ENABLE);    
  
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_OTG2_HS) ; // D0
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5, GPIO_AF_OTG2_HS) ; // CLK
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource0, GPIO_AF_OTG2_HS) ; // D1
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource1, GPIO_AF_OTG2_HS) ; // D2
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_OTG2_HS) ; // D7
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_OTG2_HS) ; // D3
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_OTG2_HS) ; // D4
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_OTG2_HS) ; // D5
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_OTG2_HS) ; // D6
  GPIO_PinAFConfig(GPIOH,GPIO_PinSource4, GPIO_AF_OTG2_HS) ; // NXT
  GPIO_PinAFConfig(GPIOI,GPIO_PinSource11,GPIO_AF_OTG2_HS) ; // DIR
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource0, GPIO_AF_OTG2_HS) ; // STP
  
  // CLK
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  // D0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3  ; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  
  
  // D1 D2 D3 D4 D5 D6 D7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  |
    GPIO_Pin_5 | GPIO_Pin_10 | 
      GPIO_Pin_11| GPIO_Pin_12 | 
        GPIO_Pin_13 ;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  
  // STP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  
  //NXT  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOH, &GPIO_InitStructure);  
  
  
  //DIR
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOI, &GPIO_InitStructure);  
  
  
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_OTG_HS | 
                         RCC_AHB1Periph_OTG_HS_ULPI, ENABLE) ;    
   
  #else
  
   #ifdef USE_I2C_PHY    
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE);  
  /* Configure RESET INTN SCL SDA (Phy/I2C) Pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | 
    GPIO_Pin_1 | 
      GPIO_Pin_10 | 
        GPIO_Pin_11;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_OTG2_FS) ; 
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_OTG2_FS) ; 
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_OTG2_FS) ; 
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_OTG2_FS);
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_OTG_HS, ENABLE) ;  
  
   #else

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12  | 
    GPIO_Pin_13 |
      GPIO_Pin_14 | 
        GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource12, GPIO_AF_OTG2_FS) ; 
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_OTG2_FS) ; 
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_OTG2_FS) ; 
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_OTG2_FS) ;
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_OTG_HS, ENABLE) ;  
   #endif    
  #endif
 #endif //USB_OTG_HS
#endif //USE_STM322xG_EVAL


  /* enable the PWR clock */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);


#ifdef USB_OTG_FS_LOW_PWR_MGMT_SUPPORT
  /* Configure the Key button in EXTI mode */
  STM32F4_Discovery_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  EXTI_ClearITPendingBit(EXTI_Line18);

  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line18);

  NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line18);
#endif

#ifdef USB_OTG_HS_LOW_PWR_MGMT_SUPPORT
  /* Configure the Key button in EXTI mode */
  STM32F4_Discovery_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  EXTI_ClearITPendingBit(EXTI_Line20);

  EXTI_InitStructure.EXTI_Line = EXTI_Line20;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line20);

  NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line20);
#endif

//  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);


}
/**
* @brief  USB_OTG_BSP_EnableInterrupt
*         Enable USB Global interrupt
* @param  None
* @retval None
*/
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
#ifdef USE_USB_OTG_HS   
  NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_IRQn;
#else
  NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;  
#endif
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_EP1_OUT_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_EP1_IN_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
#endif
}
/**
* @brief  USB_OTG_BSP_uDelay
*         This function provides delay time in micro sec
* @param  usec : Value of delay required in micro sec
* @retval None
*/
void USB_OTG_BSP_uDelay (const uint32_t usec)
{
  uint32_t count = 0;
  const uint32_t utime = (120 * usec / 7);
  do
  {
    if ( ++count > utime )
    {
      return ;
    }
  }
  while (1);
}


/**
* @brief  USB_OTG_BSP_mDelay
*          This function provides delay time in milli sec
* @param  msec : Value of delay required in milli sec
* @retval None
*/
void USB_OTG_BSP_mDelay (const uint32_t msec)
{
  USB_OTG_BSP_uDelay(msec * 1000);   
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern void DISCOVERY_EXTI_IRQHandler(void);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif


/**
  * @brief  This function handles OTG_FS_WKUP_IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_FS
void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

/**
  * @brief  This function handles OTG_HS_WKUP_IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_HS
void OTG_HS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

/**
  * @brief  This function handles OTG_xx_IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_HS
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
/**
  * @brief  This function handles EP1_IN Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

/**
  * @brief  This function handles EP1_OUT Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/


/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */


/*� ������� ������ ������� ��������������� �������� ����� ����������, ��� ���
�������� ����� ���������� ���� � ����� main.c

void EXTI0_IRQHandler(void)
{
	//DISCOVERY_EXTI_IRQHandler();
	//Clear the EXTI line pending bit
//	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);

}
*/

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/