/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   USB host MSC class demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "dual_core_demo.h"
#include "stdio.h"
#include "usbh_ioreq.h"

#include "usb_bsp.h"
#include "usbh_hcs.h"
#include "usbh_stdreq.h"
#include "usbh_core.h"
#include "usb_hcd_int.h"

//#include "usbd_cdc_vcp.h"
/** @addtogroup USBH_USER
* @{
*/

/** @defgroup USBH_USR_MAIN
* @brief This file is the MSC demo main file
* @{
*/ 

/** @defgroup USBH_USR_MAIN_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBH_USR_MAIN_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_USR_MAIN_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_USR_MAIN_Private_Variables
* @{
*/

/**
* @}
*/ 


/** @defgroup USBH_USR_MAIN_Private_FunctionPrototypes
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_USR_MAIN_Private_Functions
* @{
*/ 

/**
* @brief  Main routine for MSC class application
* @param  None
* @retval int
*/
#if 1
//#define EVAL_COM1           USART1
//#define EVAL_COM1_IRQn      USART1_IRQn
    //#else
#define EVAL_COM4           UART4
#define EVAL_COM4_IRQn      UART4_IRQn
#endif
    //#define EVAL_COM_IRQHandler USART1_IRQHandler
    
    //#define  USART              USART1
    //#define  USART_IRQn         USART1_IRQn
    //#define  USART_IRQHandler   USART1_IRQHandler
    
#define UART_DEFAULT_BAUD   115200//921600
    
#define  UART_CLK           ((uint32_t)84000000)
    
#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))
USART_InitTypeDef USART_InitStructure;
extern USB_OTG_CORE_HANDLE          USB_OTG_FS_Core;
extern USB_OTG_CORE_HANDLE          USB_OTG_Core;

void STM_COMInit(USART_InitTypeDef* USART_InitStruct)
{
      GPIO_InitTypeDef GPIO_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;
      
    //  static int com_init_f = 0;
    //  if(com_init_f == 1) return;
    //  com_init_f = 1;
    
      /* Enable GPIO clock */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
    
      /* Enable UART clock */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
      /* Connect PXx to USARTx_Tx*/
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    
      /* Connect PXx to USARTx_Rx*/
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
      /* Configure USART Tx Rx as alternate function  */
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
    
      /* USART configuration */
      USART_Cmd(USART1, DISABLE);
    //  EVAL_COM1->CR1 = 0;
    
      USART_Init(USART1, USART_InitStruct);
      USART1->BRR = __USART_BRR(UART_CLK, USART_InitStruct->USART_BaudRate);
    
      /* Enable USART */
      USART_Cmd(USART1, ENABLE);
  #if 1//TOUCH_USB_AUTO_TEST//add by huangcaihui 180627
      {
      /* EVAL_COM4 configured as follow:
            - BaudRate = 57600 baud  
            - Word Length = 8 Bits
            - One Stop Bit
            - Parity Odd
            - Hardware flow control disabled
            - Receive and transmit enabled
      */
    
      /* Enable GPIO clock */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
    
      /* Enable UART clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    
      //串口4对应引脚复用映射
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UART4
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4
    
      /* Configure USART Tx Rx as alternate function  */
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOC, &GPIO_InitStructure);
    
      /* USART configuration */
      USART_Cmd(EVAL_COM4, DISABLE);
    //  EVAL_COM1->CR1 = 0;
    
      USART_Init(EVAL_COM4, USART_InitStruct);
      EVAL_COM4->BRR = __USART_BRR(UART_CLK, USART_InitStruct->USART_BaudRate);
    
      /* Enable USART */
      USART_Cmd(EVAL_COM4, ENABLE);

      USART_ITConfig(EVAL_COM4, USART_IT_RXNE, ENABLE);

      //UART4 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
        }
  #endif
    }


void COMInit(void)
{
    USART_InitStructure.USART_BaudRate = UART_DEFAULT_BAUD;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;//USART_Parity_Odd;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure and enable the USART */
  STM_COMInit(&USART_InitStructure);
}


int main(void)
{
  __IO uint32_t i = 0;
  int k,stop;
  
#if 0
  uint8_t buf_fs[4]; 
  uint8_t buf_hs[4]; 
  buf_fs[0]=0;  
  buf_fs[1]=7;  
  buf_fs[2]=7;  
  buf_fs[3]=0;  
   
  buf_hs[0]=0;  
  buf_hs[1]=8;  
  buf_hs[2]=8;  
  buf_hs[3]=0;  
#endif
  stop = 0;
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32fxxx_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32fxxx.c file
  */  
  //APP_VCP_FOPS.pIf_Init();
  COMInit();
  Demo_Init();

  while (1)
  {

    Demo_Process();
    
    if (i++ == 0x10000)
    {
        #if 1
        
        if(stop == 0)
        {
            for(k=0;k<10;k++)
            {
                
#if 0//add by huangcaihui 180621
                    //SysTick_Handler();//add by huangcaihui 180620 for test
                    //printf("11111111111111111111 \ n");
                    USBH_InterruptSendData (&USB_OTG_FS_Core,buf_fs,4,1);  
                    USB_OTG_BSP_mDelay(500);

                    //USBH_InterruptSendData (&USB_OTG_Core,buf_hs,4,1);  
                   
                    //USB_OTG_BSP_mDelay(500);
#endif
                printf(" is host 22222222222222222 \n");
                //USART_SendData(UART4, data);
                //USART_SendData(USART1, data);
                //data++;
                //while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);
            }
            stop = 1;
        }
        #endif
        
      
      i = 0;
    }      
  }
}

#if 1
/*
 * serial port print callback
 */
 void EVAL_COM_TX_BYTE(char c)
{
    #if 1//TOUCH_USB_AUTO_TEST//add by huangcaihui 180627
        while(USART_GetFlagStatus(EVAL_COM4, USART_FLAG_TXE) == RESET);
        USART_SendData(EVAL_COM4, c );
    //#else
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, c );
    #endif
}
int fputc(int ch, FILE *f)
{
    extern void EVAL_COM_TX_BYTE(char c);
    //UART_WriteData((uint8_t*)&ch,1);
//    APP_FOPS.pIf_DataRx((uint8_t*)&ch,1);
    EVAL_COM_TX_BYTE(ch);
    return ch;
}
#endif

#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif


/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
