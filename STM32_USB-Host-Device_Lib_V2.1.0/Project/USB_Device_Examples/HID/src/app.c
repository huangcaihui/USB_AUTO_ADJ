/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the Application firmware functions.
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
#include  "usbd_hid_core.h"
#include  "usbd_usr.h"
#include  "usbd_desc.h"
#include  "stdio.h"
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup APP_HID 
  * @brief Mass storage application module
  * @{
  */ 

/** @defgroup APP_HID_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Defines
  * @{
  */ 


/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Variables
  * @{
  */ 
  
    static void USART_Config(void);
    
#ifdef __GNUC__
      /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
         set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/**
  * @}
  */ 


/** @defgroup APP_HID_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Functions
  * @{
  */ 

/**
  * @brief  Program entry point
  * @param  None
  * @retval None
  */
#if 0
 int fputc(int ch, FILE *f)
 {   
     while((UART4->SR&0X40)==0);//循环发送,直到发送完毕   
     UART4->DR = (u8) ch;      
     return ch;
 }
#endif

#if  0
 int fputc(int ch, FILE *f)
 {
     /* Place your implementation of fputc here */
     USART_SendData(UART4, (uint8_t) ch);
     /* Loop until the end of transmission */
     while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
     {}
     return ch;
 }

//#else
int fputc(int ch, FILE *f)
{   
        while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
        USART_SendData(UART4, (uint8_t) ch);
        while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
        return ch;
}


int fgetc(FILE *f) 
{
    int ch;
    while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
    ch = USART_ReceiveData(UART4);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
    USART_SendData(UART4, (uint8_t) ch);
    return ch;
}
#endif

#if 0
u8 USART_RX_BUF[100];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

void UART4_IRQHandler(void)                	//串口4中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(UART4);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(100-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 

#else
u8 USART_RX_BUF[100];     //接收缓冲,最大64个字节.
u8 uc = 0;
u8 USART_RX_STA=0;       //接收状态标记

void UART4_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断()
	{
	    USART_ClearITPendingBit(UART4,USART_IT_RXNE);	   //清除中断接受标致
	   	Res =USART_ReceiveData(UART4);//(USART1->DR);	//读取接收到的数据
	 	USART_RX_BUF[USART_RX_STA&0XFF] = Res;
		USART_RX_STA++;		 
    }	 
    if(USART_RX_BUF[0] == 0x01)
    {
        GPIO_SetBits(GPIOC,GPIO_Pin_3);
        //USB_OTG_BSP_uDelay(500 * 1000);
    }
    else if((USART_RX_BUF[0] == 0x02))
    {
        GPIO_ResetBits(GPIOC,GPIO_Pin_3);
        //USB_OTG_BSP_uDelay(500 * 1000);
    }    
} 

#endif
//GPIO_TypeDef* io_led=GPIOC;//定义一个指向结构体<span style="font-family:Arial, Helvetica, sans-serif;">GPIO_TypeDef</span><span style="font-family:Arial, Helvetica, sans-serif;">的io_led  </span>  
//const u16 pin_led=GPIO_Pin_3;//引脚  
void Delay(u32 time)  
{  
    u32 t=time;  
    while(t--);  
}  

void Led_Init()  
{  
    GPIO_InitTypeDef GPIO_init_l;//用于初始化的结构体  
      
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   //时钟  
    GPIO_init_l.GPIO_Pin=GPIO_Pin_3;  
    GPIO_init_l.GPIO_Mode=GPIO_Mode_OUT;  
    GPIO_init_l.GPIO_OType=GPIO_OType_PP;  
    GPIO_init_l.GPIO_Speed=GPIO_Speed_100MHz;  
    GPIO_init_l.GPIO_PuPd=GPIO_PuPd_NOPULL;    
    GPIO_Init(GPIOC,&GPIO_init_l);  
    //GPIOC->BSRRL=GPIO_Pin_3;
    GPIO_ResetBits(GPIOC,GPIO_Pin_3);
    
}  

int main(void)
{
  __IO uint32_t i = 0;
  int k,data;
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32fxxx_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32fxxx.c file
  */  
  
  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc, 
            &USBD_HID_cb, 
            &USR_cb);
  Led_Init(); 
  
  USART_Config();
  #if 1
  while (1)
  {
          
    if (i++ == 0x100000)
    {
        data = 0x22;
        for(k=0;k<30;k++)
        {
            USART_SendData(UART4, data);
            data++;
            while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);
        }
        //printf("yuanbao");
        //printf("hch 1111111111111111111111 \n");
        //io_led->BSRRL=pin_led;
        //Delay(5000000);  
        //io_led->BSRRH=pin_led;
        //Delay(5000000);  
#if 1//add by huangcaihui 180621
        //SysTick_Handler();//add by huangcaihui 180620 for test
        uint8_t buf[4];  
        buf[0]=0;  
        buf[1]=7;  
        buf[2]=7;  
        buf[3]=0;  
        USBD_HID_SendReport (&USB_OTG_dev,   
                                                 buf,  
                                                 4);  
#endif
#if 0
      STM_EVAL_LEDToggle(LED1);
      STM_EVAL_LEDToggle(LED2);
      STM_EVAL_LEDToggle(LED3);
      STM_EVAL_LEDToggle(LED4);
      i = 0;
#endif
    }
  }
  #endif
} 

static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  STM_EVAL_COMInit(COM1, &USART_InitStructure);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

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
