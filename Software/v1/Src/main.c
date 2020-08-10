/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "NEXTION.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//-------------------------------------------------  USB var -------------------
char RxBuffer[12];
uint8_t counter = 0, Buffer_full = 0;

int number;
uint8_t cnt, dim = 0;
uint8_t Val_buf[6];
uint16_t Bit_value = 0;
float Dec_number = 0;
//------------------------------------------------ main variables --------------
uint8_t DataRX[8],prevButtState=0,lastButtonState ;

uint8_t lastOnOFFState, prevOnOFFState; 
uint16_t Counter1 = 0, Measure, Counter2, Mode,  Check_USB;

unsigned char string[20], string2[20], string3[20], string4[20], string5[20] ;

uint8_t  Rx_data[1], Rx_indx, data,  SendProgBarVal, undim = 0, counter3 = 0 ;

uint16_t tmp=0,Value=3740,Value2=0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Byte_process(uint8_t data)
{
  if(data == 0x65) // if buttons pushed
  { 
    DataRX[0] = data;
    Rx_indx=0;
  }
  else if(data == 0x71) // if slider moved
  { 
    DataRX[0] = data;
    Rx_indx=0;
  }
  else if(data == 0x1A) // ignore TSD response
  {
    Rx_indx=0;
  }
   else if(data == 0x68) // ignore TSD response
  {
    Rx_indx=0;
  }
    DataRX[Rx_indx ++] = data;  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // UART RX complete interupt callback
{
  if (huart->Instance == USART2){ // if UART2 	
  data = Rx_data[0]; // collect data
  HAL_UART_Receive_IT(&huart2, Rx_data, 1); // reinicialize Receive_IT
   __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // enable ISR when buffer not empty
  Byte_process(data); // process received data
  }
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); //Shut down SMPS
  HAL_Delay(1000);
  MCP4725_Write(3740);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); //Shut down SMPS
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(1000);
  dim = 0;
  undim = 1;
  HAL_UART_Init(&huart2); 
  HAL_Delay(100); 
  HAL_UART_Receive_IT(&huart2, Rx_data, 1);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // interupt (buffer not empty)
  HAL_Delay(100); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    if (Check_USB == 1)
  {
     if (hUsbDeviceFS.dev_state == 0x03) // if USB is connected          
    {
    Mode = 1;    // USB mode
    
    }  
    else if (hUsbDeviceFS.dev_state != 0x03) // if USB disconected
    { 
    //MCP4725_Write(3740);
    //HAL_Delay(10);
    Mode = 0; // TSD mode
    Check_USB = 0;
    dim = 1;
    
    }
  }
    switch (Mode) 
  {
//=============================================================================    
    case 1: // USB mode switch case
      while(dim == 1)
      { 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); //Shut down SMPS
    MCP4725_Write(3740);
    HAL_Delay(10);
    sprintf(string4,"dims=%03d",0); 
    string4[8]=0xff;
    string4[9]=0xff;
    string4[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string4 ,11); // dim the TSD screen
    HAL_Delay(10);
    sprintf(string4,"tsw h0,%d\n",0); // disable slider touch function 
    string4[8]=0xff;
    string4[9]=0xff;
    string4[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string4 ,11);
    HAL_Delay(10);
    sprintf(string4,"tsw b0,%d\n",0); // disable button + touch function 
    string4[8]=0xff;
    string4[9]=0xff;
    string4[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string4 ,11);
    HAL_Delay(10);
    sprintf(string4,"tsw b1,%d\n",0); // disable button - touch function 
    string4[8]=0xff;
    string4[9]=0xff;
    string4[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string4 ,11);
    HAL_Delay(10);
    sprintf(string4,"tsw b2,%d\n",0); // disable button on/off touch function 
    string4[8]=0xff;
    string4[9]=0xff;
    string4[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string4 ,11);
    HAL_Delay(10);
    sprintf(string4,"tsw r0,%d\n",0); // disable on/off state touch function 
    string4[8]=0xff;
    string4[9]=0xff;
    string4[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string4 ,11);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); //Shut down SMPS
    dim = 0;
    undim = 1;
    if(Buffer_full == 1) 
  
  { 
    for(int i = 0; i<12; i++) 
    {
      RxBuffer[i] = 0; }
      Buffer_full = 0; 
  }
      }  
//---------------------------------------- flag of sucessfull reception from USB   
   if(Buffer_full == 1)
    {
    if(RxBuffer[0] == 'o' && RxBuffer[1] == 'u'&& RxBuffer[2] == 't' && RxBuffer[3] == '='
    && isdigit(RxBuffer[4]) && isdigit(RxBuffer[5]) && isdigit(RxBuffer[6]) 
    && RxBuffer[7] == '.' && isdigit(RxBuffer[8]) && isdigit(RxBuffer[9])) //---------------- only if this statment is confirmed
    { //------------------------------------------------------------------------------------- the out value is converted 
        Val_buf[0] = RxBuffer[4];
        Val_buf[1] = RxBuffer[5];
        Val_buf[2] = RxBuffer[6];
        Val_buf[3] = RxBuffer[7];
        Val_buf[4] = RxBuffer[8];
        Val_buf[5] = RxBuffer[9];
        Dec_number = strtof(Val_buf, NULL);
    if(Dec_number >= 0 && Dec_number <= 298)
    {
    Bit_value = (298.08 - Dec_number)/0.0797;
    MCP4725_Write(Bit_value);    
    }
    else     MCP4725_Write(5);
    }
      else  
      {      
    while(cnt == 0)
      {
      CDC_Transmit_FS("\r Entered Command Is Incorrect\n", 30);       
      HAL_Delay(1000);
      CDC_Transmit_FS("\r Command Format Is: out=000.00\n", 31);
      cnt = 1;
      }
     }
    }      
    break; // ----------------------------------------------- end of USB mode case
      
 //=============================================================================    
    case 0: // ---------------------------------------------- TSD mode switch case
    
 
  
    if(DataRX[0] == 0x71 ) //---------------------------------- if slider moved
    { 
  
        tmp = DataRX[2] << 8;
 
        Value = 3740 - (tmp + DataRX[1]);
  
        Value2 = tmp + DataRX[1];
  
        SendProgBarVal = Value2 / 37.4;
        sprintf(string,"j0.val=%03d\n",(uint8_t)SendProgBarVal); // send value to prgress bar
        string[10]=0xff;
        string[11]=0xff;
        string[12]=0xff;
        HAL_UART_Transmit_IT(&huart2, string, 13);
        MCP4725_Write(Value);
 
    }

        else if( DataRX[0] == 0x65 && DataRX[2] == 0x06 ){ // button " + " 
          
          if (DataRX[3] != lastButtonState ){
            
             if ( Value >= 7 && Value2 <= 3733) {
               
                 if ( DataRX[3] == 0x01 ){
          
                    Value = Value - 7;
          
                    Value2 = Value2 + 7;
          
                    MCP4725_Write(Value);

                    SendProgBarVal = Value2 / 37.4;
                    sprintf(string,"j0.val=%03d\n",(uint8_t)SendProgBarVal);
                    string[10]=0xff;
                    string[11]=0xff;
                    string[12]=0xff;
                    HAL_UART_Transmit_IT(&huart2, string, 13);
                 }
              }
            }
          } 
   
        else if(DataRX[0] == 0x65 && DataRX[2] == 0x07){ // button " --- " 
          
          if (DataRX[3] != lastButtonState ){
            
            if( Value < 3733 && Value2 >= 7 ){
              
                if (DataRX[3] == 0x01 )     
             {  
                Value = Value + 7;
          
                Value2 = Value2 - 7;
         
                MCP4725_Write(Value);
           
                SendProgBarVal = Value2 / 37.4;
                sprintf(string,"j0.val=%03d\n",(uint8_t)SendProgBarVal);
                string[10]=0xff;
                string[11]=0xff;
                string[12]=0xff;
                HAL_UART_Transmit_IT(&huart2, string, 13);
                     }
             }
            }
          } 
        else if( DataRX[0] == 0x65 && DataRX[2] == 0x08) //-- ON/OFF state button
        { 

          if (DataRX[3] != lastOnOFFState && DataRX[3] == 1 ) {counter3++;}
          
          if (counter3 == 1) 
          {  
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            sprintf(string,"r0.val=%d\n",0); // arg 1 - enable, 0- disable
            string[8]=0xff;
            string[9]=0xff;
            string[10]=0xff;
            HAL_UART_Transmit_IT(&huart2, string ,11);
          }
        else if(counter3 == 2) 
        {  
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
          sprintf(string,"r0.val=%d\n",1); 
          string[8]=0xff;
          string[9]=0xff;
          string[10]=0xff;
          HAL_UART_Transmit_IT(&huart2, string ,11);
          MCP4725_Write(Value);
          counter3 = 0;
        }
      }
lastButtonState = DataRX[3];
lastOnOFFState = DataRX[3];
  /* USER CODE BEGIN 3 */
  HAL_Delay(10);

//====================================================== Read adc ch and update TSD
  if( Measure == 1)
  {
   
  sprintf(string3,"t3.txt=\"%6.2f\"",HVVPlus(20)); 
  string3[15]=0xff;
  string3[16]=0xff;
  string3[17]=0xff;
  HAL_UART_Transmit_IT(&huart2, string3, 18);
  
  sprintf(string3,"t5.txt=\"-%3.3f\"",HVCMinus(20));
  string3[15]=0xff;
  string3[16]=0xff;
  string3[17]=0xff;
  HAL_UART_Transmit_IT(&huart2, string3 ,18);
  
  sprintf(string3,"t1.txt=\"%2.3f\"",HVCPlus(20));
  string3[14]=0xff;
  string3[15]=0xff;
  string3[16]=0xff;
  HAL_UART_Transmit_IT(&huart2, string3, 17);
  
  sprintf(string3,"t7.txt=\"-%6.2f\"",HVVMinus(20));
  string3[16]=0xff;
  string3[17]=0xff;
  string3[18]=0xff;
  HAL_UART_Transmit_IT(&huart2, string3 ,19);
  
  sprintf(string3,"t9.txt=\"%2.3f\"",HVVPlus(20)*HVCPlus(20));
  string3[14]=0xff;
  string3[15]=0xff;
  string3[16]=0xff;
  HAL_UART_Transmit_IT(&huart2, string3, 17);
  
 //  HAL_Delay(10);
 // sprintf(string5,"dims=%03d",100); 
 //   string5[8]=0xff;
 //   string5[9]=0xff;
 //   string5[10]=0xff;
 //   HAL_UART_Transmit_IT(&huart2, string5 ,11); // dim the TSD screen
   
  while(undim == 1)
      { 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
    MCP4725_Write(3740);
    HAL_Delay(10);
    sprintf(string5,"dims=%03d",100); 
    string5[8]=0xff;
    string5[9]=0xff;
    string5[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string5 ,11); // dim the TSD screen
    HAL_Delay(10);
    sprintf(string5,"tsw h0,%d\n",1); // disable slider touch function 
    string5[8]=0xff;
    string5[9]=0xff;
    string5[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string5 ,11);
    HAL_Delay(10);
    sprintf(string5,"tsw b0,%d\n",1); // disable button + touch function 
    string5[8]=0xff;
    string5[9]=0xff;
    string5[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string5 ,11);
    HAL_Delay(10);
    sprintf(string5,"tsw b1,%d\n",1); // disable button - touch function 
    string5[8]=0xff;
    string5[9]=0xff;
    string5[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string5 ,11);
    HAL_Delay(10);
    sprintf(string5,"tsw b2,%d\n",1); // disable button on/off touch function 
    string5[8]=0xff;
    string5[9]=0xff;
    string5[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string5 ,11);
    HAL_Delay(10);
    sprintf(string5,"tsw r0,%d\n",1); // disable on/off state touch function 
    string5[8]=0xff;
    string5[9]=0xff;
    string5[10]=0xff;
    HAL_UART_Transmit_IT(&huart2, string5 ,11);
    dim = 1;
    undim = 0;
      }
  Measure = 0; 
  }
break;

  /* USER CODE END 3 */
  } 
  }
 
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
