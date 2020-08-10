#include "NEXTION.h"
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;



// Sending

void IOutMinusTXT(float Val)
{
  unsigned char string[20];
  sprintf(string,"t5.txt=\"-%3.3f\"",Val);
  string[15]=0xff;
  string[16]=0xff;
  string[17]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,18);
}

void OutMinusTXT(float Val)
{
  unsigned char string[20];
  sprintf(string,"t7.txt=\"-%6.2f\"",Val);
  string[16]=0xff;
  string[17]=0xff;
  string[18]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,19);
}

void OutPlusTXT(float Val)
{
  unsigned char string[20];
  sprintf(string,"t3.txt=\"%6.2f\"",Val);
  string[15]=0xff;
  string[16]=0xff;
  string[17]=0xff;
  HAL_UART_Transmit_IT(&huart2, string, 18);
}

void IOutPlusTXT(float Val)
{
  unsigned char string[20];
  sprintf(string,"t1.txt=\"%2.3f\"",Val);
  string[14]=0xff;
  string[15]=0xff;
  string[16]=0xff;
  HAL_UART_Transmit_IT(&huart2, string, 17);
}

void PowerTXT(float Val)
{
  unsigned char string[20];
  sprintf(string,"t9.txt=\"%2.3f\"",Val);
  string[14]=0xff;
  string[15]=0xff;
  string[16]=0xff;
  HAL_UART_Transmit_IT(&huart2, string, 17);
 }

void SendText9OFF()
{
  unsigned char string[15];
  sprintf(string,"t9.txt=\"OFF\"");
  string[12]=0xff;
  string[13]=0xff;
  string[14]=0xff;
  HAL_UART_Transmit_IT(&huart2, string, 15);
        }

void SendText9ON()
{
  unsigned char string[14];
  sprintf(string,"t9.txt=\"ON\"");
  string[11]=0xff;
  string[12]=0xff;
  string[13]=0xff;
  HAL_UART_Transmit_IT(&huart2, string, 14);
 }

void RefreshPage()
{
  unsigned char string[20];
  sprintf(string,"page 0");
  string[6]=0xff;
  string[7]=0xff;
  string[8]=0xff;
  HAL_UART_Transmit_IT (&huart2, string, 9);
 }

void SendProgBar(uint8_t Val)
{
  unsigned char string[14];
  sprintf(string,"j0.val=%03d\n",Val);
  string[10]=0xff;
  string[11]=0xff;
  string[12]=0xff;
  HAL_UART_Transmit_IT(&huart2, string, 13);

 }

void Screen_Dim(uint8_t Val)
{
  
  unsigned char stringg[12];
  sprintf(stringg,"dims=%03d",(char *)Val);
  stringg[8]=0xff;
  stringg[9]=0xff;
  stringg[10]=0xff;
  stringg[11]=0xff;

  HAL_UART_Transmit_IT(&huart2, stringg ,11);
}

void Sleep_mode(uint8_t Val)
{
   unsigned char string[20];
  sprintf(string,"thsp=%03d\n",Val);
  string[8]=0xff;
  string[9]=0xff;
  string[10]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,11); 
  HAL_Delay(10);
  for (int i = 0; i<20; i++)
  {string[i] = 0;}
  sprintf(string,"thup=1");
  string[6]=0xff;
  string[7]=0xff;
  string[8]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,9);
  HAL_Delay(1);
}

void Component_b0_touch(uint8_t Val)
{
  unsigned char string[20];
  sprintf(string,"tsw b0,%d\n",Val); // arg 1 - enable, 0- disable
  string[8]=0xff;
  string[9]=0xff;
  string[10]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,11);
}

void Component_b1_touch(uint8_t Val)
{
  unsigned char string[20];
  sprintf(string,"tsw b1,%d\n",Val); // arg 1 - enable, 0- disable
  string[8]=0xff;
  string[9]=0xff;
  string[10]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,11);
}

void Component_b2_touch(uint8_t Val)
{
  unsigned char string[20];
  sprintf(string,"tsw b2,%d\n",Val); // arg 1 - enable, 0- disable
  string[8]=0xff;
  string[9]=0xff;
  string[10]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,11);
}

void Component_h0_touch(uint8_t Val)
{
  unsigned char string[20];
  sprintf(string,"tsw h0,%d\n",Val); // arg 1 - enable, 0- disable
  string[8]=0xff;
  string[9]=0xff;
  string[10]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,11);
}

void Component_r0_touch(uint8_t Val)
{
  unsigned char string[20];
  sprintf(string,"tsw r0,%d\n",Val); // arg 1 - enable, 0- disable
  string[8]=0xff;
  string[9]=0xff;
  string[10]=0xff;
  HAL_UART_Transmit_IT(&huart2, string ,11);
}
// AD5172 function
void AD5172_Write( uint16_t Address, uint8_t Channel, uint8_t Value)
{
 uint8_t ChannelAr[3]={0x0,0x8,0x88};
 uint8_t Data[2];
 Data[0]=ChannelAr[Channel];
 Data[1]=Value;
 // HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 HAL_I2C_Master_Transmit(&hi2c1,Address, Data, 2, 1);
}

// MCP4725 function
void MCP4725_Write( uint16_t Value)
{
 uint8_t Data[2];
 Data[0]=(Value & 0x0F00)>>8;
 Data[1]=Value & 0x00FF;
 // HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 HAL_I2C_Master_Transmit(&hi2c1,0xC0, Data, 2, 1);  
}

float HVCMinus(uint8_t N)
{ 
  
  uint8_t Setup[1] = {0x82};
  uint8_t ScanAIN0[1] = {0x61};
  
  float Summ = 0, Rezult = 0, Current = 0;
  uint8_t ADC_0[2];
  uint16_t i = 0;
  uint16_t ADC0Val = 0;
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, Setup, 1, 10);//setup byte
  //HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, ScanAIN0, 1, 10);//setup byte
 
	  for(i=0;i<N;i++)
	     {
            HAL_I2C_Master_Receive(&hi2c1, RADDR, ADC_0, 2, 50);//setup byte
            ADC0Val = ((ADC_0[0]&0x0F)<<8) | ADC_0[1];

	 Summ = Summ + ADC0Val;				 
	}
        Summ = Summ/N;
	Rezult = (Summ*5/4096);       
        Current = (-1.0537* (Rezult*Rezult*Rezult)) + (4.9151*(Rezult*Rezult)) + ((6.3271*Rezult) + 0.3273);
        
    return Current/1000;	        
}

float HVCPlus(uint8_t N)
{ 
  
  uint8_t Setup[1] = {0x82};
  uint8_t ScanAIN1[1] = {0x63};
  
  float Summ = 0, Rezult = 0, Current = 0;
  uint8_t ADC_1[2];
  uint16_t i = 0;
  uint16_t ADC1Val = 0;
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, Setup, 1, 10);//setup byte
  //HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, ScanAIN1, 1, 10);//setup byte
 
	  for(i=0;i<N;i++)
	     {
            HAL_I2C_Master_Receive(&hi2c1, RADDR, ADC_1, 2, 50);//setup byte
            ADC1Val = ((ADC_1[0]&0x0F)<<8) | ADC_1[1];

	 Summ = Summ + ADC1Val;				 
	}
	Summ = Summ/N;
        Rezult = (Summ*5/4096);
        Current = (-1.0537* (Rezult*Rezult*Rezult)) + (4.9151*(Rezult*Rezult)) + ((6.3271*Rezult) + 0.3273);
   
    return Current/1000;	        
}

float HVVPlus(uint8_t N)
{ 
  
  uint8_t Setup[1] = {0x82};
  float Output = 0;
  uint8_t ScanAIN2[1] = {0x65};
  
  float Summ = 0;
  uint8_t ADC_2[2];
  uint16_t i = 0;
  uint16_t ADC2Val = 0;
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, Setup, 1, 10);//setup byte
  //HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, ScanAIN2, 1, 10);//setup byte
 
	  for(i=0;i<N;i++)
	     {
            HAL_I2C_Master_Receive(&hi2c1, RADDR, ADC_2, 2, 50);//setup byte
            ADC2Val = ((ADC_2[0]&0x0F)<<8) | ADC_2[1];

	 Summ = Summ + ADC2Val;				 
	}
	Summ = (Summ/N);
        
   Output = (Summ*5/4096)*100;
    return(Output);	        
}

float HVVMinus(uint8_t N)
{ 
  
  uint8_t Setup[1] = {0x82};
  uint8_t ScanAIN3[1] = {0x67};
  
  float Summ = 0;
  uint8_t ADC_3[2];
  uint16_t i = 0;
  uint16_t ADC3Val = 0;
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, Setup, 1, 10);//setup byte
  //HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, WADDR, ScanAIN3, 1, 10);//setup byte
 
	  for(i=0;i<N;i++)
	     {
            HAL_I2C_Master_Receive(&hi2c1, RADDR, ADC_3, 2, 50);//setup byte
            ADC3Val = ((ADC_3[0]&0x0F)<<8) | ADC_3[1];

	 Summ = Summ + ADC3Val;				 
	}
	Summ = (Summ/N);         
   
    return((Summ*5/4096)*100);	        
}





