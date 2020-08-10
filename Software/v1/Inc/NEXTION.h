#ifndef __NEXTION_H
#define __NEXTION_H

#include <string.h>
#include <stdint.h>


#define WADDR 0x68
#define RADDR 0x69


// Sending
void IOutMinusTXT(float Val);
void OutMinusTXT(float Val);
void OutPlusTXT(float Val);
void IOutPlusTXT(float Val);
void PowerTXT(float Val);
void SendText9OFF();
void SendText9ON();
void RefreshPage();
void SendProgBar(uint8_t Val);
void Screen_Dim(uint8_t Val);
void Sleep_mode(uint8_t Val);
void Component_b0_touch(uint8_t Val);
void Component_b1_touch(uint8_t Val);
void Component_b2_touch(uint8_t Val);
void Component_h0_touch(uint8_t Val);
void Component_r0_touch(uint8_t Val);
void AD5172_Write( uint16_t Address, uint8_t Channel, uint8_t Value);
void MCP4725_Write( uint16_t Value);
float HVCMinus(uint8_t N);
float HVCPlus(uint8_t N);
float HVVPlus(uint8_t N);
float HVVMinus(uint8_t N);

#endif 




