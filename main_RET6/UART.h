#ifndef __UART_H
#define __UART_H
#include <stdint.h>

void UART4_Init(void);
void send_data(float *data, uint8_t size);

#define CH_COUNT 22  // the max channel count
#define TAIL {0x00, 0x00, 0x80, 0x7f}  // Data frame end

extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial_SendPacket(void);
uint8_t Serial_GetRxFlag(void);

#endif
