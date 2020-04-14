/*
 * UartCommunication.h
 *
 *  Created on: 02.03.2020
 *      Author: Przemek
 */

#ifndef UARTCOMMUNICATION_H_
#define UARTCOMMUNICATION_H_


void UartCommunication_Init(void);
void UartCommunication_Send(unsigned char id,void*data, int size);
void UartCommunication_NewFrame(unsigned char frameId, char* data,int size);

#endif /* UARTCOMMUNICATION_H_ */
