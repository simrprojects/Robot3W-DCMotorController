/*
 *
 *
 *  Created on: 02.03.2020
 *      Author: Przemek
 */
/**
  ******************************************************************************
  * @file	 UartCommunication.c
  * @author  Przemek
  * @version V1.0.0
  * @date    02.03.2020
  * @brief   Modu³ odpowiedzialny za komunikacjê po UART
  * Ramka danych
  * [1B] PRE1: 0x12
  * [2B] PRE2: 0x34
  * [3B] SIZE
  * [4B] - [nB] dane
  * [nB+1] CRC: 0xAA
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "UartCommunication.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
#define UART_RX_BUFF_SIZE	64
#define UART_TX_BUFF_SIZE	32

typedef struct{
	xQueueHandle queue;
	unsigned char rxBuf[UART_RX_BUFF_SIZE];
	unsigned char txBuf[UART_TX_BUFF_SIZE];
}tUartCommunication;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tUartCommunication uart;
/* Private function prototypes -----------------------------------------------*/
void UartCommunication_Thread(void* ptrt);
extern uint32_t ITM_SendChar (uint32_t ch);

/* Public  functions ---------------------------------------------------------*/
/**
  * @brief Modu³ odpowiedzialny za komunikacjê
  * @param[in]  None
  * @retval None
  */
void UartCommunication_Init(void){
	uart.queue=xQueueCreate(10,1);
	huart2.RxISR=HAL_UART_RxCpltCallback;
	xTaskCreate(UartCommunication_Thread,"uart",250,0,2,0);
}
/**
  * @brief Modu³ odpowiedzialny za wysy³anie danych w ramce
  * @param[in]  None
  * @retval None
  */
void UartCommunication_Send(unsigned char id,void*data, int size){
	if(size<(UART_TX_BUFF_SIZE-(2+1+1+1))){
		uart.txBuf[0]=0x12;
		uart.txBuf[1]=0x34;
		uart.txBuf[2]=size+1;
		uart.txBuf[3]=id;
		memcpy(&uart.txBuf[4],data,size);
		uart.txBuf[4+size]=0xAA;
		HAL_UART_Transmit_IT(&huart2,uart.txBuf,size+5);
	}
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief W¹tek zajmuje siê przetwarzaniem odczytanych danych i formatowaniem ich
  * @param[in]  None
  * @retval None
  */
void UartCommunication_Thread(void* ptrt){
	unsigned int state=0;
	unsigned char rx,size,ptr;
	//aktywuje przerwanie odbiorcze uart
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE);

	while(1){
		if(xQueueReceive(uart.queue,&rx,50)==pdTRUE){
			//odczyta³em
			switch(state){
			case 0:
				if(rx==0x12){
					state=1;
				}
				break;
			case 1:
				if(rx==0x34){
					state=2;
				}else{
					state=0;
				}
				break;
			case 2:
				size=rx;
				ptr=0;
				state=3;
				break;
			case 3:
				if(ptr<size){
					uart.rxBuf[ptr++]=rx;
				}else{
					//crc
					if(rx==0xAA){
						printf("UC: New frame; id:%d, size:%d\n\r",uart.rxBuf[0],size-1);
						UartCommunication_NewFrame(uart.rxBuf[0],(char*)&uart.rxBuf[1],size-1);
					}
					state=0;
				}
			}
		}else{
			//nieodczyta³em
			state=0;
		}
	}
}
/**
  * @brief Funkcja zwrotna przekazujaca odebrane dane
  * @param[in]  None
  * @retval None
  */
__weak void UartCommunication_NewFrame(unsigned char frameId, char* data,int size){

}
/**
  * @brief odebrano dane z uart
  * @param[in]  None
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t xHigherPriorityTaskWokenByPost=pdFALSE;
	unsigned char rx=USART2->RDR;
	xQueueSendFromISR(uart.queue,&rx,&xHigherPriorityTaskWokenByPost);
	ITM_SendChar(rx);
	if( xHigherPriorityTaskWokenByPost ){
		taskYIELD();
	}
}
/**
  * @brief
  * @param[in]  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}
