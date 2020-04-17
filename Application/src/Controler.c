/**
  ******************************************************************************
  * @file	 Controler.c
  * @author  Przemek
  * @version V1.0.0
  * @date    28.02.2020
  * @brief	 Modu³ kontrolera zapewniaj¹cy sterowanie silnikami.
  * @note 	 Obecnie obs³ógiwany jest tylko tryb sterowania PWM
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "UartCommunication.h"
#include "L6206.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	xQueueHandle motorAqueue;
	xQueueHandle nweUartFrame;
	xQueueHandle motorqueue;
	float sp_rpm_a;
	float sp_rpm_b;
	float rpm_a;
	float rpm_b;
	signed short sp_pwm_a;
	signed short sp_pwm_b;
	unsigned short encoder_a;
	unsigned short encoder_b;
	struct{
		unsigned int rpm_pid_ctrl_enable:1;
	}flags;
}tControler;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tControler controler;
/* Private function prototypes -----------------------------------------------*/
void ControlerTask(void* ptr);
void ControlerSpervisorTask(void* ptr);
void Controler_SendFrame(void);
/* Public  functions ---------------------------------------------------------*/
void Controler_Init(void){
	//inicjuje pracê enkoderów
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	//inicjuje kolejkê
	controler.motorqueue = xQueueCreate(10,4);
	controler.nweUartFrame = xQueueCreate(10,1);
	//controler.motorBqueue = xQueueCreate(10,2);
	//inicjuje w¹tek kontroler
	xTaskCreate(ControlerTask,"controler",500,0,5,0);
	xTaskCreate(ControlerSpervisorTask,"ctrl_sup",250,0,2,0);
	HAL_TIM_Base_Start_IT(&htim7);
	//inicjuej modu³ komunikacyjny
	UartCommunication_Init();
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief W¹tek kontrolera
  * @param[in]  None
  * @retval None
  */
volatile unsigned int tim1cnt,tim4cnt;
volatile float rpm_a;
void ControlerTask(void* ptr){
	unsigned int encoder=0,enc_a,last_enc_a=0,enc_b,last_enc_b=0;
	signed int dEnc;
	float rpm;
	while(1){
		if(xQueueReceive(controler.motorqueue,&encoder,100)==pdTRUE){
			//nowy pomiar
			tim1cnt=encoder&0xFFFF;
			tim4cnt = (encoder>>16)&0xFFFF;
			enc_a=encoder&0xFFFF;
			enc_b=(encoder>>16)&0xFFFF;
			//obliczenia dla ko³a A
			dEnc = (signed int)enc_a-(signed int)last_enc_a;
			last_enc_a=enc_a;
			rpm = (float)dEnc*/*(100*60/24/75)*/3.33333333;
			rpm_a = rpm;
			controler.rpm_a=rpm;
			//obliczenia dla ko³a B
			dEnc = (signed int)enc_b-(signed int)last_enc_b;
			last_enc_b=enc_b;
			rpm = (float)dEnc*/*(100*60/24/75)*/3.33333333;

			controler.rpm_b=rpm;
			controler.encoder_a=enc_a;
			controler.encoder_b=enc_b;
			if(controler.flags.rpm_pid_ctrl_enable){
				//sterowanie prêdkosci¹ obrotow¹ silników
			}else{
				//bezpoœrednie sterowanie PWM
				L6206_setDuty(controler.sp_pwm_a,controler.sp_pwm_b);
			}
		}else{
			//problem z licznikiem
		}
	}
}
/**
  * @brief Watek kontrolny nadzorujacy proces komunikacyjny. W przypadku braku ramek uart zatrzymje silnik
  * @param[in]  None
  * @retval None
  */
void ControlerSpervisorTask(void* ptr){
	unsigned char id;
	while(1){
		if(xQueueReceive(controler.nweUartFrame,&id,500)){
			//nowe wysterowanie do przekazania
			//zwracam wartosc pomiarów
			Controler_SendFrame();
			L6206_enable(1,1);
		}else{
			//pzekroczono timeout, wylaczam silniki
			controler.sp_rpm_a=0;
			controler.sp_rpm_b=0;
			L6206_enable(0,0);
		}
		Controler_SendFrame();
	}
}
/**
  * @brief Funkcja zwrotna przenosz¹ca dane z ramek uart
  * @param[in]  None
  * @retval None
  */
void UartCommunication_NewFrame(unsigned char frameId, char* data,int size){
	switch(frameId){
	case 1://sterowanie rpm silnika A i B
		if(size>=8){
			//poprawny rozmiar
			float rpm_a,rpm_b;
			rpm_a=*(float*)data;
			rpm_b=*(float*)&data[4];
			//przekazuje dane
			controler.sp_rpm_a=rpm_a;
			controler.sp_rpm_b=rpm_b;
			//zg³aszam do w¹tku nadzorczego odebranie danych
			xQueueSend(controler.nweUartFrame,&frameId,0);
		}
		break;
	case 3://sterowanie napiêciem silnika A i B
		if(size>=2){
			signed char a,b;
			a = (*(signed char*)&data[0]);
			b = *(signed char*)&data[1];
			if(a>100){
				a = 100;
			}else if(a<-100){
				a = -100;
			}
			if(b>100){
				b = 100;
			}else if(b<-100){
				b = -100;
			}
			controler.flags.rpm_pid_ctrl_enable=0;
			controler.sp_pwm_a = 5*(signed int)a;
			controler.sp_pwm_b = 5*(signed int)b;
			//zg³aszam do w¹tku nadzorczego odebranie danych
			xQueueSend(controler.nweUartFrame,&frameId,0);
		}
		break;
	}
}
/**
  * @brief Funkcja wysy³a dane na port szeregowy
  * @param[in]  None
  * @retval None
  */
void Controler_SendFrame(void){
	char data[4+4+2+2];
	*(int*)&data[0]=*(int*)&controler.rpm_a;
	*(int*)&data[4]=*(int*)&controler.rpm_b;
	*(short*)&data[8]=*(short*)&controler.encoder_a;
	*(short*)&data[10]=*(short*)&controler.encoder_b;
	UartCommunication_Send(2,data,12);
}
/**
  * @brief Przerwanie od licznika synchronizuj¹cego
  * @param[in]  None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWokenByPost=pdFALSE;
	__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	unsigned int encoder = TIM1->CNT | ((unsigned int)TIM4->CNT<<16)&0xFFFF0000;
	xQueueSendFromISR(controler.motorqueue,&encoder,&xHigherPriorityTaskWokenByPost);
	if( xHigherPriorityTaskWokenByPost ){
		taskYIELD();
	}

}
