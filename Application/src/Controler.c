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
#include "LinearModules.h"
#include "math.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	float kp;//wzmocneinei proporcjonalne
	float ki;//wzmocnienei ca³kuj¹ce
	float kff;//wzmocnienei do przodu
	float kv;//wsp. rpm/v
	float u;//napiêcie zasilania
	float u_max;//limit napiecia silnika
	unsigned int revers;
	float tp;//okres regulacji
	float rpm_max;
}tPIDCfg;
typedef struct{
	float kp;//wzmocneinei proporcjonalne
	float ki;//wzmocnienei ca³kuj¹ce
	float kff;//wzmocnienei do przodu
	signed int saturation;
	float kv;//wsp. rpm/v
	float u;//napiêcie zasilania
	float u_max;//limit napiecia silnika
	float rpm_max;
	unsigned int revers;//praca silnika w rewersie
	tIntegrator integrator;
}tRpmControler;
typedef struct{
	xQueueHandle motorAqueue;
	xQueueHandle nweUartFrame;
	xQueueHandle motorqueue;
	tRpmControler aPID;
	tRpmControler bPID;
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
int Controler_RPMcontrolerExecute(tRpmControler *rc,float rpm_sp,float rpm_cv);
int Controler_RPMcontrolerInit(tRpmControler *rc,tPIDCfg *cfg);
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
	//inicjujê parametry kontrolera
	tPIDCfg cfg;
	cfg.kff=0.4;
	cfg.kp=5.;
	cfg.ki=10.;
	cfg.kv=21.67;//rpm/V
	cfg.revers=0;
	cfg.tp=0.01;
	cfg.u=7.3;
	cfg.u_max=6;
	cfg.rpm_max=130;
	Controler_RPMcontrolerInit(&controler.aPID,&cfg);
	cfg.revers=1;
	Controler_RPMcontrolerInit(&controler.bPID,&cfg);

	controler.flags.rpm_pid_ctrl_enable=1;
	controler.sp_rpm_a=0;
	controler.sp_rpm_b=0;
	controler.sp_pwm_a=0;
	controler.sp_pwm_b=0;
	L6206_enable(1,1);
	//inicjuje w¹tek kontroler
	xTaskCreate(ControlerTask,"controler",500,0,5,0);
	xTaskCreate(ControlerSpervisorTask,"ctrl_sup",250,0,2,0);
	HAL_TIM_Base_Start_IT(&htim7);
	//inicjuej modu³ komunikacyjny
//	UartCommunication_Init();
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
			dEnc = (signed int)((signed int)enc_a-(signed int)last_enc_a);
			if(dEnc>(65536/2)){
				dEnc=dEnc-65536;
			}else if(dEnc<-(65536/2)){
				dEnc = 65536+dEnc;
			}
			last_enc_a=enc_a;
			rpm = (float)dEnc*/*(100*60/24/75)*/3.33333333;
			controler.rpm_a+=(rpm-controler.rpm_a)*0.4;
			//obliczenia dla ko³a B
			dEnc = (signed int)enc_b-(signed int)last_enc_b;
			if(dEnc>(65536/2)){
				dEnc=dEnc-65536;
			}else if(dEnc<-(65536/2)){
				dEnc = 65536+dEnc;
			}
			last_enc_b=enc_b;
			rpm = (float)dEnc*/*(100*60/24/75)*/3.33333333;
			controler.rpm_b+=(rpm-controler.rpm_b)*0.4;
			controler.encoder_a=enc_a;
			controler.encoder_b=enc_b;
			if(controler.flags.rpm_pid_ctrl_enable){
				//sterowanie prêdkosci¹ obrotow¹ silników
				controler.sp_pwm_a=Controler_RPMcontrolerExecute(&controler.aPID,controler.sp_rpm_a,controler.rpm_a);
				controler.sp_pwm_b=Controler_RPMcontrolerExecute(&controler.bPID,controler.sp_rpm_b,controler.rpm_b);
				L6206_setDuty(controler.sp_pwm_a,controler.sp_pwm_b);
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
			//controler.sp_rpm_a=0;
			//controler.sp_rpm_b=0;
			//L6206_enable(0,0);
		}
		//Controler_SendFrame();
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
  * @brief	Funkcja kontrolera prêdkoœci wykonuje pojedynczy cykl obliczeniowy
  * @param[in]  None
  * @retval None
  */
int Controler_RPMcontrolerExecute(tRpmControler *rc,float rpm_sp,float rpm_cv){
	float cv_ff;
	//ograniczam sterowanie
	if(fabs(rpm_sp)>rc->rpm_max){
		rpm_sp = rpm_sp/fabs(rpm_sp)*rc->rpm_max;
	}
	//odwrócenie kierunku obrotów
	if(rc->revers){
		rpm_sp=-rpm_sp;
		rpm_cv=-rpm_cv;
	}
	//wyzcznaczam sk³¹dow¹ sterowania od feedforward
	cv_ff = rpm_sp*rc->kff/rc->kv;
	//wyznaczam odchy³kê
	float e = rpm_sp-rpm_cv;
	float cv_pid = (rc->kp*e+Integrator_Execute(&rc->integrator,rc->saturation,e)*rc->ki)/rc->kv;
	//wyznaczam ca³kowite napiêcie sterowania
	float u = cv_ff+cv_pid;
	//sprawdzam limity napiêcia sterowania
	if(u>rc->u_max){
		u=rc->u_max;
		rc->saturation=1;
	}else if(u<-rc->u_max){
		u=-rc->u_max;
		rc->saturation=-1;
	}else{
		rc->saturation=0;
	}
	//przeliczam na sterownaie PWM w zakresie +-1000
	return 1000*u/rc->u;
}
/**
  * @brief Funkcja inicjuje kontroler prêdkoœci
  * @param[in]  None
  * @retval None
  */
int Controler_RPMcontrolerInit(tRpmControler *rc,tPIDCfg *cfg){
	//inicjuje parametry
	Integrator_SetTime(&rc->integrator,cfg->tp);
	rc->saturation=0;
	rc->kff=cfg->kff;
	rc->ki=cfg->ki;
	rc->kp=cfg->kp;
	rc->kv=cfg->kv;
	rc->revers=cfg->revers;
	rc->rpm_max=cfg->rpm_max;
	rc->u=cfg->u;
	rc->u_max=cfg->u_max;
	return 0;
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
