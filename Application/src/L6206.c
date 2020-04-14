/**
  ******************************************************************************
  * @file	 L6206.c
  * @author  Przemek
  * @version 1.0
  * @date    28.02.2020
  * @brief Pod³aczenie silnika Pololu 2275
  * 75:1 Metal Gearmotor 25Dx66L mm HP 6V with 48 CPR Encoder
  * Red 	motor power (connects to one motor terminal)
  * Black 	motor power (connects to the other motor terminal)
  * Green 	encoder GND
  * Blue 	encoder Vcc (3.5 V to 20 V)
  * Yellow 	encoder A output
  *White 	encoder B output
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TIMA_CHP_PWM	TIM16->CCR1
#define TIMA_CHN_PWM	TIM17->CCR1
#define SET_EN_A()			(GPIOA->ODR|=(0x1<<10))
#define RESET_EN_A()		(GPIOA->ODR&=~(0x1<<10))

#define TIMB_CHP_PWM	TIM2->CCR1
#define TIMB_CHN_PWM	TIM2->CCR2
#define SET_EN_B()			(GPIOC->ODR|=0x1<<1)
#define RESET_EN_B()		(GPIOC->ODR&=~(0x1<<1))
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja aktywuje niezbêdne peryferia
  * @param[in]  None
  * @retval None
  */
void L6206_init(void){
	//deaktywuje modu³
	RESET_EN_A();
	RESET_EN_B();
	//aktywuje liniczki i kana³y
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim16);
	HAL_TIM_Base_Start(&htim17);

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);

	TIMA_CHP_PWM=0;
	TIMA_CHN_PWM=0;
	TIMB_CHP_PWM=0;
	TIMB_CHN_PWM=0;
}
/**
  * @brief  Funkcja aktywuje lub deaktywuje mostek nie modyfikuj¹c wysterowania PWM
  * @param[in]  None
  * @retval None
  */
void L6206_enable(int ch_A, int ch_B){
	if(ch_B){
		SET_EN_B();
	}else{
		RESET_EN_B();
	}
	if(ch_A){
		SET_EN_A();
	}else{
		RESET_EN_A();
	}
}
/**
  * @brief  Funkcja ustawia wype³enie na kana³ach PWM w zakresie -1000 do 1000
  * @param[in]  None
  * @retval None
  */
void L6206_setDuty(signed int duty_A,signed int duty_B){
	if(duty_A>=0){
		TIMA_CHP_PWM=duty_A;
		TIMA_CHN_PWM=0;
	}else{
		TIMA_CHN_PWM=-duty_A;
		TIMA_CHP_PWM=0;
	}
	if(duty_B>=0){
		TIMB_CHP_PWM=duty_B;
		TIMB_CHN_PWM=0;
	}else{
		TIMB_CHN_PWM=-duty_B;
		TIMB_CHP_PWM=0;
	}

}
/* Private functions ---------------------------------------------------------*/

