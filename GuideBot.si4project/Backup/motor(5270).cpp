#include "motor.h"
#include "main.h"
//#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

// GPIO_TypeDef* MOTOR_PORT[MOTORn] = {RIKI_MOTOR1_GPIO_PORT, RIKI_MOTOR2_GPIO_PORT};
GPIO_TypeDef* MOTOR_PWM_PORT[MOTORn] = {RIKI_MOTOR1_PWM_PORT, RIKI_MOTOR2_PWM_PORT};
TIM_TypeDef*  MOTOR_PWM_TIM[MOTORn] = {RIKI_MOTOR1_PWM_TIM, RIKI_MOTOR2_PWM_TIM};
// const uint32_t  MOTOR_PORT_CLK[MOTORn] = {RIKI_MOTOR1_GPIO_CLK, RIKI_MOTOR2_GPIO_CLK};
// const uint32_t  MOTOR_PWM_PORT_CLK[MOTORn] = {RIKI_MOTOR1_PWM_CLK, RIKI_MOTOR2_PWM_CLK};
// const uint32_t  MOTOR_PWM_TIM_CLK[MOTORn] = {RIKI_MOTOR1_PWM_TIM_CLK, RIKI_MOTOR2_PWM_TIM_CLK};
const uint16_t  MOTOR_A_PIN[MOTORn] = {RIKI_MOTOR1_A_PIN, RIKI_MOTOR2_A_PIN};
const uint16_t  MOTOR_B_PIN[MOTORn] = {RIKI_MOTOR1_B_PIN, RIKI_MOTOR2_B_PIN};
const uint16_t	MOTOR_C_PIN[MOTORn] = {RIKI_MOTOR1_C_PIN, RIKI_MOTOR2_C_PIN};
const uint16_t  MOTOR_PWM_PIN[MOTORn] = {RIKI_MOTOR1_PWM_PIN, RIKI_MOTOR2_PWM_PIN};


Motor::Motor(Motor_TypeDef _motor, uint32_t _arr, uint32_t _psc)
{
	motor = _motor;
	arr = _arr;
	psc = _psc;
}

void Motor::init()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	// RCC_APB2PeriphClockCmd(MOTOR_PORT_CLK[this->motor] | MOTOR_PWM_PORT_CLK[this->motor], ENABLE);
	 /** init motor gpio **/
	GPIO_InitStructure.Pin     = MOTOR_A_PIN[this->motor];
	GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed   = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin     = MOTOR_B_PIN[this->motor]|MOTOR_C_PIN[this->motor];
	GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed   = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	/** 继电器使能 **/
	GPIO_InitStructure.Pin     = GPIO_PIN_3;
	GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed   = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull    = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);

	/** init motor pwm gpio **/
	GPIO_InitStructure.Pin     = MOTOR_PWM_PIN[this->motor];
	GPIO_InitStructure.Mode    = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed   = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(MOTOR_PWM_PORT[this->motor], &GPIO_InitStructure);

	// motor_pwm_init();
}

void Motor::motor_pwm_init()
{
	//pwm value ((1 + psc)/72M)*(1+arr)
	//eg: ((1+143)/72M)*(1+9999) = 0.02s --10000 count use 0.02s
	//set arduino pwm value 490hz 255 count 
	//((1 + 575)/72M)(1 + 254) = (1 / 490)
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
 	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 163-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.RepetitionCounter = 0;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if( HAL_TIM_Base_Init(&htim3) != HAL_OK){
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if( HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK){
		Error_Handler();
	}
	/*
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(MOTOR_PWM_TIM_CLK[this->motor], ENABLE);

	TIM_BaseInitStructure.TIM_Period                = this->arr;
	TIM_BaseInitStructure.TIM_Prescaler             = this->psc;
	TIM_BaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter     = 0;

	TIM_TimeBaseInit(MOTOR_PWM_TIM[this->motor], &TIM_BaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	if(this->motor == MOTOR1){
		TIM_OC1Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}

	if(this->motor == MOTOR2) {
		TIM_OC2Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}
	TIM_ARRPreloadConfig(MOTOR_PWM_TIM[this->motor], ENABLE);

	TIM_CtrlPWMOutputs(MOTOR_PWM_TIM[this->motor], ENABLE);
	TIM_Cmd(MOTOR_PWM_TIM[this->motor], ENABLE);
	*/
}

void Motor::spin(int pwm)
{
	if(pwm > 0){
		HAL_GPIO_WritePin(MOTOR_PORT[this->motor], MOTOR_A_PIN[this->motor], GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_PORT[this->motor], MOTOR_B_PIN[this->motor], GPIO_PIN_RESET);
	}else if(pwm < 0) {
		HAL_GPIO_WritePin(MOTOR_PORT[this->motor], MOTOR_B_PIN[this->motor], GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_PORT[this->motor], MOTOR_A_PIN[this->motor], GPIO_PIN_RESET);
	}
	if(this->motor == MOTOR1){
		//TIM_SetCompare1(MOTOR_PWM_TIM[this->motor], abs(pwm));
		htim3.Instance->CCR1 = pwm;
	}
	if(this->motor == MOTOR2){
		//TIM_SetCompare2(MOTOR_PWM_TIM[this->motor], abs(pwm));
		htim3.Instance->CCR2 = pwm;
	}
}
