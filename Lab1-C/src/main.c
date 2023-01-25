// The SPL included in this project has to be added to PlatformIO following the instructions in this forum post:
// https://community.platformio.org/t/stm32-standard-library/7086/15
// Since it hasn't been added to the mainline
// Otherwise, it won't work

#include "stm32f10x.h"

// TIM3_CH1, PA6, for PWM output
#define PWMrcc		 	RCC_APB2Periph_GPIOA
#define PWMgpio			GPIOA
#define PWMpin			GPIO_Pin_6

void initTIM3CH1() {
	
	// GPIO setup
	RCC_APB2PeriphClockCmd(PWMrcc, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIOstruct;

	GPIOstruct.GPIO_Pin = PWMpin;
	GPIOstruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOstruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PWMgpio, &GPIOstruct);
	
	// Timer3 setup
	// 72,000,000 / 125 (TIM_Period) / 1152 (Prescaler) = 500Hz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIMstruct;
	
	TIMstruct.TIM_Prescaler = 1152 - 1;
	TIMstruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIMstruct.TIM_Period = 125 - 1;
	TIMstruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIMstruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIMstruct);
	TIM_Cmd(TIM3, ENABLE);
	
	TIM_OCInitTypeDef PWMstruct;
	PWMstruct.TIM_OCMode = TIM_OCMode_PWM1;
	PWMstruct.TIM_Pulse = 63 - 1;
	PWMstruct.TIM_OutputState = TIM_OutputState_Enable;
	PWMstruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &PWMstruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

}

int main() {

	initTIM3CH1();

	while(1) {
	}
}