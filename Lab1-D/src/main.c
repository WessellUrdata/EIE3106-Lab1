// The SPL included in this project has to be added to PlatformIO following the instructions in this forum post:
// https://community.platformio.org/t/stm32-standard-library/7086/15
// Since it hasn't been added to the mainline
// Otherwise, it won't work

#include "stm32f10x.h"
#include <string.h>
#include <stdbool.h>

// TIM3_CH1, PA6, for PWM output
#define PWMen		 	(RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE))
#define PWMgpio			GPIOA
#define PWMpin			GPIO_Pin_6

// TIM4_CH1, PB6, for Input Capture
#define ICen			(RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE))
#define ICgpio			GPIOB
#define ICpin			GPIO_Pin_6


char welcomeMessage[] = "Enter integer between 10 and 99 to set as pulse width: ";
static uint8_t welcomeMessage_i = 0;
bool welcomeOver = false;

volatile unsigned char character;

void PWMinit() {
	
	// GPIO setup
	PWMen;
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

void ICinit() {

	// GPIO setup
	ICen;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIOstruct;

	GPIOstruct.GPIO_Pin = ICpin;
	GPIOstruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOstruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ICgpio, &GPIOstruct);

	// Timer4 setup
	// same setup as Timer3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIMstruct;

	TIMstruct.TIM_Prescaler = 1152 - 1;
	TIMstruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIMstruct.TIM_Period = 125 - 1;
	TIMstruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIMstruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIMstruct);
	TIM_Cmd(TIM4, ENABLE);

	// IC setup
	TIM_ICInitTypeDef ICstruct;
	ICstruct.TIM_Channel = TIM_Channel_1; // Select IC1
	ICstruct.TIM_ICPolarity = TIM_ICPolarity_Rising; // Capture rising
	ICstruct.TIM_ICSelection = TIM_ICSelection_DirectTI; // Map to TI1
	ICstruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; // Configure input frequency
	ICstruct.TIM_ICFilter = 0; // no filter
	TIM_ICInit(TIM4, &ICstruct);

	//Enable Input Capture Interrupt
	NVIC_InitTypeDef IC_NVICstruct;
	IC_NVICstruct.NVIC_IRQChannel = TIM4_IRQn; // TIM4 interrupt
	IC_NVICstruct.NVIC_IRQChannelPreemptionPriority = 2; // Preemptive priority level 2
	IC_NVICstruct.NVIC_IRQChannelSubPriority = 0; // From the priority level 0
	IC_NVICstruct.NVIC_IRQChannelCmd = ENABLE; // Enable IRQ channel
	NVIC_Init(&IC_NVICstruct);
	TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, ENABLE); // Allow updates to interrupt, allows the CC1IE to capture interrupt

}

void USART2init(void) {

	//USART2 TX RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	//USART2 ST-LINK USB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStructure; 
	
	USART_InitStructure.USART_BaudRate = 9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
	
	
	NVIC_InitTypeDef UART_NVICinit;
	// Enable the USART2 TX Interrupt 
	USART_ITConfig(USART2, USART_IT_TC, ENABLE );
	UART_NVICinit.NVIC_IRQChannel = USART2_IRQn;
	UART_NVICinit.NVIC_IRQChannelPreemptionPriority = 0;
	UART_NVICinit.NVIC_IRQChannelSubPriority = 0;
	UART_NVICinit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&UART_NVICinit);
	// Enable the USART2 RX Interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );
	UART_NVICinit.NVIC_IRQChannel = USART2_IRQn;
	UART_NVICinit.NVIC_IRQChannelPreemptionPriority = 0;
	UART_NVICinit.NVIC_IRQChannelSubPriority = 0;
	UART_NVICinit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&UART_NVICinit);

}

int main() {

	USART2init();
	PWMinit();
	ICinit();

	while (1) {
		
	}
}

void USART2_IRQHandler() {

	if (USART_GetITStatus(USART2, USART_IT_TC) != RESET) {

		if (welcomeMessage_i < strlen(welcomeMessage)) {
			USART_SendData(USART2, welcomeMessage[welcomeMessage_i++]);
		} 
		else if (welcomeMessage_i == strlen(welcomeMessage)) {
			welcomeOver = true;
		}

		USART_ClearITPendingBit(USART2, USART_IT_TC);

	}

	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

		character = (unsigned char) USART_ReceiveData(USART2);
		USART_SendData(USART2, character);

		USART_ClearITPendingBit(USART2, USART_IT_RXNE);


	}

}

void TIM4_IRQHandler(void) {

}