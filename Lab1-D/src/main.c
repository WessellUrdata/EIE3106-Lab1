// The SPL included in this project has to be added to PlatformIO following the instructions in this forum post:
// https://community.platformio.org/t/stm32-standard-library/7086/15
// Since it hasn't been added to the mainline
// Otherwise, it won't work

#include "stm32f10x.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// TIM3_CH1, PA6, for PWM output
#define PWMen		 	(RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE))
#define PWMgpio			GPIOA
#define PWMpin			GPIO_Pin_6

// TIM4_CH1, PB6, for Input Capture
#define ICen			(RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE))
#define ICgpio			GPIOB
#define ICpin			GPIO_Pin_6

// Welcome message as constant saves RAM and Flash
#define welcomeMessage 	("Enter integer between 10 and 99 to set as pulse width: ")
#define reportMessage	("The pulse width is: ")

bool pulseHigh = false;
uint16_t pulseWidth = 0;

void PWMinit(uint8_t width) {
	
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
	PWMstruct.TIM_Pulse = width - 1;
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

	GPIO_InitTypeDef GPIOstruct;
	GPIOstruct.GPIO_Pin = GPIO_Pin_2;
  	GPIOstruct.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIOstruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIOstruct);

	GPIOstruct.GPIO_Pin = GPIO_Pin_3;
  	GPIOstruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOstruct); 
	
	//USART2 ST-LINK USB
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2 in RCC_APB1
	
	USART_InitTypeDef USARTstruct;
	//USART_ClockInitTypeDef USART_ClockInitStructure; 
	
	USARTstruct.USART_BaudRate = 9600;
  	USARTstruct.USART_WordLength = USART_WordLength_8b;
 	USARTstruct.USART_StopBits = USART_StopBits_1;
  	USARTstruct.USART_Parity = USART_Parity_No;
  	USARTstruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	USARTstruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USARTstruct);
	USART_Cmd(USART2, ENABLE);
	
}

void transmit(uint16_t Data) { // modified from USART_SendData func, hard-coded for USART2

	// Wait for empty transmit buffer
	while (!(USART2->SR & USART_SR_TXE));

	// Transmit Data
	USART2->DR = (Data & (uint16_t)0x01FF);

}

unsigned char receive() { // modified from USART_ReceiveData, hard-coded for USART2

	// Wait for data to be received
	while (!(USART2->SR & USART_SR_RXNE));

	// Get and return received data from buffer
	return (unsigned char)(USART2->DR & (uint16_t)0x01FF);

}

int main() {

	USART2init();

	for (int i = 0; i < strlen(welcomeMessage); i++) transmit(welcomeMessage[i]);

	char receivedInt[2];
	for (int i = 0; i < 2; i++) {
		receivedInt[i] = receive();
		transmit(receivedInt[i]);
	}
	transmit('\n');

	uint8_t setPulseWidth = (receivedInt[0] - 0x30) * 10 + (receivedInt[1] - 0x30);
	PWMinit(setPulseWidth);

	ICinit();

	while (1) {

	}
}

void TIM4_IRQHandler(void) {

	if (TIM4->DIER & TIM_DIER_CC1IE && TIM4->SR & TIM_SR_CC1IF) { // if CC1 interrupt enabled and flag set

		if (!pulseHigh) {
			TIM4->CNT = 0;
			TIM4->CCER |= TIM_CCER_CC1P; // change to detect falling

			pulseHigh = true; // high pulse starts

		} else {
			pulseWidth += TIM4->CNT;
			TIM4->CCER &= ~TIM_CCER_CC1P; // change to detect raising
			
			// convert pulseWidth int to charArray and report/transmit over UART
			char buffer[2];
			buffer[0] = (pulseWidth / 10) + 0x30;
			buffer[1] = (pulseWidth % 10) + 0x30;

			for (int i = 0; i < strlen(reportMessage); i++) transmit(reportMessage[i]);
			for (int i = 0; i < strlen(buffer); i++) transmit(buffer[i]);
			transmit('\r');

			pulseWidth = 0;
			pulseHigh = false; // high pulse ends (reset)
		}

	}

	//Clear interrupt flag
	TIM4->SR = ~TIM_SR_CC1IF;

}