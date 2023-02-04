// The SPL included in this project has to be added to PlatformIO following the instructions in this forum post:
// https://community.platformio.org/t/stm32-standard-library/7086/15
// Since it hasn't been added to the mainline
// Otherwise, it won't work

#include "stm32f1xx.h"
// #include "stm32f10x.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define reportMessage	("The pulse width is: ")

bool pulseHigh = false;
uint16_t pulseWidth = 0;


void transmit(uint16_t Data) { // hard-coded for USART2

	// Wait for empty transmit buffer
	while (!(USART2->SR & USART_SR_TXE));

	// Transmit Data
	USART2->DR = (Data & (uint16_t)0x01FF);

}

void USART2init(void) { // PA2 TX

	// USART2 TX-only (PA2)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; // enable PORTA and AFIO in RCC_APB2

	// PA2 / Mode 10: Output 2MHz / Config 10: Alt Func Push Pull
	GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;
	
	// USART2 ST-LINK USB
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2 in RCC_APB1

	// USART2 / 9600 baud / 8 bit word length / 1 stop bit / no parity / no flow control / TX mode
	// Fraction part (4-bit): 36,000,000Hz/Baud/16
	// Mantissa part (12-bit): the remainder of 36,000,000Hz/Baud/16 multiplied by 16
	USART2->BRR |= (234 << 4) | (6 & USART_BRR_DIV_Fraction);
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE;
	
}

void ICinit() { // PB6, 500Hz Timer Interrupt

	// PB6, Input Capture
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; // enable PORTB and AFIO in RCC_APB2

	// PB6 / Mode 10: Output 2MHz / Config 10: Alt Func Push Pull
	GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;

	// Timer4 setup
	// 72,000,000 / 125 (TIM_Period) / 1152 (Prescaler) = 500Hz
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 in RCC_APB1

	TIM4->PSC = 1152 - 1;
	TIM4->ARR = 125 - 1;
	TIM4->EGR |= TIM_EGR_UG; // Update registers of TIM4
	TIM4->CR1 |= TIM_CR1_CEN; // Enable Clock

	// IC1 setup
	// Capture Rising edge, DirectTI, No Prescaler, No Filter
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0; // CC1 select: input, IC1 to TI1 (directTI)
	TIM4->CCER |= TIM_CCER_CC1E; // Enable CC1 Capture

	// this is a mess that I don't want to touch anymore

	IRQn_Type NVIC_IRQChannel = TIM4_IRQn; // TIM4 interrupt
	int NVIC_IRQChannelPreemptionPriority = 2; // Preemptive priority level 2
	int NVIC_IRQChannelSubPriority = 0; // From the priority level 0

  	uint32_t tmppriority = 0, tmppre = 0, tmpsub = 0x0F;

    /* Compute the Corresponding IRQ Priority --------------------------------*/
    tmppriority = (uint32_t)NVIC_GetPriorityGrouping; // read priority PRIGROUP from AIRCR, then shift it back
    tmppre = (4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = (uint32_t)NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |= NVIC_IRQChannelSubPriority & tmpsub;
    tmppriority = tmppriority << 4;
        
    NVIC->IP[NVIC_IRQChannel] = tmppriority;

    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[NVIC_IRQChannel >> 5] = 1 << (NVIC_IRQChannel & (uint8_t)0x1F);

	TIM4->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // Update Interrupt Enable & CC1 Interrupt Enable

}

void clockSetup() {
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV2 ; // PLL multiply by 9 = 72MHz, PLL clock source = HSE, APB1 = 36MHz
	RCC->CR |= RCC_CR_HSEON; // enable HSE clock
	while (!(RCC->CR & RCC_CR_HSERDY)); // wait until HSE ready
	RCC->CR |= RCC_CR_PLLON; // enable PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)); // wait until PLL ready
	FLASH->ACR |= FLASH_ACR_LATENCY_2; // flash latency = 2 @ 72MHz
	RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // wait until PLL is selected as system clock
	RCC->CR &= ~RCC_CR_HSION; // turn off internal 8MHz internal high-speed clock
}

int main() {

	clockSetup();

	USART2init();
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