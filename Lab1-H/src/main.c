// The SPL included in this project has to be added to PlatformIO following the instructions in this forum post:
// https://community.platformio.org/t/stm32-standard-library/7086/15
// Since it hasn't been added to the mainline
// Otherwise, it won't work

#include "stm32f1xx.h"
#include <string.h>

void USART2init(void) {

	// USART2 TX (PA2) RX (PA3)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; // enable PORTA and AFIO in RCC_APB2

	// PA2 / Mode 10: Output 2MHz / Config 10: Alt Func Push Pull
	// PA3 / Mode 00: Input / Config 01: Input Floating
	uint32_t tempGPIOACRL = GPIOA->CRL;
	tempGPIOACRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2 | GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
	tempGPIOACRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_0;
	GPIOA->CRL = tempGPIOACRL;
	
	// USART2 ST-LINK USB
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2 in RCC_APB1

	// USART2 / 115200 baud / 8 bit word length / 1 stop bit / no parity / no flow control / TX-RX mode
	// Fraction part (4-bit): 36,000,000Hz/Baud/16
	// Mantissa part (12-bit): the remainder of 36,000,000Hz/Baud/16 multiplied by 16
	USART2->BRR |= (19 << 4) | (9 & USART_BRR_DIV_Fraction);
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	
}

void transmit(uint8_t Data) { // hard-coded for USART2

	// Wait for empty transmit buffer
	while (!(USART2->SR & USART_SR_TXE));

	// Transmit Data
	USART2->DR = (Data & 0xFF);

}

/*
Pins for PWM:
TIM3_CH1: PA6 / Red
TIM3_CH2: PA7 / Green
TIM3_CH3: PB0 / Blue
*/
void PWMinit(uint8_t width) {
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;  // enable PORTA, PORTB and AFIO in RCC_APB2

	// PA6, PA7, PB0
	// Mode 10: Output 2MHz / Config 10: Alt Func Push Pull

	uint32_t tempGPIOACRL = GPIOA->CRL;
	tempGPIOACRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
	tempGPIOACRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1; 
	GPIOA->CRL = tempGPIOACRL;

	uint32_t tempGPIOBCRL = GPIOB->CRL;
	tempGPIOBCRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
	tempGPIOBCRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1; 
	GPIOB->CRL = tempGPIOBCRL;
	
	// Timer3 setup
	// 72,000,000 / 125 (TIM_Period) / 1152 (Prescaler) = 500Hz
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 in RCC_APB1

	TIM3->PSC = 1152 - 1;
	TIM3->ARR = 125 - 1;
	TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = width - 1; // Compare target
	
	// PWM setup
	// For TIM3_CH1, TIM3_CH2, TIM3_CH3: Output Compare 1 Mode: 110/PWM1 & enable Preload
	TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
	TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; // CC1/CC2/CC3 output enable

	TIM3->EGR |= TIM_EGR_UG; // Update registers of TIM3
	TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable Clock
	

}

/*
Pins for ADC:
ADC_0: PA0 / Red
ADC_1: PA1 / Green
ADC_4: PA4 / Blue
*/
void ADCinit() {

	// set prescaler for ADC
	// ADC clock cannot exceed 14MHz, so we'll select a prescaler of 6 (72MHz/6 = 12MHz)
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

	// enable ADC1 in APB2
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Reset ADC1 to its power-on default
	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;

	// PA0, PA1, PA4
	// Mode 00: Input / Config 00: Analog In
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1 | GPIO_CRL_MODE4 | GPIO_CRL_CNF4);

	// setup ADC
	// Dual mode disabled, Scan mode enable
	uint32_t tempADC1CR1 = ADC1->CR1;
	tempADC1CR1 &= ~(ADC_CR1_DUALMOD);
	tempADC1CR1 |= ADC_CR1_SCAN;
	ADC1->CR1 = tempADC1CR1;

	// Continuous Conversion enable, External trigger disabled (SWSTART), DMA enabled
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART | ADC_CR2_DMA;

	// 3 regular channels
	ADC1->SQR1 |= (3-1 << ADC_SQR1_L_Pos);

	// Set sampling time at 239.5 cycles for ADC_0, ADC_1, ADC_4
	ADC1->SMPR2 = (ADC_SMPR2_SMP0_Msk | ADC_SMPR2_SMP1_Msk | ADC_SMPR2_SMP1_Msk);

	// Set ADC conversion sequence
	// Red (ADC_0) first, Green (ADC_1) second, Blue (ADC_4) third
	ADC1->SQR3 = (0 << ADC_SQR3_SQ1_Pos) | (1 << ADC_SQR3_SQ2_Pos) | (4 << ADC_SQR3_SQ3_Pos);

	// Turn on the ADC
	ADC1->CR2 |= ADC_CR2_ADON;

	// Calibration
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while (ADC1->CR2 & ADC_CR2_RSTCAL); // while it's still resetting calibration
	ADC1->CR2 |= ADC_CR2_CAL;
	while (ADC1->CR2 & ADC_CR2_CAL); // while it's still calibrating

}

void clockSetup() {
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV2 ; // PLL * 9 = 72MHz, PLL clock source = HSE, APB1 = 72MHz / 2 = 36MHz
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

	// clockSetup();
	USART2init();

	PWMinit(120);

	while (1);
}