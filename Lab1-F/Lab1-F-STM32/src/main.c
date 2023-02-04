// The SPL included in this project has to be added to PlatformIO following the instructions in this forum post:
// https://community.platformio.org/t/stm32-standard-library/7086/15
// Since it hasn't been added to the mainline
// Otherwise, it won't work

#include "stm32f1xx.h"
#include <string.h>

#define welcomeMessage 	("Enter integer between 10 and 99 to set as pulse width: ")
#define confirmMessage	("Pulse width has been set at ")

void transmit(uint8_t Data) { // modified from USART_SendData func, hard-coded for USART2

	// Wait for empty transmit buffer
	while (!(USART2->SR & USART_SR_TXE));

	// Transmit Data
	USART2->DR = (Data & (uint16_t)0x0FF);

}

char receive() { // modified from USART_ReceiveData, hard-coded for USART2

	// Wait for data to be received
	while (!(USART2->SR & USART_SR_RXNE));

	// Get and return received data from buffer
	return (char)(USART2->DR & (uint16_t)0x01FF);

}

void PWMinit(uint8_t width) { // PA6 TIM3_CH1, PWM1
	
	// GPIO setup
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;  // enable PORTA and AFIO in RCC_APB2

	// PA6 / Mode 10: Output 2MHz / Config 10: Alt Func Push Pull
	uint32_t tempGPIOACRL = GPIOA->CRL;
	tempGPIOACRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
	tempGPIOACRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;
	GPIOA->CRL = tempGPIOACRL;
	
	// Timer3 setup
	// 72,000,000 / 125 (TIM_Period) / 1152 (Prescaler) = 500Hz
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 in RCC_APB1

	TIM3->PSC = 1152 - 1;
	TIM3->ARR = 125 - 1;
	TIM3->CCR1 = width - 1; // Compare target
	
	// PWM setup
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; // Output Compare 1 Mode: 110/PWM1 & enable Preload
	TIM3->CCER |= TIM_CCER_CC1E; // CC1 output enable

	TIM3->EGR |= TIM_EGR_UG; // Update registers of TIM3
	TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable Clock
	

}

void USART2init(void) {

	// USART2 TX (PA2) RX (PA3)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; // enable PORTA and AFIO in RCC_APB2

	// PA2 / Mode 10: Output 2MHz / Config 10: Alt Func Push Pull
	// PA3 / Mode 00: Input / Config 01: Input Floating
	GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_0;
	
	// USART2 ST-LINK USB
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2 in RCC_APB1

	// USART2 / 115200 baud / 8 bit word length / 1 stop bit / no parity / no flow control / TX-RX mode
	// Fraction part (4-bit): 36,000,000Hz/Baud/16
	// Mantissa part (12-bit): the remainder of 36,000,000Hz/Baud/16 multiplied by 16
	USART2->BRR |= (19 << 4) | (9 & USART_BRR_DIV_Fraction);
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	
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

int power(int x, int y) {

	int returnValue = 1;
	for (int i = 0; i < y; i++) {
		returnValue = returnValue * x;
	}
	return returnValue;
}

int main() {

	clockSetup();
	USART2init();

	for (int i = 0; i < strlen(welcomeMessage); i++) transmit(welcomeMessage[i]);

	char receivedChar[2];
	uint8_t setPulseWidth = 0;
	for (int i = 1; i >= 0; i--) {
		receivedChar[i] = receive();
		setPulseWidth += (receivedChar[i] - 0x30) * power(10, i);
		transmit(receivedChar[i]);
	}

	PWMinit(setPulseWidth);

	transmit('\n');
	for (int i = 0; i < strlen(confirmMessage); i++) transmit(confirmMessage[i]);
	for (int i = 1; i >= 0; i--) transmit(receivedChar[i]);
	transmit('.');

	while (1);
}