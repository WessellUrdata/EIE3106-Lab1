#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/BAUD/16-1

#define reportMessage	("The pulse width is: ")

void usart_init(void) { // USART initialization

	// Set baud rate (9600) to Low register (value not large enough to use High reg)
	UBRR0L = MYUBRR;

	// Enable receiver and transmitter
	UCSR0B = (1 << TXEN0); // enable TX

	// Set frame format: Async, 8 data bits, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

}

void transmit(unsigned char data) { // code snippet modified from AVR datasheet
  
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));

	// Put data into buffer, sends the data
	UDR0 = data;

}

int main() {

	// initialize UART and transmit greeting message
	usart_init();

	TIMSK1 |= (1 << ICIE1);
	TCCR1B |= (1 << CS12); // capture rising edge, prescaler 256

	sei();

	while (1);

}

volatile static bool fallingEdge = false; // I'm not really sure which edge I'm capturing right now
volatile uint8_t captureValue = 0;
volatile char buffer[3];

ISR(TIMER1_CAPT_vect) {

	if (!fallingEdge) {

		TCCR1B &= ~(1 << ICES1); // capture falling edge
		captureValue = ICR1;
		
	}

	else {

		TCCR1B |= (1 << ICES1); // capture rising edge

		// store the value received from getPulseWidth and transmit over UART
		uint8_t pulseWidth = ICR1 - captureValue;

		buffer[0] = (pulseWidth / 10) + 0x30;
		buffer[1] = (pulseWidth % 10) + 0x30;
		buffer[2] = '\r';

		UCSR0B |= (1 << UDRIE0); // turn on UART TX interrupt

	}

	fallingEdge = !fallingEdge;


}

volatile uint8_t reportMessage_i = 0;
volatile uint8_t buffer_i = 0;

ISR(USART_UDRE_vect) {

	if (reportMessage_i < strlen(reportMessage)) {
		UDR0 = reportMessage[reportMessage_i];
		reportMessage_i++;
	}
	else if (reportMessage_i == strlen(reportMessage) && buffer_i < 3) {
		UDR0 = buffer[buffer_i];
		buffer_i++;
	}
	else {
		reportMessage_i = buffer_i = 0;
		_delay_ms(300); // this delay is a very crude hack to make the Arduino not print stuff when serial monitor connects, it really shouldn't be here
		UCSR0B &= ~(1 << UDRIE0); // turn off the interrupt
	}

}