#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/BAUD/16-1

void usart_init(void) { // USART initialization

  // Set baud rate (9600) to High and Low register
  UBRR0H = (unsigned char)((MYUBRR) >> (8));
  UBRR0L = (unsigned char)((MYUBRR) & 0xFF);

  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0); 

  // Set frame format: Async, 8 data bits, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

}

void transmit(unsigned char data) { // code snippet modified from AVR datasheet
  
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));
	
	// Put data into buffer, sends the data
	UDR0 = data;

}

unsigned char receive(void) { // code snippet modified from AVR datasheet

	// Wait for data to be received
	while (!(UCSR0A & (1 << RXC0)));

	// Get and return received data from buffer
	return UDR0;

}

unsigned int getPulseWidth(int width) { // measure the pulse width of a pulse (count from rising edge to falling edge)

	TCCR1A = 0; // normal mode
	TCCR1B = (1 << ICES1) | (TCCR0B & 0b00000111); // capture rising edge, same as prescaler as Timer0

	TIFR1 = (1 << ICF1); // clear ICF1 (The Input Capture Flag)
	while (!(TIFR1 & (1 << ICF1))); // wait while ICF1 is clear

	// get the TCNT count at input capture of rising edge
	// For a 16-bit read, the low byte must be read before the high byte. (ATmega328p Datasheet p91)
	unsigned int risingEdgeCount = ICR1L;
	risingEdgeCount += (ICR1H << 8);
	TIFR1 = (1 << ICF1); // clear ICF1

	TCCR1B = (0 << ICES1) | (TCCR0B & 0b00000111); // capture falling edge, same prescaler as Timer0
	while (!(TIFR1 & (1 << ICF1))); // wait while ICF1 is clear

	// get the TCNT count at input capture of falling edge
	unsigned int fallingEdgeCount = ICR1L;
	fallingEdgeCount += (ICR1H << 8);
	TIFR1 = (1 << ICF1); // clear ICF1

	return fallingEdgeCount - risingEdgeCount; // return pulse width

}

int main() {

	// initialize UART and transmit greeting message
	usart_init();
	
	char welcomeMessage[] = "Enter integer between 10 and 99 to set as pulse width: ";
	for (int i = 0; i < strlen(welcomeMessage); i++) {
		transmit(welcomeMessage[i]);
	}

	// receive 2 digits (and echo them)
	char widthString[2];
	for (int i = 0; i < 2; i++) {
		widthString[i] = receive();
		transmit(widthString[i]);
	}
	unsigned int width = atoi(widthString);

	transmit('\n');

	/*
	Clock config:
	Compare Match Output A: 00: Normal port operation, OC0A disconnected
	Compare Match Output B: 10: Clear OC0B on compare match, set OC0B at BOTTOM, (non-inverting mode)
	Waveform Generation Mode: 111: Mode 7, Fast PWM
	Clock Select: 100: clk/256 (from prescaler)

	1/16,000,000 * 256 (prescaler) * 125 (OCR0A) = 0.002s = 500Hz
	*/

  TCCR0A =
		(0b00 << COM0A0) |
		(0b10 << COM0B0) |
		(0b11 << WGM00);  
  TCCR0B =
		(0b1 << WGM02) |
		(0b100 << CS00);

	OCR0A = 125 - 1; // 500Hz (OCR0A = TOP)
  OCR0B = width - 1; // 50% duty cycle (Update of OCR0B at BOTTOM)

  DDRD = (1 << PD5); // PD5 (OC0B)

	while (1) {

		// store the value received from getPulseWidth and transmit over UART
		char t1array[3];
		itoa(getPulseWidth(width), t1array, 10);

		for (int i = 0; i < strlen(t1array); i++) {
			transmit(t1array[i]);
		}
		transmit('\n');
		
		_delay_ms(500);
		
	}

}