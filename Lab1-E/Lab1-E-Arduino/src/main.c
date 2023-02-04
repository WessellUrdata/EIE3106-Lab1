#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/BAUD/16-1

#define welcomeMessage	("Enter integer between 10 and 99 to set as pulse width: ")
#define confirmMessage	("Pulse width has been set at ")

// PWM pulse from Arduino Uno

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

int main() {

	usart_init();
	
	// send greeting message

	for (int i = 0; i < strlen(welcomeMessage); i++) transmit(welcomeMessage[i]);

	// receive 2 digits (and echo them)
	char widthString[2];
	for (int i = 0; i < 2; i++) {
		widthString[i] = receive();
		transmit(widthString[i]);
	}
	uint8_t width = (widthString[0] - 0x30) * 10 + (widthString[1] - 0x30);

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

	// send confirm message
	for (int i = 0; i < strlen(confirmMessage); i++) transmit(confirmMessage[i]);
	for (int i = 0; i < 2; i++) transmit(widthString[i]);
	transmit('.');

	while(1) {}

}