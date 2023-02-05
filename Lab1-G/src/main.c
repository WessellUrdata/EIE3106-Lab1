#include <avr/io.h>

/*
Clock config:
Compare Match Output A: 00: Normal port operation, OCxA disconnected
Compare Match Output B: 10: Clear OCxA on compare match, set OCxB at BOTTOM, (non-inverting mode)
Waveform Generation Mode: 111: Mode 7, Fast PWM / toggle WGM13 in Timer1 for the equivalent mode (Mode 15)
Clock Select: 100: clk/256 (from prescaler) / 110 in Timer2 for equivalent setting

1/16,000,000 * 256 (prescaler) * 125 (OCRxA) = 0.002s = 500Hz

Red = Timer0 = OC0B = PD5
Green = Timer1 = OC1B = PB2
Blue = Timer2 = OC2B = PD3
*/

void TimersSetup() {

	TCCR0A = TCCR1A = TCCR2A = (1 << COM0B1) | (0b11 << WGM00); // common settings
	TCCR0B = TCCR1B = TCCR2B = (0b11 << WGM02) | (0b100 << CS00); // common settings
	TCCR2B |= (1 << CS21);

	OCR0A = OCR1A = OCR2A = 125-1; // 500Hz (OCR0A = TOP)

	DDRD = (1 << PD5) | (1 << PD3); // PD5 (OC0B, Red), PD3 (OC2B, Blue)
	DDRB = (1 << PB2); // PB2 (OC1B, Green)

}

int main() {

	TimersSetup();

	while (1) {
		for (int i = 1; i < 125 * 262; i++) {
			OCR0B = i / 262;
		}
		for (int i = 125 * 262; i >= 0; i--) {
			OCR0B = i / 262;
		}
		for (int i = 1; i < 125 * 262; i++) {
			OCR1B = i / 262;
		}
		for (int i = 125 * 262; i >= 0; i--) {
			OCR1B = i / 262;
		}
		for (int i = 1; i < 125 * 262; i++) {
			OCR2B = i / 262;
		}
		for (int i = 125 * 262; i >= 0; i--) {
			OCR2B = i / 262;
		}
	}

}