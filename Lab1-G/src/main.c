#include <avr/io.h>
#include <avr/interrupt.h>

void TimersSetup() {

	/*
	Clock config:
	Compare Match Output A: 00: Normal port operation, OCxA disconnected
	Compare Match Output B: 10: Clear OCxA on compare match, set OCxB at BOTTOM, (non-inverting mode)
	Waveform Generation Mode: 111: Mode 7, Fast PWM / toggle WGM13 in Timer1 for equivalent mode (Mode 15)
	Clock Select: 100: clk/256 (from prescaler) / 110 in Timer2 for equivalent setting

	1/16,000,000 * 256 (prescaler) * 125 (OCRxA) = 0.002s = 500Hz

	Red = Timer0 = OC0B = PD5
	Green = Timer1 = OC1B = PB2
	Blue = Timer2 = OC2B = PD3
	*/

	TCCR0A = TCCR1A = TCCR2A = (1 << COM0B1) | (0b11 << WGM00); // common settings
	TCCR0B = TCCR1B = TCCR2B = (0b11 << WGM02) | (0b100 << CS00); // common settings
	TCCR2B |= (1 << CS21);

	OCR0A = OCR1A = OCR2A = 125-1; // 500Hz (OCR0A = TOP)

	DDRD = (1 << PD5) | (1 << PD3); // PD5 (OC0B, Red), PD3 (OC2B, Blue)
	DDRB = (1 << PB2); // PB2 (OC1B, Green)

}

void ADCSetup() {
	// AREF is hooked up to 5V
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (0b111 << ADPS0); // enable ADC, ADC Interrupt and set prescaler to 128
	ADCSRA |= (1 << ADSC);
}

enum LED {
	RED,
	GREEN,
	BLUE
};

volatile enum LED currentLED = RED;
ISR(ADC_vect) {

	uint8_t OCRvalue = (ADC > 400) ? 125 : 0; // threshhold at 300, max is around 900 for me

	switch (currentLED) {
		case RED: 
			OCR0B = OCRvalue;
			ADMUX = currentLED = GREEN;
			break;
		case GREEN:
			OCR1B = OCRvalue;
			ADMUX = currentLED = BLUE;
			break;
		case BLUE:
			OCR2B = OCRvalue;
			ADMUX = currentLED = RED;
			break;
	}

	ADCSRA |= (1 << ADSC);

}


int main() {

	TimersSetup();
	ADCSetup();
	sei();

	while (1) {
	}

}
