#include <avr/io.h>

/*
Clock config:
Compare Match Output A: 00: Normal port operation, OC0A disconnected
Compare Match Output B: 10: Clear OC0B on compare match, set OC0B at BOTTOM, (non-inverting mode)
Waveform Generation Mode: 111: Mode 7, Fast PWM
Clock Select: 100: clk/256 (from prescaler)

1/16,000,000 * 256 (prescaler) * 125 (OCR0A) = 0.002s = 500Hz
*/

int main() {

  TCCR0A =
        (0b00 << COM0A0) |
        (0b10 << COM0B0) |
        (0b11 << WGM00);  
  TCCR0B =
        (0b1 << WGM02) |
        (0b100 << CS00);

  OCR0A = 125-1; // 500Hz (OCR0A = TOP)
  OCR0B = (125/2)-1; // 50% duty cycle (Update of OCR0B at BOTTOM)

  DDRD = (1 << PD5); // PD5 (OC0B)

  while (1);

}