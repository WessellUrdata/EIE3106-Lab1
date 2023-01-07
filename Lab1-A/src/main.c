#include <avr/io.h>

int main() {

/*
Clock config:
Compare Match Output A: 10: Clear OC0A on compare match, set OC0A at BOTTOM, (non-inverting mode)
Compare Match Output B: 10: Clear OC0B on compare match, set OC0B at BOTTOM, (non-inverting mode)
Waveform Generation Mode: 111: Mode 7, Fast PWM
Clock Select: 100: clk/256 (from prescaler)

1/16,000,000 * 256 (prescaler) * 125 (OCR0A) = 0.002s = 500Hz
*/

  TCCR0A = (0b10 << COM0A0) |
          (0b10 << COM0B0) |
          (0b11 << WGM00);  
  TCCR0B = (0b1 << WGM02) |
          (0b101 << CS00);

  OCR0A = 125 - 1; // 500Hz
  OCR0B = 63 - 1; // 50% duty cycle

  DDRD = (1 << PD5); // PD5 (OC0B)

  while (1);

}