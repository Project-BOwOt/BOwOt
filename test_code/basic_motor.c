#define F_CPU 1000000UL
#include <avr/io.h>

int main(void)
{
    // Direction pins
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) |
            (1 << PD5) | (1 << PD6);
    DDRC |= (1 << PC0);

    // Timer1 Fast PWM, 8-bit
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Left motor: AIN1=1, AIN2=0
    PORTD |= (1 << PD2);
    PORTD &= ~(1 << PD3);

    // Right motor: BIN1=1, BIN2=0
    PORTD |= (1 << PD6);
    PORTC &= ~(1 << PC0);

    // Full speed
    OCR1A = 255;
    OCR1B = 255;

    while (1)
    {
    }

    return 0;
}