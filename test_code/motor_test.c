#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) |
            (1 << PD5) | (1 << PD6);
    DDRC |= (1 << PC0);

    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Left motor forward
    PORTD |= (1 << PD2);
    PORTD &= ~(1 << PD3);

    // Right motor forward
    PORTD |= (1 << PD6);
    PORTC &= ~(1 << PC0);

    OCR1A = 255;
    OCR1B = 255;

    while (1);

    return 0;
}