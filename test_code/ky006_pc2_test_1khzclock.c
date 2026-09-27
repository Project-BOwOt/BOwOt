/*
 * ATmega32 + KY-006 passive buzzer test
 * Buzzer signal: PC2 (DIP pin 24)
 * Actual MCU clock: 1 MHz
 *
 * IMPORTANT:
 * The KY-006 is PASSIVE, so it needs a square-wave signal.
 * This test generates ~2 kHz, which is near the buzzer's useful range.
 */

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define BUZZER_PIN PC2

static void buzzer_tone(uint16_t frequency, uint16_t duration_ms)
{
    /*
     * Half-period in microseconds:
     * T/2 = 500000 / frequency
     *
     * At 2 kHz:
     * half-period = 250 us
     */
    uint16_t half_period_us = (uint16_t)(500000UL / frequency);
    uint32_t cycles = ((uint32_t)duration_ms * 1000UL) /
                      (uint32_t)(half_period_us * 2UL);

    while (cycles--) {
        PORTC |= _BV(BUZZER_PIN);
        _delay_us(250);   /* 2 kHz: HIGH for 250 us */

        PORTC &= (uint8_t)~_BV(BUZZER_PIN);
        _delay_us(250);   /* 2 kHz: LOW for 250 us */
    }
}

int main(void)
{
    /*
     * PC2 is also JTAG TCK on ATmega32.
     * Disable JTAG so PC2 works as GPIO.
     */
    MCUCSR |= _BV(JTD);
    MCUCSR |= _BV(JTD);

    DDRC |= _BV(BUZZER_PIN);
    PORTC &= (uint8_t)~_BV(BUZZER_PIN);

    while (1) {
        /* Loud test tone near KY-006 resonance */
        buzzer_tone(2000, 1000);

        /* Short pause */
        _delay_ms(500);

        /* Repeat */
    }
}
