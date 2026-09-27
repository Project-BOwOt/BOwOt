#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define CLIFF_LEVEL       1u
#define STABLE_SAMPLES    5u

void motors_init(void)
{
    DDRD |= (1 << PD2) |
            (1 << PD3) |
            (1 << PD4) |
            (1 << PD5) |
            (1 << PD6);

    DDRC |= (1 << PC0);

    // Timer1: 8-bit Fast PWM
    TCCR1A = (1 << COM1A1) |
             (1 << COM1B1) |
             (1 << WGM10);

    TCCR1B = (1 << WGM12) |
             (1 << CS11);
}

void motors_forward(uint8_t speedA, uint8_t speedB)
{
    // Motor A: IN1=HIGH, IN2=LOW
    PORTD |=  (1 << PD2);
    PORTD &= ~(1 << PD3);

    // Motor B requires opposite electrical direction
    // IN3=LOW, IN4=HIGH
    PORTD &= ~(1 << PD6);
    PORTC |=  (1 << PC0);

    // ENA and ENB
    OCR1A = speedA;
    OCR1B = speedB;
}

void motors_stop(void)
{
    // Disable both motors
    OCR1A = 0;
    OCR1B = 0;

    // Clear direction inputs
    PORTD &= ~((1 << PD2) |
               (1 << PD3) |
               (1 << PD6));

    PORTC &= ~(1 << PC0);
}

void sensor_init(void)
{
    // PB0 input with internal pull-up
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);
}

uint8_t cliff_detected(void)
{
    // HIGH means no surface/cliff
    return (PINB & (1 << PB0)) != 0;
}

int main(void)
{
    uint8_t cliff_samples = 0;
    uint8_t surface_samples = 0;
    uint8_t motors_running = 0;

    motors_init();
    sensor_init();
    motors_stop();

    while (1)
    {
        if (cliff_detected())
        {
            surface_samples = 0;

            if (cliff_samples < STABLE_SAMPLES)
                cliff_samples++;

            /*
             * Stop after approximately 10 ms of continuous
             * cliff detection.
             */
            if (cliff_samples >= STABLE_SAMPLES &&
                motors_running)
            {
                motors_stop();
                motors_running = 0;
            }
        }
        else
        {
            cliff_samples = 0;

            if (surface_samples < STABLE_SAMPLES)
                surface_samples++;

            /*
             * Restart after approximately 10 ms of continuous
             * surface detection.
             */
            if (surface_samples >= STABLE_SAMPLES &&
                !motors_running)
            {
                motors_forward(255, 255);
                motors_running = 1;
            }
        }

        _delay_ms(2);
    }
}
