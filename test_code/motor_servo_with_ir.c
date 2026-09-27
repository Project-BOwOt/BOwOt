#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// IR logic already confirmed
#define STABLE_SAMPLES 5u

/*
 * Timer2 tick:
 * 1 MHz / 8 = 125 kHz
 * One tick = 8 microseconds
 *
 * 125 ticks = 1000 us  -> approximately 0 degrees
 * 188 ticks = 1504 us  -> approximately 90 degrees
 */
#define SERVO_HOME_TICKS   125u
#define SERVO_CLIFF_TICKS  188u

#define SERVO_FRAME_OVERFLOWS 10u

#define STATE_UNKNOWN 0u
#define STATE_SURFACE 1u
#define STATE_CLIFF   2u

volatile uint8_t servo_target_ticks = SERVO_HOME_TICKS;
volatile uint8_t servo_frame_counter = 9;
volatile uint8_t servo_pulse_active = 0;

/* ---------------- Motor control ---------------- */

void motors_init(void)
{
    /*
     * PD2 = IN1
     * PD3 = IN2
     * PD4 = ENB / OC1B
     * PD5 = ENA / OC1A
     * PD6 = IN3
     */
    DDRD |= (1 << PD2) |
            (1 << PD3) |
            (1 << PD4) |
            (1 << PD5) |
            (1 << PD6);

    // PC0 = IN4
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

    /*
     * Motor B requires the opposite electrical direction:
     * IN3=LOW, IN4=HIGH
     */
    PORTD &= ~(1 << PD6);
    PORTC |=  (1 << PC0);

    OCR1A = speedA;
    OCR1B = speedB;
}

void motors_stop(void)
{
    OCR1A = 0;
    OCR1B = 0;

    PORTD &= ~((1 << PD2) |
               (1 << PD3) |
               (1 << PD6));

    PORTC &= ~(1 << PC0);
}

/* ---------------- IR sensor ---------------- */

void sensor_init(void)
{
    // PB0 input with pull-up
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);
}

uint8_t cliff_detected(void)
{
    /*
     * Confirmed behavior:
     * PB0 LOW  = surface
     * PB0 HIGH = cliff/no surface
     */
    return (PINB & (1 << PB0)) != 0;
}

/* ---------------- Servo control ---------------- */

void servos_init(void)
{
    // PB1 = SG90 signal
    // PB2 = SG92 signal
    DDRB |= (1 << PB1) | (1 << PB2);

    // Both signals initially LOW
    PORTB &= ~((1 << PB1) | (1 << PB2));

    servo_target_ticks = SERVO_HOME_TICKS;
    servo_frame_counter = 9;
    servo_pulse_active = 0;

    /*
     * Timer2 synchronous normal mode.
     * Prescaler = 8.
     */
    ASSR &= ~(1 << AS2);

    TCCR2 = 0;
    TCNT2 = 0;
    OCR2 = SERVO_HOME_TICKS;

    // Clear pending Timer2 flags
    TIFR = (1 << TOV2) | (1 << OCF2);

    // Enable Timer2 overflow and compare interrupts
    TIMSK |= (1 << TOIE2) | (1 << OCIE2);

    // Start Timer2 with clock/8
    TCCR2 = (1 << CS21);
}

void servos_home(void)
{
    servo_target_ticks = SERVO_HOME_TICKS;
}

void servos_turn_90(void)
{
    servo_target_ticks = SERVO_CLIFF_TICKS;
}

/*
 * Timer2 overflows every:
 * 256 × 8 us = 2.048 ms
 *
 * Ten overflows create a 20.48 ms servo frame.
 */
ISR(TIMER2_OVF_vect)
{
    servo_frame_counter++;

    if (servo_frame_counter >= SERVO_FRAME_OVERFLOWS)
    {
        servo_frame_counter = 0;

        // Apply the currently requested pulse duration
        OCR2 = servo_target_ticks;

        // Start pulse for both servos simultaneously
        PORTB |= (1 << PB1) | (1 << PB2);

        servo_pulse_active = 1;
    }
}

/*
 * End both servo pulses when Timer2 reaches OCR2.
 */
ISR(TIMER2_COMP_vect)
{
    if (servo_pulse_active)
    {
        PORTB &= ~((1 << PB1) | (1 << PB2));
        servo_pulse_active = 0;
    }
}

/* ---------------- Main program ---------------- */

int main(void)
{
    uint8_t cliff_samples = 0;
    uint8_t surface_samples = 0;
    uint8_t system_state = STATE_UNKNOWN;

    motors_init();
    sensor_init();
    servos_init();

    motors_stop();
    servos_home();

    // Enable global interrupts for Timer2 servo generation
    sei();

    while (1)
    {
        if (cliff_detected())
        {
            surface_samples = 0;

            if (cliff_samples < STABLE_SAMPLES)
                cliff_samples++;

            if (cliff_samples >= STABLE_SAMPLES &&
                system_state != STATE_CLIFF)
            {
                // Cliff: stop motors and turn both servos 90 degrees
                motors_stop();
                servos_turn_90();

                system_state = STATE_CLIFF;
            }
        }
        else
        {
            cliff_samples = 0;

            if (surface_samples < STABLE_SAMPLES)
                surface_samples++;

            if (surface_samples >= STABLE_SAMPLES &&
                system_state != STATE_SURFACE)
            {
                // Surface: restore servos and run both motors
                servos_home();
                motors_forward(255, 255);

                system_state = STATE_SURFACE;
            }
        }

        _delay_ms(2);
    }
}
