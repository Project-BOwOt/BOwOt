#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/* =========================================================
 * Configuration
 * ========================================================= */

#define IR_STABLE_SAMPLES       5u

// Adjustable obstacle distance: keep between 2 and 35 cm
#define OBSTACLE_DISTANCE_CM    20u

// One ultrasonic measurement approximately every 60 ms
#define ULTRASONIC_PERIOD_LOOPS 30u

// Require two clear readings before restarting
#define CLEAR_READINGS_REQUIRED 2u

/*
 * Timer0:
 * 1 MHz / 8 = 125 kHz
 * One timer tick = 8 microseconds
 *
 * Echo time = distance_cm × 58 microseconds
 */
#define TIMER0_TICK_US 8UL

#define OBSTACLE_THRESHOLD_TICKS \
    ((OBSTACLE_DISTANCE_CM * 58UL + 7UL) / TIMER0_TICK_US)

#define MINIMUM_VALID_TICKS \
    ((2UL * 58UL + 7UL) / TIMER0_TICK_US)

#if OBSTACLE_DISTANCE_CM > 35
#error "OBSTACLE_DISTANCE_CM must not exceed 35 with this Timer0 setup"
#endif

/* Servo pulse settings */

#define SERVO_HOME_TICKS   125u  // Approximately 1.0 ms
#define SERVO_CLIFF_TICKS  188u  // Approximately 1.5 ms
#define SERVO_FRAME_COUNT  10u

/* IR states */

#define IR_STATE_UNKNOWN 0u
#define IR_STATE_SURFACE 1u
#define IR_STATE_CLIFF   2u

/* Robot actions */

#define ACTION_UNKNOWN   0u
#define ACTION_DRIVE     1u
#define ACTION_OBSTACLE  2u
#define ACTION_CLIFF     3u

/* =========================================================
 * Servo variables
 * ========================================================= */

volatile uint8_t servo_target_ticks = SERVO_HOME_TICKS;
volatile uint8_t servo_frame_counter = 9u;
volatile uint8_t servo_pulse_active = 0u;

/* =========================================================
 * Motor control
 * ========================================================= */

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

    // Motor B uses opposite electrical direction
    // IN3=LOW, IN4=HIGH
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

/* =========================================================
 * IR cliff sensor
 * ========================================================= */

void ir_sensor_init(void)
{
    // PB0 input with internal pull-up
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);
}

uint8_t cliff_detected(void)
{
    /*
     * PB0 LOW  = surface
     * PB0 HIGH = cliff
     */
    return (PINB & (1 << PB0)) != 0;
}

/* =========================================================
 * Servo control using Timer2
 * ========================================================= */

void servos_init(void)
{
    // PB1 = SG90 signal
    // PB2 = SG92 signal
    DDRB |= (1 << PB1) | (1 << PB2);

    PORTB &= ~((1 << PB1) | (1 << PB2));

    servo_target_ticks = SERVO_HOME_TICKS;
    servo_frame_counter = 9u;
    servo_pulse_active = 0u;

    // Timer2 synchronous normal mode
    ASSR &= ~(1 << AS2);

    TCCR2 = 0;
    TCNT2 = 0;
    OCR2 = SERVO_HOME_TICKS;

    // Clear pending Timer2 flags
    TIFR = (1 << TOV2) | (1 << OCF2);

    // Enable Timer2 overflow and compare interrupts
    TIMSK |= (1 << TOIE2) | (1 << OCIE2);

    // Timer2 clock divided by 8
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

ISR(TIMER2_OVF_vect)
{
    servo_frame_counter++;

    if (servo_frame_counter >= SERVO_FRAME_COUNT)
    {
        servo_frame_counter = 0;

        OCR2 = servo_target_ticks;

        // Begin both servo pulses
        PORTB |= (1 << PB1) | (1 << PB2);
        servo_pulse_active = 1u;
    }
}

ISR(TIMER2_COMP_vect)
{
    if (servo_pulse_active)
    {
        // End both servo pulses
        PORTB &= ~((1 << PB1) | (1 << PB2));
        servo_pulse_active = 0u;
    }
}

/* =========================================================
 * HC-SR04 ultrasonic sensor using Timer0
 * ========================================================= */

void ultrasonic_init(void)
{
    // PC1 = TRIG output
    DDRC |= (1 << PC1);
    PORTC &= ~(1 << PC1);

    // PD0 = ECHO input, pull-up disabled
    DDRD &= ~(1 << PD0);
    PORTD &= ~(1 << PD0);

    /*
     * Timer0 normal mode, clock divided by 8.
     * One timer tick = 8 microseconds.
     */
    TCCR0 = (1 << CS01);
    TCNT0 = 0;

    // Clear Timer0 overflow flag
    TIFR = (1 << TOV0);
}

void ultrasonic_trigger(void)
{
    uint8_t saved_sreg = SREG;

    /*
     * Temporarily disable interrupts so the trigger pulse
     * remains exactly timed.
     */
    cli();

    PORTC &= ~(1 << PC1);
    _delay_us(2);

    PORTC |= (1 << PC1);
    _delay_us(10);

    PORTC &= ~(1 << PC1);

    SREG = saved_sreg;
}

uint8_t wait_for_echo_start(void)
{
    uint8_t overflow_count = 0;

    TCNT0 = 0;
    TIFR = (1 << TOV0);

    /*
     * Wait approximately 4 ms for ECHO to become HIGH.
     */
    while (!(PIND & (1 << PD0)))
    {
        if (TIFR & (1 << TOV0))
        {
            TIFR = (1 << TOV0);
            overflow_count++;

            if (overflow_count >= 2u)
                return 0;
        }
    }

    return 1;
}

uint8_t ultrasonic_obstacle_near(void)
{
    uint8_t pulse_ticks;

    /*
     * ECHO should normally be LOW before starting.
     * If it is stuck HIGH, stop as a safety precaution.
     */
    if (PIND & (1 << PD0))
        return 1;

    ultrasonic_trigger();

    if (!wait_for_echo_start())
    {
        // No echo means no nearby obstacle
        return 0;
    }

    // Begin measuring the HIGH pulse
    TCNT0 = 0;
    TIFR = (1 << TOV0);

    while (PIND & (1 << PD0))
    {
        /*
         * Once the pulse exceeds the chosen distance,
         * the object is outside the stopping range.
         */
        if (TCNT0 > OBSTACLE_THRESHOLD_TICKS)
            return 0;

        if (TIFR & (1 << TOV0))
        {
            // Any overflow is beyond the supported threshold
            TIFR = (1 << TOV0);
            return 0;
        }
    }

    pulse_ticks = TCNT0;

    /*
     * Reject unrealistically short glitches.
     */
    if (pulse_ticks < MINIMUM_VALID_TICKS)
        return 0;

    return pulse_ticks <= OBSTACLE_THRESHOLD_TICKS;
}

/* =========================================================
 * Main program
 * ========================================================= */

int main(void)
{
    uint8_t cliff_samples = 0;
    uint8_t surface_samples = 0;
    uint8_t ir_state = IR_STATE_UNKNOWN;

    uint8_t ultrasonic_counter = ULTRASONIC_PERIOD_LOOPS;
    uint8_t obstacle_present = 0;
    uint8_t clear_readings = 0;

    uint8_t applied_action = ACTION_UNKNOWN;
    uint8_t desired_action;

    motors_init();
    ir_sensor_init();
    servos_init();
    ultrasonic_init();

    motors_stop();
    servos_home();

    // Enable Timer2 servo interrupts
    sei();

    while (1)
    {
        /* ---------- Update cliff state ---------- */

        if (cliff_detected())
        {
            surface_samples = 0;

            if (cliff_samples < IR_STABLE_SAMPLES)
                cliff_samples++;

            if (cliff_samples >= IR_STABLE_SAMPLES)
                ir_state = IR_STATE_CLIFF;
        }
        else
        {
            cliff_samples = 0;

            if (surface_samples < IR_STABLE_SAMPLES)
                surface_samples++;

            if (surface_samples >= IR_STABLE_SAMPLES)
                ir_state = IR_STATE_SURFACE;
        }

        /* ---------- Update obstacle state ---------- */

        ultrasonic_counter++;

        if (ultrasonic_counter >= ULTRASONIC_PERIOD_LOOPS)
        {
            ultrasonic_counter = 0;

            if (ultrasonic_obstacle_near())
            {
                // Stop immediately after one valid close reading
                obstacle_present = 1;
                clear_readings = 0;
            }
            else
            {
                /*
                 * Require two clear measurements before restarting
                 * to avoid motor jitter around the threshold.
                 */
                if (clear_readings < CLEAR_READINGS_REQUIRED)
                    clear_readings++;

                if (clear_readings >= CLEAR_READINGS_REQUIRED)
                    obstacle_present = 0;
            }
        }

        /* ---------- Select action by priority ---------- */

        if (ir_state == IR_STATE_CLIFF)
        {
            // Highest priority
            desired_action = ACTION_CLIFF;
        }
        else if (ir_state == IR_STATE_SURFACE &&
                 obstacle_present)
        {
            desired_action = ACTION_OBSTACLE;
        }
        else if (ir_state == IR_STATE_SURFACE)
        {
            desired_action = ACTION_DRIVE;
        }
        else
        {
            // Sensor state not established yet
            desired_action = ACTION_UNKNOWN;
        }

        /* ---------- Apply only when state changes ---------- */

        if (desired_action != applied_action)
        {
            switch (desired_action)
            {
                case ACTION_DRIVE:
                    servos_home();
                    motors_forward(255, 255);
                    break;

                case ACTION_OBSTACLE:
                    // Obstacle only stops the motors
                    servos_home();
                    motors_stop();
                    break;

                case ACTION_CLIFF:
                    // Cliff stops motors and turns servos
                    motors_stop();
                    servos_turn_90();
                    break;

                default:
                    motors_stop();
                    servos_home();
                    break;
            }

            applied_action = desired_action;
        }

        _delay_ms(2);
    }
}
