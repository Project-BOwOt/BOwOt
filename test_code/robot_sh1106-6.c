/*
 * Robot firmware: priority FSM + directional/searching gaze, 2026-09-11.
 * ATmega32, actual CPU clock 1 MHz; defining F_CPU does not set fuse bits.
 * Wiring is unchanged from the working touch-enabled SH1106 version:
 *   L298N ENA PD5, ENB PD4; IN1 PD2, IN2 PD3, IN3 PD6, IN4 PD1.
 *   IR OUT PB0 (HIGH = cliff); SG90 PB1; SG92 PB2.
 *   HC-SR04 TRIG PB3 / ECHO PD0; OLED SCL PC0 / SDA PC1.
 *   TTP223 left PA0 / right PA1; AVCC powered; all grounds common.
 * Remove L298N ENA/ENB jumpers. No automatic reverse is used.
 *
 * Build (avr-gcc/avr-libc installed):
 * avr-gcc -mmcu=atmega32 -std=gnu99 -Os -Wall -Wextra -Werror \
 *   robot_sh1106.c -o robot_fsm.elf
 * avr-objcopy -O ihex -R .eeprom robot_fsm.elf robot_fsm.hex
 * avr-size -C --mcu=atmega32 robot_fsm.elf
 *
 * Test with wheels raised, then at floor level with a tether/catch surface.
 * PWM duty is NOT measured wheel speed. Tune to the actual loaded robot.
 * Braking is not zero stopping distance; a single IR sensor cannot protect
 * both wheel paths during a turn or distinguish all dark floors from cliffs.
 * L298 braking current must remain within the driver's/motor's limits.
 */
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#if F_CPU != 1000000UL
//#error "Timer constants in this firmware require an actual 1 MHz CPU clock"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>

/* --------------------------- User settings --------------------------- */

#define MOTOR_SPEED                  50u  /* 47% PWM; was 255/full duty. */
#define TURN_FAST_PWM                140u
#define TURN_SLOW_PWM                 60u  /* Both wheels commanded forward. */
#define MOTOR_A_IS_LEFT                1u  /* Confirmed: left motor OUT1/OUT2. */
#define MOTOR_RAMP_STEP                8u
#define MOTOR_RAMP_INTERVAL_TICKS     10u  /* 20.48 ms; braking never ramps. */
#define MOTOR_PWM_ARM_TICKS            3u  /* Let buffered OCR values load. */
#define OBSTACLE_DISTANCE_CM          25u
#define OBSTACLE_CLEAR_CM             30u  /* Hysteresis: don't chatter at 25. */
#define OBSTACLE_TOO_CLOSE_CM         10u  /* Stop; forward arcs are unsafe. */
#define ULTRASONIC_PERIOD_TICKS      30u   /* 30 x 2.048 ms = 61.44 ms */
#define CLEAR_READINGS_REQUIRED      2u
#define SONAR_STALE_TICKS           123u   /* About 252 ms without a sample. */
#define IR_SURFACE_STABLE_TICKS     150u   /* 307 ms before cliff recovery. */
#define SETTLE_TICKS                170u   /* 348 ms before resuming. */
#define TURN_TIMEOUT_TICKS         1953u   /* About 4 s, then brake/wait. */
#define STARTUP_TICKS               250u
#define IDLE_MIN_TICKS             2441u   /* Curious pause every 5-10 s. */
#define IDLE_MAX_TICKS             4882u
#define CURIOUS_MIN_TICKS            60u   /* About 123-246 ms total pause. */
#define CURIOUS_MAX_TICKS           120u   /* No extra SETTLE after curiosity. */
#define OBSTACLE_PAUSE_MIN_TICKS     60u   /* Notice -> choose -> turn. */
#define OBSTACLE_PAUSE_MAX_TICKS    117u

/* Default TTP223 boards are momentary and active HIGH. */
#define TOUCH_ACTIVE_HIGH            1u
#define TOUCH_DEBOUNCE_TICKS         8u    /* About 16.4 ms. */
#define TOUCH_STARTUP_IGNORE_TICKS   250u  /* About 0.51 s calibration time. */

#define SERVO_HOME_TICKS             125u  /* 1.000 ms at 8 us/tick */
#define SERVO_CLIFF_TICKS            188u  /* 1.504 ms: about 90 degrees */
#define SERVO_PET_TICKS              145u  /* 1.160 ms: gentle petting wiggle */
#define SERVO_PET_HALF_PERIOD_TICKS  120u  /* About 246 ms each way. */

#define SH1106_COLUMN_OFFSET         2u
#define OLED_EYE_X                   24u
#define OLED_EYE_WIDTH               80u   /* Room for a full pop + turn gaze. */
#define OLED_FIRST_EYE_PAGE          2u
#define OLED_EYE_PAGE_COUNT          4u

#define PAT_EYE_X_SHIFT              5
#define PAT_EYE_Y_SHIFT             -5
#define CLOSED_EYE_HALF_WIDTH        8
#define TURN_GAZE_SHIFT              4
/* OLED viewed from in front: robot's left is the viewer's right (+X).
 * Change only this sign if your display mounting makes travel gaze mirrored.
 * The established left/right petting-pad expressions are independent. */
#define TURN_GAZE_POLARITY           1
#define SEARCH_WIGGLE_STEP_TICKS    35u   /* About 72 ms per wiggle step. */

/* Timer0 runs at F_CPU/8, so one timer count is 8 microseconds. */
#define TIMER0_TICK_US               8UL
#define OBSTACLE_THRESHOLD_TICKS \
    ((OBSTACLE_DISTANCE_CM * 58UL + TIMER0_TICK_US - 1UL) / TIMER0_TICK_US)
#define CLEAR_THRESHOLD_TICKS \
    ((OBSTACLE_CLEAR_CM * 58UL + TIMER0_TICK_US - 1UL) / TIMER0_TICK_US)
#define TOO_CLOSE_THRESHOLD_TICKS \
    ((OBSTACLE_TOO_CLOSE_CM * 58UL + TIMER0_TICK_US - 1UL) / TIMER0_TICK_US)
#define MINIMUM_VALID_ECHO_TICKS \
    ((2UL * 58UL + TIMER0_TICK_US - 1UL) / TIMER0_TICK_US)

#if OBSTACLE_CLEAR_CM > 35 || OBSTACLE_CLEAR_CM <= OBSTACLE_DISTANCE_CM || \
    OBSTACLE_TOO_CLOSE_CM >= OBSTACLE_DISTANCE_CM || OBSTACLE_TOO_CLOSE_CM < 2
#error "Require 2 <= TOO_CLOSE < DISTANCE < CLEAR <= 35 cm"
#endif
#if MOTOR_SPEED > 255 || TURN_FAST_PWM > 255 || TURN_SLOW_PWM >= TURN_FAST_PWM
#error "Use 8-bit PWM values, with TURN_SLOW_PWM less than TURN_FAST_PWM"
#endif
#if IR_SURFACE_STABLE_TICKS > 255 || MOTOR_RAMP_STEP < 1
#error "Surface counter must fit uint8_t and ramp step must be positive"
#endif
#if TURN_GAZE_SHIFT < 1 || TURN_GAZE_SHIFT > 4 || \
    (TURN_GAZE_POLARITY != 1 && TURN_GAZE_POLARITY != -1)
#error "Use gaze shift 1..4 and gaze polarity +1 or -1 for this eye window"
#endif

/* ------------------------------ States ------------------------------- */

typedef enum {
    ACTION_UNKNOWN,
    ACTION_DRIVE,
    ACTION_PETTING,
    ACTION_OBSTACLE,
    ACTION_CLIFF
} action_t;

typedef enum {
    TOUCH_NONE  = 0,
    TOUCH_LEFT  = 1,
    TOUCH_RIGHT = 2,
    TOUCH_BOTH  = 3
} touch_state_t;

typedef enum {
    STATE_STARTUP,
    STATE_CRUISE,
    STATE_CURIOUS_PAUSE,
    STATE_OBSTACLE_PAUSE,
    STATE_TURN,
    STATE_SETTLE,
    STATE_PETTING,
    STATE_CLIFF,
    STATE_BLOCKED,
    STATE_SENSOR_WAIT
} robot_state_t;

static volatile uint8_t cliff_latched = 1u;
static volatile uint8_t surface_stable_ticks = 0u;

/* ------------------------------- Motors ------------------------------ */
/*
 * L298N:
 * ENA=PD5/OC1A, IN1=PD2, IN2=PD3
 * ENB=PD4/OC1B, IN3=PD6, IN4=PD1
 *
 * Motor B has the opposite electrical polarity so both wheels move the
 * robot physically forward.
 */

typedef enum { MOTOR_BRAKED, MOTOR_ARMING, MOTOR_RUNNING } motor_hw_mode_t;
static volatile motor_hw_mode_t motor_hw_mode = MOTOR_BRAKED;
static uint8_t motor_target_a = 0, motor_target_b = 0;
static uint8_t motor_current_a = 0, motor_current_b = 0;
static uint16_t motor_arm_since = 0, motor_last_ramp = 0;

#define MOTOR_DIRECTION_MASK (_BV(PD1) | _BV(PD2) | _BV(PD3) | _BV(PD6))
#define MOTOR_ENABLE_MASK (_BV(PD4) | _BV(PD5))

/* Caller has interrupts disabled (main atomic section or ISR). */
static inline void motors_brake_now(void)
{
    /* Equal inputs FIRST, then force ENA/ENB HIGH as GPIO. Unlike EN=0,
     * this dynamically brakes. Disconnect PWM to avoid its buffered delay.
     * Keep Timer1 running; only its output ownership changes. */
    PORTD &= (uint8_t)~MOTOR_DIRECTION_MASK;
    PORTD |= MOTOR_ENABLE_MASK;
    TCCR1A = _BV(WGM10);
    motor_hw_mode = MOTOR_BRAKED;
}

static void motors_brake(void)
{
    uint8_t saved_sreg = SREG;
    cli();
    motors_brake_now();
    SREG = saved_sreg;
    motor_current_a = motor_current_b = 0;
}

static void motors_init(void)
{
    DDRD |= _BV(PD1) | _BV(PD2) | _BV(PD3) |
            _BV(PD4) | _BV(PD5) | _BV(PD6);

    /* Same Timer1 frequency as before; start with outputs in brake mode. */
    motors_brake_now();
    TCCR1B = _BV(WGM12) | _BV(CS11);
    OCR1A = 0;
    OCR1B = 0;
}

static void motors_set_targets(uint8_t speed_a, uint8_t speed_b)
{
    motor_target_a = speed_a;
    motor_target_b = speed_b;
    if (!speed_a && !speed_b) {
        motors_brake();
    }
}

static uint8_t motor_ramp(uint8_t current, uint8_t target)
{
    if (current >= target) {
        return target;               /* Reductions take effect immediately. */
    }
    if ((uint16_t)current + MOTOR_RAMP_STEP >= target) {
        return target;
    }
    return (uint8_t)(current + MOTOR_RAMP_STEP);
}

static void motors_ramp_pair(void)
{
    /* Keep the turning ratio even during soft start. Ramping both channels
     * by the same increment would initially drive straight at the obstacle. */
    if (motor_target_a >= motor_target_b) {
        motor_current_a = motor_ramp(motor_current_a, motor_target_a);
        motor_current_b = motor_target_a ? (uint8_t)(
            (uint16_t)motor_target_b * motor_current_a / motor_target_a) : 0u;
    } else {
        motor_current_b = motor_ramp(motor_current_b, motor_target_b);
        motor_current_a = (uint8_t)(
            (uint16_t)motor_target_a * motor_current_b / motor_target_b);
    }
}

static void motors_service(uint16_t now)
{
    uint8_t saved_sreg = SREG;
    cli();

    /* Never let a stale main-loop command undo an interrupt's cliff stop. */
    if (PINB & _BV(PB0)) {
        cliff_latched = 1u;
        surface_stable_ticks = 0u;
    }
    if (cliff_latched ||
        (!motor_target_a && !motor_target_b)) {
        motors_brake_now();
        motor_current_a = motor_current_b = 0;
        SREG = saved_sreg;
        return;
    }

    if (motor_hw_mode == MOTOR_BRAKED) {
        motor_current_a = motor_current_b = 0;
        motors_ramp_pair();
        OCR1A = motor_current_a;
        OCR1B = motor_current_b;
        /* Keep both direction pairs 00 while buffered PWM duties settle.
         * This prevents a stale full-duty value causing a restart kick. */
        TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
        motor_arm_since = motor_last_ramp = now;
        motor_hw_mode = MOTOR_ARMING;
    } else if (motor_hw_mode == MOTOR_ARMING) {
        if ((uint16_t)(now - motor_arm_since) >= MOTOR_PWM_ARM_TICKS) {
            PORTD = (uint8_t)((PORTD & (uint8_t)~MOTOR_DIRECTION_MASK) |
                             _BV(PD2) | _BV(PD1));
            motor_hw_mode = MOTOR_RUNNING;
            motor_last_ramp = now;
        }
    } else if ((uint16_t)(now - motor_last_ramp) >=
               MOTOR_RAMP_INTERVAL_TICKS) {
        motor_last_ramp = now;
        motors_ramp_pair();
        OCR1A = motor_current_a;
        OCR1B = motor_current_b;
    }
    SREG = saved_sreg;
}

/* -------------------------- Cliff IR sensor -------------------------- */
/* PB0 LOW = surface, PB0 HIGH = cliff, as established on this robot. */

static void ir_init(void)
{
    DDRB &= (uint8_t)~_BV(PB0);
    PORTB |= _BV(PB0);                 /* Harmless pull-up for open output. */
}

/* --------------------------- Two servos ------------------------------ */
/* PB1=SG90 signal, PB2=SG92 signal. Timer2 tick = 8 us. */

static volatile uint8_t servo_target_ticks = SERVO_HOME_TICKS;
static volatile uint8_t servo_frame_count = 9;
static volatile uint8_t servo_pulse_active = 0;
static volatile uint16_t system_ticks_2ms = 0;

static void servos_init(void)
{
    DDRB |= _BV(PB1) | _BV(PB2);
    PORTB &= (uint8_t)~(_BV(PB1) | _BV(PB2));

    TCNT2 = 0;
    OCR2 = SERVO_HOME_TICKS;
    TIFR = _BV(TOV2) | _BV(OCF2);
    TIMSK |= _BV(TOIE2) | _BV(OCIE2);
    TCCR2 = _BV(CS21);                 /* Normal mode, prescaler /8. */
}

static void servos_home(void)
{
    servo_target_ticks = SERVO_HOME_TICKS;
}

static void servos_cliff_position(void)
{
    servo_target_ticks = SERVO_CLIFF_TICKS;
}

ISR(TIMER2_OVF_vect)
{
    ++system_ticks_2ms;

    /* Safety is sampled every ~2.048 ms, even during OLED/sonar work.
     * One HIGH sample brakes. Only the main FSM can clear the latch after
     * a continuously stable surface; short LOW glitches cannot restart. */
    if (PINB & _BV(PB0)) {
        cliff_latched = 1u;
        surface_stable_ticks = 0u;
        if (motor_hw_mode != MOTOR_BRAKED) {
            motors_brake_now();
        }
    } else if (surface_stable_ticks < IR_SURFACE_STABLE_TICKS) {
        ++surface_stable_ticks;
    }

    if (++servo_frame_count >= 10u) {
        servo_frame_count = 0;
        OCR2 = servo_target_ticks;
        PORTB |= _BV(PB1) | _BV(PB2);
        servo_pulse_active = 1;
    }
}

ISR(TIMER2_COMP_vect)
{
    if (servo_pulse_active) {
        PORTB &= (uint8_t)~(_BV(PB1) | _BV(PB2));
        servo_pulse_active = 0;
    }
}

static uint16_t system_ticks_read(void)
{
    uint8_t saved_sreg = SREG;
    uint16_t value;

    cli();
    value = system_ticks_2ms;
    SREG = saved_sreg;
    return value;
}

/* ------------------------ TTP223 touch pads ------------------------- */
/*
 * Left OUT=PA0, right OUT=PA1. TTP223 OUT is push-pull, so the AVR
 * pull-ups stay disabled. AVCC must be powered for Port A to operate.
 */

typedef struct {
    uint8_t candidate;
    uint8_t stable;
    uint16_t candidate_since;
} touch_debouncer_t;

static touch_debouncer_t left_touch_debouncer = {0, 0, 0};
static touch_debouncer_t right_touch_debouncer = {0, 0, 0};

static void touch_init(void)
{
    DDRA &= (uint8_t)~(_BV(PA0) | _BV(PA1));
    PORTA &= (uint8_t)~(_BV(PA0) | _BV(PA1));
}

static uint8_t touch_raw(uint8_t pin_mask)
{
    uint8_t high = (PINA & pin_mask) ? 1u : 0u;

#if TOUCH_ACTIVE_HIGH
    return high;
#else
    return (uint8_t)!high;
#endif
}

static uint8_t touch_debounce(touch_debouncer_t *debouncer,
                              uint8_t sample, uint16_t now)
{
    if (sample != debouncer->candidate) {
        debouncer->candidate = sample;
        debouncer->candidate_since = now;
    } else if (sample != debouncer->stable &&
               (uint16_t)(now - debouncer->candidate_since) >=
                   TOUCH_DEBOUNCE_TICKS) {
        debouncer->stable = sample;
    }

    return debouncer->stable;
}

static touch_state_t touch_update(uint16_t now)
{
    uint8_t state = TOUCH_NONE;

    if (touch_debounce(&left_touch_debouncer,
                       touch_raw(_BV(PA0)), now)) {
        state |= TOUCH_LEFT;
    }
    if (touch_debounce(&right_touch_debouncer,
                       touch_raw(_BV(PA1)), now)) {
        state |= TOUCH_RIGHT;
    }

    return (touch_state_t)state;
}

/* ---------------------- Non-blocking pet motion --------------------- */

static void servos_update(action_t action, touch_state_t touch,
                          uint16_t now)
{
    static uint8_t petting = 0;
    static uint8_t pet_outward = 0;
    static uint16_t last_pet_change = 0;

    /* A cliff always overrides the playful movement immediately. */
    if (action == ACTION_CLIFF) {
        petting = 0;
        pet_outward = 0;
        servos_cliff_position();
        return;
    }

    if ((action == ACTION_DRIVE || action == ACTION_PETTING ||
         action == ACTION_OBSTACLE) &&
        touch != TOUCH_NONE) {
        if (!petting) {
            petting = 1;
            pet_outward = 1;
            last_pet_change = now;
            servo_target_ticks = SERVO_PET_TICKS;
        } else if ((uint16_t)(now - last_pet_change) >=
                   SERVO_PET_HALF_PERIOD_TICKS) {
            last_pet_change = now;
            pet_outward = (uint8_t)!pet_outward;
            servo_target_ticks = pet_outward ?
                SERVO_PET_TICKS : SERVO_HOME_TICKS;
        }
        return;
    }

    petting = 0;
    pet_outward = 0;
    servos_home();
}

/* -------------------------- HC-SR04 sensor --------------------------- */
/* TRIG=PB3, ECHO=PD0. Timer0 gives 8 us measurement resolution. */

typedef enum {
    SONAR_UNKNOWN, SONAR_NEAR, SONAR_MID, SONAR_CLEAR,
    SONAR_TOO_CLOSE, SONAR_FAULT
} sonar_status_t;

static sonar_status_t sonar_status = SONAR_UNKNOWN;
static uint8_t sonar_obstacle = 1u;   /* Require evidence before driving. */
static uint8_t sonar_clear_count = 0u;
static uint16_t sonar_last_sample_tick = 0;
static uint16_t sonar_last_trigger_tick = 0;
static uint16_t random_state = 0xACE1u;

/* Lightweight pseudorandom sequence. Sensor timing mixes in environmental
 * variation; this is not a true hardware entropy source. */
static uint16_t random_next(void)
{
    uint16_t value = random_state;
    value ^= (uint16_t)(value << 7);
    value ^= (uint16_t)(value >> 9);
    value ^= (uint16_t)(value << 8);
    random_state = value ? value : 0xACE1u;
    return random_state;
}

static uint16_t random_between(uint16_t low, uint16_t high)
{
    return (uint16_t)(low + random_next() % (uint16_t)(high - low + 1u));
}

static void ultrasonic_init(void)
{
    DDRB |= _BV(PB3);
    PORTB &= (uint8_t)~_BV(PB3);

    DDRD &= (uint8_t)~_BV(PD0);
    PORTD &= (uint8_t)~_BV(PD0);       /* No pull-up on ECHO. */

    TCNT0 = 0;
    TIFR = _BV(TOV0);
    TCCR0 = _BV(CS01);                 /* Normal mode, prescaler /8. */
}

static void ultrasonic_trigger(void)
{
    /* Prevent a PORTB read/modify/write collision with the servo ISRs. */
    uint8_t saved_sreg = SREG;
    cli();

    PORTB &= (uint8_t)~_BV(PB3);
    _delay_us(2);
    PORTB |= _BV(PB3);
    _delay_us(12);
    PORTB &= (uint8_t)~_BV(PB3);

    SREG = saved_sreg;
}

static sonar_status_t ultrasonic_measure(void)
{
    uint8_t rise_timeouts = 0;
    uint8_t pulse_ticks;

    /* A stuck HIGH or missing rising edge is a fault, not a clear path. */
    if (PIND & _BV(PD0)) {
        return SONAR_FAULT;
    }

    ultrasonic_trigger();

    /* The rising edge normally appears quickly; allow about 4.1 ms. */
    TCNT0 = 0;
    TIFR = _BV(TOV0);
    while (!(PIND & _BV(PD0))) {
        if (TIFR & _BV(TOV0)) {
            TIFR = _BV(TOV0);
            if (++rise_timeouts >= 2u) {
                return SONAR_FAULT;
            }
        }
    }

    TCNT0 = 0;
    TIFR = _BV(TOV0);
    while (PIND & _BV(PD0)) {
        if (TIFR & _BV(TOV0)) {
            return SONAR_CLEAR;       /* Pulse longer than ~35 cm. */
        }
        if (TCNT0 > (uint8_t)CLEAR_THRESHOLD_TICKS) {
            return SONAR_CLEAR;
        }
    }

    pulse_ticks = TCNT0;
    if (TIFR & _BV(TOV0)) {
        return SONAR_CLEAR;
    }
    if (pulse_ticks < (uint8_t)MINIMUM_VALID_ECHO_TICKS) {
        return SONAR_FAULT;           /* Glitch/inside blind zone: brake. */
    }
    if (pulse_ticks <= (uint8_t)TOO_CLOSE_THRESHOLD_TICKS) {
        return SONAR_TOO_CLOSE;
    }
    if (pulse_ticks <= (uint8_t)OBSTACLE_THRESHOLD_TICKS) {
        return SONAR_NEAR;
    }
    return pulse_ticks > (uint8_t)CLEAR_THRESHOLD_TICKS ?
        SONAR_CLEAR : SONAR_MID;
}

static void sonar_accept_sample(sonar_status_t sample, uint16_t now)
{
    sonar_status = sample;
    sonar_last_sample_tick = now;
    if (sample == SONAR_CLEAR) {
        if (sonar_clear_count < CLEAR_READINGS_REQUIRED) {
            ++sonar_clear_count;
        }
        if (sonar_clear_count >= CLEAR_READINGS_REQUIRED) {
            sonar_obstacle = 0u;
        }
    } else {
        sonar_clear_count = 0u;
        if (sample != SONAR_MID) {
            sonar_obstacle = 1u;
        }
    }
}

static void sonar_service(uint16_t now)
{
    if ((uint16_t)(now - sonar_last_trigger_tick) >= ULTRASONIC_PERIOD_TICKS) {
        sonar_status_t sample;
        sonar_last_trigger_tick = now;
        /* PD0 has no pin-change interrupt on ATmega32. Keep this short,
         * bounded polling measurement (~6 ms worst case) for the proven
         * wiring; servo pulses and emergency braking remain interrupt-driven.
         * A long pulse/no reflected echo after a valid rise exceeds the
         * clearance range. Repeatedly stuck HIGH is detected next cycle. */
        sample = ultrasonic_measure();
        now = system_ticks_read();
        random_state ^= (uint16_t)(now ^ ((uint16_t)TCNT0 << 8));
        (void)random_next();
        sonar_accept_sample(sample, now);
    }
}

/* ----------------------- Timed behavior FSM -------------------------- */

static robot_state_t robot_state = STATE_STARTUP;
static uint16_t state_since = 0, state_duration = 0;
static uint8_t turn_left = 0;
static uint8_t avoidance_blocked = 0;
static touch_state_t previous_touch = TOUCH_NONE;

static void robot_enter(robot_state_t state, uint16_t now)
{
    if (state == robot_state) {
        return;
    }
    robot_state = state;
    state_since = now;
    state_duration = 0;

    switch (state) {
    case STATE_CRUISE:
        state_duration = random_between(IDLE_MIN_TICKS, IDLE_MAX_TICKS);
        motors_set_targets(MOTOR_SPEED, MOTOR_SPEED);
        break;
    case STATE_TURN: {
        uint8_t a_is_inner = (turn_left == MOTOR_A_IS_LEFT);
        motors_set_targets(a_is_inner ? TURN_SLOW_PWM : TURN_FAST_PWM,
                           a_is_inner ? TURN_FAST_PWM : TURN_SLOW_PWM);
        break;
    }
    case STATE_CURIOUS_PAUSE:
        state_duration = random_between(CURIOUS_MIN_TICKS, CURIOUS_MAX_TICKS);
        motors_set_targets(0, 0);
        break;
    case STATE_OBSTACLE_PAUSE:
        /* One choice per encounter, never randomized on every sensor sample. */
        turn_left = (uint8_t)(random_next() & 1u);
        state_duration = random_between(OBSTACLE_PAUSE_MIN_TICKS,
                                        OBSTACLE_PAUSE_MAX_TICKS);
        motors_set_targets(0, 0);
        break;
    case STATE_SETTLE:
        state_duration = SETTLE_TICKS;
        motors_set_targets(0, 0);
        break;
    default:
        motors_set_targets(0, 0);
        break;
    }
}

static void robot_begin(uint16_t now)
{
    robot_state = STATE_STARTUP;
    state_since = now;
    state_duration = STARTUP_TICKS;
    previous_touch = TOUCH_NONE;
    avoidance_blocked = 0;
    motors_set_targets(0, 0);
}

static uint8_t cliff_release_if_safe(void)
{
    uint8_t saved_sreg = SREG;
    uint8_t safe = 0;
    cli();
    if (!(PINB & _BV(PB0)) &&
        surface_stable_ticks >= IR_SURFACE_STABLE_TICKS) {
        cliff_latched = 0u;
        safe = 1u;
    }
    SREG = saved_sreg;
    return safe;
}

static void robot_update(uint16_t now, touch_state_t touch)
{
    uint16_t elapsed;

    if (touch != previous_touch) {
        random_state ^= (uint16_t)(now + (uint16_t)touch * 257u);
        (void)random_next();
        previous_touch = touch;
    }

    /* Initial inhibit is not a real cliff encounter. Complete calibration
     * and establish the floor before leaving STARTUP; a real HIGH below
     * still enters CLIFF immediately. */
    if (robot_state == STATE_STARTUP && !(PINB & _BV(PB0))) {
        if ((uint16_t)(now - state_since) < STARTUP_TICKS ||
            !cliff_release_if_safe()) {
            return;
        }
        robot_enter(STATE_SETTLE, now);
    }

    /* Highest priority, also serviced independently by Timer2. */
    if (cliff_latched) {
        if (robot_state != STATE_CLIFF) {
            robot_enter(STATE_CLIFF, now);
            return;
        }
        if (surface_stable_ticks < IR_SURFACE_STABLE_TICKS) {
            return;
        }
        if (!cliff_release_if_safe()) {
            return;
        }
        robot_enter(STATE_SETTLE, now);
    }

    if (touch != TOUCH_NONE) {
        robot_enter(STATE_PETTING, now);
        return;
    }

    if (sonar_status == SONAR_UNKNOWN || sonar_status == SONAR_FAULT ||
        (uint16_t)(now - sonar_last_sample_tick) > SONAR_STALE_TICKS) {
        robot_enter(STATE_SENSOR_WAIT, now);
        return;
    }
    if (sonar_status == SONAR_TOO_CLOSE) {
        avoidance_blocked = 1u;
    }
    if (!sonar_obstacle) {
        avoidance_blocked = 0u;
    }
    if (avoidance_blocked) {
        robot_enter(STATE_BLOCKED, now);
        return;
    }

    elapsed = (uint16_t)(now - state_since); /* Wrap-safe elapsed time. */
    switch (robot_state) {
    case STATE_STARTUP:
        if (elapsed >= STARTUP_TICKS) {
            robot_enter(STATE_SETTLE, now);
        }
        break;
    case STATE_CLIFF:
    case STATE_PETTING:
    case STATE_SENSOR_WAIT:
        robot_enter(STATE_SETTLE, now);
        break;
    case STATE_BLOCKED:
        /* No blind retries or reversing. Wait for space or repositioning. */
        if (!sonar_obstacle) {
            robot_enter(STATE_SETTLE, now);
        }
        break;
    case STATE_SETTLE:
        if (elapsed >= state_duration) {
            robot_enter(sonar_obstacle ? STATE_OBSTACLE_PAUSE : STATE_CRUISE,
                        now);
        }
        break;
    case STATE_CRUISE:
        if (sonar_obstacle) {
            robot_enter(STATE_OBSTACLE_PAUSE, now);
        } else if (elapsed >= state_duration) {
            robot_enter(STATE_CURIOUS_PAUSE, now);
        }
        break;
    case STATE_CURIOUS_PAUSE:
        if (sonar_obstacle) {
            robot_enter(STATE_OBSTACLE_PAUSE, now);
        } else if (elapsed >= state_duration) {
            /* Already stopped for the curious glance. No second pause;
             * the ordinary motor ramp still softens the restart. */
            robot_enter(STATE_CRUISE, now);
        }
        break;
    case STATE_OBSTACLE_PAUSE:
        if (!sonar_obstacle) {
            robot_enter(STATE_SETTLE, now);
        } else if (elapsed >= state_duration) {
            robot_enter(STATE_TURN, now);
        }
        break;
    case STATE_TURN:
        if (!sonar_obstacle) {
            robot_enter(STATE_SETTLE, now);
        } else if (elapsed >= TURN_TIMEOUT_TICKS) {
            avoidance_blocked = 1u;
            robot_enter(STATE_BLOCKED, now);
        }
        break;
    default:
        robot_enter(STATE_SENSOR_WAIT, now);
        break;
    }
}

/* ------------------------- Hardware TWI/I2C -------------------------- */

static uint8_t twi_wait(void)
{
    uint16_t timeout = 5000u;

    while (!(TWCR & _BV(TWINT))) {
        if (--timeout == 0u) {
            TWCR = _BV(TWEN);          /* Recover instead of freezing robot. */
            return 0;
        }
    }
    return 1;
}

static void twi_init(void)
{
    /* PC0=SCL and PC1=SDA. External module pull-ups are normally fitted. */
    DDRC &= (uint8_t)~(_BV(PC0) | _BV(PC1));
    PORTC |= _BV(PC0) | _BV(PC1);     /* Weak backup pull-ups. */

    TWSR = 0;                          /* Prescaler = 1. */
    TWBR = 0;                          /* 62.5 kHz SCL at F_CPU=1 MHz. */
    TWCR = _BV(TWEN);
}

static void twi_bus_recover(void)
{
    uint8_t pulse;

    /*
     * Abort the failed hardware transaction, release SDA, and clock SCL nine
     * times. This lets a slave that stopped mid-byte finish and release SDA.
     * DDR=1 with PORT=0 drives a line low; DDR=0 with PORT=1 releases it and
     * leaves the weak internal pull-up enabled as a backup.
     */
    TWCR = 0;
    DDRC &= (uint8_t)~(_BV(PC0) | _BV(PC1));
    PORTC |= _BV(PC0) | _BV(PC1);
    _delay_us(5);

    for (pulse = 0; pulse < 9u; ++pulse) {
        PORTC &= (uint8_t)~_BV(PC0);
        DDRC |= _BV(PC0);              /* SCL low. */
        _delay_us(5);
        DDRC &= (uint8_t)~_BV(PC0);    /* Release SCL high. */
        PORTC |= _BV(PC0);
        _delay_us(5);
    }

    /* Generate a STOP: SDA changes low-to-high while SCL is released high. */
    PORTC &= (uint8_t)~_BV(PC1);
    DDRC |= _BV(PC1);
    _delay_us(5);
    DDRC &= (uint8_t)~_BV(PC1);
    PORTC |= _BV(PC1);
    _delay_us(5);

    twi_init();
}

static uint8_t twi_start_write(uint8_t address)
{
    uint8_t status;

    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    if (!twi_wait()) {
        return 0;
    }

    status = TWSR & 0xF8u;
    if (status != TW_START && status != TW_REP_START) {
        return 0;
    }

    TWDR = (uint8_t)(address << 1);
    TWCR = _BV(TWINT) | _BV(TWEN);
    if (!twi_wait()) {
        return 0;
    }

    return (TWSR & 0xF8u) == TW_MT_SLA_ACK;
}

static uint8_t twi_write(uint8_t value)
{
    TWDR = value;
    TWCR = _BV(TWINT) | _BV(TWEN);
    if (!twi_wait()) {
        return 0;
    }
    return (TWSR & 0xF8u) == TW_MT_DATA_ACK;
}

static void twi_stop(void)
{
    uint16_t timeout = 1000u;

    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    while ((TWCR & _BV(TWSTO)) && --timeout != 0u) {
        /* Wait briefly for STOP to appear on the bus. */
    }
}

static uint8_t twi_probe(uint8_t address)
{
    uint8_t acknowledged = twi_start_write(address);
    twi_stop();
    return acknowledged;
}

/* ------------------------------ SH1106 ------------------------------- */

static uint8_t oled_address = 0x3Cu;
static uint8_t oled_ok = 0;
static uint8_t eye_frame_buffer[OLED_EYE_PAGE_COUNT][OLED_EYE_WIDTH];

static const uint8_t sh1106_init_sequence[] PROGMEM = {
    0xAE,             /* Display off. */
    0xD5, 0x80,       /* Clock divide/oscillator. */
    0xA8, 0x3F,       /* Multiplex 1/64. */
    0xD3, 0x00,       /* Display offset 0. */
    0x40,             /* Start line 0. */
    0xAD, 0x8B,       /* Internal DC-DC on. */
    0xA1,             /* Segment remap. */
    0xC8,             /* COM scan direction. */
    0xDA, 0x12,       /* COM pins. */
    0x81, 0xCF,       /* Contrast. */
    0xD9, 0x1F,       /* Pre-charge. */
    0xDB, 0x40,       /* VCOM deselect. */
    0x33,             /* Pump voltage. */
    0xA6,             /* Normal (not inverted). */
    0x20, 0x10,       /* Page/column addressing state. */
    0xA4              /* Use display RAM. */
};

static uint8_t oled_commands_P(const uint8_t *commands, uint8_t count)
{
    uint8_t i;

    if (!twi_start_write(oled_address)) {
        twi_stop();
        return 0;
    }
    if (!twi_write(0x00)) {            /* Following bytes are commands. */
        twi_stop();
        return 0;
    }
    for (i = 0; i < count; ++i) {
        if (!twi_write(pgm_read_byte(&commands[i]))) {
            twi_stop();
            return 0;
        }
    }
    twi_stop();
    return 1;
}

static uint8_t oled_command(uint8_t command)
{
    if (!twi_start_write(oled_address)) {
        twi_stop();
        return 0;
    }
    if (!twi_write(0x00) || !twi_write(command)) {
        twi_stop();
        return 0;
    }
    twi_stop();
    return 1;
}

static uint8_t oled_set_position(uint8_t page, uint8_t x)
{
    uint8_t column = (uint8_t)(x + SH1106_COLUMN_OFFSET);

    if (!twi_start_write(oled_address)) {
        twi_stop();
        return 0;
    }
    if (!twi_write(0x00) ||
        !twi_write((uint8_t)(0xB0u | page)) ||
        !twi_write((uint8_t)(column & 0x0Fu)) ||
        !twi_write((uint8_t)(0x10u | (column >> 4)))) {
        twi_stop();
        return 0;
    }
    twi_stop();
    return 1;
}

static uint8_t oled_write_data(const uint8_t *data, uint8_t count)
{
    uint8_t i;

    if (!twi_start_write(oled_address)) {
        twi_stop();
        return 0;
    }
    if (!twi_write(0x40)) {            /* Following bytes are display data. */
        twi_stop();
        return 0;
    }
    for (i = 0; i < count; ++i) {
        if (!twi_write(data[i])) {
            twi_stop();
            return 0;
        }
    }
    twi_stop();
    return 1;
}

static uint8_t oled_clear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0; page < 8u; ++page) {
        if (!oled_set_position(page, 0)) {
            return 0;
        }
        if (!twi_start_write(oled_address)) {
            twi_stop();
            return 0;
        }
        if (!twi_write(0x40)) {
            twi_stop();
            return 0;
        }
        for (x = 0; x < 128u; ++x) {
            if (!twi_write(0x00)) {
                twi_stop();
                return 0;
            }
        }
        twi_stop();
    }
    return 1;
}

static void oled_init(void)
{
    twi_init();
    _delay_ms(20);

    if (twi_probe(0x3Cu)) {
        oled_address = 0x3Cu;
    } else if (twi_probe(0x3Du)) {
        oled_address = 0x3Du;
    } else {
        oled_ok = 0;
        return;
    }

    if (!oled_commands_P(sh1106_init_sequence,
                         (uint8_t)sizeof(sh1106_init_sequence))) {
        oled_ok = 0;
        return;
    }

    _delay_ms(100);
    /* Clear while still off so no random power-on pixels flash on screen. */
    if (!oled_clear() || !oled_command(0xAF)) {
        oled_ok = 0;
        return;
    }
    oled_ok = 1;
}

/* -------------------------- Eye animation ---------------------------- */

static const int8_t gaze_x_table[] PROGMEM = {
     0,  4,  4,  0, -4, -4,  0,  3, -3,  0
};
static const int8_t gaze_y_table[] PROGMEM = {
     0,  0, -2, -3, -2,  1,  3,  2,  2,  0
};
static const uint8_t alert_growth_table[] PROGMEM = {
     8, 6, 3, 0
};
static const uint8_t blink_height_table[] PROGMEM = {
     22, 16, 8, 3, 8, 16, 22
};
static const uint8_t closed_eye_rise_table[] PROGMEM = {
     4, 4, 4, 3, 3, 2, 1, 1, 0
};
static const int8_t search_gaze_x_table[] PROGMEM = {
    -4, -2, 0, 2, 4, 2, 0, -2
};
static const int8_t search_gaze_y_table[] PROGMEM = {
     0, -1, -2, -1, 0, 1, 2, 1
};

typedef enum {
    GAZE_ROAM, GAZE_FORWARD, GAZE_TURN_LEFT, GAZE_TURN_RIGHT, GAZE_SEARCH
} gaze_mode_t;

static gaze_mode_t eyes_gaze_mode = GAZE_ROAM;
static uint8_t search_phase = 0;
static uint16_t search_last_step = 0;

static int8_t gaze_x = 0;
static int8_t gaze_y = 0;
static int8_t target_gaze_x = 0;
static int8_t target_gaze_y = 0;
static int8_t frame_gaze_x = 0;
static int8_t frame_gaze_y = 0;
static uint8_t frame_eye_growth = 0;
static uint8_t frame_eye_height = 22;
static uint8_t gaze_index = 0;
static uint8_t gaze_hold = 0;
static uint8_t alert_phase = 0;
static uint8_t eyes_alert = 0;
static touch_state_t eyes_touch = TOUCH_NONE;
static uint8_t frame_eyes_closed = 0;
static uint8_t eye_page_index = 0;
static uint8_t blink_countdown = 70;
static uint8_t blink_phase = 0xFFu;

static int8_t step_toward(int8_t value, int8_t target)
{
    if (value < target) {
        ++value;
    } else if (value > target) {
        --value;
    }
    return value;
}

static void eyes_set_alert(uint8_t enabled)
{
    enabled = enabled ? 1u : 0u;
    if (enabled != eyes_alert) {
        eyes_alert = enabled;
        alert_phase = 0;
        blink_phase = 0xFFu;

        /* Center the first large alert frame so it cannot be clipped. */
        if (enabled) {
            gaze_x = 0;
            gaze_y = 0;
        }
    }
}

static void eyes_set_touch(touch_state_t touch)
{
    if (touch != eyes_touch) {
        eyes_touch = touch;
        blink_phase = 0xFFu;
        blink_countdown = 70u;
    }
}

static void eyes_set_gaze_mode(gaze_mode_t mode, uint16_t now)
{
    if (mode != eyes_gaze_mode) {
        eyes_gaze_mode = mode;
        search_phase = 0;
        search_last_step = now;
        gaze_hold = 0;
        if (mode == GAZE_ROAM) {
            /* A curious pause is now short; start a new glance immediately
             * instead of waiting twelve complete display frames first. */
            gaze_index = (uint8_t)random_between(1u,
                              (uint16_t)sizeof(gaze_x_table) - 2u);
            target_gaze_x = (int8_t)pgm_read_byte(&gaze_x_table[gaze_index]);
            target_gaze_y = (int8_t)pgm_read_byte(&gaze_y_table[gaze_index]);
            gaze_x = target_gaze_x;
            gaze_y = target_gaze_y;
        }
    }
}

static int8_t eyes_turn_x(void)
{
    if (eyes_gaze_mode == GAZE_TURN_LEFT) {
        return TURN_GAZE_POLARITY * TURN_GAZE_SHIFT;
    }
    if (eyes_gaze_mode == GAZE_TURN_RIGHT) {
        return -TURN_GAZE_POLARITY * TURN_GAZE_SHIFT;
    }
    return 0;
}

static void eyes_advance_frame(uint16_t now)
{
    if (eyes_alert) {
        frame_eyes_closed = 0;
        /* Turn intent is visible from the first avoidance frame, including
         * OBSTACLE_PAUSE before the wheels start. Cliff mode uses center. */
        gaze_x = eyes_turn_x();
        gaze_y = 0;
        frame_eye_growth = pgm_read_byte(&alert_growth_table[alert_phase]);
        if (++alert_phase >= (uint8_t)sizeof(alert_growth_table)) {
            alert_phase = 0;
        }
        frame_eye_height = (uint8_t)(22u + frame_eye_growth);
    } else if (eyes_touch != TOUCH_NONE) {
        int8_t pat_x = 0;

        if (eyes_touch == TOUCH_LEFT) {
            pat_x = -PAT_EYE_X_SHIFT;
        } else if (eyes_touch == TOUCH_RIGHT) {
            pat_x = PAT_EYE_X_SHIFT;
        }
        /* TOUCH_BOTH deliberately gives centered, upward closed eyes. */

        frame_eyes_closed = 1;
        frame_eye_growth = 0;
        frame_eye_height = 3u;
        blink_phase = 0xFFu;

        /* A pat should feel immediate; normal gaze resumes smoothly later. */
        gaze_x = pat_x;
        gaze_y = PAT_EYE_Y_SHIFT;
    } else if (eyes_gaze_mode == GAZE_SEARCH) {
        frame_eyes_closed = 0;
        frame_eye_growth = 0;
        frame_eye_height = 22u;
        blink_phase = 0xFFu;
        if ((uint16_t)(now - search_last_step) >= SEARCH_WIGGLE_STEP_TICKS) {
            search_last_step = now;
            if (++search_phase >= (uint8_t)sizeof(search_gaze_x_table)) {
                search_phase = 0;
            }
        }
        gaze_x = (int8_t)pgm_read_byte(&search_gaze_x_table[search_phase]);
        gaze_y = (int8_t)pgm_read_byte(&search_gaze_y_table[search_phase]);
    } else {
        frame_eyes_closed = 0;
        frame_eye_growth = 0;

        if (eyes_gaze_mode == GAZE_FORWARD ||
            eyes_gaze_mode == GAZE_TURN_LEFT ||
            eyes_gaze_mode == GAZE_TURN_RIGHT) {
            gaze_x = eyes_turn_x();
            gaze_y = 0;
        } else {
            if (++gaze_hold >= 12u) {
                gaze_hold = 0;
                if (++gaze_index >= (uint8_t)sizeof(gaze_x_table)) {
                    gaze_index = 0;
                }
                target_gaze_x = (int8_t)pgm_read_byte(&gaze_x_table[gaze_index]);
                target_gaze_y = (int8_t)pgm_read_byte(&gaze_y_table[gaze_index]);
            }

            gaze_x = step_toward(gaze_x, target_gaze_x);
            gaze_y = step_toward(gaze_y, target_gaze_y);
        }

        if (blink_phase != 0xFFu) {
            frame_eye_height = pgm_read_byte(&blink_height_table[blink_phase]);
            if (++blink_phase >= (uint8_t)sizeof(blink_height_table)) {
                blink_phase = 0xFFu;
                blink_countdown = 70u;
            }
        } else if (blink_countdown == 0u) {
            blink_phase = 0;
            frame_eye_height = pgm_read_byte(&blink_height_table[0]);
        } else {
            --blink_countdown;
            frame_eye_height = 22u;
        }
    }

    frame_gaze_x = gaze_x;
    frame_gaze_y = gaze_y;
}

static void robot_expression_service(touch_state_t touch, uint16_t now)
{
    action_t action = ACTION_DRIVE;
    gaze_mode_t mode = GAZE_FORWARD;
    uint8_t alert = 0;

    /* Expressions follow the same priorities as movement. Search wiggles
     * replace the obstacle pop only for BLOCKED and SENSOR_WAIT. */
    if (cliff_latched || robot_state == STATE_CLIFF) {
        action = ACTION_CLIFF;
        alert = 1u;
    } else if (robot_state == STATE_PETTING) {
        action = ACTION_PETTING;
    } else if (robot_state == STATE_BLOCKED || robot_state == STATE_SENSOR_WAIT) {
        action = ACTION_OBSTACLE;
        mode = GAZE_SEARCH;
    } else if (robot_state == STATE_OBSTACLE_PAUSE || robot_state == STATE_TURN) {
        action = ACTION_OBSTACLE;
        mode = turn_left ? GAZE_TURN_LEFT : GAZE_TURN_RIGHT;
        alert = 1u;
    } else if (robot_state == STATE_CURIOUS_PAUSE || robot_state == STATE_STARTUP) {
        mode = GAZE_ROAM;
    }

    servos_update(action, touch, now);
    eyes_set_touch(action == ACTION_PETTING ? touch : TOUCH_NONE);
    eyes_set_alert(alert);
    eyes_set_gaze_mode(mode, now);
}

static void eye_buffer_clear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0; page < OLED_EYE_PAGE_COUNT; ++page) {
        for (x = 0; x < OLED_EYE_WIDTH; ++x) {
            eye_frame_buffer[page][x] = 0;
        }
    }
}

static void eye_buffer_set_pixel(int16_t x, int16_t y)
{
    const int16_t first_y = (int16_t)OLED_FIRST_EYE_PAGE * 8;
    int16_t local_x;
    int16_t local_y;

    if (x < (int16_t)OLED_EYE_X ||
        x >= (int16_t)(OLED_EYE_X + OLED_EYE_WIDTH) ||
        y < first_y ||
        y >= first_y + (int16_t)OLED_EYE_PAGE_COUNT * 8) {
        return;
    }

    local_x = x - OLED_EYE_X;
    local_y = y - first_y;
    eye_frame_buffer[(uint8_t)local_y >> 3][(uint8_t)local_x] |=
        (uint8_t)_BV((uint8_t)local_y & 7u);
}

static void draw_closed_eye(int16_t center_x, int16_t baseline_y)
{
    int8_t dx;

    for (dx = -CLOSED_EYE_HALF_WIDTH;
         dx <= CLOSED_EYE_HALF_WIDTH; ++dx) {
        uint8_t distance = dx < 0 ? (uint8_t)(-dx) : (uint8_t)dx;
        uint8_t rise = pgm_read_byte(&closed_eye_rise_table[distance]);
        int16_t y = baseline_y - rise;

        /* A three-pixel-thick happy arch instead of a flat eyelid. */
        eye_buffer_set_pixel(center_x + dx, y);
        eye_buffer_set_pixel(center_x + dx, y + 1);
        eye_buffer_set_pixel(center_x + dx, y + 2);
    }
}

static void eyes_render_closed_frame(void)
{
    int16_t baseline_y = 32 + frame_gaze_y;

    eye_buffer_clear();
    draw_closed_eye(42 + frame_gaze_x, baseline_y);
    draw_closed_eye(86 + frame_gaze_x, baseline_y);
}

static int8_t eye_vertical_half_span(int16_t x, int16_t center_x,
                                     uint8_t width, uint8_t height,
                                     uint8_t radius)
{
    int16_t dx = x - center_x;
    uint8_t half_width = width / 2u;
    uint8_t half_height = height / 2u;
    uint8_t straight;
    uint8_t corner_x;
    uint16_t remaining;
    uint8_t corner_y = 0;

    if (dx < 0) {
        dx = -dx;
    }
    if ((uint16_t)dx > half_width) {
        return -1;
    }

    if (radius > half_width) {
        radius = half_width;
    }
    if (radius > half_height) {
        radius = half_height;
    }

    straight = (uint8_t)(half_width - radius);
    if ((uint16_t)dx <= straight) {
        return (int8_t)half_height;
    }

    corner_x = (uint8_t)((uint16_t)dx - straight);
    remaining = (uint16_t)radius * radius - (uint16_t)corner_x * corner_x;
    while ((uint16_t)(corner_y + 1u) * (corner_y + 1u) <= remaining) {
        ++corner_y;
    }

    return (int8_t)(half_height - radius + corner_y);
}

static uint8_t point_inside_eye(int16_t y, int16_t center_y, int8_t half_span)
{
    if (half_span < 0) {
        return 0;
    }
    return y >= (center_y - half_span) && y <= (center_y + half_span);
}

static void eyes_render_frame(void)
{
    uint8_t i;
    uint8_t local_y;
    uint8_t width = (uint8_t)(18u + frame_eye_growth);
    uint8_t height = frame_eye_height;
    uint8_t radius = (uint8_t)(5u + frame_eye_growth / 3u);
    int16_t left_center_x = 42 + frame_gaze_x;
    int16_t right_center_x = 86 + frame_gaze_x;
    int16_t center_y = 32 + frame_gaze_y;

    if (frame_eyes_closed) {
        eyes_render_closed_frame();
        return;
    }

    for (i = 0; i < OLED_EYE_WIDTH; ++i) {
        int16_t x = OLED_EYE_X + i;
        int8_t left_span = eye_vertical_half_span(
            x, left_center_x, width, height, radius);
        int8_t right_span = eye_vertical_half_span(
            x, right_center_x, width, height, radius);
        uint8_t page;

        for (page = 0; page < OLED_EYE_PAGE_COUNT; ++page) {
            eye_frame_buffer[page][i] = 0;
        }

        for (local_y = 0; local_y < OLED_EYE_PAGE_COUNT * 8u; ++local_y) {
            int16_t y = (int16_t)OLED_FIRST_EYE_PAGE * 8 + local_y;
            if (point_inside_eye(y, center_y, left_span) ||
                point_inside_eye(y, center_y, right_span)) {
                eye_frame_buffer[local_y >> 3][i] |= _BV(local_y & 7u);
            }
        }
    }
}

static void oled_animation_service(void)
{
    uint8_t display_page;

    if (!oled_ok) {
        return;
    }

    /* One 80-byte page per call; keep an entire frame's buffer stable until
     * its four pages finish. Mode changes never restart a partial transfer. */
    if (eye_page_index == 0u) {
        eyes_advance_frame(system_ticks_read());
        eyes_render_frame();
    }

    display_page = (uint8_t)(OLED_FIRST_EYE_PAGE + eye_page_index);

    if (!oled_set_position(display_page, OLED_EYE_X) ||
        !oled_write_data(eye_frame_buffer[eye_page_index], OLED_EYE_WIDTH)) {
        /*
         * Do not disable animation forever because of one motor-noise glitch.
         * Recover the bus and retry this same page on the next main-loop pass.
         */
        twi_bus_recover();
        return;
    }

    if (++eye_page_index >= OLED_EYE_PAGE_COUNT) {
        eye_page_index = 0;
    }
}

/* ------------------------------- Main -------------------------------- */

int main(void)
{
    touch_state_t touch_state = TOUCH_NONE;
    uint8_t touch_ready = 0;
    uint16_t touch_start_tick;

    motors_init();
    ir_init();
    touch_init();
    ultrasonic_init();
    servos_init();
    servos_home();
    sei();

    /* Motors remain actively braked during display power-up/recovery. */
    oled_init();
    sonar_last_trigger_tick =
        (uint16_t)(system_ticks_read() - ULTRASONIC_PERIOD_TICKS);
    touch_start_tick = system_ticks_read();
    robot_begin(touch_start_tick);

    while (1) {
        uint16_t now = system_ticks_read();

        /* Let both TTP223 boards finish power-on calibration untouched. */
        if (!touch_ready &&
            (uint16_t)(now - touch_start_tick) >=
                TOUCH_STARTUP_IGNORE_TICKS) {
            touch_ready = 1;
        }

        if (touch_ready) {
            touch_state = touch_update(now);
        } else {
            touch_state = TOUCH_NONE;
        }

        /* Apply known safety/touch state BEFORE starting another sample. */
        robot_update(now, touch_state);
        motors_service(now);

        sonar_service(now);
        now = system_ticks_read();
        robot_update(now, touch_state);
        motors_service(now);

        robot_expression_service(touch_state, now);
        oled_animation_service();
        /* No behavior delays here: all pause/turn/ramp deadlines are timed.
         * OLED page I/O and bounded sonar polling leave Timer2 IRQ enabled. */
    }
}
