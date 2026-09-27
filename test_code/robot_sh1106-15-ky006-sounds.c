/*
 * Robot firmware: RoboEyes-inspired C expressions, working microphone and FSM.
 * ATmega32, actual CPU clock 1 MHz; defining F_CPU does not set fuse bits.
 * Existing wiring is unchanged; add MAX9814 OUT to PA2/ADC2 (DIP pin 38):
 *   L298N ENA PD5, ENB PD4; IN1 PD2, IN2 PD3, IN3 PD6, IN4 PD1.
 *   IR OUT PB0 (LOW = cliff, HIGH = surface); SG90 PB1; SG92 PB2.
 *   HC-SR04 TRIG PB3 / ECHO PD0; OLED SCL PC0 / SDA PC1.
 *   TTP223 left PA0 / right PA1; KY-006 S PC2; SW-18015P DO PC3; AVCC powered; all grounds common.
 *   MAX9814 VDD=regulated 5V, GND=common, OUT=PA2, GAIN=VDD, A/R=GND.
 *   AVCC pin 30=5V; AREF pin 32=100nF to GND (no external voltage).
 *   Never wire AREF directly to GND. AVCC is required even with MIC_ENABLED=0.
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

#define MOTOR_SPEED                  120u  /* 47% PWM; was 255/full duty. */
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
#ifndef IR_CLIFF_LEVEL
#define IR_CLIFF_LEVEL                1u   /* 0: LOW=cliff; 1: HIGH=cliff. */
#endif
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

/* MAX9814 OUT -> PA2/ADC2. Threshold units are 8-bit ADC peak-to-peak,
 * NOT decibels: one count is about 19.5 mV with a 5V AVCC reference.
 * Increase MIN/RISING thresholds if the robot's own mechanisms trigger it. */
#define MIC_ENABLED                   1u
#define MIC_MIN_P2P                 100u   /* User-confirmed threshold. */
#define MIC_RISE_P2P                 50u   /* User-confirmed rise above background. */
#define MIC_WINDOW_TICKS              8u   /* At least 16.4 ms per envelope. */
#define MIC_CALIBRATION_TICKS       488u   /* About 1 second after OLED setup. */
#define MIC_COOLDOWN_TICKS          600u   /* About 1.23 s between reactions. */
#define MIC_REARM_WINDOWS             3u   /* Require quieter windows again. */
#define STARTLE_PAUSE_TICKS         240u   /* 491.5 ms; no blocking delay. */
#define DIZZY_PAUSE_TICKS           500u   /* About 1.02 s of dizziness. */
#define SHOCK_ACTIVE_LOW              1u   /* Common modules: DO LOW on shock. */
#define SHOCK_REARM_TICKS            25u   /* About 51 ms quiet before re-arm. */

/* KY-006 passive buzzer: signal on PC2. The buzzer needs a square wave,
 * so Timer0 compare generates the audio carrier while Timer2 keeps the note duration.
 * Because ultrasonic measurement also uses Timer0, sonar sampling is paused
 * during the short sound effect. Cliff/shock/servo interrupts remain active. */
#define BUZZER_PIN                   PC2
#define BUZZER_ENABLED                1u
#define BUZZER_MAX_NOTE_TICKS        70u

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
#define EYE_POP_STEP_TICKS           28u   /* 57 ms; four-step cliff pop. */

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
#if IR_CLIFF_LEVEL != 0 && IR_CLIFF_LEVEL != 1
#error "IR_CLIFF_LEVEL must be 0 or 1"
#endif
#if MIC_MIN_P2P < 1 || MIC_MIN_P2P > 255 || MIC_RISE_P2P < 2 || \
    MIC_RISE_P2P > 127 || MIC_REARM_WINDOWS < 1 || MIC_REARM_WINDOWS > 255
#error "Use valid 8-bit microphone thresholds and a nonzero re-arm count"
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
    STATE_SENSOR_WAIT,
    STATE_STARTLED,
    STATE_DIZZY
} robot_state_t;

static volatile uint8_t cliff_latched = 1u;
static volatile uint8_t surface_stable_ticks = 0u;

/* All four safety checks use this same polarity, including the Timer2 ISR. */
static inline uint8_t ir_cliff_now(void)
{
    return ((PINB & _BV(PB0)) ? 1u : 0u) == IR_CLIFF_LEVEL;
}

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
    if (ir_cliff_now()) {
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
/* PB0 polarity is selected only by IR_CLIFF_LEVEL above. */

static void ir_init(void)
{
    DDRB &= (uint8_t)~_BV(PB0);
    PORTB |= _BV(PB0);                 /* Retain the existing input pull-up. */
}

/* --------------------------- Two servos ------------------------------ */
/* PB1=SG90 signal, PB2=SG92 signal. Timer2 tick = 8 us. */

static volatile uint8_t servo_target_ticks = SERVO_HOME_TICKS;
static volatile uint8_t servo_frame_count = 9;
static volatile uint8_t servo_pulse_active = 0;
static volatile uint16_t system_ticks_2ms = 0;

static inline void shock_sample_isr(void);
static void buzzer_init(void);
static void buzzer_stop(void);
static void buzzer_duration_tick_isr(void);

static void buzzer_init(void)
{
#if BUZZER_ENABLED
    DDRC |= _BV(BUZZER_PIN);
    PORTC &= (uint8_t)~_BV(BUZZER_PIN);
    buzzer_stop();
#endif
}

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
    shock_sample_isr();
    buzzer_duration_tick_isr();

    /* Safety is sampled every ~2.048 ms, even during OLED/sonar work.
     * One cliff sample brakes. Only the main FSM can clear the latch after
     * a continuously stable surface; short surface glitches cannot restart. */
    if (ir_cliff_now()) {
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

/* --------------------- MAX9814 analog microphone -------------------- */
/* Free-running ADC needs no timer. At 1 MHz, /16 gives a 62.5 kHz ADC
 * clock and about 4,808 samples/s. Read left-adjusted ADCH for 8-bit data.
 * The ISR only collects extrema; all detection/FSM work stays in main.
 * MAX9814 output is biased near 1.25 V, NOT VCC/2. Peak-to-peak detection
 * removes that DC bias. This detects sharp sounds, not speech or a clap's
 * identity; physical isolation and threshold tuning are still necessary. */
static volatile uint8_t mic_adc_min = 255u, mic_adc_max = 0u;
static uint16_t mic_started_at = 0, mic_last_window = 0;
static uint16_t mic_last_event = 0, mic_floor_q4 = 0;
static uint8_t mic_ready = 0, mic_armed = 0, mic_in_cooldown = 0;
static uint8_t mic_quiet_windows = 0, mic_previous_level = 0;
static uint8_t mic_event_pending = 0;

ISR(ADC_vect)
{
    uint8_t sample = ADCH;
    if (sample < mic_adc_min) {
        mic_adc_min = sample;
    }
    if (sample > mic_adc_max) {
        mic_adc_max = sample;
    }
}

static void microphone_init(uint16_t now)
{
    mic_adc_min = 255u;
    mic_adc_max = 0u;
    mic_started_at = mic_last_window = mic_last_event = now;
    mic_floor_q4 = 0;
    mic_ready = mic_armed = mic_in_cooldown = 0;
    mic_quiet_windows = mic_previous_level = mic_event_pending = 0;
#if MIC_ENABLED
    DDRA &= (uint8_t)~_BV(PA2);
    PORTA &= (uint8_t)~_BV(PA2);       /* Analog input: no internal pull-up. */
    ADCSRA = 0;
    ADMUX = _BV(REFS0) | _BV(ADLAR) | 2u; /* AVCC reference, ADC2. */
    SFIOR &= (uint8_t)~(_BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0));
    ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADIF) |
             _BV(ADPS2) | _BV(ADSC);  /* /16; clear old flag, start once. */
#endif
}

static void microphone_accept_level(uint8_t level, uint16_t now)
{
    uint16_t threshold = (mic_floor_q4 >> 4) + MIC_RISE_P2P;
    uint16_t target = (uint16_t)level << 4;
    uint8_t event = 0;

    if (threshold < MIC_MIN_P2P) {
        threshold = MIC_MIN_P2P;
    }
    if (mic_in_cooldown &&
        (uint16_t)(now - mic_last_event) >= MIC_COOLDOWN_TICKS) {
        mic_in_cooldown = 0;
    }

    if (!mic_ready) {
        /* Learn without reacting to power-up transients. The flag prevents
         * calibration from recurring when the 16-bit clock wraps. */
        if ((uint16_t)(now - mic_started_at) >= MIC_CALIBRATION_TICKS) {
            mic_ready = mic_armed = 1u;
        }
    } else {
        /* Compare against the learned background, not the immediately prior
         * peak window. OLED work makes those windows unequal in duration:
         * one clap can rise across two windows and fail a prior-window test.
         * Arming/cooldown still prevent repeated events from a held level. */
        if (mic_armed && !mic_in_cooldown && level >= threshold) {
            event = mic_event_pending = 1u;
            mic_last_event = now;
            mic_in_cooldown = 1u;
            mic_armed = 0;
            mic_quiet_windows = 0;
        }
        if (!mic_armed && !event) {
            if ((uint16_t)level + 6u < threshold) {
                if (mic_quiet_windows < MIC_REARM_WINDOWS) {
                    ++mic_quiet_windows;
                }
                if (mic_quiet_windows >= MIC_REARM_WINDOWS) {
                    mic_armed = 1u;
                }
            } else {
                mic_quiet_windows = 0;
            }
        }
    }

    /* Slowly follow ordinary background changes, faster when it gets quiet.
     * Do not teach a detected impulse to the background estimator. */
    if (!event) {
        if (target > mic_floor_q4) {
            mic_floor_q4 += (uint16_t)((target - mic_floor_q4 + 15u) >> 4);
        } else {
            mic_floor_q4 -= (uint16_t)((mic_floor_q4 - target + 7u) >> 3);
        }
    }
    mic_previous_level = level;
}

static void microphone_service(uint16_t now)
{
    uint8_t saved_sreg, low, high;
    if (!MIC_ENABLED ||
        (uint16_t)(now - mic_last_window) < MIC_WINDOW_TICKS) {
        return;
    }
    mic_last_window = now;
    saved_sreg = SREG;
    cli();
    low = mic_adc_min;
    high = mic_adc_max;
    mic_adc_min = 255u;
    mic_adc_max = 0u;
    SREG = saved_sreg;
    if (high >= low) {                /* Skip a window with no ADC samples. */
        microphone_accept_level((uint8_t)(high - low), now);
    }
}

/* -------------------------- SW-18015P shock --------------------------- */
/* The common SW-18015P comparator module presents a digital DO signal and
 * typically goes LOW on vibration. PC3 is currently unused in this robot.
 * We sample it from the existing ~2.048 ms Timer2 ISR so a short shock pulse
 * is not lost while OLED/sonar code is running in the main loop. */
static volatile uint8_t shock_event_pending = 0;
static volatile uint8_t shock_armed = 1u;
static volatile uint8_t shock_rearm_ticks = 0u;

static inline uint8_t shock_active_now(void)
{
    uint8_t high = (PINC & _BV(PC3)) ? 1u : 0u;
#if SHOCK_ACTIVE_LOW
    return (uint8_t)!high;
#else
    return high;
#endif
}

static void shock_init(void)
{
    MCUCSR |= _BV(JTD);
    MCUCSR |= _BV(JTD);             /* Disable JTAG so PC3 is normal GPIO. */

    DDRC &= (uint8_t)~_BV(PC3);     /* PC3 = digital input. */
    PORTC |= _BV(PC3);              /* Idle-high bias for active-low modules. */
}

static inline void shock_sample_isr(void)
{
    if (shock_active_now()) {
        if (shock_armed) {
            shock_event_pending = 1u;
            shock_armed = 0u;
        }
        shock_rearm_ticks = SHOCK_REARM_TICKS;
    } else if (shock_rearm_ticks != 0u) {
        --shock_rearm_ticks;
    } else {
        shock_armed = 1u;
    }
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

static volatile uint8_t buzzer_active = 0u;

static void sonar_service(uint16_t now)
{
#if BUZZER_ENABLED
    if (buzzer_active) {
        return;
    }
#endif
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
static uint8_t startle_was_turning = 0;
static uint16_t startle_turn_since = 0;

/* Shock dizziness is a visual overlay, not a motion state.  A shock reacts
 * immediately from any robot state without changing motor targets. */
static uint8_t dizzy_overlay = 0u;
static uint16_t dizzy_overlay_since = 0u;

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
    case STATE_STARTLED:
        state_duration = STARTLE_PAUSE_TICKS;
        motors_set_targets(0, 0);
        break;
    case STATE_DIZZY:
        /* Kept for compatibility; shock handling no longer enters this state. */
        state_duration = DIZZY_PAUSE_TICKS;
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
    if (!ir_cliff_now() &&
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
    uint8_t clap = mic_event_pending;
    uint8_t shock = shock_event_pending;
    /* Consume once even if a higher-priority state ignores this event.
     * A shake during a cliff/pat must never be queued for a later restart. */
    mic_event_pending = 0;
    shock_event_pending = 0;

    /* GLOBAL shock reaction.  This is checked before all normal FSM logic,
     * including cliff, touch, sonar, startup, and blocked states.
     * IMPORTANT: it does not change robot_state or motor targets. */
    if (shock) {
        dizzy_overlay = 1u;
        dizzy_overlay_since = now;
    }

    /* Each shake gets a fresh dizziness window. */
    if (dizzy_overlay &&
        (uint16_t)(now - dizzy_overlay_since) >= DIZZY_PAUSE_TICKS) {
        dizzy_overlay = 0u;
    }

    if (touch != previous_touch) {
        random_state ^= (uint16_t)(now + (uint16_t)touch * 257u);
        (void)random_next();
        previous_touch = touch;
    }

    /* Initial inhibit is not a real cliff encounter. Complete calibration
     * and establish the floor before leaving STARTUP; a real cliff below
     * still enters CLIFF immediately. */
    if (robot_state == STATE_STARTUP && !ir_cliff_now()) {
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

    /* A sound can interrupt motion/ordinary pauses, but never overrides
     * cliff, touch, invalid sonar, or a latched blocked encounter above. */
    if (clap && (robot_state == STATE_CRUISE ||
                 robot_state == STATE_CURIOUS_PAUSE ||
                 robot_state == STATE_OBSTACLE_PAUSE ||
                 robot_state == STATE_TURN || robot_state == STATE_SETTLE)) {
        startle_was_turning = (robot_state == STATE_TURN);
        startle_turn_since = state_since;
        robot_enter(STATE_STARTLED, now);
        return;
    }

    elapsed = (uint16_t)(now - state_since); /* Wrap-safe elapsed time. */
    switch (robot_state) {
    case STATE_STARTLED:
        if (elapsed >= state_duration) {
            /* Re-evaluate today's path, not the path when the clap happened.
             * Retain the chosen turn and its original timeout when resuming. */
            if (sonar_obstacle && startle_was_turning) {
                if ((uint16_t)(now - startle_turn_since) >= TURN_TIMEOUT_TICKS) {
                    avoidance_blocked = 1u;
                    robot_enter(STATE_BLOCKED, now);
                } else {
                    robot_enter(STATE_TURN, now);
                    state_since = startle_turn_since;
                }
            } else {
                robot_enter(sonar_obstacle ? STATE_OBSTACLE_PAUSE : STATE_CRUISE,
                            now);
            }
        }
        break;
    case STATE_DIZZY:
        if (elapsed >= state_duration) {
            robot_enter(sonar_obstacle ? STATE_OBSTACLE_PAUSE : STATE_CRUISE,
                        now);
        }
        break;
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

/* -------------------- Expressive eyes: fixed-memory C ------------------ */
/* Visual ideas inspired by FluxGarage/RoboEyes: expressive lids, curiosity,
 * happy bounce, confusion, and autonomous blinking. This is an independently
 * written C engine for the existing SH1106 driver, not copied Arduino code.
 * Reference: https://github.com/FluxGarage/RoboEyes (GPL-3.0-or-later).
 *
 * Geometry is integer-only. Poses depend on elapsed 2.048 ms ticks, not FPS.
 * Eye randomness has its own generator and never consumes the motion RNG.
 * Drawing touches only the existing 80x32 (320-byte) display window.
 */
typedef enum {
    FACE_WAKE, FACE_DRIVE, FACE_CURIOUS, FACE_AVOID, FACE_BLOCKED,
    FACE_SENSOR_WAIT, FACE_PET, FACE_CLIFF, FACE_STARTLED, FACE_DIZZY, FACE_RECOVER
} face_mode_t;

typedef struct {
    int8_t x, y;
    uint8_t width, height;
    uint8_t lid_drop;
    int8_t lid_slope;
} eye_shape_t;

typedef struct {
    uint8_t half_period_counts;   /* Timer0 counts per half-cycle (8 us each). */
    uint8_t duration_ticks;       /* Number of ~2.048 ms Timer2 ticks. */
} buzzer_note_t;

/* Short, distinct sound signatures for every face animation. Frequencies are
 * intentionally clustered around the KY-006's useful acoustic range. */
static const buzzer_note_t buzzer_wake[] PROGMEM = {
    {42u, 10u}, {32u, 10u}
};
static const buzzer_note_t buzzer_drive[] PROGMEM = {
    {31u, 12u}
};
static const buzzer_note_t buzzer_curious[] PROGMEM = {
    {38u, 7u}, {31u, 7u}, {38u, 7u}
};
static const buzzer_note_t buzzer_avoid[] PROGMEM = {
    {26u, 8u}, {35u, 8u}
};
static const buzzer_note_t buzzer_blocked[] PROGMEM = {
    {42u, 9u}, {42u, 9u}
};
static const buzzer_note_t buzzer_sensor_wait[] PROGMEM = {
    {34u, 7u}, {28u, 7u}
};
static const buzzer_note_t buzzer_pet[] PROGMEM = {
    {36u, 6u}, {30u, 6u}, {25u, 8u}
};
static const buzzer_note_t buzzer_cliff[] PROGMEM = {
    {25u, 8u}, {50u, 10u}
};
static const buzzer_note_t buzzer_startled[] PROGMEM = {
    {25u, 6u}, {35u, 9u}
};
static const buzzer_note_t buzzer_dizzy[] PROGMEM = {
    {28u, 6u}, {34u, 6u}, {40u, 6u}, {34u, 6u}
};
static const buzzer_note_t buzzer_recover[] PROGMEM = {
    {40u, 7u}, {34u, 8u}
};

static volatile uint8_t buzzer_note_index = 0u;
static volatile uint8_t buzzer_note_ticks_left = 0u;
static volatile uint8_t buzzer_half_period = 31u;
static volatile const buzzer_note_t *buzzer_sequence = 0;
static volatile uint8_t buzzer_sequence_length = 0u;

static uint8_t buzzer_half_period_for(uint8_t counts)
{
    return counts < 2u ? 2u : counts;
}

static void buzzer_stop(void)
{
#if BUZZER_ENABLED
    TIMSK &= (uint8_t)~_BV(OCIE0);
#endif
    buzzer_active = 0u;
    buzzer_note_index = 0u;
    buzzer_note_ticks_left = 0u;
    buzzer_sequence = 0;
    buzzer_sequence_length = 0u;
    PORTC &= (uint8_t)~_BV(BUZZER_PIN);
}

static void buzzer_load_note(void)
{
    buzzer_note_t note;
    if (!buzzer_sequence || buzzer_note_index >= buzzer_sequence_length) {
        buzzer_stop();
        return;
    }

    note.half_period_counts = pgm_read_byte(&buzzer_sequence[buzzer_note_index].half_period_counts);
    note.duration_ticks = pgm_read_byte(&buzzer_sequence[buzzer_note_index].duration_ticks);
    buzzer_half_period = buzzer_half_period_for(note.half_period_counts);
    buzzer_note_ticks_left = note.duration_ticks > BUZZER_MAX_NOTE_TICKS ?
                             BUZZER_MAX_NOTE_TICKS : note.duration_ticks;

    /* Schedule the first compare relative to the current Timer0 count. */
    TIFR = _BV(OCF0);
    OCR0 = (uint8_t)(TCNT0 + buzzer_half_period);
}

static void buzzer_start_sequence(const buzzer_note_t *sequence, uint8_t length)
{
#if BUZZER_ENABLED
    uint8_t saved_sreg = SREG;
    cli();

    /* Stop any previous sound so the new animation gets its own signature. */
    TIMSK &= (uint8_t)~_BV(OCIE0);
    PORTC &= (uint8_t)~_BV(BUZZER_PIN);

    buzzer_sequence = sequence;
    buzzer_sequence_length = length;
    buzzer_note_index = 0u;
    buzzer_active = 1u;
    buzzer_load_note();
    TIMSK |= _BV(OCIE0);

    SREG = saved_sreg;
#else
    (void)sequence;
    (void)length;
#endif
}

static void buzzer_start_for_face(face_mode_t mode)
{
#if BUZZER_ENABLED
    switch (mode) {
    case FACE_WAKE:
        buzzer_start_sequence(buzzer_wake, (uint8_t)(sizeof(buzzer_wake) / sizeof(buzzer_wake[0])));
        break;
    case FACE_DRIVE:
        buzzer_start_sequence(buzzer_drive, (uint8_t)(sizeof(buzzer_drive) / sizeof(buzzer_drive[0])));
        break;
    case FACE_CURIOUS:
        buzzer_start_sequence(buzzer_curious, (uint8_t)(sizeof(buzzer_curious) / sizeof(buzzer_curious[0])));
        break;
    case FACE_AVOID:
        buzzer_start_sequence(buzzer_avoid, (uint8_t)(sizeof(buzzer_avoid) / sizeof(buzzer_avoid[0])));
        break;
    case FACE_BLOCKED:
        buzzer_start_sequence(buzzer_blocked, (uint8_t)(sizeof(buzzer_blocked) / sizeof(buzzer_blocked[0])));
        break;
    case FACE_SENSOR_WAIT:
        buzzer_start_sequence(buzzer_sensor_wait, (uint8_t)(sizeof(buzzer_sensor_wait) / sizeof(buzzer_sensor_wait[0])));
        break;
    case FACE_PET:
        buzzer_start_sequence(buzzer_pet, (uint8_t)(sizeof(buzzer_pet) / sizeof(buzzer_pet[0])));
        break;
    case FACE_CLIFF:
        buzzer_start_sequence(buzzer_cliff, (uint8_t)(sizeof(buzzer_cliff) / sizeof(buzzer_cliff[0])));
        break;
    case FACE_STARTLED:
        buzzer_start_sequence(buzzer_startled, (uint8_t)(sizeof(buzzer_startled) / sizeof(buzzer_startled[0])));
        break;
    case FACE_DIZZY:
        buzzer_start_sequence(buzzer_dizzy, (uint8_t)(sizeof(buzzer_dizzy) / sizeof(buzzer_dizzy[0])));
        break;
    case FACE_RECOVER:
        buzzer_start_sequence(buzzer_recover, (uint8_t)(sizeof(buzzer_recover) / sizeof(buzzer_recover[0])));
        break;
    default:
        buzzer_stop();
        break;
    }
#else
    (void)mode;
#endif
}

ISR(TIMER0_COMP_vect)
{
    if (!buzzer_active) return;

    PORTC ^= _BV(BUZZER_PIN);
    OCR0 = (uint8_t)(OCR0 + buzzer_half_period);
}

static void buzzer_duration_tick_isr(void)
{
#if BUZZER_ENABLED
    if (!buzzer_active) return;

    if (buzzer_note_ticks_left != 0u) {
        --buzzer_note_ticks_left;
    }
    if (buzzer_note_ticks_left == 0u) {
        ++buzzer_note_index;
        buzzer_load_note();
    }
#endif
}

static face_mode_t face_mode = FACE_WAKE;
static uint8_t face_initialized = 0;
static uint16_t face_since = 0;
static uint16_t eye_rng = 0xB47Du;
static uint8_t face_turn_left = 0;
static touch_state_t face_touch = TOUCH_NONE;
static int8_t curious_x = 4;
static eye_shape_t eye_pose[2];
static uint8_t frame_smile = 0, frame_heart_y = 0;
static uint8_t frame_alert_mark = 0, frame_sparkle = 0, frame_dizzy = 0;
static uint8_t eye_page_index = 0;
static uint8_t blink_active = 0, blink_min_shown = 0, blink_double_pending = 0;
static uint16_t blink_since = 0, blink_wait_since = 0, blink_wait_ticks = 1200u;

static const int8_t eye_breath_table[] PROGMEM = {
    0, 0, 1, 1, 2, 2, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0
};
static const int8_t eye_scan_table[] PROGMEM = {
    -4, -4, -3, -1, 1, 3, 4, 4, 4, 3, 1, -1, -3, -4, -4, -4
};
static const uint8_t eye_pop_table[] PROGMEM = {8, 6, 3, 0};
static const uint8_t eye_smile_rise[] PROGMEM = {4, 4, 4, 3, 3, 2, 1, 1, 0};
static const uint8_t eye_heart_columns[] PROGMEM = {6, 15, 30, 15, 6};
static const uint8_t low_bit_masks[] PROGMEM = {0, 1, 3, 7, 15, 31, 63, 127, 255};
/* floor(sqrt(radius^2 - corner_x^2)), radius/corner_x each 0..7. */
static const uint8_t eye_corner_lut[8][8] PROGMEM = {
    {0,0,0,0,0,0,0,0}, {1,0,0,0,0,0,0,0},
    {2,1,0,0,0,0,0,0}, {3,2,2,0,0,0,0,0},
    {4,3,3,2,0,0,0,0}, {5,4,4,4,3,0,0,0},
    {6,5,5,5,4,3,0,0}, {7,6,6,6,5,4,3,0}
};

static uint16_t eye_random(void)
{
    uint16_t x = eye_rng;
    x ^= (uint16_t)(x << 7);
    x ^= (uint16_t)(x >> 9);
    x ^= (uint16_t)(x << 8);
    eye_rng = x ? x : 0xB47Du;
    return eye_rng;
}

static int8_t eye_lerp(int8_t from, int8_t to, uint16_t elapsed, uint16_t duration)
{
    if (elapsed >= duration) return to;
    /* Callers use |delta| <= 30 and duration <= 180, fitting signed 16-bit. */
    return (int8_t)(from + ((int16_t)(to - from) * (int16_t)elapsed) /
                            (int16_t)duration);
}

static void eyes_set_mode(face_mode_t mode, uint16_t now)
{
    if (face_initialized && mode == face_mode) return;
    face_initialized = 1u;
    face_mode = mode;
    face_since = now;
    buzzer_start_for_face(mode);
    blink_active = blink_min_shown = blink_double_pending = 0;
    blink_wait_since = now;
    blink_wait_ticks = (uint16_t)(1000u + eye_random() % 1400u);
    if (mode == FACE_CURIOUS) {
        curious_x = (eye_random() & 1u) ? 4 : -4;
    }
}

/* 0..16 opening factor: fast close, a visible closed frame, gentler reopen.
 * The minimum-frame guard keeps blinks visible even after a delayed OLED page.
 * No sleep/delay: safety, touch, sonar and servo work continue in main/ISRs. */
static uint8_t eyes_blink_opening(uint16_t now)
{
    uint16_t elapsed;
    if (!blink_active) {
        if ((uint16_t)(now - blink_wait_since) < blink_wait_ticks) return 16u;
        blink_active = 1u;
        blink_min_shown = 0u;
        blink_since = now;
    }
    elapsed = (uint16_t)(now - blink_since);
    if (elapsed < 20u) return (uint8_t)(16u - (elapsed * 16u) / 20u);
    if (elapsed < 50u || !blink_min_shown) {
        blink_min_shown = 1u;
        return 0u;
    }
    if (elapsed < 90u) return (uint8_t)(((elapsed - 50u) * 16u) / 40u);
    blink_active = 0;
    blink_wait_since = now;
    if (blink_double_pending) {
        blink_double_pending = 0;
        blink_wait_ticks = (uint16_t)(1000u + eye_random() % 1400u);
    } else if ((eye_random() & 7u) == 0u) {
        blink_double_pending = 1u;
        blink_wait_ticks = 65u;
    } else {
        blink_wait_ticks = (uint16_t)(1000u + eye_random() % 1400u);
    }
    return 16u;
}

static void eyes_advance_frame(uint16_t now)
{
    uint16_t elapsed = (uint16_t)(now - face_since);
    int8_t gx = 0, gy = 0;
    uint8_t width = 20u, left_height = 22u, right_height = 22u;
    uint8_t left_drop = 0, right_drop = 0, allow_blink = 0;
    int8_t left_slope = 0, right_slope = 0;
    uint8_t index = (uint8_t)((elapsed >> 6) & 15u);
    frame_smile = frame_heart_y = frame_alert_mark = frame_sparkle = frame_dizzy = 0;

    switch (face_mode) {
    case FACE_WAKE:
        /* Sleepy line -> wide awake -> settle, within the existing startup. */
        if (elapsed < 120u) {
            left_height = (uint8_t)eye_lerp(2, 28, elapsed, 120u);
        } else {
            left_height = (uint8_t)eye_lerp(28, 22, elapsed - 120u, 80u);
        }
        right_height = left_height;
        break;
    case FACE_DRIVE:
        /* Attention stays forward while the robot drives. */
        left_height = (uint8_t)(22 + (int8_t)pgm_read_byte(&eye_breath_table[index]));
        right_height = left_height;
        allow_blink = 1u;
        break;
    case FACE_CURIOUS:
        gx = eye_lerp(0, curious_x, elapsed, 24u);
        gy = -1;
        /* Raise the eye toward the glance, while the other eye squints. */
        left_height = gx < 0 ? 26u : 16u;
        right_height = gx < 0 ? 16u : 26u;
        if (gx < 0) right_drop = 3u;
        else left_drop = 3u;
        break;
    case FACE_AVOID:
        gx = face_turn_left ? TURN_GAZE_POLARITY * TURN_GAZE_SHIFT :
                             -TURN_GAZE_POLARITY * TURN_GAZE_SHIFT;
        if (elapsed < 70u) {
            /* Notice the obstacle first, then narrow into a focused look. */
            uint8_t growth = pgm_read_byte(&eye_pop_table[(elapsed / 24u) & 3u]);
            width = (uint8_t)(18u + growth);
            left_height = right_height = (uint8_t)(22u + growth);
        } else {
            left_height = right_height = 24u;
            left_drop = right_drop = 4u;
            left_slope = 4;
            right_slope = -4;
        }
        break;
    case FACE_BLOCKED:
    case FACE_SENSOR_WAIT:
        /* Continuous scanning, with alternating skeptical/asymmetric lids. */
        gx = (int8_t)pgm_read_byte(&eye_scan_table[(elapsed >> 5) & 15u]);
        if (face_mode == FACE_BLOCKED) {
            left_height = right_height = 20u;
            left_drop = right_drop = 4u;
            left_slope = -3;
            right_slope = 3;
            gy = (int8_t)((elapsed >> 7) & 1u);
        } else {
            left_height = gx < 0 ? 26u : 16u;
            right_height = gx < 0 ? 16u : 26u;
            left_drop = gx < 0 ? 0u : 3u;
            right_drop = gx < 0 ? 3u : 0u;
            gy = -1;
        }
        allow_blink = 1u;
        break;
    case FACE_PET:
        gx = face_touch == TOUCH_LEFT ? -PAT_EYE_X_SHIFT :
             face_touch == TOUCH_RIGHT ? PAT_EYE_X_SHIFT : 0;
        gy = (int8_t)(PAT_EYE_Y_SHIFT - (int8_t)((elapsed >> 6) & 1u));
        frame_smile = 1u;
        frame_heart_y = (uint8_t)(23u - (elapsed >> 6) % 6u);
        left_height = right_height = 3u;
        break;
    case FACE_CLIFF: {
        uint8_t growth = pgm_read_byte(&eye_pop_table[(elapsed / EYE_POP_STEP_TICKS) & 3u]);
        width = (uint8_t)(18u + growth);
        left_height = right_height = (uint8_t)(22u + growth);
        frame_alert_mark = (uint8_t)(((elapsed >> 6) & 1u) == 0u);
        break;
    }
    case FACE_STARTLED:
        /* Keep the requested thin vertical shock eyes, then recover shape. */
        if (elapsed < 110u) {
            width = 8u;
            left_height = right_height = 30u;
            gx = ((elapsed >> 4) & 1u) ? 1 : -1;
            frame_sparkle = 1u;
        } else {
            width = (uint8_t)eye_lerp(8, 20, elapsed - 110u, 110u);
            left_height = right_height =
                (uint8_t)eye_lerp(30, 22, elapsed - 110u, 110u);
        }
        break;
    case FACE_DIZZY: {
        /* Keep the established rounded eyes recognizable. During the first
         * part of the reaction the eyes sway and tilt; later they settle.
         * The pupils roll in an orbit so the dizziness reads naturally. */
        uint16_t motion_elapsed = elapsed < 350u ? elapsed : 350u;
        uint8_t dizzy_index = (uint8_t)((motion_elapsed >> 4) & 15u);
        int8_t wobble = (int8_t)pgm_read_byte(&eye_scan_table[dizzy_index]);
        uint8_t pulse = (uint8_t)((motion_elapsed >> 5) & 3u);
        int8_t settle_shift;

        if (elapsed >= 350u) {
            settle_shift = eye_lerp(4, 0,
                                    (uint16_t)(elapsed - 350u), 150u);
        } else {
            settle_shift = 4;
        }

        gx = (int8_t)((wobble * settle_shift) / 4);
        gy = (pulse == 0u) ? -2 :
             (pulse == 2u) ? 2 : 0;

        /* Alternate the eyelids slightly to make the face feel unsteady. */
        left_height = (pulse == 1u) ? 20u : 23u;
        right_height = (pulse == 3u) ? 20u : 23u;
        left_drop = (dizzy_index & 4u) ? 2u : 0u;
        right_drop = (dizzy_index & 4u) ? 0u : 2u;
        left_slope = (dizzy_index & 4u) ? -2 : 2;
        right_slope = -left_slope;

        frame_dizzy = 1u;
        break;
    }
    case FACE_RECOVER:
        width = (uint8_t)eye_lerp(24, 20, elapsed, 75u);
        left_height = right_height = (uint8_t)eye_lerp(8, 22, elapsed, 75u);
        break;
    }

    if (allow_blink) {
        uint8_t open = eyes_blink_opening(now);
        left_height = (uint8_t)(((uint16_t)left_height * open) >> 4);
        right_height = (uint8_t)(((uint16_t)right_height * open) >> 4);
        if (left_height < 2u) left_height = 2u;
        if (right_height < 2u) right_height = 2u;
        /* Eyelids must not erase a nearly closed eye. */
        if (open < 8u) left_drop = right_drop = 0;
        if (open < 8u) left_slope = right_slope = 0;
    }
    eye_pose[0] = (eye_shape_t){(int8_t)(42 + gx), (int8_t)(32 + gy),
                               width, left_height, left_drop, left_slope};
    eye_pose[1] = (eye_shape_t){(int8_t)(86 + gx), (int8_t)(32 + gy),
                               width, right_height, right_drop, right_slope};
}

static void robot_expression_service(touch_state_t touch, uint16_t now)
{
    action_t action = ACTION_DRIVE;
    face_mode_t mode = FACE_DRIVE;
    if (dizzy_overlay) {
        mode = FACE_DIZZY;
    } else if (cliff_latched || robot_state == STATE_CLIFF) {
        action = ACTION_CLIFF;
        mode = FACE_CLIFF;
    } else if (robot_state == STATE_PETTING) {
        action = ACTION_PETTING;
        mode = FACE_PET;
    } else if (robot_state == STATE_STARTLED) {
        mode = FACE_STARTLED;

    } else if (robot_state == STATE_BLOCKED) {
        action = ACTION_OBSTACLE;
        mode = FACE_BLOCKED;
    } else if (robot_state == STATE_SENSOR_WAIT) {
        action = ACTION_OBSTACLE;
        mode = FACE_SENSOR_WAIT;
    } else if (robot_state == STATE_OBSTACLE_PAUSE || robot_state == STATE_TURN) {
        action = ACTION_OBSTACLE;
        mode = FACE_AVOID;
    } else if (robot_state == STATE_CURIOUS_PAUSE) {
        mode = FACE_CURIOUS;
    } else if (robot_state == STATE_STARTUP) {
        mode = FACE_WAKE;
    } else if (robot_state == STATE_SETTLE) {
        mode = FACE_RECOVER;
    }
    /* Immediate actuator priorities are unchanged; drawing never drives them. */
    servos_update(action, touch, now);
    face_touch = action == ACTION_PETTING ? touch : TOUCH_NONE;
    face_turn_left = turn_left;
    eyes_set_mode(mode, now);
}

/* -------------------- Column-span OLED rasterizer -------------------- */
/* A rounded eye is a short vertical span for each X. Build at most four page
 * masks per span instead of testing all 2,560 window pixels every frame. */
static void eye_buffer_clear(void)
{
    uint8_t page, x;
    for (page = 0; page < OLED_EYE_PAGE_COUNT; ++page)
        for (x = 0; x < OLED_EYE_WIDTH; ++x) eye_frame_buffer[page][x] = 0;
}

static void eye_buffer_span(int16_t x, int16_t top, int16_t bottom)
{
    uint8_t local_x, first, last, page;
    const int16_t first_y = (int16_t)OLED_FIRST_EYE_PAGE * 8;
    const int16_t end_y = first_y + (int16_t)OLED_EYE_PAGE_COUNT * 8;
    if (x < (int16_t)OLED_EYE_X || x >= (int16_t)(OLED_EYE_X + OLED_EYE_WIDTH) ||
        bottom < first_y || top >= end_y || top > bottom)
        return;
    if (top < first_y) top = first_y;
    if (bottom >= end_y) bottom = end_y - 1;
    local_x = (uint8_t)(x - OLED_EYE_X);
    first = (uint8_t)(top - first_y);
    last = (uint8_t)(bottom - first_y);
    for (page = (uint8_t)(first >> 3); page <= (uint8_t)(last >> 3); ++page) {
        uint8_t low = page == (first >> 3) ? (first & 7u) : 0u;
        uint8_t high = page == (last >> 3) ? (uint8_t)((last & 7u) + 1u) : 8u;
        uint8_t mask = (uint8_t)(pgm_read_byte(&low_bit_masks[high]) &
                                 (uint8_t)~pgm_read_byte(&low_bit_masks[low]));
        eye_frame_buffer[page][local_x] |= mask;
    }
}

static void draw_eye_shape(const eye_shape_t *eye)
{
    int8_t dx;
    uint8_t half_w = eye->width / 2u, half_h = eye->height / 2u;
    uint8_t radius = eye->width < 12u ? 2u : 5u;
    if (radius > half_w) radius = half_w;
    if (radius > half_h) radius = half_h;
    for (dx = -(int8_t)half_w; dx <= (int8_t)half_w; ++dx) {
        uint8_t distance = (uint8_t)(dx < 0 ? -dx : dx);
        uint8_t span = half_h;
        int16_t top, bottom;
        if (distance > half_w - radius) {
            uint8_t corner_x = (uint8_t)(distance - (half_w - radius));
            span = (uint8_t)(half_h - radius +
                            pgm_read_byte(&eye_corner_lut[radius][corner_x]));
        }
        top = (int16_t)eye->y - span;
        bottom = (int16_t)eye->y + span;
        if (eye->lid_drop || eye->lid_slope) {
            int16_t lid = (int16_t)eye->y - half_h + eye->lid_drop;
            if (half_w) lid += ((int16_t)eye->lid_slope * dx) / half_w;
            if (lid > top) top = lid;
        }
        eye_buffer_span((int16_t)eye->x + dx, top, bottom);
    }
}


static void eye_buffer_clear_span(int16_t x, int16_t top, int16_t bottom)
{
    uint8_t local_x, first, last, page;
    const int16_t first_y = (int16_t)OLED_FIRST_EYE_PAGE * 8;
    const int16_t end_y = first_y + (int16_t)OLED_EYE_PAGE_COUNT * 8;
    if (x < (int16_t)OLED_EYE_X ||
        x >= (int16_t)(OLED_EYE_X + OLED_EYE_WIDTH) ||
        bottom < first_y || top >= end_y || top > bottom)
        return;
    if (top < first_y) top = first_y;
    if (bottom >= end_y) bottom = end_y - 1;
    local_x = (uint8_t)(x - OLED_EYE_X);
    first = (uint8_t)(top - first_y);
    last = (uint8_t)(bottom - first_y);
    for (page = (uint8_t)(first >> 3); page <= (uint8_t)(last >> 3); ++page) {
        uint8_t low = page == (first >> 3) ? (first & 7u) : 0u;
        uint8_t high = page == (last >> 3) ? (uint8_t)((last & 7u) + 1u) : 8u;
        uint8_t mask = (uint8_t)(pgm_read_byte(&low_bit_masks[high]) &
                                 (uint8_t)~pgm_read_byte(&low_bit_masks[low]));
        eye_frame_buffer[page][local_x] &= (uint8_t)~mask;
    }
}

static void draw_dizzy_pupil(int16_t cx, int16_t cy, uint8_t phase)
{
    /* 8-point oval orbit. Left/right pupils are deliberately out of phase. */
    static const int8_t pupil_x[] PROGMEM = {0, 3, 5, 3, 0, -3, -5, -3};
    static const int8_t pupil_y[] PROGMEM = {-2, -2, 0, 2, 3, 2, 0, -2};
    uint8_t p = (uint8_t)(phase & 7u);
    int16_t px = cx + (int8_t)pgm_read_byte(&pupil_x[p]);
    int16_t py = cy + (int8_t)pgm_read_byte(&pupil_y[p]);
    int8_t dx;

    /* Carve a small rounded black pupil from the filled eye. */
    for (dx = -2; dx <= 2; ++dx) {
        uint8_t distance = (uint8_t)(dx < 0 ? -dx : dx);
        uint8_t half_h = distance == 2u ? 1u : 2u;
        eye_buffer_clear_span(px + dx, py - half_h, py + half_h);
    }

    /* One white highlight gives the pupil a more natural eye-like appearance. */
    eye_buffer_span(px - 1, py - 1, py - 1);
}

static void eyes_render_frame(void)
{
    uint8_t i;
    eye_buffer_clear();
    if (frame_smile) {
        for (i = 0; i < 2u; ++i) {
            int8_t dx;
            for (dx = -CLOSED_EYE_HALF_WIDTH; dx <= CLOSED_EYE_HALF_WIDTH; ++dx) {
                uint8_t distance = (uint8_t)(dx < 0 ? -dx : dx);
                int16_t y = (int16_t)eye_pose[i].y - pgm_read_byte(&eye_smile_rise[distance]);
                eye_buffer_span((int16_t)eye_pose[i].x + dx, y, y + 2);
            }
        }
    } else {
        draw_eye_shape(&eye_pose[0]);
        draw_eye_shape(&eye_pose[1]);
        if (frame_dizzy) {
            uint8_t phase = (uint8_t)((system_ticks_read() >> 4) & 7u);
            draw_dizzy_pupil(eye_pose[0].x, eye_pose[0].y, phase);
            draw_dizzy_pupil(eye_pose[1].x, eye_pose[1].y, (uint8_t)(phase + 4u));
        }
    }
    if (frame_heart_y) {
        uint8_t x, y;
        for (x = 0; x < 5u; ++x) {
            uint8_t column = pgm_read_byte(&eye_heart_columns[x]);
            for (y = 0; y < 5u; ++y)
                if (column & _BV(y)) eye_buffer_span(62 + x, frame_heart_y + y, frame_heart_y + y);
        }
    }
    if (frame_alert_mark) {
        eye_buffer_span(63, 19, 24);
        eye_buffer_span(64, 19, 24);
        eye_buffer_span(63, 27, 28);
        eye_buffer_span(64, 27, 28);
    }
    if (frame_sparkle) {
        eye_buffer_span(63, 20, 26);
        for (i = 0; i < 5u; ++i) eye_buffer_span(61 + i, 23, 23);
        eye_buffer_span(58, 20, 20);
        eye_buffer_span(68, 27, 27);
    }
}

static void oled_animation_service(void)
{
    uint8_t display_page;
    if (!oled_ok) return;
    /* Keep every frame stable until all four pages finish, including changes
     * of expression during a transfer. New poses are sampled at a boundary. */
    if (eye_page_index == 0u) {
        eyes_advance_frame(system_ticks_read());
        eyes_render_frame();
    }
    display_page = (uint8_t)(OLED_FIRST_EYE_PAGE + eye_page_index);
    if (!oled_set_position(display_page, OLED_EYE_X) ||
        !oled_write_data(eye_frame_buffer[eye_page_index], OLED_EYE_WIDTH)) {
        twi_bus_recover();
        return;
    }
    if (++eye_page_index >= OLED_EYE_PAGE_COUNT) eye_page_index = 0;
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
    shock_init();
    buzzer_init();
    sei();

    /* Motors remain actively braked during display power-up/recovery. */
    oled_init();
    sonar_last_trigger_tick =
        (uint16_t)(system_ticks_read() - ULTRASONIC_PERIOD_TICKS);
    touch_start_tick = system_ticks_read();
    robot_begin(touch_start_tick);
    microphone_init(touch_start_tick);

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

        microphone_service(now);
        /* Apply known safety/touch/sound state before another sonar sample. */
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
