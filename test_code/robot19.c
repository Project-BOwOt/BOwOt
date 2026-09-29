/*
 * Robot firmware: RoboEyes-inspired C expressions, working microphone and FSM.
 * ATmega32, actual CPU clock 1 MHz; defining F_CPU does not set fuse bits.
 * Existing wiring is unchanged; add MAX9814 OUT to PA2/ADC2 (DIP pin 38):
 *   L298N ENA PD5, ENB PD4; IN1 PD2, IN2 PD3, IN3 PD6, IN4 PD1.
 *   IR OUT PB0 (polarity: IR_CLIFF_LEVEL); SG90 PB1; SG92 PB2.
 *   HC-SR04 TRIG PB3 / ECHO PD0; OLED SCL PC0 / SDA PC1.
 *   TTP223 left PA0 / right PA1; SW-18015P DO PC3; AVCC powered; all grounds common.
 *   MAX9814 VDD=regulated 5V, GND=common, OUT=PA2, GAIN=VDD, A/R=GND.
 *   AVCC pin 30=5V; AREF pin 32=100nF to GND (no external voltage).
 *   Never wire AREF directly to GND. AVCC is required even with MIC_ENABLED=0.
 * Remove L298N ENA/ENB jumpers. The only automatic reverse is a short,
 * capped backup away from a latched cliff (STATE_CLIFF_BACKUP below);
 * every other state still only ever drives forward.
 *
 * ADDED (all on free PORTC pins; JTAG is already disabled by shock_init()):
 *   KY-040 CLK PC4, DT PC5, SW PC6 (+ to 5V, GND common).
 *   DHT11 DATA PC7 (needs a 4.7-10 k pull-up to 5V; 3-pin modules have one).
 *   KY-040: turn = pick a mode, push = select (or exit Weather Mode).
 *   WEATHER, POMODORO and GAME are stationary.
 *
 * Front cliff sensors: left PB0, right PB4; polarity is IR_CLIFF_LEVEL.
 * Cliff recovery: longer straight reverse, confirm floor, then randomly
 * turn left/right approximately 90 degrees. Tune CLIFF_TURN_TICKS on the
 * actual chassis: there is no wheel encoder/gyro to measure an exact angle.
 *
 * NEW: active buzzer control PA3 (40-pin DIP pin 37), active HIGH.
 * Use a transistor driver for a bare buzzer; see robot19_notes.md.
 * Modes: ROAM, WEATHER, POMODORO, GAME. Existing sensor wiring is unchanged.
 * Pomodoro: turn to set minutes, push; set seconds, push to start.
 * Push while running/done returns to the mode menu and silences the buzzer.
 * Game: CATCH BALLS, FIND YOUR TIMING, HIGH SCORES, BACK. Push selects/back.
 * Catch: turn to move paddle; three misses end a round. Push exits a round.
 * Timing: release both pads, push to start; after 5 seconds a beep signals
 * touch either pad. Early touches invalidate the round. Push returns.
 * Best catch score and fastest valid reaction persist in EEPROM.
 *
 * Build (avr-gcc/avr-libc installed):
 * avr-gcc -mmcu=atmega32 -std=gnu99 -Os -Wall -Wextra -Werror \
 *   robot19.c -o robot_fsm.elf
 * avr-objcopy -O ihex -R .eeprom robot_fsm.elf robot_fsm.hex
 * avr-size -C --mcu=atmega32 robot_fsm.elf
 *
 * Test with wheels raised, then at floor level with a tether/catch surface.
 * PWM duty is NOT measured wheel speed. Tune to the actual loaded robot.
 * Braking is not zero stopping distance; two front IR sensors still cannot
 * protect the wheel paths mid-turn or distinguish every dark floor from a
 * real cliff. Calibrate the turn on the actual surface, away from an edge.
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
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>

/* --------------------------- User settings --------------------------- */

#define MOTOR_SPEED                  60u  /* 47% PWM; was 255/full duty. */
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
/* One corner sensor per PORTB pin; see the wiring comment at the top. */
#define IR_FL_PIN                    PB0   /* Front-left  (original sensor). */
#define IR_FR_PIN                    PB4   /* Front-right. */
#define IR_SURFACE_STABLE_TICKS     150u   /* 307 ms before cliff recovery. */
#define CLIFF_CONFIRM_TICKS          90u   /* ~184 ms braked before backing up;
                                             * lets a one-sample glitch clear
                                             * on its own without moving. */
#define CLIFF_BACKUP_SPEED            60u  /* Equal wheel speeds: straight back. */
#define CLIFF_BACKUP_TICKS           400u  /* ~819 ms, formerly ~410 ms. */
#define CLIFF_BACKUP_TICKS_GROWTH     70u
#define CLIFF_BACKUP_TICKS_MAX       680u  /* ~1.39 s maximum per attempt. */
#define CLIFF_BACKUP_MAX_ATTEMPTS      5u
#define CLIFF_TURN_PWM               110u  /* Outer wheel; inner wheel stopped. */
#define CLIFF_TURN_TICKS             440u  /* ~901 ms: CALIBRATE for 90 degrees. */
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
#define TOUCH_DEBOUNCE_TICKS         80u    /* About 16.4 ms. */
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
#define SHOCK_REARM_TICKS             10u  /* About 20 ms quiet before re-arm:
                                             * the sensitivity/responsiveness
                                             * knob -- see the comment above
                                             * shock_sample_isr() below. */

#define SERVO_HOME_TICKS             125u  /* 1.000 ms at 8 us/tick */
#define SERVO_CLIFF_TICKS            188u  /* 1.504 ms: about 90 degrees */
#define SERVO_PET_TICKS              145u  /* 1.160 ms: gentle petting wiggle */
#define SERVO_PET_HALF_PERIOD_TICKS  120u  /* About 246 ms each way. */
#define SERVO_ACTIVE_HOLD_TICKS      200u  /* ~410 ms of pulses after any
                                             * target change -- enough for a
                                             * full sweep plus settle -- then
                                             * pulses stop until the next
                                             * change, so the servos aren't
                                             * driven/buzzing while just
                                             * holding a steady position. */

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

/* ---- KY-040 rotary encoder (PC4/PC5/PC6) and DHT11 (PC7) settings ---- */
#define ENC_CLK_PIN                  PC4
#define ENC_DT_PIN                   PC5
#define ENC_SW_PIN                   PC6
#define ENC_REVERSE                  0u    /* 1 swaps CW/CCW rotation. */
#define ENC_BUTTON_DEBOUNCE_TICKS    8u    /* About 16 ms, counted in the ISR. */
#define ENC_MIN_TRANSITIONS          2u    /* Of 4 per detent; forgives a miss. */
#define UI_MENU_TIMEOUT_TICKS     2441u    /* About 5 s idle closes the menu. */

#define DHT_PIN                      PC7
#define DHT_WARMUP_TICKS           500u    /* DHT11 needs about 1 s after power. */
#define DHT_PERIOD_TICKS          1000u    /* About 2.05 s between samples. */
#define DHT_START_LOW_TICKS         10u    /* 20.48 ms start pulse (>= 18 ms). */
#define DHT_SYNC_FRAME               2u    /* Servo-frame slot for the read. */
#define DHT_FAIL_LIMIT               3u    /* Consecutive misses => sensor error. */
#define DHT_EDGE_TIMEOUT_TICKS      20u    /* 160 us at 8 us per Timer0 tick. */
#define DHT_BIT_ONE_TICKS            5u    /* HIGH longer than this = a 1 bit. */

/* Offline weather classification, whole degrees C and %RH (DHT11 resolution:
 * about +/-2 C and +/-5 %RH).  A class is entered at its limit and left only
 * after the reading moves HYST past it, so it cannot flicker at a threshold. */
#define WEATHER_HOT_C               30u    /* HOT   when T >= this. */
#define WEATHER_COLD_C              18u    /* COLD  when T <= this. */
#define WEATHER_HUMID_RH            70u    /* HUMID when RH >= this. */
#define WEATHER_DRY_RH              30u    /* DRY   when RH <= this. */
#define WEATHER_TEMP_HYST_C          2u
#define WEATHER_RH_HYST              5u
/* One label is shown.  1: temperature wins if both apply (HOT beats HUMID);
 * 0: humidity wins (HUMID beats HOT).  The numbers are always displayed. */
#define WEATHER_TEMP_PRIORITY        1u

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
#if CLIFF_BACKUP_TICKS_MAX < CLIFF_BACKUP_TICKS
#error "CLIFF_BACKUP_TICKS_MAX must be >= CLIFF_BACKUP_TICKS"
#endif
#if MIC_MIN_P2P < 1 || MIC_MIN_P2P > 255 || MIC_RISE_P2P < 2 || \
    MIC_RISE_P2P > 127 || MIC_REARM_WINDOWS < 1 || MIC_REARM_WINDOWS > 255
#error "Use valid 8-bit microphone thresholds and a nonzero re-arm count"
#endif
#if TURN_GAZE_SHIFT < 1 || TURN_GAZE_SHIFT > 4 || \
    (TURN_GAZE_POLARITY != 1 && TURN_GAZE_POLARITY != -1)
#error "Use gaze shift 1..4 and gaze polarity +1 or -1 for this eye window"
#endif
#if WEATHER_COLD_C + WEATHER_TEMP_HYST_C >= WEATHER_HOT_C - WEATHER_TEMP_HYST_C || \
    WEATHER_DRY_RH + WEATHER_RH_HYST >= WEATHER_HUMID_RH - WEATHER_RH_HYST || \
    WEATHER_HOT_C < WEATHER_TEMP_HYST_C || WEATHER_HUMID_RH < WEATHER_RH_HYST
#error "Weather thresholds and their hysteresis bands must not overlap"
#endif
/* The sensor reply must land after the servo pulse and end before the next. */
#if DHT_START_LOW_TICKS < 10 || \
    ((DHT_SYNC_FRAME + DHT_START_LOW_TICKS) % 10) < 1 || \
    ((DHT_SYNC_FRAME + DHT_START_LOW_TICKS) % 10) > 5
#error "DHT11 start pulse must be >= 18 ms and end in servo-frame slots 1..5"
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
    STATE_CLIFF_BACKUP,     /* Bounded reverse away from a latched cliff. */
    STATE_CLIFF_TURN,       /* Random calibrated quarter turn after retreat. */
    STATE_BLOCKED,
    STATE_SENSOR_WAIT,
    STATE_STARTLED,
    STATE_DIZZY
} robot_state_t;

/* User-selectable operating modes (KY-040).  MODE_ROAM is the complete,
 * unchanged autonomous behavior implemented by the FSM below. */
typedef enum {
    MODE_ROAM, MODE_WEATHER, MODE_POMODORO, MODE_GAME, MODE_COUNT
} robot_mode_t;

typedef enum {
    WX_NONE, WX_COMFORTABLE, WX_HOT, WX_COLD, WX_HUMID, WX_DRY
} weather_class_t;

static robot_mode_t robot_mode = MODE_ROAM;

/* OLED text bands around the eye window: pages 0,1 (top) and 6,7 (bottom).
 * One dirty bit per band page; oled_ui_service() redraws one page per pass. */
#define UI_BAND_ALL                  0x0Fu
static uint8_t ui_dirty = 0u;

static volatile uint8_t cliff_latched = 1u;
static volatile uint8_t surface_stable_ticks = 0u;
/* Normally ANY cliff reading forces an immediate brake, no exceptions -- see
 * the ISR and motors_service() below. This flag is the one deliberate,
 * tightly-scoped carve-out: while STATE_CLIFF_BACKUP has it set, a bounded
 * reverse move is allowed to run despite cliff_latched being true, so the
 * robot can back away from an edge it would otherwise sit at forever (the
 * sensor never clears on its own if the robot never moves). robot_enter()
 * is the only place that sets or clears it, and it defaults to 0 on every
 * single state change, so it can never stay "on" outside that one state. */
static volatile uint8_t cliff_backup_authorized = 0u;

/* Bit assignment for the 2-sensor cliff bitmask returned by ir_cliff_mask().
 * Used only to pick a backup direction; every safety latch/brake path below
 * still treats "either sensor" the same as the original single-sensor code. */
#define CLIFF_BIT_FL                  _BV(0)
#define CLIFF_BIT_FR                  _BV(1)

/* Reads both front corner sensors and returns which one(s) currently see a
 * cliff, using the same IR_CLIFF_LEVEL polarity as the original sensor.
 * Single register read (PINB), so this is safe to call from the ISR or
 * from the main loop without disabling interrupts, exactly like the old
 * single-pin ir_cliff_now() it replaces here. */
static inline uint8_t ir_cliff_mask(void)
{
    uint8_t m = 0u;
    if (((PINB & _BV(IR_FL_PIN)) ? 1u : 0u) == IR_CLIFF_LEVEL) {
        m |= CLIFF_BIT_FL;
    }
    if (((PINB & _BV(IR_FR_PIN)) ? 1u : 0u) == IR_CLIFF_LEVEL) {
        m |= CLIFF_BIT_FR;
    }
    return m;
}

/* Both safety checks use this same polarity, including the Timer2 ISR.
 * Preserved name/signature: true the instant EITHER sensor sees a cliff,
 * same as the original single-sensor behavior, so every existing call site
 * (ISR latch, motors_service() brake gate, cliff_release_if_safe(), the
 * STARTUP check) stays correct with no changes of its own. */
static inline uint8_t ir_cliff_now(void)
{
    return ir_cliff_mask() != 0u;
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
/* Direction latched at the ARMING -> RUNNING transition below. 0 = forward
 * (the only direction this firmware used to drive), 1 = reverse. Only
 * motors_set_targets_reverse() (the cliff-backup maneuver) ever sets this. */
static volatile uint8_t motor_reverse = 0u;

#define MOTOR_DIRECTION_MASK (_BV(PD1) | _BV(PD2) | _BV(PD3) | _BV(PD6))
#define MOTOR_ENABLE_MASK (_BV(PD4) | _BV(PD5))
#define MOTOR_FORWARD_BITS (_BV(PD2) | _BV(PD1))   /* IN1 + IN4 high. */
#define MOTOR_REVERSE_BITS (_BV(PD3) | _BV(PD6))   /* IN2 + IN3 high. */

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
    motor_reverse = 0u;
    motor_target_a = speed_a;
    motor_target_b = speed_b;
    if (!speed_a && !speed_b) {
        motors_brake();
    }
}

/* Used only by the bounded STATE_CLIFF_BACKUP maneuver below -- everywhere
 * else in the firmware still only ever drives forward, unchanged. */
static void motors_set_targets_reverse(uint8_t speed_a, uint8_t speed_b)
{
    motor_reverse = 1u;
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

    /* Never let a stale main-loop command undo an interrupt's cliff stop.
     * cliff_backup_authorized is the one deliberate, bounded exception: it
     * lets STATE_CLIFF_BACKUP's reverse move run despite cliff_latched. */
    if (ir_cliff_now()) {
        cliff_latched = 1u;
        surface_stable_ticks = 0u;
    }
    if ((cliff_latched && !cliff_backup_authorized) ||
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
                             (motor_reverse ? MOTOR_REVERSE_BITS
                                            : MOTOR_FORWARD_BITS));
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

/* ------------------------ Cliff IR sensors (x2) ----------------------- */
/* Polarity for both front corners is selected only by IR_CLIFF_LEVEL above. */

static void ir_init(void)
{
    DDRB &= (uint8_t)~(_BV(IR_FL_PIN) | _BV(IR_FR_PIN));
    PORTB |= _BV(IR_FL_PIN) | _BV(IR_FR_PIN); /* Same pull-up convention as
                                                * the original PB0 sensor. */
}

/* --------------------------- Two servos ------------------------------ */
/* PB1=SG90 signal, PB2=SG92 signal. Timer2 tick = 8 us. */

static volatile uint8_t servo_target_ticks = SERVO_HOME_TICKS;
static volatile uint8_t servo_frame_count = 9;
static volatile uint8_t servo_pulse_active = 0;
static volatile uint16_t system_ticks_2ms = 0;

static inline void shock_sample_isr(void);
static inline void encoder_sample_isr(void);
static inline void activities_tick_isr(void);

#define BUZZER_PIN PA7
#define BUZZER_ACTIVE_HIGH 1u /* Set 0 for an active-LOW three-pin module. */
#define REACTION_COUNTDOWN_TICKS 2442u /* ceil(5 s / 2.048 ms). */
#define REACTION_TIMEOUT_TICKS 4883u   /* 10 seconds after the cue. */
enum { RX_OFF, RX_COUNTDOWN, RX_WAIT, RX_HIT, RX_EARLY, RX_TIMEOUT };
static volatile uint8_t rx_phase = RX_OFF;
static volatile uint16_t rx_started = 0u, rx_hit_tick = 0u;
static uint16_t rx_candidate_tick = 0u;
static uint8_t rx_touch_count = 0u;
static volatile uint16_t buzzer_ticks = 0u;

static inline void buzzer_output(uint8_t on)
{
    if (on == BUZZER_ACTIVE_HIGH) PORTA |= _BV(BUZZER_PIN);
    else PORTA &= (uint8_t)~_BV(BUZZER_PIN);
}

static void buzzer_beep(uint16_t ticks)
{
    uint8_t saved = SREG;
    cli();
    buzzer_ticks = ticks;
    buzzer_output(ticks != 0u);
    SREG = saved;
}

static void buzzer_init(void)
{
    buzzer_output(0u);
    DDRA |= _BV(BUZZER_PIN);
}

static inline void activities_tick_isr(void)
{
    uint8_t touched;
    if (buzzer_ticks && --buzzer_ticks == 0u) buzzer_output(0u);
    if (rx_phase != RX_COUNTDOWN && rx_phase != RX_WAIT) return;
    touched = PINA & (_BV(PA0) | _BV(PA1));
#if !TOUCH_ACTIVE_HIGH
    touched = (uint8_t)(~touched) & (_BV(PA0) | _BV(PA1));
#endif
    if (rx_phase == RX_COUNTDOWN) {
        /* Held pads/early touches must never become a zero-ms high score. */
        if (touched) {
            rx_phase = RX_EARLY;
        } else if ((uint16_t)(system_ticks_2ms - rx_started) >=
                   REACTION_COUNTDOWN_TICKS) {
            rx_started = system_ticks_2ms;
            rx_touch_count = 0u;
            buzzer_ticks = 60u;
            buzzer_output(1u);
            rx_phase = RX_WAIT;
        }
    } else if ((uint16_t)(system_ticks_2ms - rx_started) >=
               REACTION_TIMEOUT_TICKS) {
        rx_phase = RX_TIMEOUT;
    } else if (touched) {
        if (rx_touch_count == 0u) rx_candidate_tick = system_ticks_2ms;
        if (++rx_touch_count > TOUCH_DEBOUNCE_TICKS) {
            rx_hit_tick = rx_candidate_tick;
            rx_phase = RX_HIT;
        }
    } else {
        rx_touch_count = 0u;
    }
}

static volatile uint8_t servo_pulses_enabled = 1u; /* See servo_note_target()
                                                     * below: pulses only run
                                                     * for a bit after the
                                                     * target actually
                                                     * changes, then stop, so
                                                     * the servos aren't
                                                     * continuously driven
                                                     * (and buzzing) at rest. */
static uint8_t servo_last_seen_ticks = SERVO_HOME_TICKS;
static uint16_t servo_change_tick = 0;

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

/* Call once per servos_update() pass with whatever servo_target_ticks was
 * just (re)requested. A real change (re)starts the SERVO_ACTIVE_HOLD_TICKS
 * hold window and turns pulses back on; once that window elapses with no
 * further change, pulses turn off until the next one. Only this function
 * and the two ISRs below touch servo_pulses_enabled. Main-loop only (not
 * ISR-safe), same as the rest of servos_update(). */
static void servo_note_target(uint8_t target, uint16_t now)
{
    if (target != servo_last_seen_ticks) {
        servo_last_seen_ticks = target;
        servo_change_tick = now;
        servo_pulses_enabled = 1u;
    } else if (servo_pulses_enabled &&
               (uint16_t)(now - servo_change_tick) >= SERVO_ACTIVE_HOLD_TICKS) {
        servo_pulses_enabled = 0u;
    }
}

ISR(TIMER2_OVF_vect)
{
    ++system_ticks_2ms;
    activities_tick_isr();
    shock_sample_isr();
    encoder_sample_isr();

    /* Safety is sampled every ~2.048 ms, even during OLED/sonar work.
     * One cliff sample brakes. Only the main FSM can clear the latch after
     * a continuously stable surface; short surface glitches cannot restart. */
    if (ir_cliff_now()) {
        cliff_latched = 1u;
        surface_stable_ticks = 0u;
        if (!cliff_backup_authorized && motor_hw_mode != MOTOR_BRAKED) {
            motors_brake_now();
        }
    } else if (surface_stable_ticks < IR_SURFACE_STABLE_TICKS) {
        ++surface_stable_ticks;
    }

    /* servo_frame_count itself always keeps counting (DHT11 timing below
     * relies on this same frame cadence); only the actual pulse output is
     * gated by servo_pulses_enabled. */
    if (++servo_frame_count >= 10u) {
        servo_frame_count = 0;
        if (servo_pulses_enabled) {
            OCR2 = servo_target_ticks;
            PORTB |= _BV(PB1) | _BV(PB2);
            servo_pulse_active = 1;
        }
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
 * is not lost while OLED/sonar code is running in the main loop.
 *
 * SENSITIVITY: shock_sample_isr() below already reacts on the very FIRST
 * ~2.048 ms sample where DO reads active -- that's as fast as this firmware
 * can possibly notice a shock, so there is no software "threshold" to lower
 * for a single shake. The one sensitivity-related knob in code is
 * SHOCK_REARM_TICKS above: it's how long DO must read quiet again before the
 * NEXT shake is allowed to register, so lowering it (already done above)
 * makes repeated/ongoing shaking register more readily. If a single firm
 * shake still needs to be hard before anything happens, that threshold is
 * set on the sensor module itself: most SW-18015P breakout boards have a
 * small onboard trimmer potentiometer feeding the comparator's reference --
 * turn it toward the more-sensitive direction (consult your specific
 * board's silkscreen/markings) rather than looking for another macro here.
 */
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

/* ---------------------- KY-040 rotary encoder ----------------------- */
/*
 * CLK=PC4, DT=PC5, SW=PC6 (active LOW).  The module has pull-ups; the AVR
 * ones are only a backup.  Like the shock sensor, the encoder is sampled from
 * the ~2.048 ms Timer2 ISR: main-loop passes are stretched by OLED page
 * transfers and sonar polling, far too slow for quadrature.  A Gray-code
 * transition table ignores contact bounce and impossible jumps; one detent is
 * reported when both contacts return to the idle HIGH/HIGH position after at
 * least ENC_MIN_TRANSITIONS valid steps one way.  Clockwise = +1.
 * The ISR only reads PINC; PORTC/DDRC are written from main context only.
 */
static volatile int8_t enc_steps = 0;
static volatile uint8_t enc_press_pending = 0u;
static uint8_t enc_prev = 3u, enc_sw_stable = 0u, enc_sw_count = 0u;
static int8_t enc_accum = 0;

/* Index = (previous << 2) | current, state = (CLK << 1) | DT.
 * Clockwise cycle: 3 -> 1 -> 0 -> 2 -> 3. */
static const int8_t enc_delta[16] PROGMEM = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static void encoder_init(void)
{
    const uint8_t mask = (uint8_t)(_BV(ENC_CLK_PIN) | _BV(ENC_DT_PIN) |
                                   _BV(ENC_SW_PIN));
    uint8_t pins;

    DDRC &= (uint8_t)~mask;
    PORTC |= mask;
    _delay_us(20);                     /* Let the pull-ups settle. */
    pins = PINC;
    enc_prev = (uint8_t)(((pins & _BV(ENC_CLK_PIN)) ? 2u : 0u) |
                         ((pins & _BV(ENC_DT_PIN)) ? 1u : 0u));
    enc_sw_stable = (pins & _BV(ENC_SW_PIN)) ? 0u : 1u;  /* Held at boot: no event. */
    enc_sw_count = 0u;
    enc_accum = 0;
    enc_steps = 0;
    enc_press_pending = 0u;
}

static inline void encoder_sample_isr(void)
{
    uint8_t pins = PINC;
    uint8_t state = (uint8_t)(((pins & _BV(ENC_CLK_PIN)) ? 2u : 0u) |
                              ((pins & _BV(ENC_DT_PIN)) ? 1u : 0u));
    uint8_t pressed = (pins & _BV(ENC_SW_PIN)) ? 0u : 1u;

    if (state != enc_prev) {
        int8_t delta = (int8_t)pgm_read_byte(
            &enc_delta[(uint8_t)((uint8_t)(enc_prev << 2) | state)]);
#if ENC_REVERSE
        delta = (int8_t)-delta;
#endif
        enc_prev = state;
        enc_accum = (int8_t)(enc_accum + delta);
        if (enc_accum > 8) {
            enc_accum = 8;
        } else if (enc_accum < -8) {
            enc_accum = -8;
        }
        if (state == 3u) {             /* Back at the detent rest position. */
            if (enc_accum >= (int8_t)ENC_MIN_TRANSITIONS) {
                if (enc_steps < 100) {
                    ++enc_steps;
                }
            } else if (enc_accum <= -(int8_t)ENC_MIN_TRANSITIONS) {
                if (enc_steps > -100) {
                    --enc_steps;
                }
            }
            enc_accum = 0;
        }
    }

    /* Debounced push button: one event per press, on the press edge. */
    if (pressed == enc_sw_stable) {
        enc_sw_count = 0u;
    } else if (++enc_sw_count >= ENC_BUTTON_DEBOUNCE_TICKS) {
        enc_sw_stable = pressed;
        enc_sw_count = 0u;
        if (pressed) {
            enc_press_pending = 1u;
        }
    }
}

static void encoder_take(int8_t *steps, uint8_t *press)
{
    uint8_t saved_sreg = SREG;
    cli();
    *steps = enc_steps;
    enc_steps = 0;
    *press = enc_press_pending;
    enc_press_pending = 0u;
    SREG = saved_sreg;
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
        servo_note_target(servo_target_ticks, now);
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
        servo_note_target(servo_target_ticks, now);
        return;
    }

    petting = 0;
    pet_outward = 0;
    servos_home();
    servo_note_target(servo_target_ticks, now);
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

/* ------------------------ DHT11 and weather class ------------------- */
/*
 * DATA=PC7.  The DHT11 is sampled ONLY in Weather Mode, where the motors are
 * braked.  A sample blocks the main loop for about 25-45 ms every ~2 s: a
 * 20.48 ms start pulse (interrupts on, timed by Timer2 ticks) and then a
 * ~4.5 ms reply that must be timed with interrupts off, because the Timer2
 * and ADC ISRs (60-100 us each) would corrupt the 26 us / 70 us bit pulses.
 * The start pulse is aligned to a Timer2 servo-frame slot so the interrupt-off
 * window falls between two servo pulses and never stretches one.  A sample
 * costs at most a couple of lost 2 ms ticks and a few missed encoder samples.
 * Timer0 (8 us/tick, already free-running for the sonar) times the edges; it
 * is only read here, never written.  Nothing leaves the chip: the class is
 * computed from fixed thresholds.
 */
static uint8_t dht_warm = 0u, dht_due = 1u, dht_fail_count = 0u;
static uint16_t dht_boot_tick = 0u, dht_last_tick = 0u;
static uint8_t wx_valid = 0u, wx_error = 0u;
static uint8_t wx_temp_c = 0u, wx_humidity = 0u;
static weather_class_t wx_class = WX_NONE;

static void dht_init(void)
{
    DDRC &= (uint8_t)~_BV(DHT_PIN);
    PORTC |= _BV(DHT_PIN);             /* Idle released (backup pull-up). */
}

static inline uint8_t dht_level(void)
{
    return (PINC & _BV(DHT_PIN)) ? 1u : 0u;
}

/* Ticks waited for the line to reach `level`, or 0xFF on timeout. */
static uint8_t dht_wait(uint8_t level)
{
    uint8_t start = TCNT0;

    while (dht_level() != level) {
        if ((uint8_t)(TCNT0 - start) > DHT_EDGE_TIMEOUT_TICKS) {
            return 0xFFu;
        }
    }
    return (uint8_t)(TCNT0 - start);
}

/* Interrupts must be off.  The line has just been released by the caller. */
static uint8_t dht_read_bits(uint8_t data[5])
{
    uint8_t i, width;

    /* Line rises, then the sensor answers: 80 us LOW, 80 us HIGH. */
    if (dht_wait(1u) == 0xFFu || dht_wait(0u) == 0xFFu ||
        dht_wait(1u) == 0xFFu || dht_wait(0u) == 0xFFu) {
        return 0u;
    }
    /* Each bit: 50 us LOW, then HIGH for ~27 us (0) or ~70 us (1). */
    for (i = 0u; i < 40u; ++i) {
        if (dht_wait(1u) == 0xFFu) {
            return 0u;
        }
        width = dht_wait(0u);
        if (width == 0xFFu) {
            return 0u;
        }
        data[i >> 3] = (uint8_t)((uint8_t)(data[i >> 3] << 1) |
                                 (width > DHT_BIT_ONE_TICKS ? 1u : 0u));
    }
    return 1u;
}

static uint8_t dht11_read(uint8_t *humidity, uint8_t *temperature)
{
    uint8_t data[5] = {0u, 0u, 0u, 0u, 0u};
    uint8_t saved_sreg, ok;
    uint16_t started, guard;

    /* Wait for the START of servo-frame slot DHT_SYNC_FRAME (bounded). */
    guard = 40000u;
    while (servo_frame_count == DHT_SYNC_FRAME && --guard != 0u) {
    }
    guard = 40000u;
    while (servo_frame_count != DHT_SYNC_FRAME && --guard != 0u) {
    }
    if (guard == 0u) {
        return 0u;
    }

    /* Host start pulse, interrupts still running: exactly 10 ticks. */
    PORTC &= (uint8_t)~_BV(DHT_PIN);
    DDRC |= _BV(DHT_PIN);
    started = system_ticks_read();
    guard = 40000u;
    while ((uint16_t)(system_ticks_read() - started) < DHT_START_LOW_TICKS &&
           --guard != 0u) {
    }

    saved_sreg = SREG;
    cli();
    DDRC &= (uint8_t)~_BV(DHT_PIN);    /* Release; the pull-up raises it. */
    PORTC |= _BV(DHT_PIN);
    ok = dht_read_bits(data);
    SREG = saved_sreg;

    if (!ok) {
        return 0u;
    }
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4] ||
        data[0] > 100u || data[2] > 60u) {
        return 0u;                     /* Bad checksum or impossible value. */
    }
    *humidity = data[0];
    *temperature = data[2];            /* DHT11: integer degrees C. */
    return 1u;
}

static weather_class_t weather_classify(uint8_t temp_c, uint8_t rh,
                                        weather_class_t previous)
{
    uint8_t hot_at   = (previous == WX_HOT)   ?
        (uint8_t)(WEATHER_HOT_C - WEATHER_TEMP_HYST_C) : WEATHER_HOT_C;
    uint8_t cold_at  = (previous == WX_COLD)  ?
        (uint8_t)(WEATHER_COLD_C + WEATHER_TEMP_HYST_C) : WEATHER_COLD_C;
    uint8_t humid_at = (previous == WX_HUMID) ?
        (uint8_t)(WEATHER_HUMID_RH - WEATHER_RH_HYST) : WEATHER_HUMID_RH;
    uint8_t dry_at   = (previous == WX_DRY)   ?
        (uint8_t)(WEATHER_DRY_RH + WEATHER_RH_HYST) : WEATHER_DRY_RH;

#if WEATHER_TEMP_PRIORITY
    if (temp_c >= hot_at)   return WX_HOT;
    if (temp_c <= cold_at)  return WX_COLD;
    if (rh >= humid_at)     return WX_HUMID;
    if (rh <= dry_at)       return WX_DRY;
#else
    if (rh >= humid_at)     return WX_HUMID;
    if (rh <= dry_at)       return WX_DRY;
    if (temp_c >= hot_at)   return WX_HOT;
    if (temp_c <= cold_at)  return WX_COLD;
#endif
    return WX_COMFORTABLE;
}

/* Called when Weather Mode is entered: show "reading" until a fresh sample. */
static void weather_begin(void)
{
    wx_valid = 0u;
    wx_error = 0u;
    wx_class = WX_NONE;
    dht_fail_count = 0u;
    dht_due = 1u;
}

static void dht_service(uint16_t now)
{
    uint8_t humidity = 0u, temperature = 0u;

    if (!dht_warm) {
        if ((uint16_t)(now - dht_boot_tick) < DHT_WARMUP_TICKS) {
            return;
        }
        dht_warm = 1u;
    }
    if (robot_mode != MODE_WEATHER) {
        return;
    }
    if (!dht_due && (uint16_t)(now - dht_last_tick) < DHT_PERIOD_TICKS) {
        return;
    }

    dht_due = 0u;
    if (dht11_read(&humidity, &temperature)) {
        weather_class_t cls = weather_classify(temperature, humidity, wx_class);
        if (!wx_valid || wx_error || temperature != wx_temp_c ||
            humidity != wx_humidity || cls != wx_class) {
            ui_dirty |= UI_BAND_ALL;
        }
        wx_temp_c = temperature;
        wx_humidity = humidity;
        wx_class = cls;
        wx_valid = 1u;
        wx_error = 0u;
        dht_fail_count = 0u;
    } else {
        if (dht_fail_count < 255u) {
            ++dht_fail_count;
        }
        if (dht_fail_count >= DHT_FAIL_LIMIT && !wx_error) {
            wx_error = 1u;
            ui_dirty |= UI_BAND_ALL;
        }
    }
    dht_last_tick = system_ticks_read();
}

/* ----------------------- Timed behavior FSM -------------------------- */

static robot_state_t robot_state = STATE_STARTUP;
static uint16_t state_since = 0, state_duration = 0;
static uint8_t turn_left = 0;
static uint8_t avoidance_blocked = 0;
static uint8_t cliff_backup_attempts = 0u;  /* Capped by CLIFF_BACKUP_MAX_ATTEMPTS. */
static uint8_t cliff_turn_needed = 0u;      /* Set by cliff_backup_start(). */
static uint8_t cliff_turn_left = 0u;        /* Direction for STATE_CLIFF_TURN. */
static touch_state_t previous_touch = TOUCH_NONE;
static uint8_t startle_was_turning = 0;
static uint16_t startle_turn_since = 0;

/* Shock dizziness is a visual overlay, not a motion state.  A shock reacts
 * immediately from any robot state without changing motor targets. */
static uint8_t dizzy_overlay = 0u;
static uint16_t dizzy_overlay_since = 0u;

/* Finish the full straight retreat before releasing the cliff latch.
 * Pick one random direction per recovery, including a BOTH-sensor cliff. */
static void cliff_backup_start(void)
{
    if (!ir_cliff_now()) {
        cliff_backup_authorized = 0u;
        motors_set_targets(0, 0);
        return;
    }
    if (!cliff_turn_needed) {
        cliff_turn_left = (uint8_t)(random_next() & 1u);
        cliff_turn_needed = 1u;
    }
    cliff_backup_authorized = 1u;
    motors_set_targets_reverse(CLIFF_BACKUP_SPEED, CLIFF_BACKUP_SPEED);
}

static void robot_enter(robot_state_t state, uint16_t now)
{
    if (state == robot_state) {
        return;
    }
    /* Default off on every transition; only the STATE_CLIFF_BACKUP case
     * below re-arms it, so the exception can never leak into another state. */
    cliff_backup_authorized = 0u;
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
    case STATE_CLIFF_BACKUP: {
        /* Bounded straight retreat, growing only if the floor stays absent. */
        uint16_t grown = CLIFF_BACKUP_TICKS + (uint16_t)
            ((cliff_backup_attempts > 1u ? cliff_backup_attempts - 1u : 0u) *
             CLIFF_BACKUP_TICKS_GROWTH);
        state_duration = grown > CLIFF_BACKUP_TICKS_MAX ?
            CLIFF_BACKUP_TICKS_MAX : grown;
        cliff_backup_start();
        break;
    }
    case STATE_CLIFF_TURN: {
        /* Floor must be stable before turning. Either sensor still brakes
         * immediately if it detects another edge during this turn. */
        uint8_t a_is_inner = (cliff_turn_left == MOTOR_A_IS_LEFT);
        state_duration = CLIFF_TURN_TICKS;
        motors_set_targets(a_is_inner ? 0u : CLIFF_TURN_PWM,
                           a_is_inner ? CLIFF_TURN_PWM : 0u);
        break;
    }
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
    cliff_backup_attempts = 0u;
    cliff_turn_needed = 0u;
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

    /* All non-ROAM modes are stationary. Motors are braked on entry
     * and no FSM state may retarget them, so the whole FSM (sonar, cliff,
     * touch and sound reactions) is suspended here.  The shock overlay above
     * is purely visual and still works.  Leaving the mode restarts the normal
     * STARTUP -> SETTLE -> CRUISE path through robot_begin(), which also
     * re-checks the cliff sensor before the wheels can move. */
    if (robot_mode != MODE_ROAM) {
        return;
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
        if (robot_state == STATE_CLIFF_BACKUP) {
            /* A bounded reverse is already under way; its own timeout in
             * the switch below decides when it ends. Don't reroute here. */
        } else if (robot_state != STATE_CLIFF) {
            robot_enter(STATE_CLIFF, now);
            return;
        } else if (surface_stable_ticks < IR_SURFACE_STABLE_TICKS) {
            /* A genuinely stationary cliff will never self-clear while the
             * robot sits still -- the sensor keeps reading the same drop
             * forever. After a short confirm delay (so a one-sample glitch
             * can clear on its own without moving), back away a small,
             * bounded amount and check again. Give up after a few tries
             * rather than shuffle back and forth indefinitely, same
             * philosophy as STATE_BLOCKED below. */
            if (touch == TOUCH_NONE &&
                cliff_backup_attempts < CLIFF_BACKUP_MAX_ATTEMPTS &&
                (uint16_t)(now - state_since) >= CLIFF_CONFIRM_TICKS) {
                ++cliff_backup_attempts;
                robot_enter(STATE_CLIFF_BACKUP, now);
            }
            return;
        } else if (!cliff_release_if_safe()) {
            return;
        } else {
            cliff_backup_attempts = 0u;
            if (cliff_turn_needed) {
                cliff_turn_needed = 0u;
                robot_enter(STATE_CLIFF_TURN, now);
            } else {
                robot_enter(STATE_SETTLE, now);
            }
        }
    }

    if (robot_state == STATE_CLIFF_BACKUP) {
        if (touch != TOUCH_NONE ||
            (uint16_t)(now - state_since) >= state_duration) {
            robot_enter(STATE_CLIFF, now);
        }
        return;
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
    case STATE_CLIFF_BACKUP:
        if (elapsed >= state_duration) {
            /* Stop and hand back to the block above on the next pass: if
             * the sensor is finally clear (and has been for the full
             * stable window) the latch releases there; otherwise it
             * retries, up to the attempt cap, or simply stays braked. */
            robot_enter(STATE_CLIFF, now);
        }
        break;
    case STATE_CLIFF_TURN:
        if (elapsed >= state_duration) {
            robot_enter(STATE_SETTLE, now);
        }
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

/* ------------------ KY-040 mode selection and menu -------------------- */
/*
 * Turn:  opens the mode menu (if closed) and moves the highlight.
 * Push:  in the menu, selects the highlighted mode (pushing on the current
 *        mode just closes it); in Weather Mode, exits back to ROAM;
 *        in ROAM with no menu, opens the menu.
 * The menu closes by itself after UI_MENU_TIMEOUT_TICKS without input.
 * It is only drawn on the OLED; ROAM keeps driving while it is open.
 */
typedef enum { UI_HIDDEN, UI_MENU } ui_state_t;
static ui_state_t ui_state = UI_HIDDEN;
static uint8_t ui_cursor = MODE_ROAM;
static uint16_t ui_last_input = 0u;

/* ---------------- Pomodoro and stationary games --------------------- */
typedef enum { POMO_MINUTES, POMO_SECONDS, POMO_RUNNING, POMO_DONE } pomo_state_t;
typedef enum {
    GAME_MENU, GAME_CATCH, GAME_CATCH_OVER, GAME_TIMING_READY,
    GAME_TIMING_ACTIVE, GAME_TIMING_RESULT, GAME_SCORES
} game_state_t;
static pomo_state_t pomo_state = POMO_MINUTES;
static game_state_t game_state = GAME_MENU;
static uint8_t pomo_minutes = 25u, pomo_seconds = 0u;
static uint16_t pomo_remaining = 0u, pomo_total = 0u, activity_last_tick = 0u;
static uint32_t pomo_fraction_us = 0UL;
static uint16_t pomo_alarm_tick = 0u;
static uint8_t game_cursor = 0u, game_lives = 3u, paddle_x = 55u;
static uint16_t game_score = 0u, reaction_ms = 0u, reaction_score = 0u;
static uint8_t reaction_result = RX_OFF, reaction_countdown = 5u;
static uint16_t activity_rng = 0x716Bu;
typedef struct { uint8_t x; int8_t y; } ball_t;
static ball_t balls[3];

/* Complements reject erased or interrupted EEPROM writes. Only improved
 * records are written, from the stationary main loop, never from an ISR. */
typedef struct { uint16_t value, inverse; } score_record_t;
static score_record_t EEMEM ee_catch;
static score_record_t EEMEM ee_reaction;
static uint16_t best_catch = 0u, best_reaction_ms = 0u;

static uint16_t score_load(const score_record_t *record)
{
    uint16_t value = eeprom_read_word(&record->value);
    uint16_t inverse = eeprom_read_word(&record->inverse);
    return (uint16_t)(value ^ inverse) == 0xFFFFu ? value : 0u;
}

static void score_store(score_record_t *record, uint16_t value)
{
    eeprom_update_word(&record->value, value);
    eeprom_update_word(&record->inverse, (uint16_t)~value);
}

static uint16_t activity_random(void)
{
    activity_rng ^= (uint16_t)(activity_rng << 7);
    activity_rng ^= (uint16_t)(activity_rng >> 9);
    activity_rng ^= (uint16_t)(activity_rng << 8);
    if (!activity_rng) activity_rng = 0x716Bu;
    return activity_rng;
}

static uint8_t selection_wrap(uint8_t current, int8_t steps, uint8_t count)
{
    int16_t n = ((int16_t)current + steps) % count;
    if (n < 0) n += count;
    return (uint8_t)n;
}

static uint16_t timing_score(uint16_t ms)
{
    /* Strictly higher score for every lower millisecond, up to 10 seconds. */
    return ms < 10000u ? (uint16_t)(10000u - ms) : 0u;
}

static void activity_stop(void)
{
    rx_phase = RX_OFF;
    buzzer_beep(0u);
}

static void activity_begin(uint16_t now)
{
    activity_stop();
    activity_rng ^= now;
    activity_last_tick = now;
    if (robot_mode == MODE_POMODORO) pomo_state = POMO_MINUTES;
    else { game_state = GAME_MENU; game_cursor = 0u; }
}

static void activity_open_modes(uint16_t now)
{
    activity_stop();
    ui_state = UI_MENU;
    ui_cursor = (uint8_t)robot_mode;
    ui_last_input = now;
    ui_dirty |= UI_BAND_ALL;
}

static void catch_finish(void)
{
    if (game_score > best_catch) {
        best_catch = game_score;
        score_store(&ee_catch, best_catch);
    }
    game_state = GAME_CATCH_OVER;
    ui_dirty |= UI_BAND_ALL;
}

static void activity_input(int8_t steps, uint8_t press, uint16_t now)
{
    uint8_t i;
    if (robot_mode == MODE_POMODORO) {
        if (pomo_state == POMO_MINUTES) {
            pomo_minutes = selection_wrap(pomo_minutes, steps, 100u);
            if (press) pomo_state = POMO_SECONDS;
        } else if (pomo_state == POMO_SECONDS) {
            pomo_seconds = selection_wrap(pomo_seconds, steps, 60u);
            if (press) {
                pomo_total = (uint16_t)pomo_minutes * 60u + pomo_seconds;
                if (pomo_total != 0u) {
                    pomo_remaining = pomo_total;
                    pomo_fraction_us = 0UL;
                    activity_last_tick = now;
                    pomo_state = POMO_RUNNING;
                }
            }
        } else if (press) {
            /* Push cancels/acknowledges and opens the mode selector. */
            pomo_state = POMO_MINUTES;
            activity_open_modes(now);
        }
    } else if (game_state == GAME_MENU) {
        game_cursor = selection_wrap(game_cursor, steps, 4u);
        if (press) {
            if (game_cursor == 0u) {
                game_score = 0u; game_lives = 3u; paddle_x = 55u;
                for (i = 0u; i < 3u; ++i) {
                    balls[i].x = (uint8_t)(26u + activity_random() % 76u);
                    balls[i].y = (int8_t)(17 - (int8_t)i * 13);
                }
                activity_last_tick = now;
                game_state = GAME_CATCH;
            } else if (game_cursor == 1u) game_state = GAME_TIMING_READY;
            else if (game_cursor == 2u) game_state = GAME_SCORES;
            else activity_open_modes(now);
        }
    } else if (game_state == GAME_CATCH) {
        int16_t x = (int16_t)paddle_x + (int16_t)steps * 4;
        if (x < 24) x = 24;
        if (x > 86) x = 86; /* 18-pixel paddle fits the 80-pixel arena. */
        paddle_x = (uint8_t)x;
        if (press) catch_finish();
    } else if (game_state == GAME_TIMING_READY) {
        /* Rotation returns to the game list; push starts a five-second round. */
        if (steps) game_state = GAME_MENU;
        else if (press) {
            uint8_t saved = SREG;
            cli();
            rx_started = system_ticks_2ms;
            rx_phase = RX_COUNTDOWN;
            SREG = saved;
            reaction_countdown = 5u;
            game_state = GAME_TIMING_ACTIVE;
        }
    } else if (press) {
        activity_stop();
        game_state = GAME_MENU;
    }
    if (steps || press) ui_dirty |= UI_BAND_ALL;
}

static void activity_service(uint16_t now)
{
    if (robot_mode == MODE_POMODORO) {
        if (pomo_state == POMO_RUNNING) {
            uint16_t delta = (uint16_t)(now - activity_last_tick);
            activity_last_tick = now;
            /* Exact 2.048-ms tick conversion, preserving the remainder.
             * Accumulated elapsed time handles the 16-bit tick wrap and
             * countdowns longer than 134 seconds without rounding drift. */
            pomo_fraction_us += (uint32_t)delta * 2048UL;
            while (pomo_fraction_us >= 1000000UL && pomo_remaining) {
                pomo_fraction_us -= 1000000UL;
                --pomo_remaining;
                ui_dirty |= UI_BAND_ALL;
            }
            if (!pomo_remaining) {
                pomo_state = POMO_DONE;
                pomo_alarm_tick = now;
                buzzer_beep(122u);
            }
        } else if (pomo_state == POMO_DONE &&
                   (uint16_t)(now - pomo_alarm_tick) >= 488u) {
            pomo_alarm_tick = now;
            buzzer_beep(122u);
        }
    } else if (robot_mode == MODE_GAME && game_state == GAME_CATCH) {
        uint8_t i;
        uint16_t interval = game_score < 400u ?
            (uint16_t)(60u - game_score / 20u) : 40u;
        if ((uint16_t)(now - activity_last_tick) < interval) return;
        activity_last_tick = now;
        for (i = 0u; i < 3u; ++i) {
            if (++balls[i].y >= 43) {
                if ((uint16_t)balls[i].x + 1u >= paddle_x &&
                    balls[i].x <= (uint16_t)paddle_x + 18u) {
                    if (game_score <= 65520u) game_score += 10u;
                } else if (--game_lives == 0u) {
                    catch_finish();
                    break;
                }
                balls[i].x = (uint8_t)(26u + activity_random() % 76u);
                balls[i].y = (int8_t)(12 - (int8_t)(activity_random() % 8u));
                ui_dirty |= UI_BAND_ALL;
            }
        }
    } else if (robot_mode == MODE_GAME && game_state == GAME_TIMING_ACTIVE) {
        uint8_t saved = SREG, phase;
        uint16_t started, hit;
        cli();
        phase = rx_phase; started = rx_started; hit = rx_hit_tick;
        now = system_ticks_2ms;
        SREG = saved;
        if (phase == RX_COUNTDOWN) {
            uint8_t n = (uint8_t)(5u -
                ((uint32_t)(uint16_t)(now - started) * 2048UL / 1000000UL));
            if (n != reaction_countdown) {
                reaction_countdown = n;
                ui_dirty |= UI_BAND_ALL;
            }
        } else if (phase == RX_WAIT) {
            if (reaction_countdown) {
                reaction_countdown = 0u;
                ui_dirty |= UI_BAND_ALL;
            }
        } else if (phase == RX_HIT || phase == RX_EARLY || phase == RX_TIMEOUT) {
            reaction_result = phase;
            if (phase == RX_HIT) {
                reaction_ms = (uint16_t)(
                    ((uint32_t)(uint16_t)(hit - started) * 2048UL + 500UL) / 1000UL);
                reaction_score = timing_score(reaction_ms);
                if (!best_reaction_ms || reaction_ms < best_reaction_ms) {
                    best_reaction_ms = reaction_ms;
                    score_store(&ee_reaction, best_reaction_ms);
                }
            }
            rx_phase = RX_OFF;
            game_state = GAME_TIMING_RESULT;
            ui_dirty |= UI_BAND_ALL;
        }
    }
}

static void mode_set(robot_mode_t mode, uint16_t now)
{
    if (mode == robot_mode) {
        return;
    }
    activity_stop();
    cliff_backup_authorized = 0u;
    robot_mode = mode;
    ui_dirty |= UI_BAND_ALL;
    if (mode != MODE_ROAM) {
        motors_set_targets(0, 0);      /* Immediate dynamic brake, then hold. */
        if (mode == MODE_WEATHER) weather_begin();
        else activity_begin(now);
    } else {
        robot_begin(now);              /* Normal safe restart, incl. cliff check. */
    }
}

static void ui_service(uint16_t now)
{
    int8_t steps;
    uint8_t press;

    encoder_take(&steps, &press);
    if (ui_state == UI_HIDDEN &&
        (robot_mode == MODE_POMODORO || robot_mode == MODE_GAME)) {
        activity_input(steps, press, now);
        return;
    }

    if (steps != 0) {
        int16_t position;
        if (ui_state == UI_HIDDEN) {
            ui_state = UI_MENU;
            ui_cursor = (uint8_t)robot_mode;
        }
        position = (int16_t)((int16_t)ui_cursor + steps);
        position = (int16_t)(position % (int16_t)MODE_COUNT);
        if (position < 0) {
            position = (int16_t)(position + (int16_t)MODE_COUNT);
        }
        ui_cursor = (uint8_t)position;
        ui_last_input = now;
        ui_dirty |= UI_BAND_ALL;
    }

    if (press) {
        ui_last_input = now;
        ui_dirty |= UI_BAND_ALL;
        if (ui_state == UI_MENU) {
            ui_state = UI_HIDDEN;
            mode_set((robot_mode_t)ui_cursor, now);
        } else if (robot_mode != MODE_ROAM) {
            mode_set(MODE_ROAM, now);
        } else {
            ui_state = UI_MENU;
            ui_cursor = (uint8_t)robot_mode;
        }
    }

    if (ui_state == UI_MENU &&
        (uint16_t)(now - ui_last_input) >= UI_MENU_TIMEOUT_TICKS) {
        ui_state = UI_HIDDEN;
        ui_dirty |= UI_BAND_ALL;
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
    FACE_SENSOR_WAIT, FACE_PET, FACE_CLIFF, FACE_STARTLED, FACE_DIZZY, FACE_RECOVER,
    /* Weather Mode reactions. */
    FACE_WX_COMFORT, FACE_WX_HOT, FACE_WX_COLD, FACE_WX_HUMID, FACE_WX_DRY
} face_mode_t;

typedef struct {
    int8_t x, y;
    uint8_t width, height;
    uint8_t lid_drop;
    int8_t lid_slope;
} eye_shape_t;

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
static uint8_t frame_drop_l = 0, frame_drop_r = 0;   /* Top Y of a drop; 0 = none. */
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
    frame_drop_l = frame_drop_r = 0;

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
    case FACE_WX_COMFORT: {
        /* Content and relaxed: slow sway, easy breathing, and a happy
         * closed-eye smile for the last part of every ~4 s cycle. */
        uint16_t phase = (uint16_t)(elapsed % 1952u);
        gx = (int8_t)((int8_t)pgm_read_byte(
                 &eye_scan_table[(elapsed >> 8) & 15u]) / 2);
        left_height = (uint8_t)(21 + (int8_t)pgm_read_byte(&eye_breath_table[index]));
        right_height = left_height;
        if (phase >= 1500u) {
            frame_smile = 1u;
            gy = -2;
        } else {
            allow_blink = 1u;
        }
        break;
    }
    case FACE_WX_HOT:
        /* Overheated: heavy drooping lids, quick panting, a slow sweat drop. */
        left_height = right_height = (uint8_t)(21u + ((elapsed >> 7) & 1u) * 2u);
        left_drop = right_drop = 6u;
        left_slope = -3;
        right_slope = 3;
        gy = 2;
        frame_drop_r = (uint8_t)(17u + (uint8_t)((elapsed >> 5) % 24u));
        allow_blink = 1u;
        break;
    case FACE_WX_COLD:
        /* Shivering: narrowed, worried eyes trembling on every frame. */
        gx = (int8_t)((int8_t)(eye_random() % 5u) - 2);
        gy = (int8_t)(eye_random() & 1u);
        left_height = right_height = 17u;
        left_drop = right_drop = 3u;
        left_slope = -3;
        right_slope = 3;
        break;
    case FACE_WX_HUMID:
        /* Muggy and unimpressed: flat half-lids, slow sway, falling drops. */
        gx = (int8_t)((int8_t)pgm_read_byte(
                 &eye_scan_table[(elapsed >> 7) & 15u]) / 2);
        gy = 1;
        left_height = right_height = 22u;
        left_drop = right_drop = 7u;
        frame_drop_l = (uint8_t)(17u + (uint8_t)((elapsed >> 4) % 24u));
        frame_drop_r = (uint8_t)(17u + (uint8_t)(((elapsed >> 4) + 12u) % 24u));
        allow_blink = 1u;
        break;
    case FACE_WX_DRY: {
        /* Scratchy and irritated: narrowed angled lids, rapid double blinks
         * (two ~50 ms blinks every ~1.3 s) and a slight jitter. */
        uint16_t p = (uint16_t)(elapsed % 640u);
        uint8_t open = 16u;
        if (p >= 40u) {
            p = (uint16_t)(p - 40u);
        }
        if (p < 24u) {
            uint8_t d = (uint8_t)(p < 12u ? 12u - p : p - 12u);
            open = (uint8_t)((d * 4u) / 3u);
        }
        left_height = right_height = (uint8_t)((20u * open) >> 4);
        if (left_height < 2u) {
            left_height = right_height = 2u;
        }
        left_drop = right_drop = (open < 8u) ? 0u : 3u;
        left_slope = (open < 8u) ? 0 : 2;
        right_slope = (int8_t)-left_slope;
        gx = (int8_t)((int8_t)(eye_random() % 3u) - 1);
        break;
    }
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

static face_mode_t weather_face(void)
{
    if (wx_error) {
        return FACE_SENSOR_WAIT;       /* Confused scanning: sensor not answering. */
    }
    switch (wx_class) {
    case WX_COMFORTABLE: return FACE_WX_COMFORT;
    case WX_HOT:         return FACE_WX_HOT;
    case WX_COLD:        return FACE_WX_COLD;
    case WX_HUMID:       return FACE_WX_HUMID;
    case WX_DRY:         return FACE_WX_DRY;
    default:             return FACE_DRIVE; /* No reading yet: neutral, attentive. */
    }
}

static face_mode_t pomodoro_face(uint16_t now)
{
    static uint8_t old_stage = 255u;
    static uint16_t changed_at = 0u, wait = 0u;
    static face_mode_t choice = FACE_DRIVE;
    uint8_t stage;
    if (pomo_state == POMO_DONE) return FACE_PET;
    if (pomo_state != POMO_RUNNING) return FACE_CURIOUS;
    stage = pomo_remaining <= 10u ? 3u :
        ((uint32_t)pomo_remaining * 4u <= pomo_total ? 2u :
        ((uint32_t)pomo_remaining * 2u <= pomo_total ? 1u : 0u));
    if (stage != old_stage || (uint16_t)(now - changed_at) >= wait) {
        uint8_t pick = (uint8_t)(eye_random() % 3u);
        old_stage = stage;
        changed_at = now;
        wait = (uint16_t)(700u + eye_random() % 800u);
        if (stage == 0u) choice = pick == 0u ? FACE_WX_COMFORT :
                                  pick == 1u ? FACE_DRIVE : FACE_CURIOUS;
        else if (stage == 1u) choice = pick == 0u ? FACE_AVOID :
                                       pick == 1u ? FACE_DRIVE : FACE_CURIOUS;
        else if (stage == 2u) choice = pick == 0u ? FACE_WX_HOT :
                                       pick == 1u ? FACE_AVOID : FACE_CURIOUS;
        else choice = pick == 0u ? FACE_STARTLED :
                      pick == 1u ? FACE_CLIFF : FACE_AVOID;
    }
    return choice;
}

static void robot_expression_service(touch_state_t touch, uint16_t now)
{
    action_t action = ACTION_DRIVE;
    face_mode_t mode = FACE_DRIVE;
    if (robot_mode == MODE_POMODORO || robot_mode == MODE_GAME) {
        servos_update(ACTION_DRIVE, TOUCH_NONE, now);
        face_touch = TOUCH_NONE;
        face_turn_left = 0u;
        mode = robot_mode == MODE_POMODORO ? pomodoro_face(now) : FACE_CURIOUS;
        eyes_set_mode(dizzy_overlay ? FACE_DIZZY : mode, now);
        return;
    }
    if (robot_mode == MODE_WEATHER) {
        /* Stationary: servos stay home and touch is ignored.  The condition's
         * expression is shown unless a shake overlays the dizzy reaction. */
        servos_update(ACTION_DRIVE, TOUCH_NONE, now);
        face_touch = TOUCH_NONE;
        face_turn_left = turn_left;
        eyes_set_mode(dizzy_overlay ? FACE_DIZZY : weather_face(), now);
        return;
    }
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

static void eye_buffer_hspan(uint8_t x1, uint8_t x2, uint8_t y)
{
    uint16_t x;
    if (x2 < x1) {
        uint8_t t = x1;
        x1 = x2;
        x2 = t;
    }
    for (x = x1; x <= x2; ++x) eye_buffer_span(x, y, y);
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

/* 5x8 sweat / humidity drop: pointed top, round bottom. */
static void draw_drop(uint8_t x, uint8_t y)
{
    static const uint8_t top[] PROGMEM = {4, 2, 0, 2, 4};
    static const uint8_t bottom[] PROGMEM = {5, 7, 7, 7, 5};
    uint8_t i;
    for (i = 0; i < 5u; ++i)
        eye_buffer_span((int16_t)(x + i), (int16_t)(y + pgm_read_byte(&top[i])),
                        (int16_t)(y + pgm_read_byte(&bottom[i])));
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
    if (frame_drop_l) draw_drop(27, frame_drop_l);
    if (frame_drop_r) draw_drop(98, frame_drop_r);
}

static void catch_render_frame(void)
{
    uint8_t i;
    eye_buffer_clear();
    for (i = 0u; i < 3u; ++i) {
        int16_t x = balls[i].x, y = balls[i].y;
        if (y < 17) continue;
        eye_buffer_span(x - 1, y, y);
        eye_buffer_span(x, y - 1, y + 1);
        eye_buffer_span(x + 1, y, y);
    }
    for (i = 44u; i <= 46u; ++i)
        eye_buffer_hspan(paddle_x, (uint8_t)(paddle_x + 17u), i);
}

static void oled_animation_service(void)
{
    uint8_t display_page;
    if (!oled_ok) return;
    /* Keep every frame stable until all four pages finish, including changes
     * of expression during a transfer. New poses are sampled at a boundary. */
    if (eye_page_index == 0u) {
        if (robot_mode == MODE_GAME &&
            (game_state == GAME_CATCH || game_state == GAME_CATCH_OVER)) {
            catch_render_frame();
        } else {
            eyes_advance_frame(system_ticks_read());
            eyes_render_frame();
        }
    }
    display_page = (uint8_t)(OLED_FIRST_EYE_PAGE + eye_page_index);
    if (!oled_set_position(display_page, OLED_EYE_X) ||
        !oled_write_data(eye_frame_buffer[eye_page_index], OLED_EYE_WIDTH)) {
        twi_bus_recover();
        return;
    }
    if (++eye_page_index >= OLED_EYE_PAGE_COUNT) eye_page_index = 0;
}

/* ------------- Text bands around the eye window (KY-040 UI) ------------ */
/* The eyes own pages 2..5 (x 24..103).  The menu and the weather readout use
 * the free bands above (pages 0-1) and below (pages 6-7), drawn only when the
 * content changes, one page per main-loop pass, so the eye animation keeps
 * running.  Built-in 5x7 font, upper-case, 1x or 2x size. */
#define UI_WIDTH                     128u
#define UI_DEG_CHAR                  '['   /* Font slot 59 is a degree sign. */
#define UI_BAND_PAGES                4u

static uint8_t ui_buf[UI_WIDTH];
static const uint8_t ui_band_page[UI_BAND_PAGES] PROGMEM = {0, 1, 6, 7};

static const uint8_t ui_font[60][5] PROGMEM = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x5F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08},
    {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    {0x00, 0x06, 0x09, 0x09, 0x06}
};

static const char ui_name_roam[] PROGMEM = "ROAM";
static const char ui_name_weather[] PROGMEM = "WEATHER";
static const char ui_name_pomodoro[] PROGMEM = "POMODORO";
static const char ui_name_game[] PROGMEM = "GAME";
/* Scroll the two visible rows as the mode selection changes. */

static const char *ui_mode_name(uint8_t mode)
{
    switch (mode) {
    case MODE_WEATHER: return ui_name_weather;
    case MODE_POMODORO: return ui_name_pomodoro;
    case MODE_GAME: return ui_name_game;
    default: return ui_name_roam;
    }
}

static uint8_t ui_str_len(const char *s, uint8_t in_flash)
{
    uint8_t n = 0u;
    while ((in_flash ? pgm_read_byte(&s[n]) : (uint8_t)s[n]) != 0u) {
        ++n;
    }
    return n;
}

/* Draw `s` into ui_buf at column x.  scale 1: one page.  scale 2: double size,
 * spanning two pages; `sub` selects the upper (0) or lower (1) page of it. */
static void ui_text(const char *s, uint8_t in_flash, uint16_t x,
                    uint8_t scale, uint8_t sub)
{
    uint8_t i, col, rep, r;
    for (i = 0u; ; ++i) {
        uint8_t c = in_flash ? pgm_read_byte(&s[i]) : (uint8_t)s[i];
        uint8_t glyph;
        if (c == 0u) {
            break;
        }
        if (c >= 'a' && c <= 'z') {
            c = (uint8_t)(c - 32u);
        }
        glyph = (c >= ' ' && c <= '[') ? (uint8_t)(c - ' ') : 0u;
        for (col = 0u; col < 6u; ++col) {          /* 5 columns + 1 gap. */
            uint8_t bits = col < 5u ? pgm_read_byte(&ui_font[glyph][col]) : 0u;
            uint8_t out = bits;
            if (scale == 2u) {
                uint16_t wide = 0u;
                for (r = 0u; r < 7u; ++r) {
                    if (bits & _BV(r)) {
                        wide |= (uint16_t)(3u << (2u * r));
                    }
                }
                out = sub ? (uint8_t)(wide >> 8) : (uint8_t)(wide & 0xFFu);
            }
            for (rep = 0u; rep < scale; ++rep) {
                if (x < UI_WIDTH) {
                    ui_buf[x] = out;
                }
                ++x;
            }
        }
    }
}

static void ui_text_centered(const char *s, uint8_t in_flash,
                             uint8_t scale, uint8_t sub)
{
    uint16_t width = (uint16_t)((uint16_t)ui_str_len(s, in_flash) * 6u * scale
                                - scale);
    ui_text(s, in_flash, width < UI_WIDTH ? (uint16_t)((UI_WIDTH - width) / 2u)
                                          : 0u, scale, sub);
}

static char *ui_put_number(char *p, uint8_t value)
{
    if (value >= 100u) {
        *p++ = (char)('0' + value / 100u);
        value = (uint8_t)(value % 100u);
        *p++ = (char)('0' + value / 10u);
    } else if (value >= 10u) {
        *p++ = (char)('0' + value / 10u);
    }
    *p++ = (char)('0' + value % 10u);
    return p;
}

static void ui_value(const char *label, uint16_t value, const char *suffix)
{
    char digits[6], *p = &digits[5];
    uint8_t len;
    *p = '\0';
    do { *--p = (char)('0' + value % 10u); value /= 10u; } while (value);
    ui_text(label, 1u, 0u, 1u, 0u);
    len = ui_str_len(label, 1u);
    ui_text(p, 0u, (uint16_t)len * 6u, 1u, 0u);
    ui_text(suffix, 1u, (uint16_t)(len + ui_str_len(p, 0u)) * 6u, 1u, 0u);
}

static const char *game_item_name(uint8_t item)
{
    switch (item) {
    case 0u: return PSTR("CATCH BALLS");
    case 1u: return PSTR("FIND YOUR TIMING");
    case 2u: return PSTR("HIGH SCORES");
    default: return PSTR("BACK");
    }
}

static void activity_render_page(uint8_t page)
{
    const char *label = 0;
    if (robot_mode == MODE_POMODORO) {
        if (page <= 1u) {
            char clock_text[6];
            uint16_t secs = (pomo_state == POMO_RUNNING || pomo_state == POMO_DONE)
                ? pomo_remaining : (uint16_t)pomo_minutes * 60u + pomo_seconds;
            uint8_t m = (uint8_t)(secs / 60u), sec = (uint8_t)(secs % 60u);
            clock_text[0] = (char)('0' + m / 10u);
            clock_text[1] = (char)('0' + m % 10u); clock_text[2] = ':';
            clock_text[3] = (char)('0' + sec / 10u);
            clock_text[4] = (char)('0' + sec % 10u); clock_text[5] = '\0';
            ui_text_centered(clock_text, 0u, 2u, page);
        } else if (page == 6u) {
            label = pomo_state == POMO_MINUTES ? PSTR("SET MINUTES") :
                    pomo_state == POMO_SECONDS ? PSTR("SET SECONDS") :
                    pomo_state == POMO_RUNNING ? PSTR("FOCUS TIME") : PSTR("TIME IS UP!");
        } else {
            label = pomo_state == POMO_MINUTES ? PSTR("PUSH: NEXT") :
                    pomo_state == POMO_SECONDS ? PSTR("PUSH: START >00:00") :
                    pomo_state == POMO_RUNNING ? PSTR("PUSH: CANCEL / MENU") :
                    PSTR("PUSH: SILENCE / MENU");
        }
    } else if (game_state == GAME_MENU) {
        if (page <= 1u) {
            uint8_t item = (uint8_t)((game_cursor / 2u) * 2u + page);
            if (item == game_cursor) ui_text(PSTR(">"), 1u, 0u, 1u, 0u);
            ui_text(game_item_name(item), 1u, 12u, 1u, 0u);
        } else if (page == 6u) label = PSTR("GAME");
        else label = PSTR("TURN / PUSH SELECT");
    } else if (game_state == GAME_CATCH || game_state == GAME_CATCH_OVER) {
        if (page == 0u) ui_value(PSTR("SCORE "), game_score, PSTR(""));
        else if (page == 1u) ui_value(PSTR("LIVES "), game_lives, PSTR(""));
        else if (page == 6u) label = game_state == GAME_CATCH ?
            PSTR("TURN TO CATCH") : PSTR("ROUND OVER");
        else label = game_state == GAME_CATCH ? PSTR("PUSH: END ROUND") : PSTR("PUSH: GAME MENU");
    } else if (game_state == GAME_TIMING_READY) {
        if (page == 0u) label = PSTR("FIND YOUR TIMING");
        else if (page == 1u) label = PSTR("RELEASE BOTH PADS");
        else if (page == 6u) label = PSTR("PUSH: 5 SEC START");
        else label = PSTR("TURN: BACK");
    } else if (game_state == GAME_TIMING_ACTIVE) {
        if (page == 0u) label = PSTR("FIND YOUR TIMING");
        else if (page == 1u) {
            if (reaction_countdown) ui_value(PSTR("WAIT "), reaction_countdown, PSTR(" SEC"));
            else label = PSTR("TOUCH NOW!");
        } else if (page == 6u) label = PSTR("TOUCH EITHER PAD");
        else label = PSTR("PUSH: CANCEL");
    } else if (game_state == GAME_TIMING_RESULT) {
        if (page == 0u) {
            if (reaction_result == RX_HIT) ui_value(PSTR("TIME "), reaction_ms, PSTR(" MS"));
            else label = reaction_result == RX_EARLY ? PSTR("TOO EARLY!") : PSTR("TIMED OUT");
        } else if (page == 1u && reaction_result == RX_HIT)
            ui_value(PSTR("SCORE "), reaction_score, PSTR(""));
        else if (page == 6u) label = PSTR("FASTER = MORE POINTS");
        else if (page == 7u) label = PSTR("PUSH: GAME MENU");
    } else if (game_state == GAME_SCORES) {
        if (page == 0u) ui_value(PSTR("CATCH BEST "), best_catch, PSTR(""));
        else if (page == 1u) {
            if (best_reaction_ms) ui_value(PSTR("FASTEST "), best_reaction_ms, PSTR(" MS"));
            else label = PSTR("NO TIMING RECORD");
        } else if (page == 6u && best_reaction_ms)
            ui_value(PSTR("TIMING BEST "), timing_score(best_reaction_ms), PSTR(""));
        else if (page == 7u) label = PSTR("PUSH: GAME MENU");
    }
    if (label) ui_text_centered(label, 1u, 1u, 0u);
}

static void ui_render_page(uint8_t page)
{
    char text[8];
    char *p;
    const char *label;
    uint8_t i, len;

    for (i = 0u; i < UI_WIDTH; ++i) {
        ui_buf[i] = 0u;
    }

    if (ui_state == UI_MENU) {
        if (page <= 1u) {
            uint8_t item = (uint8_t)((ui_cursor / 2u) * 2u + page);
            const char *name = ui_mode_name(item);
            len = ui_str_len(name, 1u);
            if (ui_cursor == item) {
                ui_text(PSTR(">"), 1u, 16u, 1u, 0u);
            }
            ui_text(name, 1u, 28u, 1u, 0u);
            if ((uint8_t)robot_mode == item) {          /* Active mode. */
                ui_text(PSTR("*"), 1u, (uint16_t)(28u + len * 6u + 4u), 1u, 0u);
            }
        } else if (page == 7u) {
            ui_text_centered(PSTR("PUSH TO SELECT"), 1u, 1u, 0u);
        }
        return;
    }
    if (robot_mode == MODE_POMODORO || robot_mode == MODE_GAME) {
        activity_render_page(page);
        return;
    }
    if (robot_mode != MODE_WEATHER) {
        return;                        /* ROAM: bands stay blank. */
    }

    if (page <= 1u) {                  /* Readout: double size, pages 0-1. */
        if (wx_error) {
            ui_text_centered(PSTR("SENSOR ERR"), 1u, 2u, page);
            return;
        }
        p = text;
        if (wx_valid) {
            p = ui_put_number(p, wx_temp_c);
        } else {
            *p++ = '-';
            *p++ = '-';
        }
        *p++ = UI_DEG_CHAR;
        *p++ = 'C';
        *p = '\0';
        ui_text(text, 0u, 4u, 2u, page);

        p = text;
        if (wx_valid) {
            p = ui_put_number(p, wx_humidity);
        } else {
            *p++ = '-';
            *p++ = '-';
        }
        *p++ = '%';
        *p = '\0';
        len = (uint8_t)(p - text);
        ui_text(text, 0u, (uint16_t)(UI_WIDTH - 4u - (len * 12u - 2u)), 2u, page);
        return;
    }

    /* Bottom band: the detected condition. */
    if (wx_error) {
        label = PSTR("CHECK DHT11");
    } else if (!wx_valid) {
        label = PSTR("READING");
    } else {
        switch (wx_class) {
        case WX_HOT:   label = PSTR("HOT");   break;
        case WX_COLD:  label = PSTR("COLD");  break;
        case WX_HUMID: label = PSTR("HUMID"); break;
        case WX_DRY:   label = PSTR("DRY");   break;
        default:       label = PSTR("COMFORTABLE"); break;
        }
    }
    len = ui_str_len(label, 1u);
    if ((uint16_t)len * 12u - 2u <= 124u) {        /* Fits at double size. */
        ui_text_centered(label, 1u, 2u, (uint8_t)(page - 6u));
    } else if (page == 7u) {                        /* e.g. COMFORTABLE. */
        ui_text_centered(label, 1u, 1u, 0u);
    }
}

static void oled_ui_service(void)
{
    uint8_t bit, page;
    if (!oled_ok || ui_dirty == 0u) {
        return;
    }
    for (bit = 0u; bit < UI_BAND_PAGES; ++bit) {
        if (ui_dirty & _BV(bit)) {
            break;
        }
    }
    page = pgm_read_byte(&ui_band_page[bit]);
    ui_render_page(page);
    if (!oled_set_position(page, 0u) || !oled_write_data(ui_buf, UI_WIDTH)) {
        twi_bus_recover();             /* Leave the bit set and retry later. */
        return;
    }
    ui_dirty &= (uint8_t)~_BV(bit);
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
    encoder_init();                    /* After shock_init(): JTAG is off. */
    dht_init();
    buzzer_init();
    best_catch = score_load(&ee_catch);
    best_reaction_ms = score_load(&ee_reaction);
    if (best_reaction_ms > 10000u) best_reaction_ms = 0u;
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

        ui_service(now);                /* KY-040: menu and mode changes. */
        activity_service(system_ticks_read());
        microphone_service(now);
        /* Apply known safety/touch/sound state before another sonar sample. */
        robot_update(now, touch_state);
        motors_service(now);

        sonar_service(now);
        now = system_ticks_read();
        robot_update(now, touch_state);
        motors_service(now);

        /* Weather Mode only (motors braked): about 25-45 ms every ~2 s. */
        dht_service(now);
        now = system_ticks_read();

        robot_expression_service(touch_state, now);
        oled_animation_service();
        oled_ui_service();
        /* No behavior delays here: all pause/turn/ramp deadlines are timed.
         * OLED page I/O and bounded sonar polling leave Timer2 IRQ enabled. */
    }
}
