#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>

/* --------------------------- User settings --------------------------- */

#define MOTOR_SPEED                  255u
#define OBSTACLE_DISTANCE_CM         20u
#define ULTRASONIC_PERIOD_TICKS      30u   /* 30 x 2.048 ms = 61.44 ms */
#define CLEAR_READINGS_REQUIRED      2u
#define IR_STABLE_SAMPLES            2u

/* Default TTP223 boards are momentary and active HIGH. */
#define TOUCH_ACTIVE_HIGH            1u
#define TOUCH_DEBOUNCE_TICKS         8u    /* About 16.4 ms. */
#define TOUCH_STARTUP_IGNORE_TICKS   250u  /* About 0.51 s calibration time. */

#define SERVO_HOME_TICKS             125u  /* 1.000 ms at 8 us/tick */
#define SERVO_CLIFF_TICKS            188u  /* 1.504 ms: about 90 degrees */
#define SERVO_PET_TICKS              145u  /* 1.160 ms: gentle petting wiggle */
#define SERVO_PET_HALF_PERIOD_TICKS  120u  /* About 246 ms each way. */

#define SH1106_COLUMN_OFFSET         2u
#define OLED_EYE_X                   28u
#define OLED_EYE_WIDTH               72u
#define OLED_FIRST_EYE_PAGE          2u
#define OLED_EYE_PAGE_COUNT          4u

#define PAT_EYE_X_SHIFT              5
#define PAT_EYE_Y_SHIFT             -5
#define CLOSED_EYE_HALF_WIDTH        8

/* Timer0 runs at F_CPU/8, so one timer count is 8 microseconds. */
#define TIMER0_TICK_US               8UL
#define OBSTACLE_THRESHOLD_TICKS \
    ((OBSTACLE_DISTANCE_CM * 58UL + TIMER0_TICK_US - 1UL) / TIMER0_TICK_US)
#define MINIMUM_VALID_ECHO_TICKS \
    ((2UL * 58UL + TIMER0_TICK_US - 1UL) / TIMER0_TICK_US)

#if OBSTACLE_DISTANCE_CM > 35
#error "At 1 MHz this short Timer0 method supports a threshold up to 35 cm"
#endif

/* ------------------------------ States ------------------------------- */

typedef enum {
    IR_UNKNOWN,
    IR_SURFACE,
    IR_CLIFF
} ir_state_t;

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

/* ------------------------------- Motors ------------------------------ */
/*
 * L298N:
 * ENA=PD5/OC1A, IN1=PD2, IN2=PD3
 * ENB=PD4/OC1B, IN3=PD6, IN4=PD1
 *
 * Motor B has the opposite electrical polarity so both wheels move the
 * robot physically forward.
 */

static void motors_init(void)
{
    DDRD |= _BV(PD1) | _BV(PD2) | _BV(PD3) |
            _BV(PD4) | _BV(PD5) | _BV(PD6);

    PORTD &= (uint8_t)~(_BV(PD1) | _BV(PD2) | _BV(PD3) | _BV(PD6));

    /* Timer1: 8-bit Fast PWM, non-inverting OC1A and OC1B, prescaler /8. */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS11);
    OCR1A = 0;
    OCR1B = 0;
}

static void motors_forward(uint8_t speed_a, uint8_t speed_b)
{
    /* Motor A: IN1=1, IN2=0. */
    PORTD |= _BV(PD2);
    PORTD &= (uint8_t)~_BV(PD3);

    /* Motor B: IN3=0, IN4=1 (opposite-mounted motor). */
    PORTD &= (uint8_t)~_BV(PD6);
    PORTD |= _BV(PD1);

    OCR1A = speed_a;
    OCR1B = speed_b;
}

static void motors_stop(void)
{
    OCR1A = 0;
    OCR1B = 0;
    PORTD &= (uint8_t)~(_BV(PD1) | _BV(PD2) | _BV(PD3) | _BV(PD6));
}

/* -------------------------- Cliff IR sensor -------------------------- */
/* PB0 LOW = surface, PB0 HIGH = cliff, as established on this robot. */

static void ir_init(void)
{
    DDRB &= (uint8_t)~_BV(PB0);
    PORTB |= _BV(PB0);                 /* Harmless pull-up for open output. */
}

static ir_state_t ir_update(void)
{
    static ir_state_t candidate = IR_UNKNOWN;
    static ir_state_t stable = IR_UNKNOWN;
    static uint8_t same_count = 0;
    ir_state_t sample;

    sample = (PINB & _BV(PB0)) ? IR_CLIFF : IR_SURFACE;

    if (sample != candidate) {
        candidate = sample;
        same_count = 1;
    } else if (same_count < IR_STABLE_SAMPLES) {
        ++same_count;
    }

    if (same_count >= IR_STABLE_SAMPLES) {
        stable = candidate;
    }

    return stable;
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

static uint8_t ultrasonic_obstacle_near(void)
{
    uint8_t rise_timeouts = 0;
    uint8_t pulse_ticks;

    /* A stuck-high ECHO is treated fail-safe as an obstacle. */
    if (PIND & _BV(PD0)) {
        return 1;
    }

    ultrasonic_trigger();

    /* The rising edge normally appears quickly; allow about 4.1 ms. */
    TCNT0 = 0;
    TIFR = _BV(TOV0);
    while (!(PIND & _BV(PD0))) {
        if (TIFR & _BV(TOV0)) {
            TIFR = _BV(TOV0);
            if (++rise_timeouts >= 2u) {
                return 0;             /* No echo means no measured obstacle. */
            }
        }
    }

    TCNT0 = 0;
    TIFR = _BV(TOV0);
    while (PIND & _BV(PD0)) {
        if (TIFR & _BV(TOV0)) {
            return 0;                 /* Farther than this configured range. */
        }
        if (TCNT0 > (uint8_t)OBSTACLE_THRESHOLD_TICKS) {
            return 0;
        }
    }

    pulse_ticks = TCNT0;
    if (pulse_ticks < (uint8_t)MINIMUM_VALID_ECHO_TICKS) {
        return 0;                     /* Reject a very short noise pulse. */
    }

    return pulse_ticks <= (uint8_t)OBSTACLE_THRESHOLD_TICKS;
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

static void eyes_advance_frame(void)
{
    if (eyes_alert) {
        frame_eyes_closed = 0;
        gaze_x = step_toward(gaze_x, 0);
        gaze_y = step_toward(gaze_y, 0);
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
    } else {
        frame_eyes_closed = 0;
        frame_eye_growth = 0;

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

    /* One 72-byte page per call keeps control sensing responsive. */
    if (eye_page_index == 0u) {
        eyes_advance_frame();
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
    ir_state_t ir_state = IR_UNKNOWN;
    action_t applied_action = ACTION_UNKNOWN;
    touch_state_t touch_state = TOUCH_NONE;
    uint8_t obstacle = 0;
    uint8_t clear_readings = 0;
    uint8_t touch_ready = 0;
    uint16_t last_ultrasonic_tick;
    uint16_t touch_start_tick;

    motors_init();
    motors_stop();
    ir_init();
    touch_init();
    ultrasonic_init();
    oled_init();
    servos_init();
    servos_home();

    sei();

    /* Cause the first ultrasonic measurement immediately. */
    last_ultrasonic_tick =
        (uint16_t)(system_ticks_read() - ULTRASONIC_PERIOD_TICKS);
    touch_start_tick = system_ticks_read();

    while (1) {
        uint16_t now;
        action_t wanted_action;

        ir_state = ir_update();
        now = system_ticks_read();

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

        if ((uint16_t)(now - last_ultrasonic_tick) >=
            ULTRASONIC_PERIOD_TICKS) {
            uint8_t near;
            last_ultrasonic_tick = now;
            near = ultrasonic_obstacle_near();

            if (near) {
                obstacle = 1;
                clear_readings = 0;
            } else if (obstacle) {
                if (++clear_readings >= CLEAR_READINGS_REQUIRED) {
                    obstacle = 0;
                    clear_readings = 0;
                }
            }
        }

        /* Cliff > petting > obstacle > normal drive. */
        if (ir_state == IR_CLIFF) {
            wanted_action = ACTION_CLIFF;
        } else if (ir_state == IR_SURFACE &&
                   touch_state != TOUCH_NONE) {
            wanted_action = ACTION_PETTING;
        } else if (ir_state == IR_SURFACE && obstacle) {
            wanted_action = ACTION_OBSTACLE;
        } else if (ir_state == IR_SURFACE) {
            wanted_action = ACTION_DRIVE;
        } else {
            wanted_action = ACTION_UNKNOWN;
        }

        if (wanted_action != applied_action) {
            switch (wanted_action) {
            case ACTION_DRIVE:
                motors_forward(MOTOR_SPEED, MOTOR_SPEED);
                break;

            case ACTION_CLIFF:
                motors_stop();
                break;

            case ACTION_PETTING:
                motors_stop();
                break;

            case ACTION_OBSTACLE:
                motors_stop();
                break;

            default:
                motors_stop();
                break;
            }
            applied_action = wanted_action;
        }

        /*
         * Petting, cliff, and obstacle states all stop the wheels. A cliff
         * also overrides the pet motion. Touch may override the obstacle face
         * so a hand near the ultrasonic sensor does not hide the reaction.
         */
        servos_update(wanted_action, touch_state, now);

        if (wanted_action == ACTION_CLIFF ||
            wanted_action == ACTION_UNKNOWN) {
            eyes_set_touch(TOUCH_NONE);
        } else {
            eyes_set_touch(touch_state);
        }

        eyes_set_alert((uint8_t)(
            wanted_action == ACTION_CLIFF ||
            (wanted_action == ACTION_OBSTACLE &&
             touch_state == TOUCH_NONE)));
        oled_animation_service();
        _delay_ms(1);
    }
}
