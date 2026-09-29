/*
 * STATIONARY ATmega32 MICROPHONE THRESHOLD TEST -- 1 MHz, SH1106 128x64 I2C.
 * Standalone file; no robot_sh1106.c dependency.
 *
 * PURPOSE: sweeps through several candidate (MIN_P2P, RISE_P2P) threshold
 * pairs -- the same two numbers the full robot firmware's
 * microphone_accept_level() uses (there: MIC_MIN_P2P / MIC_RISE_P2P) --
 * and, for each one, measures how many times it triggers while you are
 * silent (false positives) versus while you deliberately make noise
 * (real hits). It uses the identical envelope/arm/cooldown algorithm as
 * the robot firmware, just re-timed to this file's coarser ~98 ms
 * measurement window instead of production's ~16 ms one. At the end it
 * reports every candidate's hit/false-trigger counts and highlights the
 * one with the best score, so you can copy its MIN/RISE values straight
 * into MIC_MIN_P2P/MIC_RISE_P2P in the main robot firmware.
 *
 * HOW TO RUN IT:
 *   1. Flash this file. The OLED shows "MIC THRESH TEST" for ~2 s.
 *   2. For each candidate listed in mic_candidate_min/_rise below:
 *        - "SETTLING"      (~1 s): stay quiet, lets the noise floor settle.
 *        - "STAY QUIET"    (~2 s): stay silent. Any trigger here counts
 *          as a FALSE POSITIVE for that candidate.
 *        - "MAKE NOISE"    (~2 s): clap/speak/tap near the mic repeatedly.
 *          Every trigger here counts as a HIT for that candidate.
 *        - "NEXT CANDIDATE"(~1 s): brief pause, then it repeats for the
 *          next candidate.
 *   3. After the last candidate, "MIC TEST RESULTS" lists every
 *      candidate's MIN/RISE/HITS/FALSE counts (one per line), with a "*"
 *      marking the best-scoring one and a summary line with its exact
 *      MIN/RISE values. This stays on screen for about 15 s, then the
 *      whole test loops automatically so you can re-run it (e.g. after
 *      moving the mic) without reflashing.
 *
 * SCORING: score = hits - 3 * false_triggers. This favors a candidate
 * that reliably detects real noise while being heavily penalized for
 * triggering on background/silence -- tune MIC_FALSE_PENALTY below if
 * you'd rather weight false positives differently.
 *
 * Build: avr-gcc -mmcu=atmega32 -DF_CPU=1000000UL -std=gnu99 -Os \
 *          -Wall -Wextra mic_threshold_test.c -o mic_test.elf
 *        avr-objcopy -O ihex -R .eeprom mic_test.elf mic_test.hex
 *
 * Wiring (DIP-40, unchanged from robot firmware): OLED SCL PC0/22,
 * SDA PC1/23; MAX9814 OUT PA2/ADC2/38. AVCC = regulated 5V, AREF = 100nF
 * to GND (never wire AREF directly to GND or to +5V).
 *
 * Motors/servos never drive, same as the original sensor diagnostic this
 * file is based on: Timer1 is off, ENA/ENB and directions are LOW, servo
 * pulses are off, sonar TRIG stays LOW. REMOVE the L298N EN jumpers as
 * usual whenever the MCU drives ENA/ENB.
 */
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#if F_CPU != 1000000UL
//#error "This diagnostic timing requires the actual MCU clock to be 1 MHz."
#endif
#ifndef DIAG_ADC_ENABLED
#define DIAG_ADC_ENABLED 1
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <string.h>

#define SH1106_COLUMN_OFFSET 2u

/* ------------------------ Candidate thresholds ------------------------ */
/* Each pair is tested in turn. Values bracket the production defaults
 * (MIC_MIN_P2P=100, MIC_RISE_P2P=50), from more sensitive to less. Edit
 * this list to add/remove candidates -- everything below adapts to
 * however many entries you put here. */
static const uint8_t mic_candidate_min[]  PROGMEM = { 40u,  60u,  80u, 100u, 130u, 160u};
static const uint8_t mic_candidate_rise[] PROGMEM = { 15u,  20u,  30u,  50u,  60u,  70u};
#define MIC_CANDIDATE_COUNT \
    ((uint8_t)(sizeof(mic_candidate_min) / sizeof(mic_candidate_min[0])))

#define MIC_FALSE_PENALTY 3 /* score = hits - MIC_FALSE_PENALTY * false */

/* Windows are ~98.3 ms each (see reading.sequence below): 6 x Timer0's
 * 16.384 ms overflow. Counts below are in units of that window. */
#define MIC_TEST_INTRO_WINDOWS        20u  /* ~2.0 s */
#define MIC_TEST_SETTLE_WINDOWS       10u  /* ~1.0 s: let the floor settle */
#define MIC_TEST_QUIET_WINDOWS        20u  /* ~2.0 s: measure false triggers */
#define MIC_TEST_ACTIVE_WINDOWS       20u  /* ~2.0 s: measure real hits */
#define MIC_TEST_NEXT_WINDOWS         10u  /* ~1.0 s pause between candidates */
#define MIC_TEST_REARM_WINDOWS         3u  /* quiet windows needed to re-arm */
#define MIC_TEST_COOLDOWN_WINDOWS      6u  /* ignore-retrigger after an event */
#define MIC_TEST_RESULTS_HOLD_WINDOWS 150u /* ~15 s before auto-restart */

/* Timer0 runs at /64: one overflow = 16.384 ms. */
typedef struct {
    uint8_t latest;
    uint8_t low;
    uint8_t high;
    uint8_t p2p;
    uint8_t hold;
    uint8_t sequence;
    uint16_t count;
} reading_t;

static reading_t reading;
static uint8_t sample_low = 255u, sample_high, sample_latest;
static uint16_t sample_count;
static uint8_t window_ticks, hold_windows, display_due;
static uint8_t page_buffer[128];

static void diagnostic_poll(void)
{
#if DIAG_ADC_ENABLED
    if (ADCSRA & _BV(ADIF)) {
        uint8_t value = ADCH;
        /* ADIF is write-one-to-clear. ADIE remains disabled. */
        ADCSRA |= _BV(ADIF);
        sample_latest = value;
        if (value < sample_low) sample_low = value;
        if (value > sample_high) sample_high = value;
        if (sample_count < 65535u) sample_count++;
    }
#endif
    if (TIFR & _BV(TOV0)) {
        TIFR = _BV(TOV0);
        display_due = 1u;
        if (++window_ticks >= 6u) {
            window_ticks = 0u;
            reading.latest = sample_latest;
            reading.count = sample_count;
            reading.low = sample_count ? sample_low : 0u;
            reading.high = sample_count ? sample_high : 0u;
            reading.p2p = (uint8_t)(reading.high - reading.low);
            reading.sequence++;
            if (hold_windows) hold_windows--;
            if (!hold_windows || reading.p2p > reading.hold) {
                reading.hold = reading.p2p;
                hold_windows = 10u;
            }
            sample_low = 255u;
            sample_high = 0u;
            sample_count = 0u;
        }
    }
}

static void stationary_hardware_init(void)
{
    cli();
    TIMSK = 0u;
    TCCR0 = 0u;
    TCCR1A = 0u;
    TCCR1B = 0u;
    TCCR2 = 0u;
    OCR1A = 0u;
    OCR1B = 0u;

    PORTD &= (uint8_t)~(_BV(PD1) | _BV(PD2) | _BV(PD3) |
                        _BV(PD4) | _BV(PD5) | _BV(PD6));
    DDRD |= _BV(PD1) | _BV(PD2) | _BV(PD3) |
            _BV(PD4) | _BV(PD5) | _BV(PD6);
    PORTB &= (uint8_t)~(_BV(PB1) | _BV(PB2) | _BV(PB3));
    DDRB |= _BV(PB1) | _BV(PB2) | _BV(PB3);
    DDRB &= (uint8_t)~_BV(PB0);
    PORTB |= _BV(PB0);             /* Same IR pull-up as robot firmware. */
    DDRA &= (uint8_t)~(_BV(PA0) | _BV(PA1) | _BV(PA2));
    PORTA &= (uint8_t)~(_BV(PA0) | _BV(PA1) | _BV(PA2));

    ADCSRA = 0u;
#if DIAG_ADC_ENABLED
    ADMUX = _BV(REFS0) | _BV(ADLAR) | 2u; /* AVCC, left adjust, ADC2 only. */
    SFIOR &= (uint8_t)~(_BV(ADTS0) | _BV(ADTS1) | _BV(ADTS2));
    /* /16 = 62.5 kHz ADC clock; free run, about 4,808 samples/s. */
    ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIF) |
             _BV(ADPS2) | _BV(ADSC);
#endif
    TCNT0 = 0u;
    TIFR = _BV(TOV0);
    TCCR0 = _BV(CS01) | _BV(CS00);
    /* Global interrupts intentionally remain disabled. */
}

/* ------------------------- Hardware TWI/I2C -------------------------- */

static uint8_t twi_wait(void)
{
    uint16_t timeout = 5000u;

    while (!(TWCR & _BV(TWINT))) {
        diagnostic_poll();
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
        diagnostic_poll();
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

/* Five columns per glyph, bit 0 at the top. Font stays in flash. Supports
 * digits, A-Z, ':', and space (blank column) -- keep on-screen strings to
 * those characters. */
static const uint8_t digits[10][5] PROGMEM = {
    {0x3e,0x51,0x49,0x45,0x3e}, {0x00,0x42,0x7f,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3c,0x4a,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1e}
};
static const uint8_t letters[26][5] PROGMEM = {
    {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36},
    {0x3e,0x41,0x41,0x41,0x22}, {0x7f,0x41,0x41,0x22,0x1c},
    {0x7f,0x49,0x49,0x49,0x41}, {0x7f,0x09,0x09,0x09,0x01},
    {0x3e,0x41,0x49,0x49,0x7a}, {0x7f,0x08,0x08,0x08,0x7f},
    {0x00,0x41,0x7f,0x41,0x00}, {0x20,0x40,0x41,0x3f,0x01},
    {0x7f,0x08,0x14,0x22,0x41}, {0x7f,0x40,0x40,0x40,0x40},
    {0x7f,0x02,0x0c,0x02,0x7f}, {0x7f,0x04,0x08,0x10,0x7f},
    {0x3e,0x41,0x41,0x41,0x3e}, {0x7f,0x09,0x09,0x09,0x06},
    {0x3e,0x41,0x51,0x21,0x5e}, {0x7f,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7f,0x01,0x01},
    {0x3f,0x40,0x40,0x40,0x3f}, {0x1f,0x20,0x40,0x20,0x1f},
    {0x3f,0x40,0x38,0x40,0x3f}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static uint8_t text_x;
static void put_char(char c)
{
    uint8_t i;
    if (text_x > 122u) return;
    for (i = 0u; i < 5u; i++) {
        uint8_t column = 0u;
        if (c >= '0' && c <= '9')
            column = pgm_read_byte(&digits[(uint8_t)(c - '0')][i]);
        else if (c >= 'A' && c <= 'Z')
            column = pgm_read_byte(&letters[(uint8_t)(c - 'A')][i]);
        else if (c == ':' && i == 2u) column = 0x24u;
        page_buffer[text_x++] = column;
    }
    page_buffer[text_x++] = 0u;
}
static void put_text(const char *s)
{
    while (*s) put_char(*s++);
}
static void put_number(uint16_t value, uint8_t width)
{
    char reversed[5];
    uint8_t count = 0u;
    do {
        reversed[count++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value && count < (uint8_t)sizeof(reversed));
    while (width > count) {
        put_char('0');
        width--;
    }
    while (count) put_char(reversed[--count]);
}

/* --------------------- Microphone threshold sweep --------------------- */

typedef enum {
    TPHASE_INTRO, TPHASE_SETTLE, TPHASE_QUIET, TPHASE_ACTIVE,
    TPHASE_NEXT, TPHASE_RESULTS
} test_phase_t;

typedef struct {
    uint8_t hits;
    uint8_t false_triggers;
    int16_t score;
} mic_result_t;

static test_phase_t test_phase = TPHASE_INTRO;
static uint8_t candidate_index = 0u;
static uint8_t phase_windows_left = MIC_TEST_INTRO_WINDOWS;
static mic_result_t mic_results[MIC_CANDIDATE_COUNT];
static uint8_t best_candidate = 0u;

/* Envelope-detector state, same algorithm as the robot firmware's
 * microphone_accept_level(), re-timed to ~98 ms windows. */
static uint16_t mic_floor_q4;
static uint8_t mic_ready, mic_armed, mic_in_cooldown, mic_quiet_windows;
static uint8_t mic_cooldown_left, mic_settle_seen;
static uint8_t last_level, last_threshold;

static uint8_t mic_candidate_min_at(uint8_t idx)
{
    return pgm_read_byte(&mic_candidate_min[idx]);
}
static uint8_t mic_candidate_rise_at(uint8_t idx)
{
    return pgm_read_byte(&mic_candidate_rise[idx]);
}

static void mic_test_reset_candidate(void)
{
    mic_floor_q4 = 0u;
    mic_ready = 0u;
    mic_armed = 0u;
    mic_in_cooldown = 0u;
    mic_quiet_windows = 0u;
    mic_cooldown_left = 0u;
    mic_settle_seen = 0u;
}

/* One call per ~98 ms window. Returns 1 if this window triggered an event
 * under the given candidate thresholds. */
static uint8_t mic_test_process(uint8_t level, uint8_t min_p2p, uint8_t rise_p2p)
{
    uint16_t threshold = (uint16_t)(mic_floor_q4 >> 4) + rise_p2p;
    uint16_t target = (uint16_t)level << 4;
    uint8_t event = 0u;

    if (threshold < min_p2p) {
        threshold = min_p2p;
    }
    last_level = level;
    last_threshold = (uint8_t)(threshold > 255u ? 255u : threshold);

    if (mic_in_cooldown) {
        if (mic_cooldown_left) mic_cooldown_left--;
        else mic_in_cooldown = 0u;
    }

    if (!mic_ready) {
        if (++mic_settle_seen >= MIC_TEST_SETTLE_WINDOWS) {
            mic_ready = mic_armed = 1u;
        }
    } else {
        if (mic_armed && !mic_in_cooldown && level >= threshold) {
            event = 1u;
            mic_in_cooldown = 1u;
            mic_cooldown_left = MIC_TEST_COOLDOWN_WINDOWS;
            mic_armed = 0u;
            mic_quiet_windows = 0u;
        }
        if (!mic_armed && !event) {
            if ((uint16_t)level + 6u < threshold) {
                if (mic_quiet_windows < MIC_TEST_REARM_WINDOWS) mic_quiet_windows++;
                if (mic_quiet_windows >= MIC_TEST_REARM_WINDOWS) mic_armed = 1u;
            } else {
                mic_quiet_windows = 0u;
            }
        }
    }

    if (!event) {
        if (target > mic_floor_q4) {
            mic_floor_q4 = (uint16_t)(mic_floor_q4 +
                ((target - mic_floor_q4 + 15u) >> 4));
        } else {
            mic_floor_q4 = (uint16_t)(mic_floor_q4 -
                ((mic_floor_q4 - target + 7u) >> 3));
        }
    }
    return event;
}

static void mic_results_clear(void)
{
    uint8_t i;
    for (i = 0u; i < MIC_CANDIDATE_COUNT; ++i) {
        mic_results[i].hits = 0u;
        mic_results[i].false_triggers = 0u;
        mic_results[i].score = 0;
    }
}

/* Advances the test state machine by exactly one ~98 ms window. Call once
 * per new reading.sequence value, passing reading.p2p as the level. */
static void mic_test_tick(uint8_t level)
{
    uint8_t min_p2p, rise_p2p;

    switch (test_phase) {
    case TPHASE_INTRO:
        if (phase_windows_left) { phase_windows_left--; return; }
        candidate_index = 0u;
        mic_test_reset_candidate();
        test_phase = TPHASE_SETTLE;
        phase_windows_left = MIC_TEST_SETTLE_WINDOWS;
        return;

    case TPHASE_SETTLE:
        min_p2p = mic_candidate_min_at(candidate_index);
        rise_p2p = mic_candidate_rise_at(candidate_index);
        (void)mic_test_process(level, min_p2p, rise_p2p); /* let floor settle */
        if (mic_ready) {
            test_phase = TPHASE_QUIET;
            phase_windows_left = MIC_TEST_QUIET_WINDOWS;
        }
        return;

    case TPHASE_QUIET:
        min_p2p = mic_candidate_min_at(candidate_index);
        rise_p2p = mic_candidate_rise_at(candidate_index);
        if (mic_test_process(level, min_p2p, rise_p2p)) {
            if (mic_results[candidate_index].false_triggers < 255u) {
                mic_results[candidate_index].false_triggers++;
            }
        }
        if (phase_windows_left) {
            phase_windows_left--;
        } else {
            test_phase = TPHASE_ACTIVE;
            phase_windows_left = MIC_TEST_ACTIVE_WINDOWS;
        }
        return;

    case TPHASE_ACTIVE:
        min_p2p = mic_candidate_min_at(candidate_index);
        rise_p2p = mic_candidate_rise_at(candidate_index);
        if (mic_test_process(level, min_p2p, rise_p2p)) {
            if (mic_results[candidate_index].hits < 255u) {
                mic_results[candidate_index].hits++;
            }
        }
        if (phase_windows_left) {
            phase_windows_left--;
        } else {
            mic_results[candidate_index].score =
                (int16_t)mic_results[candidate_index].hits -
                (int16_t)(MIC_FALSE_PENALTY *
                          (int16_t)mic_results[candidate_index].false_triggers);
            candidate_index++;
            if (candidate_index < MIC_CANDIDATE_COUNT) {
                test_phase = TPHASE_NEXT;
                phase_windows_left = MIC_TEST_NEXT_WINDOWS;
            } else {
                uint8_t i;
                best_candidate = 0u;
                for (i = 1u; i < MIC_CANDIDATE_COUNT; ++i) {
                    if (mic_results[i].score > mic_results[best_candidate].score) {
                        best_candidate = i;
                    }
                }
                test_phase = TPHASE_RESULTS;
                phase_windows_left = MIC_TEST_RESULTS_HOLD_WINDOWS;
            }
        }
        return;

    case TPHASE_NEXT:
        if (phase_windows_left) { phase_windows_left--; return; }
        mic_test_reset_candidate();
        test_phase = TPHASE_SETTLE;
        phase_windows_left = MIC_TEST_SETTLE_WINDOWS;
        return;

    case TPHASE_RESULTS:
    default:
        if (phase_windows_left) {
            phase_windows_left--;
        } else {
            mic_results_clear();
            candidate_index = 0u;
            test_phase = TPHASE_INTRO;
            phase_windows_left = MIC_TEST_INTRO_WINDOWS;
        }
        return;
    }
}

/* ------------------------- OLED page rendering ------------------------ */

static uint8_t draw_mic_test_page(uint8_t page)
{
    memset(page_buffer, 0, sizeof(page_buffer));
    text_x = 0u;

    if (test_phase == TPHASE_INTRO) {
        switch (page) {
        case 0: put_text("MIC THRESH TEST"); break;
        case 1: put_text("TESTS "); put_number(MIC_CANDIDATE_COUNT, 1u);
                put_text(" SETTINGS"); break;
        case 2: put_text("EACH: SETTLE"); break;
        case 3: put_text("THEN QUIET 2S"); break;
        case 4: put_text("THEN NOISE 2S"); break;
        case 5: put_text("REPEAT FOR ALL"); break;
        case 6: put_text("THEN SHOWS BEST"); break;
        default: put_text("STARTING"); break;
        }
    } else if (test_phase == TPHASE_RESULTS) {
        if (page == 0u) {
            put_text("MIC TEST RESULTS");
        } else if (page == 7u) {
            put_text("BEST C"); put_number((uint16_t)(best_candidate + 1u), 1u);
            put_text(" M"); put_number(mic_candidate_min_at(best_candidate), 3u);
            put_text(" R"); put_number(mic_candidate_rise_at(best_candidate), 3u);
        } else {
            uint8_t idx = (uint8_t)(page - 1u);
            if (idx < MIC_CANDIDATE_COUNT) {
                put_text("C"); put_number((uint16_t)(idx + 1u), 1u);
                put_text(" M"); put_number(mic_candidate_min_at(idx), 3u);
                put_text(" R"); put_number(mic_candidate_rise_at(idx), 3u);
                put_text(" H"); put_number(mic_results[idx].hits, 3u);
                put_text(" F"); put_number(mic_results[idx].false_triggers, 3u);
                if (idx == best_candidate) put_text(" X");
            }
        }
    } else {
        uint8_t min_p2p = mic_candidate_min_at(candidate_index);
        uint8_t rise_p2p = mic_candidate_rise_at(candidate_index);
        switch (page) {
        case 0:
            if (test_phase == TPHASE_SETTLE) put_text("SETTLING");
            else if (test_phase == TPHASE_QUIET) put_text("STAY QUIET");
            else if (test_phase == TPHASE_ACTIVE) put_text("MAKE NOISE");
            else put_text("NEXT CANDIDATE");
            break;
        case 1:
            put_text("CAND "); put_number((uint16_t)(candidate_index + 1u), 1u);
            put_text(" OF "); put_number(MIC_CANDIDATE_COUNT, 1u);
            break;
        case 2:
            put_text("MIN"); put_number(min_p2p, 3u);
            put_text(" RISE"); put_number(rise_p2p, 3u);
            break;
        case 3:
            put_text("TIME "); put_number(phase_windows_left, 3u);
            break;
        case 4:
            put_text("LVL"); put_number(last_level, 3u);
            put_text(" THR"); put_number(last_threshold, 3u);
            break;
        case 5:
            put_text("FLOOR"); put_number((uint16_t)(mic_floor_q4 >> 4), 3u);
            break;
        case 6:
            if (test_phase == TPHASE_QUIET) {
                put_text("FALSE ");
                put_number(mic_results[candidate_index].false_triggers, 3u);
            } else if (test_phase == TPHASE_ACTIVE) {
                put_text("HITS ");
                put_number(mic_results[candidate_index].hits, 3u);
            } else {
                put_text("WAITING");
            }
            break;
        default:
            if (test_phase == TPHASE_QUIET) put_text("BE SILENT");
            else if (test_phase == TPHASE_ACTIVE) put_text("CLAP OR SPEAK");
            else if (test_phase == TPHASE_SETTLE) put_text("HOLD STILL");
            else put_text("GET READY");
            break;
        }
    }

    return oled_set_position(page, 0u) &&
           oled_write_data(page_buffer, (uint8_t)sizeof(page_buffer));
}

int main(void)
{
    uint8_t page = 0u;
    uint8_t last_sequence = 0xFFu;

    stationary_hardware_init();
    oled_init();
    display_due = 1u;

    for (;;) {
        diagnostic_poll();
        if (!display_due) continue;
        display_due = 0u;
        if (!oled_ok) {
            twi_bus_recover();
            oled_init();
            page = 0u;
            continue;
        }
        if (!page && reading.sequence != last_sequence) {
            last_sequence = reading.sequence;
            mic_test_tick(reading.p2p);
        }
        if (!draw_mic_test_page(page)) {
            oled_ok = 0u;
            continue;
        }
        page = (uint8_t)((page + 1u) & 7u);
    }
}
