/*
 * STATIONARY ATmega32 sensor diagnostic -- 1 MHz, SH1106 128x64 I2C.
 * Standalone file; no robot_sh1106.c dependency.
 *
 * Build: avr-gcc -mmcu=atmega32 -DF_CPU=1000000UL -std=gnu99 -Os \
 *          -Wall -Wextra robot_sensor_diagnostic.c -o sensor_diag.elf
 *        avr-objcopy -O ihex -R .eeprom sensor_diag.elf sensor_diag.hex
 *
 * For an ADC-OFF comparison, add -DDIAG_ADC_ENABLED=0 to avr-gcc.
 * Leave AVCC connected to regulated +5 V in BOTH builds. AREF uses a
 * 100 nF capacitor to GND, not a direct +5 V connection and not a direct
 * GND connection either.
 *
 * Wiring (DIP-40): OLED SCL PC0/22, SDA PC1/23; touch L PA0/40,
 * touch R PA1/39; IR PB0/1; MAX9814 OUT PA2/ADC2/38.
 * All GPIO values displayed are RAW 0/1 values, without filtering/FSM.
 * ADC numbers are 8-bit counts (0..255), not volts or sound pressure.
 * MAX9814's approximately 1.25 V DC bias reads near 64 with 5 V AVCC.
 * MIN/MAX/P2P use about 98 ms windows. HOLD keeps the recent highest
 * P2P for approximately one second. SEQ shows the window timer runs;
 * COUNT > 0 indicates ADC conversions were read.
 *
 * Motors never drive: Timer1 is OFF, ENA/ENB and directions are LOW.
 * Servo pulses are OFF. Sonar TRIG stays LOW. No motion test occurs.
 * REMOVE the L298N EN jumpers when the MCU drives ENA/ENB, as usual.
 * ADC is polled without an interrupt; ADC polling also runs inside TWI
 * waits. This reduces, but cannot eliminate, display-related sample gaps.
 * COUNT is how many ADC conversions were actually read in that window.
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
#ifndef IR_CLIFF_LEVEL
#define IR_CLIFF_LEVEL 0u
#endif
#if (IR_CLIFF_LEVEL != 0) && (IR_CLIFF_LEVEL != 1)
#error "IR_CLIFF_LEVEL must be 0 or 1."
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <string.h>

#define SH1106_COLUMN_OFFSET 2u

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


/* Five columns per glyph, bit 0 at the top. Font stays in flash. */
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

static uint8_t draw_diagnostic_page(uint8_t page, const reading_t *frame,
                                    uint8_t left, uint8_t right, uint8_t ir)
{
    memset(page_buffer, 0, sizeof(page_buffer));
    text_x = 0u;
    switch (page) {
    case 0:
#if DIAG_ADC_ENABLED
        put_text("SENSOR DIAG ADC ON");
#else
        put_text("SENSOR DIAG ADC OFF");
#endif
        break;
    case 1:
        put_text("L:"); put_number(left, 1u);
        put_text(" R:"); put_number(right, 1u);
        put_text(" IR:"); put_number(ir, 1u);
        break;
    case 2:
        put_text("CLIFF:"); put_number(ir == IR_CLIFF_LEVEL, 1u);
        put_text(" RAW INPUT");
        break;
    case 3:
        put_text("ADC2:"); put_number(frame->latest, 3u);
        put_text(" 8BIT");
        break;
    case 4:
        put_text("MIN:"); put_number(frame->low, 3u);
        put_text(" MAX:"); put_number(frame->high, 3u);
        break;
    case 5:
        put_text("P2P:"); put_number(frame->p2p, 3u);
        put_text(" HOLD:"); put_number(frame->hold, 3u);
        break;
    case 6:
        put_text("COUNT:"); put_number(frame->count, 3u);
        put_text(" SEQ:"); put_number(frame->sequence, 3u);
        break;
    default:
        put_text("MOTORS AND SERVOS OFF");
        break;
    }
    return oled_set_position(page, 0u) &&
           oled_write_data(page_buffer, (uint8_t)sizeof(page_buffer));
}

int main(void)
{
    reading_t frame;
    uint8_t page = 0u, left = 0u, right = 0u, ir = 0u;
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
        if (!page) {
            frame = reading;
            left = (PINA & _BV(PA0)) ? 1u : 0u;
            right = (PINA & _BV(PA1)) ? 1u : 0u;
            ir = (PINB & _BV(PB0)) ? 1u : 0u;
        }
        if (!draw_diagnostic_page(page, &frame, left, right, ir)) {
            oled_ok = 0u;
            continue;
        }
        page = (uint8_t)((page + 1u) & 7u);
    }
}
