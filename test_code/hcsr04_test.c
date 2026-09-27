#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

/* =========================================================
   OLED CONFIGURATION (SH1106 / SSD1306)
   PC0 = SCL
   PC1 = SDA
   Address = 0x3C
   ========================================================= */

#define OLED_ADDR 0x3C
#define WIDTH 128
#define HEIGHT 64

uint8_t framebuffer[128 * 8];


/* =========================================================
   I2C (TWI) FUNCTIONS
   ========================================================= */

void I2C_init(void)
{
    TWSR = 0x00;
    TWBR = 32; // 100 kHz bit rate at 8 MHz CPU
    TWCR = (1 << TWEN);
}

void I2C_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(10);
}

void I2C_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void OLED_command(uint8_t command)
{
    I2C_start();
    I2C_write(OLED_ADDR << 1);
    I2C_write(0x00);
    I2C_write(command);
    I2C_stop();
}

void OLED_init(void)
{
    _delay_ms(100);

    OLED_command(0xAE); // Display OFF
    OLED_command(0xD5); OLED_command(0x80);
    OLED_command(0xA8); OLED_command(0x3F);
    OLED_command(0xD3); OLED_command(0x00);
    OLED_command(0x40);
    OLED_command(0xAD); OLED_command(0x8B); // Enable SH1106 DC-DC charge pump
    OLED_command(0xA1);
    OLED_command(0xC8);
    OLED_command(0xDA); OLED_command(0x12);
    OLED_command(0x81); OLED_command(0x80);
    OLED_command(0xD9); OLED_command(0x1F);
    OLED_command(0xDB); OLED_command(0x40);
    OLED_command(0xA4);
    OLED_command(0xA6);
    OLED_command(0xAF); // Display ON
}


/* =========================================================
   FRAMEBUFFER & GRAPHICS
   ========================================================= */

void clear_buffer(void)
{
    memset(framebuffer, 0x00, sizeof(framebuffer));
}

void set_pixel(uint8_t x, uint8_t y)
{
    if (x >= 128 || y >= 64)
        return;

    framebuffer[x + (y / 8) * 128] |= (1 << (y % 8));
}

const uint8_t font5x7[][5] =
{
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}  // 9
};

void draw_digit(uint8_t digit, uint8_t x, uint8_t y)
{
    for (uint8_t col = 0; col < 5; col++)
    {
        uint8_t data = font5x7[digit][col];
        for (uint8_t row = 0; row < 7; row++)
        {
            if (data & (1 << row))
            {
                set_pixel(x + col, y + row);
            }
        }
    }
}

void draw_number(uint16_t number, uint8_t x, uint8_t y)
{
    uint8_t hundreds = number / 100;
    uint8_t tens = (number / 10) % 10;
    uint8_t ones = number % 10;

    if (hundreds > 0)
    {
        draw_digit(hundreds, x, y);
        draw_digit(tens, x + 7, y);
        draw_digit(ones, x + 14, y);
    }
    else if (tens > 0)
    {
        draw_digit(tens, x, y);
        draw_digit(ones, x + 7, y);
    }
    else
    {
        draw_digit(ones, x, y);
    }
}

void OLED_update(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        OLED_command(0xB0 + page);
        OLED_command(0x02); // SH1106 2-column offset
        OLED_command(0x10);

        I2C_start();
        I2C_write(OLED_ADDR << 1);
        I2C_write(0x40);

        for (uint8_t col = 0; col < 128; col++)
        {
            I2C_write(framebuffer[page * 128 + col]);
        }

        I2C_stop();
    }
}


/* =========================================================
   HC-SR04 ULTRASONIC SENSOR
   PB0 = TRIG
   PD6 = ECHO (Timer1 ICP1)
   ========================================================= */

void ultrasonic_init(void)
{
    DDRB |= (1 << PB0);   // PB0 as Output (TRIG)
    DDRD &= ~(1 << PD6);  // PD6 as Input (ECHO)
    PORTB &= ~(1 << PB0); // TRIG LOW

    TCCR1A = 0;
    TCCR1B = (1 << CS11); // Timer1 Prescaler = 8 (1 tick = 1 us at 8 MHz)
}

uint16_t ultrasonic_read(void)
{
    uint16_t timeout;
    uint16_t duration;

    PORTB &= ~(1 << PB0);
    _delay_us(2);

    // 10 us pulse to TRIG pin
    PORTB |= (1 << PB0);
    _delay_us(10);
    PORTB &= ~(1 << PB0);

    // Wait for ECHO pulse HIGH
    timeout = 0;
    while (!(PIND & (1 << PD6)))
    {
        _delay_us(1);
        timeout++;
        if (timeout > 30000) return 0; // Sensor timeout
    }

    TCNT1 = 0; // Reset Timer1

    // Wait for ECHO pulse LOW
    timeout = 0;
    while (PIND & (1 << PD6))
    {
        _delay_us(1);
        timeout++;
        if (timeout > 30000) return 0; // Sensor timeout
    }

    duration = TCNT1;
    return duration / 58; // Convert microseconds to centimeters
}


/* =========================================================
   MAIN LOOP
   ========================================================= */

int main(void)
{
    uint16_t distance;

    I2C_init();
    OLED_init();
    ultrasonic_init();

    while (1)
    {
        distance = ultrasonic_read();
        clear_buffer();

        if (distance == 0)
        {
            draw_number(0, 60, 28);
        }
        else
        {
            draw_number(distance, 55, 28);
        }

        OLED_update();
        _delay_ms(100);
    }

    return 0;
}