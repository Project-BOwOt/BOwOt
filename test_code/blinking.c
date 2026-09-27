#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

#define OLED_ADDR 0x3C

#define WIDTH  128
#define HEIGHT 64

uint8_t framebuffer[128 * 8];


/* =========================
   I2C INITIALIZATION
   PC0 = SCL
   PC1 = SDA
   ========================= */

void I2C_init(void)
{
    TWSR = 0x00;

    // 100 kHz at 8 MHz CPU
    TWBR = 32;

    TWCR = (1 << TWEN);
}


/* =========================
   I2C START
   ========================= */

void I2C_start(void)
{
    TWCR = (1 << TWINT) |
           (1 << TWSTA) |
           (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));
}


/* =========================
   I2C STOP
   ========================= */

void I2C_stop(void)
{
    TWCR = (1 << TWINT) |
           (1 << TWSTO) |
           (1 << TWEN);

    _delay_us(10);
}


/* =========================
   I2C WRITE
   ========================= */

void I2C_write(uint8_t data)
{
    TWDR = data;

    TWCR = (1 << TWINT) |
           (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));
}


/* =========================
   OLED COMMAND
   ========================= */

void OLED_command(uint8_t command)
{
    I2C_start();

    I2C_write(OLED_ADDR << 1);

    // Command
    I2C_write(0x00);

    I2C_write(command);

    I2C_stop();
}


/* =========================
   OLED INIT
   ========================= */

void OLED_init(void)
{
    _delay_ms(100);

    OLED_command(0xAE); // Display OFF

    OLED_command(0xD5);
    OLED_command(0x80);

    OLED_command(0xA8);
    OLED_command(0x3F);

    OLED_command(0xD3);
    OLED_command(0x00);

    OLED_command(0x40);

    OLED_command(0xAD);
    OLED_command(0x8B);

    OLED_command(0xA1);

    OLED_command(0xC8);

    OLED_command(0xDA);
    OLED_command(0x12);

    OLED_command(0x81);
    OLED_command(0x80);

    OLED_command(0xD9);
    OLED_command(0x1F);

    OLED_command(0xDB);
    OLED_command(0x40);

    OLED_command(0xA4);

    OLED_command(0xA6);

    OLED_command(0xAF); // Display ON
}


/* =========================
   CLEAR BUFFER
   ========================= */

void clear_buffer(void)
{
    memset(framebuffer, 0x00, sizeof(framebuffer));
}


/* =========================
   SET PIXEL
   ========================= */

void set_pixel(uint8_t x, uint8_t y)
{
    if (x >= 128 || y >= 64)
        return;

    framebuffer[x + (y / 8) * 128]
        |= (1 << (y % 8));
}


/* =========================
   LINE
   ========================= */

void draw_line(int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0;
    int dy = y1 - y0;

    int sx = (dx >= 0) ? 1 : -1;
    int sy = (dy >= 0) ? 1 : -1;

    if (dx < 0)
        dx = -dx;

    if (dy < 0)
        dy = -dy;

    int err = dx - dy;

    while (1)
    {
        set_pixel(x0, y0);

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}


/* =========================
   THICK LINE
   ========================= */

void draw_thick_line(int x0, int y0, int x1, int y1)
{
    draw_line(x0, y0, x1, y1);
    draw_line(x0, y0 + 1, x1, y1 + 1);
    draw_line(x0, y0 - 1, x1, y1 - 1);
}


/* =========================
   OPEN EYE
   ========================= */

void draw_open_eye(void)
{
    clear_buffer();

    // Upper eyelid
    draw_thick_line(25, 32, 40, 20);
    draw_thick_line(40, 20, 64, 16);
    draw_thick_line(64, 16, 88, 20);
    draw_thick_line(88, 20, 103, 32);

    // Lower eyelid
    draw_thick_line(25, 32, 40, 44);
    draw_thick_line(40, 44, 64, 48);
    draw_thick_line(64, 48, 88, 44);
    draw_thick_line(88, 44, 103, 32);

    // Pupil
    for (uint8_t y = 25; y <= 39; y++)
    {
        for (uint8_t x = 57; x <= 71; x++)
        {
            int dx = x - 64;
            int dy = y - 32;

            if ((dx * dx + dy * dy) <= 49)
            {
                set_pixel(x, y);
            }
        }
    }
}


/* =========================
   CLOSED EYE
   ========================= */

void draw_closed_eye(void)
{
    clear_buffer();

    draw_thick_line(25, 32, 40, 29);
    draw_thick_line(40, 29, 64, 28);
    draw_thick_line(64, 28, 88, 29);
    draw_thick_line(88, 29, 103, 32);

    // Eyelashes
    draw_line(35, 30, 30, 26);
    draw_line(93, 30, 98, 26);
}


/* =========================
   UPDATE OLED
   ========================= */

void OLED_update(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0; page < 8; page++)
    {
        OLED_command(0xB0 + page);

        // SH1106 column offset
        OLED_command(0x02);
        OLED_command(0x10);

        I2C_start();

        I2C_write(OLED_ADDR << 1);

        // Display data
        I2C_write(0x40);

        for (col = 0; col < 128; col++)
        {
            I2C_write(framebuffer[page * 128 + col]);
        }

        I2C_stop();
    }
}


/* =========================
   MAIN
   ========================= */

int main(void)
{
    I2C_init();

    OLED_init();

    while (1)
    {
        // OPEN EYE
        draw_open_eye();
        OLED_update();

        _delay_ms(1500);

        // CLOSED EYE
        draw_closed_eye();
        OLED_update();

        _delay_ms(150);
    }

    return 0;
}