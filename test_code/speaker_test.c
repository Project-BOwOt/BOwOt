#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* =========================================================
   PIN CONFIGURATION
   ========================================================= */

#define OLED_SCL PC0
#define OLED_SDA PC1

#define OLED_ADDR 0x3C

#define MIC_ADC_CHANNEL 2       // PA2 = ADC2


/* =========================================================
   OLED FRAMEBUFFER
   SH1106 = 128 x 64
   ========================================================= */

uint8_t framebuffer[1024];


/* =========================================================
   I2C - SOFTWARE I2C
   SCL = PC0
   SDA = PC1
   ========================================================= */

void I2C_delay(void)
{
    _delay_us(5);
}

void I2C_init(void)
{
    DDRC |= (1 << OLED_SCL) | (1 << OLED_SDA);

    PORTC |= (1 << OLED_SCL) | (1 << OLED_SDA);
}

void I2C_start(void)
{
    PORTC |= (1 << OLED_SDA);
    PORTC |= (1 << OLED_SCL);

    I2C_delay();

    PORTC &= ~(1 << OLED_SDA);

    I2C_delay();

    PORTC &= ~(1 << OLED_SCL);
}

void I2C_stop(void)
{
    PORTC &= ~(1 << OLED_SDA);

    PORTC |= (1 << OLED_SCL);

    I2C_delay();

    PORTC |= (1 << OLED_SDA);

    I2C_delay();
}

void I2C_write(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 0x80)
            PORTC |= (1 << OLED_SDA);
        else
            PORTC &= ~(1 << OLED_SDA);

        PORTC |= (1 << OLED_SCL);
        I2C_delay();

        PORTC &= ~(1 << OLED_SCL);
        I2C_delay();

        data <<= 1;
    }

    /* Ignore ACK */
    PORTC |= (1 << OLED_SDA);

    PORTC |= (1 << OLED_SCL);
    I2C_delay();

    PORTC &= ~(1 << OLED_SCL);
}


/* =========================================================
   OLED LOW LEVEL
   ========================================================= */

void OLED_command(uint8_t cmd)
{
    I2C_start();

    I2C_write(OLED_ADDR << 1);
    I2C_write(0x00);
    I2C_write(cmd);

    I2C_stop();
}

void OLED_init(void)
{
    _delay_ms(100);

    OLED_command(0xAE);     // Display OFF

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

    OLED_command(0xAF);     // Display ON

    _delay_ms(100);
}


/* =========================================================
   FRAMEBUFFER FUNCTIONS
   ========================================================= */

void OLED_clear_buffer(void)
{
    for (uint16_t i = 0; i < 1024; i++)
        framebuffer[i] = 0;
}

void OLED_set_pixel(uint8_t x, uint8_t y)
{
    if (x >= 128 || y >= 64)
        return;

    framebuffer[(y / 8) * 128 + x] |= (1 << (y % 8));
}

void OLED_draw_pixel_scaled(uint8_t x,
                            uint8_t y,
                            uint8_t scale)
{
    for (uint8_t dx = 0; dx < scale; dx++)
    {
        for (uint8_t dy = 0; dy < scale; dy++)
        {
            OLED_set_pixel(x + dx, y + dy);
        }
    }
}


/* =========================================================
   FONT
   5x7 FONT

   Supported:
   0-9
   A-Z
   :
   space
   ========================================================= */

const uint8_t font_digits[10][5] =
{
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};


/* Return font for uppercase letter */
void get_letter_font(char c, uint8_t *f)
{
    /* Default = blank */

    for (uint8_t i = 0; i < 5; i++)
        f[i] = 0x00;

    switch (c)
    {
        case 'A':
            f[0]=0x7E; f[1]=0x09; f[2]=0x09;
            f[3]=0x09; f[4]=0x7E;
            break;

        case 'B':
            f[0]=0x7F; f[1]=0x49; f[2]=0x49;
            f[3]=0x49; f[4]=0x36;
            break;

        case 'C':
            f[0]=0x3E; f[1]=0x41; f[2]=0x41;
            f[3]=0x41; f[4]=0x22;
            break;

        case 'D':
            f[0]=0x7F; f[1]=0x41; f[2]=0x41;
            f[3]=0x22; f[4]=0x1C;
            break;

        case 'E':
            f[0]=0x7F; f[1]=0x49; f[2]=0x49;
            f[3]=0x49; f[4]=0x41;
            break;

        case 'F':
            f[0]=0x7F; f[1]=0x09; f[2]=0x09;
            f[3]=0x09; f[4]=0x01;
            break;

        case 'G':
            f[0]=0x3E; f[1]=0x41; f[2]=0x49;
            f[3]=0x49; f[4]=0x7A;
            break;

        case 'H':
            f[0]=0x7F; f[1]=0x08; f[2]=0x08;
            f[3]=0x08; f[4]=0x7F;
            break;

        case 'I':
            f[0]=0x00; f[1]=0x41; f[2]=0x7F;
            f[3]=0x41; f[4]=0x00;
            break;

        case 'J':
            f[0]=0x20; f[1]=0x40; f[2]=0x41;
            f[3]=0x3F; f[4]=0x01;
            break;

        case 'K':
            f[0]=0x7F; f[1]=0x08; f[2]=0x14;
            f[3]=0x22; f[4]=0x41;
            break;

        case 'L':
            f[0]=0x7F; f[1]=0x40; f[2]=0x40;
            f[3]=0x40; f[4]=0x40;
            break;

        case 'M':
            f[0]=0x7F; f[1]=0x02; f[2]=0x0C;
            f[3]=0x02; f[4]=0x7F;
            break;

        case 'N':
            f[0]=0x7F; f[1]=0x04; f[2]=0x08;
            f[3]=0x10; f[4]=0x7F;
            break;

        case 'O':
            f[0]=0x3E; f[1]=0x41; f[2]=0x41;
            f[3]=0x41; f[4]=0x3E;
            break;

        case 'P':
            f[0]=0x7F; f[1]=0x09; f[2]=0x09;
            f[3]=0x09; f[4]=0x06;
            break;

        case 'Q':
            f[0]=0x3E; f[1]=0x41; f[2]=0x51;
            f[3]=0x21; f[4]=0x5E;
            break;

        case 'R':
            f[0]=0x7F; f[1]=0x09; f[2]=0x19;
            f[3]=0x29; f[4]=0x46;
            break;

        case 'S':
            f[0]=0x46; f[1]=0x49; f[2]=0x49;
            f[3]=0x49; f[4]=0x31;
            break;

        case 'T':
            f[0]=0x01; f[1]=0x01; f[2]=0x7F;
            f[3]=0x01; f[4]=0x01;
            break;

        case 'U':
            f[0]=0x3F; f[1]=0x40; f[2]=0x40;
            f[3]=0x40; f[4]=0x3F;
            break;

        case 'V':
            f[0]=0x1F; f[1]=0x20; f[2]=0x40;
            f[3]=0x20; f[4]=0x1F;
            break;

        case 'W':
            f[0]=0x3F; f[1]=0x40; f[2]=0x38;
            f[3]=0x40; f[4]=0x3F;
            break;

        case 'X':
            f[0]=0x63; f[1]=0x14; f[2]=0x08;
            f[3]=0x14; f[4]=0x63;
            break;

        case 'Y':
            f[0]=0x07; f[1]=0x08; f[2]=0x70;
            f[3]=0x08; f[4]=0x07;
            break;

        case 'Z':
            f[0]=0x61; f[1]=0x51; f[2]=0x49;
            f[3]=0x45; f[4]=0x43;
            break;

        case ':':
            f[0]=0x00; f[1]=0x36; f[2]=0x36;
            f[3]=0x00; f[4]=0x00;
            break;

        case ' ':
            break;
    }
}


/* =========================================================
   DRAW BIG CHARACTER
   SCALE = 2

   Character size:
   10 x 14 pixels
   ========================================================= */

void OLED_char_big(char c, uint8_t x, uint8_t y)
{
    uint8_t f[5];

    if (c >= '0' && c <= '9')
    {
        for (uint8_t i = 0; i < 5; i++)
            f[i] = font_digits[c - '0'][i];
    }
    else
    {
        get_letter_font(c, f);
    }

    for (uint8_t col = 0; col < 5; col++)
    {
        for (uint8_t row = 0; row < 7; row++)
        {
            if (f[col] & (1 << row))
            {
                OLED_draw_pixel_scaled(
                    x + col * 2,
                    y + row * 2,
                    2
                );
            }
        }
    }
}


/* =========================================================
   DRAW BIG STRING
   ========================================================= */

void OLED_string_big(const char *str,
                     uint8_t x,
                     uint8_t y)
{
    while (*str)
    {
        OLED_char_big(*str, x, y);

        x += 12;

        if (x > 116)
            break;

        str++;
    }
}


/* =========================================================
   DRAW NUMBER BIG
   ========================================================= */

void OLED_number_big(uint16_t number,
                     uint8_t x,
                     uint8_t y)
{
    char digits[6];
    uint8_t count = 0;

    if (number == 0)
    {
        OLED_char_big('0', x, y);
        return;
    }

    while (number > 0)
    {
        digits[count++] = '0' + (number % 10);
        number /= 10;
    }

    for (int8_t i = count - 1; i >= 0; i--)
    {
        OLED_char_big(digits[i], x, y);
        x += 12;
    }
}


/* =========================================================
   SEND FRAMEBUFFER TO SH1106
   ========================================================= */

void OLED_update(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        OLED_command(0xB0 + page);

        // SH1106 has column offset
        OLED_command(0x02);
        OLED_command(0x10);

        I2C_start();

        I2C_write(OLED_ADDR << 1);
        I2C_write(0x40);

        for (uint8_t x = 0; x < 128; x++)
        {
            I2C_write(framebuffer[page * 128 + x]);
        }

        I2C_stop();
    }
}


/* =========================================================
   ADC
   ========================================================= */

void ADC_init(void)
{
    /*
     * AVCC = ADC reference
     * ADC2 = PA2
     */

    ADMUX = (1 << REFS0) | MIC_ADC_CHANNEL;

    /*
     * ADC enable
     * Prescaler = 8
     */

    ADCSRA = (1 << ADEN) |
             (1 << ADPS1) |
             (1 << ADPS0);
}

uint16_t ADC_read(void)
{
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
        ;

    return ADC;
}


/* =========================================================
   FIND MICROPHONE PEAK
   ========================================================= */

uint16_t microphone_peak(uint16_t center)
{
    uint16_t peak = 0;

    for (uint8_t i = 0; i < 100; i++)
    {
        uint16_t sample = ADC_read();

        int16_t difference =
            (int16_t)sample - (int16_t)center;

        if (difference < 0)
            difference = -difference;

        if ((uint16_t)difference > peak)
            peak = difference;
    }

    return peak;
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    /* Initialize OLED */
    I2C_init();
    OLED_init();

    /* Initialize ADC */
    ADC_init();

    _delay_ms(500);


    /*
     * First determine the DC center of MAX9814.
     *
     * MAX9814 output has a DC bias around ~1.25V.
     * With 5V ADC reference this is around ADC 256.
     */

    uint32_t sum = 0;

    for (uint8_t i = 0; i < 32; i++)
    {
        sum += ADC_read();
        _delay_ms(2);
    }

    uint16_t center = sum / 32;


    while (1)
    {
        /*
         * Read current ADC sample
         */

        uint16_t adc_value = ADC_read();


        /*
         * Measure microphone activity
         */

        uint16_t peak =
            microphone_peak(center);


        /* ==============================================
           BUILD OLED SCREEN
           ============================================== */

        OLED_clear_buffer();


        /*
         * Title
         */

        OLED_string_big("MIC", 8, 2);


        /*
         * ADC value
         */

        OLED_string_big("ADC:", 8, 20);

        OLED_number_big(
            adc_value,
            60,
            20
        );


        /*
         * Peak value
         */

        OLED_string_big("PEAK:", 8, 38);

        OLED_number_big(
            peak,
            68,
            38
        );


        /*
         * Sound detection
         */

        if (peak > 15)
        {
            OLED_string_big("YES", 72, 52);
        }
        else
        {
            OLED_string_big("NO", 84, 52);
        }


        /*
         * Send framebuffer to OLED
         */

        OLED_update();


        _delay_ms(100);
    }
}