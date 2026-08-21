#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define OLED_ADDR 0x78

// PORTA pin allocation
#define IR_PIN           PA0    // IR module OUT, active LOW
#define LEFT_TOUCH_PIN   PA1    // left touch sensor, active HIGH
#define RIGHT_TOUCH_PIN  PA2    // right touch sensor, active HIGH
#define TRIG_PIN         PA3    // HC-SR04 TRIG
#define ECHO_PIN         PA4    // HC-SR04 ECHO

#define ULTRA_INVALID  0xFFFFu
#define ULTRA_LIMIT_CM 35u

// Gaze target for the movable eyeballs.
#define GAZE_X_SHIFT 6
#define GAZE_Y_SHIFT 4

typedef enum {
    EXPR_NORMAL = 0,
    EXPR_SCARED,
    EXPR_HAPPY,
    EXPR_LOOK_UP_LEFT,
    EXPR_LOOK_UP_RIGHT
} ExpressionState;

// ============================================================
// I2C
// ============================================================
static void I2C_Init(void) {
    TWSR = 0x00;
    // ATmega32 is running at 1 MHz (LFUSE E1 from your programmer log).
    // With prescaler 1 and TWBR=2, SCL is about 50 kHz.
    TWBR = 2;
    TWCR = (1 << TWEN);
}

static void I2C_Start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { }
}

static void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { }
}

static void I2C_Stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

// ============================================================
// OLED
// ============================================================
static void OLED_Command(uint8_t cmd) {
    I2C_Start();
    I2C_Write(OLED_ADDR);
    I2C_Write(0x00);
    I2C_Write(cmd);
    I2C_Stop();
}

static void OLED_Data(uint8_t data) {
    I2C_Start();
    I2C_Write(OLED_ADDR);
    I2C_Write(0x40);
    I2C_Write(data);
    I2C_Stop();
}

// Send many display bytes in one I2C transaction. This is dramatically
// faster than calling OLED_Data() once for every byte.
static void OLED_WriteBlock(const uint8_t *data, uint8_t length) {
    I2C_Start();
    I2C_Write(OLED_ADDR);
    I2C_Write(0x40);

    for (uint8_t i = 0; i < length; i++) {
        I2C_Write(data[i]);
    }

    I2C_Stop();
}

static void OLED_FillBlock(uint8_t value, uint8_t length) {
    I2C_Start();
    I2C_Write(OLED_ADDR);
    I2C_Write(0x40);

    for (uint8_t i = 0; i < length; i++) {
        I2C_Write(value);
    }

    I2C_Stop();
}

static void OLED_Init(void) {
    OLED_Command(0xAE);                 // display off
    OLED_Command(0x20); OLED_Command(0x00); // horizontal addressing
    OLED_Command(0x8D); OLED_Command(0x14); // charge pump on
    OLED_Command(0xAF);                 // display on
}

static void OLED_SetCursor(uint8_t page, uint8_t col) {
    OLED_Command((uint8_t)(0xB0u + page));
    OLED_Command((uint8_t)(0x00u + (col & 0x0Fu)));
    OLED_Command((uint8_t)(0x10u + ((col >> 4) & 0x0Fu)));
}

// Gaze animation can move the eye upward into page 1, so clear pages 1..4.
static void ClearEyeRegion(void) {
    for (uint8_t page = 1; page <= 4; page++) {
        OLED_SetCursor(page, 0);
        OLED_FillBlock(0x00, 128u);
    }
}

// ============================================================
// EYE BITMAPS
// ============================================================
const uint8_t eye_normal_top[32] = {
    0x00,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFE,0xFC,0xF8,0xF0,0xE0,0xC0,0x00,0x00,0x00
};

const uint8_t eye_normal_mid[32] = {
    0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00
};

const uint8_t eye_normal_bot[32] = {
    0x00,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0x3F,0x1F,0x0F,0x07,0x03,0x00,0x00,0x00
};

const uint8_t eye_scared_top[32] = {
    0x00,0x0F,0x30,0x40,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x40,0x30,0x0F,0x00,0x00
};

const uint8_t eye_scared_mid[32] = {
    0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x3C,0x3C,0x3C,
    0x3C,0x3C,0x3C,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00
};

const uint8_t eye_scared_bot[32] = {
    0x00,0xF0,0x0C,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
    0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x0C,0xF0,0x00,0x00
};

const uint8_t eye_happy_top[32] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x06,0x06,
    0x06,0x06,0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

const uint8_t eye_happy_mid[32] = {
    0x00,0x00,0xC0,0xF0,0x3C,0x0F,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x0F,0x3C,0xF0,0xC0,0x00,0x00,0x00
};

const uint8_t eye_happy_bot[32] = {
    0x00,0x03,0x0F,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x0F,0x03,0x00,0x00
};

// ============================================================
// EYE DRAWING
// ============================================================
static void DrawEyePair(const uint8_t *top, const uint8_t *mid, const uint8_t *bot) {
    const uint8_t left_x = 20;
    const uint8_t right_x = 76;

    OLED_SetCursor(2, left_x);
    OLED_WriteBlock(top, 32u);
    OLED_SetCursor(2, right_x);
    OLED_WriteBlock(top, 32u);

    OLED_SetCursor(3, left_x);
    OLED_WriteBlock(mid, 32u);
    OLED_SetCursor(3, right_x);
    OLED_WriteBlock(mid, 32u);

    OLED_SetCursor(4, left_x);
    OLED_WriteBlock(bot, 32u);
    OLED_SetCursor(4, right_x);
    OLED_WriteBlock(bot, 32u);
}

/*
 * Smooth gaze renderer
 * --------------------
 * The previous version cleared the OLED first and then redrew the eyes at the
 * destination. On a real SSD1306 over I2C that looks like "eyes disappear,
 * then reappear".
 *
 * Here we build the COMPLETE next frame in RAM and then overwrite the gaze
 * area directly. There is never an intentional all-black frame.
 *
 * The whole normal eye/eyeball shapes move a few pixels diagonally. No pupils
 * are added. Because we animate through intermediate offsets, the motion looks
 * like the eyeballs actually glance toward the touched side.
 */
static uint8_t gaze_buf[4][128];       // OLED pages 1..4 only (512 bytes)
static int8_t gaze_x = 0;              // current horizontal offset
static uint8_t gaze_y = 0;             // current upward offset
static uint8_t gaze_active = 0;

static uint8_t NormalEyeByte(uint8_t row, uint8_t col) {
    if (row == 0u) return eye_normal_top[col];
    if (row == 1u) return eye_normal_mid[col];
    return eye_normal_bot[col];
}

static void BuildGazeFrame(int8_t x_shift, uint8_t y_up) {
    // Clear only the RAM frame, not the physical OLED.
    for (uint8_t p = 0; p < 4u; p++) {
        for (uint8_t x = 0; x < 128u; x++) {
            gaze_buf[p][x] = 0x00;
        }
    }

    const int16_t bases[2] = {20, 76};

    // Convert each set source pixel of the original 32x24 eye into its shifted
    // destination position. This handles arbitrary 1-pixel vertical motion,
    // not just whole 8-pixel OLED pages.
    for (uint8_t eye = 0; eye < 2u; eye++) {
        const int16_t base_x = bases[eye] + x_shift;

        for (uint8_t src_row = 0; src_row < 3u; src_row++) {
            for (uint8_t col = 0; col < 32u; col++) {
                const uint8_t b = NormalEyeByte(src_row, col);
                if (b == 0u) continue;

                const int16_t dst_x = base_x + col;
                if (dst_x < 0 || dst_x >= 128) continue;

                for (uint8_t bit = 0; bit < 8u; bit++) {
                    if (!(b & (1u << bit))) continue;

                    // Original normal eye occupies y=16..39 (pages 2..4).
                    const int16_t src_y = 16 + ((int16_t)src_row * 8) + bit;
                    const int16_t dst_y = src_y - y_up;

                    // Our buffer represents physical OLED pages 1..4 => y=8..39.
                    if (dst_y < 8 || dst_y > 39) continue;

                    const uint8_t buf_page = (uint8_t)((dst_y >> 3) - 1);
                    const uint8_t dst_bit = (uint8_t)(dst_y & 7);
                    gaze_buf[buf_page][dst_x] |= (uint8_t)(1u << dst_bit);
                }
            }
        }
    }
}

static void PushGazeFrame(void) {
    // One I2C burst per 128-byte page. Avoids hundreds of START/STOP cycles
    // per animation frame and is the main smoothness improvement.
    for (uint8_t p = 0; p < 4u; p++) {
        OLED_SetCursor((uint8_t)(p + 1u), 0);
        OLED_WriteBlock(gaze_buf[p], 128u);
    }
}

static void DrawGazeFrame(int8_t x_shift, uint8_t y_up) {
    BuildGazeFrame(x_shift, y_up);
    PushGazeFrame();
}

static void AnimateGazeTo(int8_t target_x, uint8_t target_y) {
    // If another expression was on screen, start the gaze from centered eyes.
    if (!gaze_active) {
        gaze_x = 0;
        gaze_y = 0;
        DrawGazeFrame(gaze_x, gaze_y);
        gaze_active = 1u;
    }

    // Move up to 2 pixels per rendered frame. With a 6x4 target this normally
    // needs only three frames, which looks much smoother on a 1 MHz AVR.
    while (gaze_x != target_x || gaze_y != target_y) {
        if (gaze_x < target_x) {
            gaze_x += 2;
            if (gaze_x > target_x) gaze_x = target_x;
        } else if (gaze_x > target_x) {
            gaze_x -= 2;
            if (gaze_x < target_x) gaze_x = target_x;
        }

        if (gaze_y < target_y) {
            gaze_y = (uint8_t)(gaze_y + 2u);
            if (gaze_y > target_y) gaze_y = target_y;
        } else if (gaze_y > target_y) {
            if (gaze_y >= 2u) gaze_y = (uint8_t)(gaze_y - 2u);
            else gaze_y = 0u;
            if (gaze_y < target_y) gaze_y = target_y;
        }

        DrawGazeFrame(gaze_x, gaze_y);
    }
}

static void RenderExpression(ExpressionState state) {
    switch (state) {
        case EXPR_LOOK_UP_LEFT:
            // Move both complete eyeballs smoothly toward upper-left.
            AnimateGazeTo(-GAZE_X_SHIFT, GAZE_Y_SHIFT);
            break;

        case EXPR_LOOK_UP_RIGHT:
            // Move both complete eyeballs smoothly toward upper-right.
            AnimateGazeTo(GAZE_X_SHIFT, GAZE_Y_SHIFT);
            break;

        case EXPR_NORMAL:
            // If we were looking somewhere, visibly return the eyeballs to center.
            if (gaze_active) {
                AnimateGazeTo(0, 0u);
            } else {
                ClearEyeRegion();
                DrawEyePair(eye_normal_top, eye_normal_mid, eye_normal_bot);
            }
            gaze_active = 0u;
            break;

        case EXPR_HAPPY:
            gaze_active = 0u;
            ClearEyeRegion();
            DrawEyePair(eye_happy_top, eye_happy_mid, eye_happy_bot);
            break;

        case EXPR_SCARED:
            gaze_active = 0u;
            ClearEyeRegion();
            DrawEyePair(eye_scared_top, eye_scared_mid, eye_scared_bot);
            break;

        default:
            gaze_active = 0u;
            ClearEyeRegion();
            DrawEyePair(eye_normal_top, eye_normal_mid, eye_normal_bot);
            break;
    }
}

// ============================================================
// HC-SR04 ULTRASONIC
// ============================================================
// With the current 1 MHz CPU clock, Timer1 with no prescaler ticks once/us.
static void Ultrasonic_Init(void) {
    DDRA |= (1 << TRIG_PIN);        // TRIG output
    DDRA &= ~(1 << ECHO_PIN);       // ECHO input
    PORTA &= ~(1 << TRIG_PIN);      // idle low
    PORTA &= ~(1 << ECHO_PIN);      // no internal pull-up

    TCCR1A = 0x00;
    TCCR1B = (1 << CS10);           // F_CPU/1 = 1 MHz -> 1 us/tick
    TCNT1 = 0;
}

static uint16_t Measure_Distance_cm(void) {
    // Clean >=10 us trigger pulse.
    PORTA &= ~(1 << TRIG_PIN);
    _delay_us(4);
    PORTA |= (1 << TRIG_PIN);
    _delay_us(12);
    PORTA &= ~(1 << TRIG_PIN);

    // Wait for ECHO to rise. Timeout prevents a disconnected sensor hanging MCU.
    TCNT1 = 0;
    while (!(PINA & (1 << ECHO_PIN))) {
        if (TCNT1 >= 30000u) return ULTRA_INVALID;
    }

    // Measure ECHO high pulse in microseconds.
    TCNT1 = 0;
    while (PINA & (1 << ECHO_PIN)) {
        if (TCNT1 >= 30000u) return ULTRA_INVALID;
    }

    const uint16_t pulse_us = TCNT1;
    if (pulse_us < 116u) return ULTRA_INVALID; // HC-SR04 unreliable below ~2 cm

    return (uint16_t)((pulse_us + 29u) / 58u); // rounded distance in cm
}

// ============================================================
// MAIN
// ============================================================
int main(void) {
    // PA0/PA1/PA2/PA4 inputs, PA3 output.
    DDRA &= ~((1 << IR_PIN) |
              (1 << LEFT_TOUCH_PIN) |
              (1 << RIGHT_TOUCH_PIN) |
              (1 << ECHO_PIN));
    DDRA |= (1 << TRIG_PIN);

    // Modules actively drive their output pins, so disable AVR pull-ups.
    PORTA &= ~((1 << IR_PIN) |
               (1 << LEFT_TOUCH_PIN) |
               (1 << RIGHT_TOUCH_PIN) |
               (1 << ECHO_PIN) |
               (1 << TRIG_PIN));

    I2C_Init();
    Ultrasonic_Init();
    _delay_ms(50);
    OLED_Init();

    ExpressionState last_expr = EXPR_NORMAL;
    RenderExpression(last_expr);

    // HC-SR04 gets pinged about every 60 ms; IR/touch are checked every loop.
    uint8_t ultra_tick = 0;
    uint8_t ultrasonic_detected = 0;

    while (1) {
        // Preserve the exact IR polarity that works on your robot.
        const uint8_t ir_detected =
            (PINA & (1 << IR_PIN)) ? 0u : 1u;

        const uint8_t left_touch =
            (PINA & (1 << LEFT_TOUCH_PIN)) ? 1u : 0u;

        const uint8_t right_touch =
            (PINA & (1 << RIGHT_TOUCH_PIN)) ? 1u : 0u;

        // Roughly 30 * 2 ms = 60 ms between ultrasonic triggers.
        if (++ultra_tick >= 30u) {
            ultra_tick = 0;
            const uint16_t dist_cm = Measure_Distance_cm();
            ultrasonic_detected =
                (dist_cm != ULTRA_INVALID && dist_cm <= ULTRA_LIMIT_CM) ? 1u : 0u;
        }

        ExpressionState current_expr;

        // Touch direction has highest priority so the gaze is always visible.
        if (left_touch && !right_touch) {
            current_expr = EXPR_LOOK_UP_LEFT;
        } else if (right_touch && !left_touch) {
            current_expr = EXPR_LOOK_UP_RIGHT;
        } else if (left_touch && right_touch) {
            // Both cheeks/sides touched together -> happy face.
            current_expr = EXPR_HAPPY;
        } else if (ir_detected || ultrasonic_detected) {
            current_expr = EXPR_SCARED;
        } else {
            current_expr = EXPR_NORMAL;
        }

        if (current_expr != last_expr) {
            RenderExpression(current_expr);
            last_expr = current_expr;
        }

        _delay_ms(2);
    }
}
