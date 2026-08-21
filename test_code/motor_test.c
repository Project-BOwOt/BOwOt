/*
 * TB6612FNG + ATmega32 + 2x N20 Bench Test
 *
 * ATmega32:
 *   PA0 -> AIN1
 *   PA1 -> AIN2
 *   PA2 -> BIN1
 *   PA3 -> BIN2
 *   PA4 -> STBY
 *
 *   PB3 / OC0 -> PWMA
 *   PD7 / OC2 -> PWMB
 *
 * Motor A -> AO1, AO2
 * Motor B -> BO1, BO2
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

/* -----------------------------
   Direction pin definitions
   ----------------------------- */

#define AIN1 PA0
#define AIN2 PA1

#define BIN1 PA2
#define BIN2 PA3

#define STBY PA4


/* =============================
   PWM INITIALIZATION
   ============================= */

void pwm_init(void)
{
    /*
     * OC0 = PB3
     * Timer0 -> Motor A
     *
     * Fast PWM
     * Non-inverting mode
     * Prescaler = 8
     */

    DDRB |= (1 << PB3);

    TCCR0 =
        (1 << WGM00) |
        (1 << WGM01) |
        (1 << COM01) |
        (1 << CS01);

    OCR0 = 0;


    /*
     * OC2 = PD7
     * Timer2 -> Motor B
     *
     * Fast PWM
     * Non-inverting mode
     * Prescaler = 8
     */

    DDRD |= (1 << PD7);

    TCCR2 =
        (1 << WGM20) |
        (1 << WGM21) |
        (1 << COM21) |
        (1 << CS21);

    OCR2 = 0;
}


/* =============================
   MOTOR GPIO INITIALIZATION
   ============================= */

void motor_gpio_init(void)
{
    /*
     * PA0 = AIN1
     * PA1 = AIN2
     * PA2 = BIN1
     * PA3 = BIN2
     * PA4 = STBY
     */

    DDRA |=
        (1 << AIN1) |
        (1 << AIN2) |
        (1 << BIN1) |
        (1 << BIN2) |
        (1 << STBY);

    /*
     * Start with all control pins LOW
     */
    PORTA &=
        ~(
            (1 << AIN1) |
            (1 << AIN2) |
            (1 << BIN1) |
            (1 << BIN2) |
            (1 << STBY)
        );
}


/* =============================
   DRIVER ENABLE / DISABLE
   ============================= */

void driver_enable(void)
{
    PORTA |= (1 << STBY);
}

void driver_disable(void)
{
    PORTA &= ~(1 << STBY);
}


/* =============================
   MOTOR A
   ============================= */

void motorA_forward(uint8_t speed)
{
    PORTA |=  (1 << AIN1);
    PORTA &= ~(1 << AIN2);

    OCR0 = speed;
}

void motorA_reverse(uint8_t speed)
{
    PORTA &= ~(1 << AIN1);
    PORTA |=  (1 << AIN2);

    OCR0 = speed;
}

void motorA_stop(void)
{
    OCR0 = 0;

    PORTA &= ~((1 << AIN1) | (1 << AIN2));
}

void motorA_brake(void)
{
    OCR0 = 255;

    PORTA |= (1 << AIN1) | (1 << AIN2);
}


/* =============================
   MOTOR B
   ============================= */

void motorB_forward(uint8_t speed)
{
    PORTA |=  (1 << BIN1);
    PORTA &= ~(1 << BIN2);

    OCR2 = speed;
}

void motorB_reverse(uint8_t speed)
{
    PORTA &= ~(1 << BIN1);
    PORTA |=  (1 << BIN2);

    OCR2 = speed;
}

void motorB_stop(void)
{
    OCR2 = 0;

    PORTA &= ~((1 << BIN1) | (1 << BIN2));
}

void motorB_brake(void)
{
    OCR2 = 255;

    PORTA |= (1 << BIN1) | (1 << BIN2);
}


/* =============================
   BOTH MOTORS
   ============================= */

void motors_forward(uint8_t speed)
{
    motorA_forward(speed);
    motorB_forward(speed);
}

void motors_reverse(uint8_t speed)
{
    motorA_reverse(speed);
    motorB_reverse(speed);
}

void motors_stop(void)
{
    motorA_stop();
    motorB_stop();
}


/* =============================
   MAIN TEST
   ============================= */

int main(void)
{
    motor_gpio_init();
    pwm_init();

    /*
     * TB6612 starts in standby.
     */
    _delay_ms(500);

    driver_enable();

    while (1)
    {
        /*
         * TEST 1:
         * Both motors forward at ~40%
         */
        motors_forward(100);
        _delay_ms(2000);

        motors_stop();
        _delay_ms(1000);


        /*
         * TEST 2:
         * Both motors forward at ~70%
         */
        motors_forward(180);
        _delay_ms(2000);

        motors_stop();
        _delay_ms(1000);


        /*
         * TEST 3:
         * Both motors reverse
         */
        motors_reverse(150);
        _delay_ms(2000);

        motors_stop();
        _delay_ms(1000);


        /*
         * TEST 4:
         * Only Motor A
         */
        motorA_forward(150);
        motorB_stop();

        _delay_ms(2000);

        motors_stop();
        _delay_ms(1000);


        /*
         * TEST 5:
         * Only Motor B
         */
        motorA_stop();
        motorB_forward(150);

        _delay_ms(2000);

        motors_stop();
        _delay_ms(1000);


        /*
         * TEST 6:
         * Differential-drive spin
         *
         * A forward
         * B reverse
         */
        motorA_forward(150);
        motorB_reverse(150);

        _delay_ms(2000);

        motors_stop();
        _delay_ms(2000);
    }
}