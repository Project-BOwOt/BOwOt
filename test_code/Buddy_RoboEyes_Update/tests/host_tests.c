/*
 * Native host regression harness: uses production C directly, with main
 * renamed and peripheral registers represented by volatile host variables.
 * This checks software logic only; it is NOT an AVR compiler or simulator.
 *
 * Build from firmware_roboeyes:
 * gcc -std=c11 -O2 -Wall -Wextra -Werror -I tests/shim \
 *     tests/host_tests.c -o tests/host_tests
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define main robot_firmware_main
#include "../robot_sh1106.c"
#undef main

static unsigned checks;
#define CHECK(expression) do { \
    ++checks; \
    if (!(expression)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); \
        exit(1); \
    } \
} while (0)

static void set_ir_cliff(uint8_t cliff)
{
    uint8_t high = cliff ? IR_CLIFF_LEVEL : !IR_CLIFF_LEVEL;
    if (high) PINB |= _BV(PB0);
    else PINB &= (uint8_t)~_BV(PB0);
}

static void reset_safe(uint16_t now)
{
    DDRA = DDRB = DDRC = DDRD = 0;
    PORTA = PORTB = PORTC = PORTD = 0;
    PINA = PINB = PINC = PIND = 0;
    set_ir_cliff(0u);
    TCCR0 = TCCR1A = TCCR1B = TCCR2 = 0;
    TCNT0 = TCNT2 = OCR2 = TIFR = TIMSK = ASSR = 0;
    TCNT1 = OCR1A = OCR1B = ICR1 = 0;
    TWCR = TWSR = TWBR = TWDR = 0;
    ADMUX = ADCSRA = ADCH = ADCL = SFIOR = 0;
    SREG = _BV(SREG_I);
    cliff_latched = 0;
    surface_stable_ticks = IR_SURFACE_STABLE_TICKS;
    system_ticks_2ms = now;
    servo_target_ticks = SERVO_HOME_TICKS;
    servo_frame_count = 9;
    servo_pulse_active = 0;
    sonar_status = SONAR_CLEAR;
    sonar_obstacle = 0;
    sonar_clear_count = CLEAR_READINGS_REQUIRED;
    sonar_last_sample_tick = now;
    sonar_last_trigger_tick = now;
    random_state = 0xACE1u;
    motor_target_a = motor_target_b = 0;
    motor_current_a = motor_current_b = 0;
    motor_arm_since = motor_last_ramp = now;
    microphone_init(now);
    face_mode = FACE_WAKE;
    face_initialized = 0;
    face_since = now;
    eye_rng = 0xB47Du;
    face_turn_left = 0;
    face_touch = TOUCH_NONE;
    curious_x = 4;
    memset(eye_pose, 0, sizeof(eye_pose));
    frame_smile = frame_heart_y = frame_alert_mark = frame_sparkle = 0;
    eye_page_index = 0;
    blink_active = blink_min_shown = blink_double_pending = 0;
    blink_since = blink_wait_since = now;
    blink_wait_ticks = 1200u;
    oled_ok = 0;
    memset(eye_frame_buffer, 0, sizeof(eye_frame_buffer));
    motors_init();
    robot_begin(now);
}

static void check_braked(void)
{
    CHECK(motor_hw_mode == MOTOR_BRAKED);
    CHECK((PORTD & MOTOR_DIRECTION_MASK) == 0);
    CHECK((PORTD & MOTOR_ENABLE_MASK) == MOTOR_ENABLE_MASK);
    CHECK(TCCR1A == _BV(WGM10));
}

static void fresh_update(uint16_t now, touch_state_t touch)
{
    sonar_last_sample_tick = now;
    robot_update(now, touch);
    motors_service(now);
}

static void tick_irq(unsigned count)
{
    while (count--) {
        TIMER2_OVF_vect();
        TIMER2_COMP_vect();
    }
}

static void start_cruise(uint16_t now)
{
    reset_safe(now);
    robot_enter(STATE_CRUISE, now);
    motors_service(now);
    motors_service((uint16_t)(now + MOTOR_PWM_ARM_TICKS));
    CHECK(motor_hw_mode == MOTOR_RUNNING);
}

static void test_motor_resume_and_ramp(void)
{
    uint16_t now = 100;
    unsigned i;
    reset_safe(now);
    PORTD |= _BV(PD0) | _BV(PD7); /* Unrelated pins must survive motor writes. */
    motors_set_targets(MOTOR_SPEED, MOTOR_SPEED);
    motors_service(now);
    CHECK(motor_hw_mode == MOTOR_ARMING);
    CHECK((PORTD & MOTOR_DIRECTION_MASK) == 0);
    CHECK(motor_current_a == MOTOR_RAMP_STEP);
    CHECK(motor_current_b == MOTOR_RAMP_STEP);
    CHECK(OCR1A == MOTOR_RAMP_STEP && OCR1B == MOTOR_RAMP_STEP);
    motors_service((uint16_t)(now + MOTOR_PWM_ARM_TICKS - 1));
    CHECK(motor_hw_mode == MOTOR_ARMING);
    motors_service((uint16_t)(now + MOTOR_PWM_ARM_TICKS));
    CHECK(motor_hw_mode == MOTOR_RUNNING);
    CHECK((PORTD & MOTOR_DIRECTION_MASK) == (_BV(PD1) | _BV(PD2)));
    CHECK((PORTD & (_BV(PD0) | _BV(PD7))) == (_BV(PD0) | _BV(PD7)));
    now = (uint16_t)(now + MOTOR_PWM_ARM_TICKS);
    motors_service((uint16_t)(now + MOTOR_RAMP_INTERVAL_TICKS - 1));
    CHECK(motor_current_a == MOTOR_RAMP_STEP);
    for (i = 0; i < 40; ++i) {
        uint8_t previous = motor_current_a;
        now = (uint16_t)(now + MOTOR_RAMP_INTERVAL_TICKS);
        motors_service(now);
        CHECK(motor_current_a <= MOTOR_SPEED);
        CHECK(motor_current_a >= previous);
        CHECK((unsigned)(motor_current_a - previous) <= MOTOR_RAMP_STEP);
    }
    CHECK(motor_current_a == MOTOR_SPEED && motor_current_b == MOTOR_SPEED);
    motors_set_targets(40, 30);
    motors_service((uint16_t)(now + MOTOR_RAMP_INTERVAL_TICKS));
    CHECK(motor_current_a == 40 && motor_current_b == 30);
    motors_set_targets(0, 0);
    check_braked();
    CHECK(motor_current_a == 0 && motor_current_b == 0);
}

static void test_cliff_irq_and_stale_command(void)
{
    start_cruise(100);
    set_ir_cliff(1u);
    TIMER2_OVF_vect();
    CHECK(cliff_latched == 1);
    CHECK(surface_stable_ticks == 0);
    check_braked();
    /* A stale foreground cruise target must never release the ISR brake. */
    motors_set_targets(255, 255);
    motors_service(103);
    check_braked();
    set_ir_cliff(0u);
    motors_service(104);
    check_braked();
    /* Even before another interrupt, direct pin sampling blocks a restart. */
    cliff_latched = 0;
    set_ir_cliff(1u);
    motors_service(105);
    CHECK(cliff_latched == 1);
    check_braked();
    CHECK((SREG & _BV(SREG_I)) != 0);
}

static void test_cliff_interrupts_arming(void)
{
    reset_safe(200);
    motors_set_targets(120, 120);
    motors_service(200);
    CHECK(motor_hw_mode == MOTOR_ARMING);
    set_ir_cliff(1u);
    TIMER2_OVF_vect();
    check_braked();
    motors_service(203);
    check_braked();
}

static void test_priority_across_states(void)
{
    robot_state_t state;
    for (state = STATE_STARTUP; state <= STATE_SENSOR_WAIT; ++state) {
        reset_safe(1000);
        robot_enter(state, 1000);
        sonar_accept_sample(SONAR_NEAR, 1000);
        cliff_latched = 1;
        surface_stable_ticks = 0;
        set_ir_cliff(1u);
        robot_update(1000, TOUCH_BOTH);
        CHECK(robot_state == STATE_CLIFF);
        check_braked();
    }
    reset_safe(1000);
    robot_enter(STATE_TURN, 1000);
    sonar_accept_sample(SONAR_NEAR, 1000);
    robot_update(1001, TOUCH_LEFT);
    CHECK(robot_state == STATE_PETTING);
    check_braked();
    sonar_accept_sample(SONAR_FAULT, 1002);
    robot_update(1002, TOUCH_LEFT);
    CHECK(robot_state == STATE_PETTING);
    check_braked();
    robot_update(1003, TOUCH_NONE);
    CHECK(robot_state == STATE_SENSOR_WAIT);
    check_braked();
}

static void test_cliff_recovery_wait(void)
{
    uint16_t now;
    start_cruise(2000);
    set_ir_cliff(1u);
    tick_irq(1);
    robot_update(system_ticks_2ms, TOUCH_NONE);
    CHECK(robot_state == STATE_CLIFF);
    set_ir_cliff(0u);
    tick_irq(IR_SURFACE_STABLE_TICKS - 1);
    now = system_ticks_2ms;
    fresh_update(now, TOUCH_NONE);
    CHECK(cliff_latched == 1 && robot_state == STATE_CLIFF);
    check_braked();
    tick_irq(1);
    now = system_ticks_2ms;
    CHECK(cliff_latched == 1); /* Only FSM, not the ISR, releases the latch. */
    fresh_update(now, TOUCH_NONE);
    CHECK(cliff_latched == 0 && robot_state == STATE_SETTLE);
    check_braked();
    fresh_update((uint16_t)(now + SETTLE_TICKS - 1), TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE);
    fresh_update((uint16_t)(now + SETTLE_TICKS), TOUCH_NONE);
    CHECK(robot_state == STATE_CRUISE);
    CHECK(motor_hw_mode == MOTOR_ARMING);
    motors_service((uint16_t)(now + SETTLE_TICKS + MOTOR_PWM_ARM_TICKS));
    CHECK(motor_hw_mode == MOTOR_RUNNING);
    /* Surface glitches cannot accumulate through intervening cliff samples. */
    set_ir_cliff(1u);
    tick_irq(1);
    set_ir_cliff(0u);
    tick_irq(IR_SURFACE_STABLE_TICKS - 1);
    set_ir_cliff(1u);
    tick_irq(1);
    CHECK(surface_stable_ticks == 0);
    set_ir_cliff(0u);
    tick_irq(IR_SURFACE_STABLE_TICKS + 10);
    CHECK(surface_stable_ticks == IR_SURFACE_STABLE_TICKS);
}

static void test_sonar_hysteresis_and_fault(void)
{
    reset_safe(3000);
    robot_enter(STATE_CRUISE, 3000);
    sonar_accept_sample(SONAR_NEAR, 3001);
    CHECK(sonar_obstacle == 1);
    sonar_accept_sample(SONAR_MID, 3002);
    CHECK(sonar_obstacle == 1 && sonar_clear_count == 0);
    sonar_accept_sample(SONAR_CLEAR, 3003);
    CHECK(sonar_obstacle == 1 && sonar_clear_count == 1);
    sonar_accept_sample(SONAR_MID, 3004);
    CHECK(sonar_obstacle == 1 && sonar_clear_count == 0);
    sonar_accept_sample(SONAR_CLEAR, 3005);
    sonar_accept_sample(SONAR_CLEAR, 3006);
    CHECK(sonar_obstacle == 0);
    sonar_accept_sample(SONAR_MID, 3007);
    CHECK(sonar_obstacle == 0); /* 25--30 cm retains previous clearance. */
    sonar_accept_sample(SONAR_FAULT, 3008);
    CHECK(sonar_obstacle == 1 && sonar_clear_count == 0);
    robot_update(3008, TOUCH_NONE);
    CHECK(robot_state == STATE_SENSOR_WAIT);
    check_braked();
    sonar_accept_sample(SONAR_FAULT, 3500);
    robot_update(3500, TOUCH_NONE);
    CHECK(robot_state == STATE_SENSOR_WAIT);
    check_braked();
    sonar_accept_sample(SONAR_CLEAR, 3501);
    robot_update(3501, TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE && sonar_obstacle == 1);
    check_braked();
    sonar_accept_sample(SONAR_CLEAR, 3502);
    robot_update(3502, TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE && sonar_obstacle == 0);
}

static void test_sonar_staleness(void)
{
    start_cruise(4000);
    robot_update((uint16_t)(4000 + SONAR_STALE_TICKS), TOUCH_NONE);
    CHECK(robot_state == STATE_CRUISE);
    robot_update((uint16_t)(4000 + SONAR_STALE_TICKS + 1), TOUCH_NONE);
    CHECK(robot_state == STATE_SENSOR_WAIT);
    check_braked();
    fresh_update((uint16_t)(4000 + SONAR_STALE_TICKS + 2), TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE);
    check_braked();
}

static void test_turn_direction_hold_and_clear(void)
{
    uint16_t now = 5000;
    uint8_t chosen;
    unsigned i;
    start_cruise(now);
    sonar_accept_sample(SONAR_NEAR, now);
    robot_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_OBSTACLE_PAUSE);
    CHECK(state_duration >= OBSTACLE_PAUSE_MIN_TICKS);
    CHECK(state_duration <= OBSTACLE_PAUSE_MAX_TICKS);
    chosen = turn_left;
    now = (uint16_t)(now + state_duration);
    fresh_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_TURN);
    CHECK(motor_target_a != motor_target_b);
    CHECK(motor_target_a == TURN_FAST_PWM || motor_target_a == TURN_SLOW_PWM);
    CHECK(motor_target_b == TURN_FAST_PWM || motor_target_b == TURN_SLOW_PWM);
    CHECK((turn_left == MOTOR_A_IS_LEFT) == (motor_target_a < motor_target_b));
    for (i = 0; i < 20; ++i) {
        now = (uint16_t)(now + ULTRASONIC_PERIOD_TICKS);
        sonar_accept_sample(i % 2 ? SONAR_MID : SONAR_NEAR, now);
        fresh_update(now, TOUCH_NONE);
        CHECK(robot_state == STATE_TURN);
        CHECK(turn_left == chosen);
    }
    sonar_accept_sample(SONAR_CLEAR, ++now);
    robot_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_TURN && turn_left == chosen);
    sonar_accept_sample(SONAR_CLEAR, ++now);
    robot_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE);
    check_braked();
}

static void test_random_turns_cover_both_directions(void)
{
    unsigned i, left = 0, right = 0;
    reset_safe(6000);
    for (i = 0; i < 100; ++i) {
        robot_enter(STATE_CRUISE, (uint16_t)(6000 + 2*i));
        robot_enter(STATE_OBSTACLE_PAUSE, (uint16_t)(6001 + 2*i));
        left += turn_left != 0;
        right += turn_left == 0;
    }
    CHECK(left > 0 && right > 0);
}

static void test_timeout_and_close_block_survive_pet(void)
{
    uint16_t now = 7000;
    reset_safe(now);
    sonar_accept_sample(SONAR_NEAR, now);
    robot_enter(STATE_TURN, now);
    now = (uint16_t)(now + TURN_TIMEOUT_TICKS - 1);
    fresh_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_TURN);
    fresh_update(++now, TOUCH_NONE);
    CHECK(robot_state == STATE_BLOCKED && avoidance_blocked == 1);
    check_braked();
    fresh_update(++now, TOUCH_RIGHT);
    CHECK(robot_state == STATE_PETTING && avoidance_blocked == 1);
    check_braked();
    fresh_update(++now, TOUCH_NONE);
    CHECK(robot_state == STATE_BLOCKED && avoidance_blocked == 1);
    check_braked();
    sonar_accept_sample(SONAR_CLEAR, ++now);
    fresh_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_BLOCKED);
    sonar_accept_sample(SONAR_CLEAR, ++now);
    fresh_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE && avoidance_blocked == 0);

    start_cruise(10000);
    sonar_accept_sample(SONAR_TOO_CLOSE, 10001);
    robot_update(10001, TOUCH_NONE);
    CHECK(robot_state == STATE_BLOCKED && avoidance_blocked == 1);
    check_braked();
    sonar_accept_sample(SONAR_NEAR, 10002);
    robot_update(10002, TOUCH_NONE);
    CHECK(robot_state == STATE_BLOCKED);
}

static void test_curious_pause_and_pet_release(void)
{
    uint16_t now = 11000;
    uint16_t duration;
    start_cruise(now);
    CHECK(state_duration >= IDLE_MIN_TICKS && state_duration <= IDLE_MAX_TICKS);
    duration = state_duration;
    fresh_update((uint16_t)(now + duration - 1), TOUCH_NONE);
    CHECK(robot_state == STATE_CRUISE);
    now = (uint16_t)(now + duration);
    fresh_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_CURIOUS_PAUSE);
    CHECK(state_duration >= CURIOUS_MIN_TICKS && state_duration <= CURIOUS_MAX_TICKS);
    check_braked();
    now = (uint16_t)(now + state_duration);
    fresh_update(now, TOUCH_NONE);
    CHECK(robot_state == STATE_CRUISE);
    fresh_update(++now, TOUCH_BOTH);
    CHECK(robot_state == STATE_PETTING);
    check_braked();
    fresh_update(++now, TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE);
    check_braked();
}

static void test_uint16_wrap(void)
{
    reset_safe(65535u);
    motors_set_targets(120, 120);
    motors_service(65535u);
    CHECK(motor_hw_mode == MOTOR_ARMING);
    motors_service((uint16_t)(65535u + MOTOR_PWM_ARM_TICKS - 1u));
    CHECK(motor_hw_mode == MOTOR_ARMING);
    motors_service((uint16_t)(65535u + MOTOR_PWM_ARM_TICKS));
    CHECK(motor_hw_mode == MOTOR_RUNNING);
    motors_service((uint16_t)(65535u + MOTOR_PWM_ARM_TICKS + MOTOR_RAMP_INTERVAL_TICKS));
    CHECK(motor_current_a == 2 * MOTOR_RAMP_STEP);
    system_ticks_2ms = 65535u;
    tick_irq(1);
    CHECK(system_ticks_2ms == 0);

    reset_safe(65500u);
    robot_enter(STATE_SETTLE, 65500u);
    fresh_update((uint16_t)(65500u + SETTLE_TICKS - 1u), TOUCH_NONE);
    CHECK(robot_state == STATE_SETTLE);
    fresh_update((uint16_t)(65500u + SETTLE_TICKS), TOUCH_NONE);
    CHECK(robot_state == STATE_CRUISE);

    reset_safe(65530u);
    robot_enter(STATE_CRUISE, 65530u);
    robot_update((uint16_t)(65530u + SONAR_STALE_TICKS), TOUCH_NONE);
    CHECK(robot_state == STATE_CRUISE);
    robot_update((uint16_t)(65530u + SONAR_STALE_TICKS + 1u), TOUCH_NONE);
    CHECK(robot_state == STATE_SENSOR_WAIT);

    reset_safe(65000u);
    sonar_accept_sample(SONAR_NEAR, 65000u);
    robot_enter(STATE_TURN, 65000u);
    fresh_update((uint16_t)(65000u + TURN_TIMEOUT_TICKS - 1u), TOUCH_NONE);
    CHECK(robot_state == STATE_TURN);
    fresh_update((uint16_t)(65000u + TURN_TIMEOUT_TICKS), TOUCH_NONE);
    CHECK(robot_state == STATE_BLOCKED);
}

#include "microphone_tests.inc"
#include "eyes_tests.inc"

int main(void)
{
    unsigned groups = 0;
#define RUN(test) do { test(); ++groups; } while (0)
    RUN(test_motor_resume_and_ramp);
    RUN(test_cliff_irq_and_stale_command);
    RUN(test_cliff_interrupts_arming);
    RUN(test_priority_across_states);
    RUN(test_cliff_recovery_wait);
    RUN(test_sonar_hysteresis_and_fault);
    RUN(test_sonar_staleness);
    RUN(test_turn_direction_hold_and_clear);
    RUN(test_random_turns_cover_both_directions);
    RUN(test_timeout_and_close_block_survive_pet);
    RUN(test_curious_pause_and_pet_release);
    RUN(test_uint16_wrap);
    RUN(test_microphone_configuration_and_samples);
    RUN(test_microphone_calibration_noise_and_rearm);
    RUN(test_microphone_clock_wrap);
    RUN(test_startle_pause_and_resume);
    RUN(test_startle_safety_priority);
    RUN(test_startle_turn_resume_and_timeout);
    RUN(test_all_face_mappings_and_priorities);
    RUN(test_drive_turn_and_curious_gazes);
    RUN(test_petting_gaze_bounce_and_heart);
    RUN(test_blocked_sensor_wait_motion_and_wrap);
    RUN(test_timed_blink_and_minimum_closed_frame);
    RUN(test_eye_randomness_does_not_change_motion);
    RUN(test_span_clipping_and_shape_math);
    RUN(test_all_poses_pixel_reference_and_bounds);
    RUN(test_partial_frame_stability_on_expression_change);
    RUN(test_startle_visual_recovery);
    printf("PASS: %u host logic regression groups, %u assertions.\n", groups, checks);
    puts("Host register shim; not a peripheral timing or physical hardware simulation.");
    return 0;
}
