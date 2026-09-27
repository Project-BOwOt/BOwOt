/* Render the production C framebuffer on a host, without executing the
 * firmware main loop or emulating any peripheral timing. Preview only.
 * Each synthetic display frame advances time by 32 * 2.048 ms.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define main robot_firmware_main
#include "../robot_sh1106.c"
#undef main

typedef struct {
    const char *key;
    const char *label;
    robot_state_t state;
    touch_state_t touch;
    unsigned frames;
    uint8_t left;
} preview_scene_t;

static const preview_scene_t scenes[] = {
    {"startup", "Waking up", STATE_STARTUP, TOUCH_NONE, 8, 0},
    {"cruise", "Cruising / alive and alert", STATE_CRUISE, TOUCH_NONE, 80, 0},
    {"curious", "A curious little glance", STATE_CURIOUS_PAUSE, TOUCH_NONE, 3, 0},
    {"prepare_left", "Obstacle / look before turning", STATE_OBSTACLE_PAUSE, TOUCH_NONE, 3, 1},
    {"turn_left", "Turning left (robot's left)", STATE_TURN, TOUCH_NONE, 22, 1},
    {"prepare_right", "New obstacle / look right", STATE_OBSTACLE_PAUSE, TOUCH_NONE, 3, 0},
    {"turn_right", "Turning right (robot's right)", STATE_TURN, TOUCH_NONE, 22, 0},
    {"blocked", "Blocked / looking for a way", STATE_BLOCKED, TOUCH_NONE, 32, 0},
    {"sensor_wait", "Sensor wait / checking around", STATE_SENSOR_WAIT, TOUCH_NONE, 32, 0},
    {"pet_left", "Left touch / happy closed eyes", STATE_PETTING, TOUCH_LEFT, 25, 0},
    {"pet_right", "Right touch / happy closed eyes", STATE_PETTING, TOUCH_RIGHT, 25, 0},
    {"pet_both", "Both touches / enjoying the pets", STATE_PETTING, TOUCH_BOTH, 25, 0},
    {"cliff", "Cliff / fast warning pop", STATE_CLIFF, TOUCH_NONE, 26, 0},
    {"recovery", "Safe again / settling down", STATE_SETTLE, TOUCH_NONE, 5, 0},
    {"cruise_again", "Back on the move", STATE_CRUISE, TOUCH_NONE, 20, 0},
    {"startled", "Clap / startled for half a second", STATE_STARTLED, TOUCH_NONE, 8, 0},
    {"resume", "And back to exploring", STATE_CRUISE, TOUCH_NONE, 30, 0}
};

static void write_frame(const char *dir, unsigned frame)
{
    char path[1024];
    unsigned x, y;
    FILE *out;
    if (snprintf(path, sizeof(path), "%s/%04u.pgm", dir, frame) >=
            (int)sizeof(path)) {
        fprintf(stderr, "Preview output path is too long\n");
        exit(2);
    }
    out = fopen(path, "wb");
    if (!out) { perror(path); exit(2); }
    fprintf(out, "P5\n128 64\n255\n");
    for (y = 0; y < 64; ++y) {
        for (x = 0; x < 128; ++x) {
            unsigned lit = 0;
            if (x >= OLED_EYE_X && x < OLED_EYE_X + OLED_EYE_WIDTH &&
                y >= OLED_FIRST_EYE_PAGE * 8u &&
                y < (OLED_FIRST_EYE_PAGE + OLED_EYE_PAGE_COUNT) * 8u) {
                unsigned bx = x - OLED_EYE_X;
                unsigned by = y - OLED_FIRST_EYE_PAGE * 8u;
                lit = (eye_frame_buffer[by / 8u][bx] >> (by % 8u)) & 1u;
            }
            fputc(lit ? 255 : 0, out);
        }
    }
    fclose(out);
}

int main(int argc, char **argv)
{
    unsigned scene, frame = 0;
    uint16_t now = 0;
    if (argc != 2) {
        fprintf(stderr, "Usage: %s FRAME_DIRECTORY\n", argv[0]);
        return 2;
    }
    random_state = 0xACE1u;
    cliff_latched = 0u;
    for (scene = 0; scene < sizeof(scenes) / sizeof(scenes[0]); ++scene) {
        const preview_scene_t *s = &scenes[scene];
        unsigned scene_frame;
        robot_state = s->state;
        state_since = now;
        turn_left = s->left;
        cliff_latched = s->state == STATE_CLIFF;
        for (scene_frame = 0; scene_frame < s->frames; ++scene_frame) {
            system_ticks_2ms = now;
            robot_expression_service(s->touch, now);
            eyes_advance_frame(now);
            eyes_render_frame();
            write_frame(argv[1], frame);
            printf("%u\t%s\t%s\t%u\t%u\n", frame, s->key, s->label,
                   scene_frame, (unsigned)now);
            now = (uint16_t)(now + 32u);
            ++frame;
        }
    }
    return 0;
}
