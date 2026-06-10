#include "robot_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_oled_ssd1315.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "robot_config.h"

typedef enum {
    EYES_MOOD_DEFAULT = 0,
    EYES_MOOD_TIRED,
    EYES_MOOD_ANGRY,
    EYES_MOOD_HAPPY,
} eyes_mood_t;

typedef enum {
    EYES_POSITION_CENTER = 0,
    EYES_POSITION_N,
    EYES_POSITION_NE,
    EYES_POSITION_E,
    EYES_POSITION_SE,
    EYES_POSITION_S,
    EYES_POSITION_SW,
    EYES_POSITION_W,
    EYES_POSITION_NW,
} eyes_position_t;

typedef struct {
    int frame_interval_ms;
    int64_t fps_timer_ms;

    bool tired;
    bool angry;
    bool happy;
    bool curious;
    bool cyclops;
    bool eye_l_open;
    bool eye_r_open;

    int eye_l_width_default;
    int eye_l_height_default;
    int eye_l_width_current;
    int eye_l_height_current;
    int eye_l_width_next;
    int eye_l_height_next;
    int eye_l_height_offset;
    int eye_l_radius_default;
    int eye_l_radius_current;
    int eye_l_radius_next;
    int eye_l_x_default;
    int eye_l_y_default;
    int eye_l_x;
    int eye_l_y;
    int eye_l_x_next;
    int eye_l_y_next;

    int eye_r_width_default;
    int eye_r_height_default;
    int eye_r_width_current;
    int eye_r_height_current;
    int eye_r_width_next;
    int eye_r_height_next;
    int eye_r_height_offset;
    int eye_r_radius_default;
    int eye_r_radius_current;
    int eye_r_radius_next;
    int eye_r_x_default;
    int eye_r_y_default;
    int eye_r_x;
    int eye_r_y;
    int eye_r_x_next;
    int eye_r_y_next;

    int eyelids_tired_height;
    int eyelids_tired_height_next;
    int eyelids_angry_height;
    int eyelids_angry_height_next;
    int eyelids_happy_bottom_offset;
    int eyelids_happy_bottom_offset_next;

    int space_between_default;
    int space_between_current;
    int space_between_next;

    bool h_flicker;
    bool h_flicker_alternate;
    int h_flicker_amplitude;
    bool v_flicker;
    bool v_flicker_alternate;
    int v_flicker_amplitude;

    bool autoblinker;
    int blink_interval_s;
    int blink_interval_variation_s;
    int64_t blink_timer_ms;

    bool idle;
    int idle_interval_s;
    int idle_interval_variation_s;
    int64_t idle_animation_timer_ms;

    bool confused;
    int64_t confused_animation_timer_ms;
    int confused_animation_duration_ms;
    bool confused_toggle;

    bool laugh;
    int64_t laugh_animation_timer_ms;
    int laugh_animation_duration_ms;
    bool laugh_toggle;
} robot_eyes_t;

typedef struct {
    char ch;
    uint8_t col[5];
} glyph_t;

static const char *TAG = "robot_display";

static bool s_oled_ready;
static uint8_t s_oled_addr = ROBOT_OLED_ADDR_PRIMARY;
static uint8_t s_oled_buffer[ROBOT_OLED_WIDTH * ROBOT_OLED_HEIGHT / 8];
static i2c_master_bus_handle_t s_oled_i2c_bus;
static esp_lcd_panel_io_handle_t s_oled_io_handle;
static esp_lcd_panel_handle_t s_oled_panel_handle;
static robot_eyes_t s_eyes;

static const glyph_t FONT_5X7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'W', {0x7F, 0x20, 0x18, 0x20, 0x7F}},
};

static uint32_t random_upto(uint32_t max)
{
    return max == 0 ? 0 : esp_random() % max;
}

static int64_t millis_now(void)
{
    return esp_timer_get_time() / 1000;
}

static esp_err_t oled_flush(void)
{
    if (!s_oled_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_lcd_panel_draw_bitmap(s_oled_panel_handle, 0, 0, ROBOT_OLED_WIDTH, ROBOT_OLED_HEIGHT, s_oled_buffer);
}

static void oled_clear(void)
{
    memset(s_oled_buffer, 0, sizeof(s_oled_buffer));
}

static void oled_draw_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= ROBOT_OLED_WIDTH || y < 0 || y >= ROBOT_OLED_HEIGHT) {
        return;
    }

    const size_t index = (size_t)x + (size_t)(y / 8) * ROBOT_OLED_WIDTH;
    const uint8_t bit = (uint8_t)(1U << (y & 7));

    if (on) {
        s_oled_buffer[index] |= bit;
    } else {
        s_oled_buffer[index] &= (uint8_t)~bit;
    }
}

static void oled_fill_rect(int x, int y, int w, int h, bool on)
{
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) {
            oled_draw_pixel(xx, yy, on);
        }
    }
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int min3_int(int a, int b, int c)
{
    const int min_value = a < b ? a : b;
    return min_value < c ? min_value : c;
}

static int max3_int(int a, int b, int c)
{
    const int max_value = a > b ? a : b;
    return max_value > c ? max_value : c;
}

static void oled_fill_round_rect(int x, int y, int w, int h, int radius, bool on)
{
    if (w <= 0 || h <= 0) {
        return;
    }

    radius = clamp_int(radius, 0, (w < h ? w : h) / 2);
    if (radius == 0) {
        oled_fill_rect(x, y, w, h, on);
        return;
    }

    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) {
            int cx = xx;
            int cy = yy;

            if (xx < x + radius) {
                cx = x + radius;
            } else if (xx >= x + w - radius) {
                cx = x + w - radius - 1;
            }

            if (yy < y + radius) {
                cy = y + radius;
            } else if (yy >= y + h - radius) {
                cy = y + h - radius - 1;
            }

            const int dx = xx - cx;
            const int dy = yy - cy;
            if ((dx * dx) + (dy * dy) <= radius * radius) {
                oled_draw_pixel(xx, yy, on);
            }
        }
    }
}

static int32_t triangle_edge(int x0, int y0, int x1, int y1, int x, int y)
{
    return (int32_t)(x - x0) * (y1 - y0) - (int32_t)(y - y0) * (x1 - x0);
}

static void oled_fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, bool on)
{
    const int min_x = clamp_int(min3_int(x0, x1, x2), 0, ROBOT_OLED_WIDTH - 1);
    const int max_x = clamp_int(max3_int(x0, x1, x2), 0, ROBOT_OLED_WIDTH - 1);
    const int min_y = clamp_int(min3_int(y0, y1, y2), 0, ROBOT_OLED_HEIGHT - 1);
    const int max_y = clamp_int(max3_int(y0, y1, y2), 0, ROBOT_OLED_HEIGHT - 1);

    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            const int32_t e0 = triangle_edge(x0, y0, x1, y1, x, y);
            const int32_t e1 = triangle_edge(x1, y1, x2, y2, x, y);
            const int32_t e2 = triangle_edge(x2, y2, x0, y0, x, y);
            if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) {
                oled_draw_pixel(x, y, on);
            }
        }
    }
}

static const uint8_t *font_find(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - 'a' + 'A');
    }

    for (size_t i = 0; i < sizeof(FONT_5X7) / sizeof(FONT_5X7[0]); i++) {
        if (FONT_5X7[i].ch == ch) {
            return FONT_5X7[i].col;
        }
    }

    return FONT_5X7[0].col;
}

static void oled_draw_char(int x, int y, char ch)
{
    const uint8_t *glyph = font_find(ch);

    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if ((glyph[col] & (1U << row)) != 0) {
                oled_draw_pixel(x + col, y + row, true);
            }
        }
    }
}

static void oled_draw_text(int x, int y, const char *text)
{
    while (*text != '\0' && x < ROBOT_OLED_WIDTH) {
        oled_draw_char(x, y, *text++);
        x += 6;
    }
}

static esp_err_t oled_detect_addr(void)
{
    esp_err_t err = i2c_master_probe(s_oled_i2c_bus, ROBOT_OLED_ADDR_PRIMARY, 50);
    if (err == ESP_OK) {
        s_oled_addr = ROBOT_OLED_ADDR_PRIMARY;
        return ESP_OK;
    }

    err = i2c_master_probe(s_oled_i2c_bus, ROBOT_OLED_ADDR_SECONDARY, 50);
    if (err == ESP_OK) {
        s_oled_addr = ROBOT_OLED_ADDR_SECONDARY;
        return ESP_OK;
    }

    return err;
}

static int roboeyes_screen_constraint_x(const robot_eyes_t *eyes)
{
    const int constraint = ROBOT_OLED_WIDTH - eyes->eye_l_width_current -
                           eyes->space_between_current - eyes->eye_r_width_current;
    return constraint > 0 ? constraint : 0;
}

static int roboeyes_screen_constraint_y(const robot_eyes_t *eyes)
{
    const int constraint = ROBOT_OLED_HEIGHT - eyes->eye_l_height_default;
    return constraint > 0 ? constraint : 0;
}

static void roboeyes_set_framerate(robot_eyes_t *eyes, int fps)
{
    if (fps <= 0) {
        fps = 50;
    }
    eyes->frame_interval_ms = 1000 / fps;
}

static void roboeyes_begin(robot_eyes_t *eyes, int fps)
{
    memset(eyes, 0, sizeof(*eyes));

    eyes->eye_l_width_default = 36;
    eyes->eye_l_height_default = 36;
    eyes->eye_l_width_current = eyes->eye_l_width_default;
    eyes->eye_l_height_current = 1;
    eyes->eye_l_width_next = eyes->eye_l_width_default;
    eyes->eye_l_height_next = eyes->eye_l_height_default;
    eyes->eye_l_radius_default = 8;
    eyes->eye_l_radius_current = eyes->eye_l_radius_default;
    eyes->eye_l_radius_next = eyes->eye_l_radius_default;

    eyes->eye_r_width_default = eyes->eye_l_width_default;
    eyes->eye_r_height_default = eyes->eye_l_height_default;
    eyes->eye_r_width_current = eyes->eye_r_width_default;
    eyes->eye_r_height_current = 1;
    eyes->eye_r_width_next = eyes->eye_r_width_default;
    eyes->eye_r_height_next = eyes->eye_r_height_default;
    eyes->eye_r_radius_default = eyes->eye_l_radius_default;
    eyes->eye_r_radius_current = eyes->eye_r_radius_default;
    eyes->eye_r_radius_next = eyes->eye_r_radius_default;

    eyes->space_between_default = 10;
    eyes->space_between_current = eyes->space_between_default;
    eyes->space_between_next = eyes->space_between_default;

    eyes->eye_l_x_default = (ROBOT_OLED_WIDTH - (eyes->eye_l_width_default + eyes->space_between_default +
                                                eyes->eye_r_width_default)) / 2;
    eyes->eye_l_y_default = (ROBOT_OLED_HEIGHT - eyes->eye_l_height_default) / 2;
    eyes->eye_l_x = eyes->eye_l_x_default;
    eyes->eye_l_y = eyes->eye_l_y_default;
    eyes->eye_l_x_next = eyes->eye_l_x;
    eyes->eye_l_y_next = eyes->eye_l_y;

    eyes->eye_r_x_default = eyes->eye_l_x + eyes->eye_l_width_current + eyes->space_between_default;
    eyes->eye_r_y_default = eyes->eye_l_y;
    eyes->eye_r_x = eyes->eye_r_x_default;
    eyes->eye_r_y = eyes->eye_r_y_default;
    eyes->eye_r_x_next = eyes->eye_r_x;
    eyes->eye_r_y_next = eyes->eye_r_y;

    eyes->h_flicker_amplitude = 2;
    eyes->v_flicker_amplitude = 10;
    eyes->autoblinker = true;
    eyes->blink_interval_s = 3;
    eyes->blink_interval_variation_s = 2;
    eyes->idle = true;
    eyes->idle_interval_s = 2;
    eyes->idle_interval_variation_s = 2;
    eyes->confused_animation_duration_ms = 500;
    eyes->confused_toggle = true;
    eyes->laugh_animation_duration_ms = 500;
    eyes->laugh_toggle = true;

    roboeyes_set_framerate(eyes, fps);
}

static void roboeyes_set_mood(robot_eyes_t *eyes, eyes_mood_t mood)
{
    eyes->tired = mood == EYES_MOOD_TIRED;
    eyes->angry = mood == EYES_MOOD_ANGRY;
    eyes->happy = mood == EYES_MOOD_HAPPY;
}

static void roboeyes_set_position(robot_eyes_t *eyes, eyes_position_t position)
{
    const int constraint_x = roboeyes_screen_constraint_x(eyes);
    const int constraint_y = roboeyes_screen_constraint_y(eyes);

    switch (position) {
    case EYES_POSITION_N:
        eyes->eye_l_x_next = constraint_x / 2;
        eyes->eye_l_y_next = 0;
        break;
    case EYES_POSITION_NE:
        eyes->eye_l_x_next = constraint_x;
        eyes->eye_l_y_next = 0;
        break;
    case EYES_POSITION_E:
        eyes->eye_l_x_next = constraint_x;
        eyes->eye_l_y_next = constraint_y / 2;
        break;
    case EYES_POSITION_SE:
        eyes->eye_l_x_next = constraint_x;
        eyes->eye_l_y_next = constraint_y;
        break;
    case EYES_POSITION_S:
        eyes->eye_l_x_next = constraint_x / 2;
        eyes->eye_l_y_next = constraint_y;
        break;
    case EYES_POSITION_SW:
        eyes->eye_l_x_next = 0;
        eyes->eye_l_y_next = constraint_y;
        break;
    case EYES_POSITION_W:
        eyes->eye_l_x_next = 0;
        eyes->eye_l_y_next = constraint_y / 2;
        break;
    case EYES_POSITION_NW:
        eyes->eye_l_x_next = 0;
        eyes->eye_l_y_next = 0;
        break;
    case EYES_POSITION_CENTER:
    default:
        eyes->eye_l_x_next = constraint_x / 2;
        eyes->eye_l_y_next = constraint_y / 2;
        break;
    }
}

static void roboeyes_close(robot_eyes_t *eyes)
{
    eyes->eye_l_height_next = 1;
    eyes->eye_r_height_next = 1;
    eyes->eye_l_open = false;
    eyes->eye_r_open = false;
}

static void roboeyes_open(robot_eyes_t *eyes)
{
    eyes->eye_l_open = true;
    eyes->eye_r_open = true;
}

static void roboeyes_wake(robot_eyes_t *eyes)
{
    roboeyes_open(eyes);
    eyes->eye_l_height_next = eyes->eye_l_height_default;
    eyes->eye_r_height_next = eyes->eye_r_height_default;
}

static void roboeyes_blink(robot_eyes_t *eyes)
{
    roboeyes_close(eyes);
    roboeyes_open(eyes);
}

static void roboeyes_set_h_flicker(robot_eyes_t *eyes, bool active, int amplitude)
{
    eyes->h_flicker = active;
    eyes->h_flicker_amplitude = amplitude;
}

static void roboeyes_set_v_flicker(robot_eyes_t *eyes, bool active, int amplitude)
{
    eyes->v_flicker = active;
    eyes->v_flicker_amplitude = amplitude;
}

static int64_t next_timer_ms(int64_t now_ms, int interval_s, int variation_s)
{
    const int variation = variation_s > 0 ? (int)random_upto((uint32_t)variation_s) : 0;
    return now_ms + ((int64_t)interval_s + variation) * 1000;
}

static void roboeyes_draw(robot_eyes_t *eyes, int64_t now_ms)
{
    if (eyes->curious) {
        if (eyes->eye_l_x_next <= 10) {
            eyes->eye_l_height_offset = 8;
        } else if (eyes->cyclops && eyes->eye_l_x_next >= roboeyes_screen_constraint_x(eyes) - 10) {
            eyes->eye_l_height_offset = 8;
        } else {
            eyes->eye_l_height_offset = 0;
        }

        if (eyes->eye_r_x_next >= ROBOT_OLED_WIDTH - eyes->eye_r_width_current - 10) {
            eyes->eye_r_height_offset = 8;
        } else {
            eyes->eye_r_height_offset = 0;
        }
    } else {
        eyes->eye_l_height_offset = 0;
        eyes->eye_r_height_offset = 0;
    }

    eyes->eye_l_height_current =
        (eyes->eye_l_height_current + eyes->eye_l_height_next + eyes->eye_l_height_offset) / 2;
    eyes->eye_l_y += (eyes->eye_l_height_default - eyes->eye_l_height_current) / 2;
    eyes->eye_l_y -= eyes->eye_l_height_offset / 2;

    eyes->eye_r_height_current =
        (eyes->eye_r_height_current + eyes->eye_r_height_next + eyes->eye_r_height_offset) / 2;
    eyes->eye_r_y += (eyes->eye_r_height_default - eyes->eye_r_height_current) / 2;
    eyes->eye_r_y -= eyes->eye_r_height_offset / 2;

    if (eyes->eye_l_open && eyes->eye_l_height_current <= 1 + eyes->eye_l_height_offset) {
        eyes->eye_l_height_next = eyes->eye_l_height_default;
    }
    if (eyes->eye_r_open && eyes->eye_r_height_current <= 1 + eyes->eye_r_height_offset) {
        eyes->eye_r_height_next = eyes->eye_r_height_default;
    }

    eyes->eye_l_width_current = (eyes->eye_l_width_current + eyes->eye_l_width_next) / 2;
    eyes->eye_r_width_current = (eyes->eye_r_width_current + eyes->eye_r_width_next) / 2;
    eyes->space_between_current = (eyes->space_between_current + eyes->space_between_next) / 2;

    eyes->eye_l_x = (eyes->eye_l_x + eyes->eye_l_x_next) / 2;
    eyes->eye_l_y = (eyes->eye_l_y + eyes->eye_l_y_next) / 2;
    eyes->eye_r_x_next = eyes->eye_l_x_next + eyes->eye_l_width_current + eyes->space_between_current;
    eyes->eye_r_y_next = eyes->eye_l_y_next;
    eyes->eye_r_x = (eyes->eye_r_x + eyes->eye_r_x_next) / 2;
    eyes->eye_r_y = (eyes->eye_r_y + eyes->eye_r_y_next) / 2;

    eyes->eye_l_radius_current = (eyes->eye_l_radius_current + eyes->eye_l_radius_next) / 2;
    eyes->eye_r_radius_current = (eyes->eye_r_radius_current + eyes->eye_r_radius_next) / 2;

    if (eyes->autoblinker && now_ms >= eyes->blink_timer_ms) {
        roboeyes_blink(eyes);
        eyes->blink_timer_ms = next_timer_ms(now_ms, eyes->blink_interval_s, eyes->blink_interval_variation_s);
    }

    if (eyes->laugh) {
        if (eyes->laugh_toggle) {
            roboeyes_set_v_flicker(eyes, true, 5);
            eyes->laugh_animation_timer_ms = now_ms;
            eyes->laugh_toggle = false;
        } else if (now_ms >= eyes->laugh_animation_timer_ms + eyes->laugh_animation_duration_ms) {
            roboeyes_set_v_flicker(eyes, false, 0);
            eyes->laugh_toggle = true;
            eyes->laugh = false;
        }
    }

    if (eyes->confused) {
        if (eyes->confused_toggle) {
            roboeyes_set_h_flicker(eyes, true, 20);
            eyes->confused_animation_timer_ms = now_ms;
            eyes->confused_toggle = false;
        } else if (now_ms >= eyes->confused_animation_timer_ms + eyes->confused_animation_duration_ms) {
            roboeyes_set_h_flicker(eyes, false, 0);
            eyes->confused_toggle = true;
            eyes->confused = false;
        }
    }

    if (eyes->idle && now_ms >= eyes->idle_animation_timer_ms) {
        const int constraint_x = roboeyes_screen_constraint_x(eyes);
        const int constraint_y = roboeyes_screen_constraint_y(eyes);
        eyes->eye_l_x_next = (int)random_upto((uint32_t)(constraint_x + 1));
        eyes->eye_l_y_next = (int)random_upto((uint32_t)(constraint_y + 1));
        eyes->idle_animation_timer_ms = next_timer_ms(now_ms, eyes->idle_interval_s, eyes->idle_interval_variation_s);
    }

    if (eyes->h_flicker) {
        const int offset = eyes->h_flicker_alternate ? eyes->h_flicker_amplitude : -eyes->h_flicker_amplitude;
        eyes->eye_l_x += offset;
        eyes->eye_r_x += offset;
        eyes->h_flicker_alternate = !eyes->h_flicker_alternate;
    }

    if (eyes->v_flicker) {
        const int offset = eyes->v_flicker_alternate ? eyes->v_flicker_amplitude : -eyes->v_flicker_amplitude;
        eyes->eye_l_y += offset;
        eyes->eye_r_y += offset;
        eyes->v_flicker_alternate = !eyes->v_flicker_alternate;
    }

    if (eyes->cyclops) {
        eyes->eye_r_width_current = 0;
        eyes->eye_r_height_current = 0;
        eyes->space_between_current = 0;
    }

    oled_clear();
    oled_fill_round_rect(eyes->eye_l_x, eyes->eye_l_y, eyes->eye_l_width_current,
                         eyes->eye_l_height_current, eyes->eye_l_radius_current, true);
    if (!eyes->cyclops) {
        oled_fill_round_rect(eyes->eye_r_x, eyes->eye_r_y, eyes->eye_r_width_current,
                             eyes->eye_r_height_current, eyes->eye_r_radius_current, true);
    }

    if (eyes->tired) {
        eyes->eyelids_tired_height_next = eyes->eye_l_height_current / 2;
        eyes->eyelids_angry_height_next = 0;
    } else {
        eyes->eyelids_tired_height_next = 0;
    }

    if (eyes->angry) {
        eyes->eyelids_angry_height_next = eyes->eye_l_height_current / 2;
        eyes->eyelids_tired_height_next = 0;
    } else {
        eyes->eyelids_angry_height_next = 0;
    }

    if (eyes->happy) {
        eyes->eyelids_happy_bottom_offset_next = eyes->eye_l_height_current / 2;
    } else {
        eyes->eyelids_happy_bottom_offset_next = 0;
    }

    eyes->eyelids_tired_height = (eyes->eyelids_tired_height + eyes->eyelids_tired_height_next) / 2;
    if (!eyes->cyclops) {
        oled_fill_triangle(eyes->eye_l_x, eyes->eye_l_y - 1,
                           eyes->eye_l_x + eyes->eye_l_width_current, eyes->eye_l_y - 1,
                           eyes->eye_l_x, eyes->eye_l_y + eyes->eyelids_tired_height - 1, false);
        oled_fill_triangle(eyes->eye_r_x, eyes->eye_r_y - 1,
                           eyes->eye_r_x + eyes->eye_r_width_current, eyes->eye_r_y - 1,
                           eyes->eye_r_x + eyes->eye_r_width_current,
                           eyes->eye_r_y + eyes->eyelids_tired_height - 1, false);
    } else {
        oled_fill_triangle(eyes->eye_l_x, eyes->eye_l_y - 1,
                           eyes->eye_l_x + (eyes->eye_l_width_current / 2), eyes->eye_l_y - 1,
                           eyes->eye_l_x, eyes->eye_l_y + eyes->eyelids_tired_height - 1, false);
        oled_fill_triangle(eyes->eye_l_x + (eyes->eye_l_width_current / 2), eyes->eye_l_y - 1,
                           eyes->eye_l_x + eyes->eye_l_width_current, eyes->eye_l_y - 1,
                           eyes->eye_l_x + eyes->eye_l_width_current,
                           eyes->eye_l_y + eyes->eyelids_tired_height - 1, false);
    }

    eyes->eyelids_angry_height = (eyes->eyelids_angry_height + eyes->eyelids_angry_height_next) / 2;
    if (!eyes->cyclops) {
        oled_fill_triangle(eyes->eye_l_x, eyes->eye_l_y - 1,
                           eyes->eye_l_x + eyes->eye_l_width_current, eyes->eye_l_y - 1,
                           eyes->eye_l_x + eyes->eye_l_width_current,
                           eyes->eye_l_y + eyes->eyelids_angry_height - 1, false);
        oled_fill_triangle(eyes->eye_r_x, eyes->eye_r_y - 1,
                           eyes->eye_r_x + eyes->eye_r_width_current, eyes->eye_r_y - 1,
                           eyes->eye_r_x, eyes->eye_r_y + eyes->eyelids_angry_height - 1, false);
    } else {
        oled_fill_triangle(eyes->eye_l_x, eyes->eye_l_y - 1,
                           eyes->eye_l_x + (eyes->eye_l_width_current / 2), eyes->eye_l_y - 1,
                           eyes->eye_l_x + (eyes->eye_l_width_current / 2),
                           eyes->eye_l_y + eyes->eyelids_angry_height - 1, false);
        oled_fill_triangle(eyes->eye_l_x + (eyes->eye_l_width_current / 2), eyes->eye_l_y - 1,
                           eyes->eye_l_x + eyes->eye_l_width_current, eyes->eye_l_y - 1,
                           eyes->eye_l_x + (eyes->eye_l_width_current / 2),
                           eyes->eye_l_y + eyes->eyelids_angry_height - 1, false);
    }

    eyes->eyelids_happy_bottom_offset =
        (eyes->eyelids_happy_bottom_offset + eyes->eyelids_happy_bottom_offset_next) / 2;
    oled_fill_round_rect(eyes->eye_l_x - 1,
                         (eyes->eye_l_y + eyes->eye_l_height_current) -
                             eyes->eyelids_happy_bottom_offset + 1,
                         eyes->eye_l_width_current + 2, eyes->eye_l_height_default,
                         eyes->eye_l_radius_current, false);
    if (!eyes->cyclops) {
        oled_fill_round_rect(eyes->eye_r_x - 1,
                             (eyes->eye_r_y + eyes->eye_r_height_current) -
                                 eyes->eyelids_happy_bottom_offset + 1,
                             eyes->eye_r_width_current + 2, eyes->eye_r_height_default,
                             eyes->eye_r_radius_current, false);
    }

    (void)oled_flush();
}

esp_err_t robot_display_init(void)
{
    roboeyes_begin(&s_eyes, 50);
    roboeyes_set_mood(&s_eyes, EYES_MOOD_DEFAULT);
    roboeyes_set_position(&s_eyes, EYES_POSITION_CENTER);

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = ROBOT_OLED_I2C_PORT,
        .sda_io_num = ROBOT_OLED_SDA_GPIO,
        .scl_io_num = ROBOT_OLED_SCL_GPIO,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_oled_i2c_bus), TAG, "I2C master bus init failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(oled_detect_addr(), TAG, "SSD1315 OLED not found on 0x3C or 0x3D");

    const esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = s_oled_addr,
        .scl_speed_hz = ROBOT_OLED_I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = ROBOT_OLED_CMD_BITS,
        .lcd_param_bits = ROBOT_OLED_CMD_BITS,
        .dc_bit_offset = 6,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(s_oled_i2c_bus, &io_config, &s_oled_io_handle),
                        TAG, "SSD1315 panel IO init failed");

    esp_lcd_panel_ssd1315_config_t ssd1315_config = {
        .height = ROBOT_OLED_HEIGHT,
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,
        .bits_per_pixel = 1,
        .vendor_config = &ssd1315_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ssd1315(s_oled_io_handle, &panel_config, &s_oled_panel_handle),
                        TAG, "SSD1315 panel driver init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_oled_panel_handle), TAG, "SSD1315 panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_oled_panel_handle, false), TAG, "SSD1315 invert config failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_oled_panel_handle, true), TAG, "SSD1315 display on failed");

    s_oled_ready = true;
    ESP_LOGI(TAG, "SSD1315 OLED initialized by waveshare/esp_oled_ssd1315 at I2C address 0x%02X", s_oled_addr);
    oled_clear();
    return oled_flush();
}

void robot_display_show_startup(void)
{
    if (!s_oled_ready) {
        return;
    }

    oled_clear();
    oled_draw_text(0, 0, "STARTING...");
    oled_draw_text(0, 16, "OLED OK");
    (void)oled_flush();
}

void robot_display_show_wifi_info(const char *sta_line)
{
    if (!s_oled_ready) {
        return;
    }

    oled_clear();
    oled_draw_text(0, 0, "WEB READY");
    oled_draw_text(0, 16, "AP: 192.168.4.1");
    oled_draw_text(0, 32, sta_line == NULL ? "STA: SET WIFI" : sta_line);
    (void)oled_flush();
}

void robot_display_set_random_mode(robot_random_mode_t mode)
{
    s_eyes.cyclops = false;
    s_eyes.confused = false;
    s_eyes.laugh = false;
    s_eyes.curious = false;
    s_eyes.h_flicker = false;
    s_eyes.v_flicker = false;
    s_eyes.eye_l_width_next = s_eyes.eye_l_width_default;
    s_eyes.eye_r_width_next = s_eyes.eye_r_width_default;
    s_eyes.space_between_next = s_eyes.space_between_default;

    switch (mode) {
    case ROBOT_RANDOM_OFF:
        roboeyes_set_mood(&s_eyes, EYES_MOOD_TIRED);
        roboeyes_set_position(&s_eyes, EYES_POSITION_CENTER);
        s_eyes.idle = false;
        s_eyes.autoblinker = false;
        roboeyes_close(&s_eyes);
        break;
    case ROBOT_RANDOM_SOFT:
        roboeyes_set_mood(&s_eyes, EYES_MOOD_HAPPY);
        s_eyes.idle = true;
        s_eyes.autoblinker = true;
        roboeyes_wake(&s_eyes);
        break;
    case ROBOT_RANDOM_NORMAL:
    default:
        roboeyes_set_mood(&s_eyes, EYES_MOOD_DEFAULT);
        s_eyes.curious = true;
        s_eyes.idle = true;
        s_eyes.autoblinker = true;
        roboeyes_wake(&s_eyes);
        break;
    }
}

void robot_display_update_eyes(void)
{
    if (!s_oled_ready) {
        return;
    }

    const int64_t now_ms = millis_now();
    if (now_ms - s_eyes.fps_timer_ms < s_eyes.frame_interval_ms) {
        return;
    }

    roboeyes_draw(&s_eyes, now_ms);
    s_eyes.fps_timer_ms = now_ms;
}
