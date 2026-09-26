/*
 * ============================================================================================
 *  TILT MAZE  -  Elecrow All-In-One Starter Kit for ESP32-P4
 *  Display + touch : same board bring-up as Tilt Racer (EK79007 MIPI-DSI panel 1024x600,
 *                     GT911 touch, LVGL v8 port) - unchanged, copy-pasted section for section.
 *  Motion sensor   : same LSM6DS3TR-C driver as Tilt Racer (I2C port 0, SDA=GPIO18, SCL=GPIO19)
 *
 *  This is a gentler "training wheels" game for Tilt Racer: a simple 3-ring circular maze.
 *  The child starts in the centre and tilts the board to roll a coloured ball outward, through
 *  a gap in each ring, until it escapes past the outermost ring. There is no crash, no timer,
 *  no score - touching a wall just gently stops the ball from crossing it, so the child can
 *  slide along the wall until they find the opening. Once they can reliably do this, the same
 *  tilt-to-move skill carries straight over into Tilt Racer's steer/speed controls.
 * ============================================================================================
 *
 *  HOW TO SET UP THE SKETCH FOLDER (Arduino IDE)
 *    Put this file in a folder called  TiltMaze_P4  together with these files, copied UNCHANGED
 *    from your working TiltRacer_P4 folder (same board, same sensor, nothing to edit in them):
 *      esp_panel_board_custom_conf.h  esp_panel_drivers_conf.h  esp_utils_conf.h  lv_conf.h
 *      lvgl_v8_port.cpp  lvgl_v8_port.h  bsp_i2c.cpp  bsp_i2c.h  bsp_lsm6ds3tr.cpp  bsp_lsm6ds3tr.h
 *
 *  NO CONFIG EDITS NEEDED
 *    esp_panel_board_custom_conf.h already has ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST set to 1
 *    (from the Tilt Racer fix) and ESP_PANEL_BOARD_USE_TOUCH set to 1 - leave both as they are.
 *
 *  HOW TO PLAY
 *    1. Tap START (the ball "arms" using whatever tilt angle you're holding right then as level).
 *    2. Tilt the board to roll the ball outward through the gap in each of the 3 rings.
 *    3. Touching a ring wall just gently stops you crossing it - slide along it to find the gap.
 *    4. Clear all 3 rings to win! Tap NEW GAME for a freshly shuffled maze.
 * ============================================================================================
 */

// ==================== SECTION 1 : INCLUDES & NAMESPACES ====================
#include "Arduino.h"                     // Arduino core: Serial, delay(), pinMode(), digitalWrite()
#include <esp_display_panel.hpp>         // ESP32_Display_Panel library: Board, LCD, Touch, Backlight classes
#include <lvgl.h>                        // LVGL v8 graphics library: objects, styles, labels, buttons, timers, arcs
#include <lvgl_v8_port.h>                // Espressif LVGL glue: lvgl_port_init(), lvgl_port_lock(), lvgl_port_unlock()
#include "esp_ldo_regulator.h"           // ESP32-P4 on-chip LDO regulator driver (needed for the MIPI PHY power rail)
#include "esp_random.h"                  // esp_random(): hardware random number generator (used for random gap angles)
#include "bsp_i2c.h"                     // I2C helper: i2c_init(), i2c_read_reg(), i2c_write_reg()
#include "bsp_lsm6ds3tr.h"               // IMU driver: register addresses + lsm6ds3_begin()
#include <math.h>                        // sinf(), cosf(), atan2f(), sqrtf(), fabsf(), fmodf()

using namespace esp_panel::drivers;      // lets us write "LCD" / "Touch" instead of esp_panel::drivers::LCD / Touch
using namespace esp_panel::board;        // lets us write "Board" instead of esp_panel::board::Board

// ==================== SECTION 2 : TUNABLE SETTINGS ====================

// ---- 2.1 Screen layout ----
static const int SCREEN_W = 1024;                       // full panel width in pixels
static const int SCREEN_H = 600;                        // full panel height in pixels
static const int ARENA_W  = 820;                         // play-field width (left side, 80.1 % of the screen)
static const int ARENA_H  = SCREEN_H;                    // play-field uses the full screen height
static const int PANEL_X  = ARENA_W;                     // the control panel begins exactly where the arena ends
static const int PANEL_W  = SCREEN_W - ARENA_W;          // control panel gets the remaining pixels (right side)

// ---- 2.2 Maze geometry ----
static const float MAZE_CX      = ARENA_W / 2.0f;        // maze centre x, in arena-local pixels
static const float MAZE_CY      = ARENA_H / 2.0f;        // maze centre y, in arena-local pixels
static const int   N_RINGS      = 3;                     // how many concentric ring walls to cross
static const float RING_R[N_RINGS] = {100.0f, 175.0f, 250.0f};  // radius of each ring wall, in pixels
static const float RING_THICK   = 13.0f;                 // wall stroke width in px (~2 mm on a 154x86 mm 7" 1024x600 panel, ~6.6 px/mm)
static const float GAP_PX       = 130.0f;                // width of the opening in EVERY ring, in pixels (not degrees) -
                                                          // this keeps the opening equally wide close in and far out,
                                                          // roughly 3x the sprite's diameter so it never needs pixel-perfect aim
static const int   SPRITE_D     = 40;                    // sprite (ball) diameter in px
static const float SPRITE_R     = SPRITE_D / 2.0f;       // sprite radius, used to keep it fully clear of a wall before we call it "through"

// ---- 2.3 Driving feel (rolling-marble physics) ----
static const float ACCEL_GAIN    = 900.0f;               // px/s^2 of sprite acceleration per 1 g of shaped tilt
static const float DRAG_PER_S    = 2.4f;                 // fraction of velocity removed per second (rolling friction / damping)
static const float MAX_SPEED     = 260.0f;                // top speed cap in px/s, so tilting hard never feels out of control
static const float TILT_DEADZONE = 0.04f;                 // tilts smaller than this (g) are ignored (hand shake)
static const float TILT_MAX      = 0.60f;                 // tilts bigger than this (g) are clamped, so extreme tilts don't go crazy
static const float TILT_FILTER   = 0.30f;                 // low-pass filter strength (1.0 = no smoothing, small = very smooth)

// ---- 2.4 Timing ----
static const uint32_t TICK_MS = 33;                       // game loop period in ms (about 30 frames per second)

// ---- 2.5 Sensor mapping (change these if tilting feels sideways or backwards on your board) ----
#define TILT_H_DIRECTION     (+1.0f)                      // sensor-X tilt -> sprite moves left/right. -1.0f to reverse.
#define TILT_V_DIRECTION     (+1.0f)                      // sensor-Y tilt -> sprite moves up/down.    -1.0f to reverse.
#define ACC_G_PER_LSB        (0.000061f)                  // LSM6DS3 at +/-2 g full-scale: 0.061 mg per LSB
#define TILT_DEBUG           1                            // 1: print tilt values + progress to Serial twice a second

// ---- 2.6 Colours ----
static const uint32_t WALL_COLOR    = 0x263238;           // dark charcoal ring walls - reads clearly against a light background
static const uint32_t ARENA_BG      = 0xEDEDF2;           // soft light background (calmer than the racer's green grass)
static const uint32_t GOAL_GLOW     = 0xFFE082;           // faint ring just outside the outer wall, marks "you made it" territory
static const uint32_t SPRITE_COLORS[] = {0xE53935, 0x1E88E5, 0xFFD600, 0x43A047, 0xFF4FA3, 0xFF9800};  // red, blue, yellow, green, pink, orange
static const int      N_SPRITE_COLORS = sizeof(SPRITE_COLORS) / sizeof(SPRITE_COLORS[0]);

// ==================== SECTION 3 : GAME DATA (global variables) ====================

enum GameState { STATE_IDLE, STATE_PLAYING, STATE_WON };   // the three phases of the game

static GameState g_state = STATE_IDLE;                     // current phase of the game

static float g_px = 0.0f, g_py = 0.0f;                      // sprite position, in px, relative to the maze centre
static float g_vx = 0.0f, g_vy = 0.0f;                      // sprite velocity, in px/s
static int   g_rings_cleared = 0;                           // how many ring walls the sprite is currently outside of (0..N_RINGS)
static int   g_rings_shown = -1;                            // last value written to the ring-progress label (avoids needless redraws)
static float g_ring_gap_center[N_RINGS];                    // each ring's gap centre angle, in degrees (0 = +x/right, clockwise, matches lv_arc)

static float g_tilt_h_f = 0.0f, g_tilt_v_f = 0.0f;           // filtered horizontal / vertical tilt, in g
static float g_neutral_h = 0.0f, g_neutral_v = 0.0f;         // tilt pose captured as "neutral" at the moment START is tapped
static bool  g_imu_ok = false;                               // true if the LSM6DS3 was found at start-up
static bool  g_touch_ok = false;                             // true if the LVGL task ended up running WITH a working touch driver
static bool  g_imu_primed = false;                           // true after the first successful sensor read
static uint32_t g_imu_errors = 0;                            // counts failed sensor reads (shown in the debug print)

static uint32_t g_last_tick_ms = 0;                          // LVGL tick value at the previous game tick (for delta time)

static lv_obj_t *ui_arena      = NULL;                       // the 820x600 play-field container
static lv_obj_t *ui_panel      = NULL;                       // the 204x600 right-hand control panel
static lv_obj_t *ui_banner     = NULL;                       // big centre-screen message label
static lv_obj_t *ui_sprite     = NULL;                       // the coloured ball the child steers
static lv_obj_t *ui_ring_lbl   = NULL;                       // "RING x / 3" progress label on the panel
static lv_obj_t *ui_start_btn  = NULL;                       // START button
static lv_obj_t *ui_new_btn    = NULL;                       // NEW GAME button
static lv_obj_t *ui_ring_seg[N_RINGS][2];                    // up to 2 arc segments per ring (a ring wall almost always
                                                              // has to wrap past the 0-degree point, so it needs 2 pieces)
static int g_sprite_color_idx  = 0;                          // index into SPRITE_COLORS of the sprite's current colour

// ==================== SECTION 4 : SMALL MATH HELPERS ====================

static const float RAD2DEG = 57.29578f;                      // 180 / pi
static const float DEG2RAD = 0.0174533f;                     // pi / 180

static float clampf(float v, float lo, float hi)             // limits v to the range [lo, hi]
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int rnd(int lo, int hi)                                // random integer between lo and hi inclusive
{
    return lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
}

static float shape_tilt(float v)                              // turns a raw tilt (in g) into a clean control value
{
    if (v > TILT_DEADZONE) v -= TILT_DEADZONE;
    else if (v < -TILT_DEADZONE) v += TILT_DEADZONE;
    else v = 0.0f;
    return clampf(v, -TILT_MAX, TILT_MAX);
}

static float angle_deg(float x, float y)                      // angle of (x,y) in degrees, 0..360.
{                                                              // 0 = +x (right), 90 = +y (down on screen) - this matches
    float a = atan2f(y, x) * RAD2DEG;                         // LVGL's arc angle convention exactly, so maze angles and
    if (a < 0.0f) a += 360.0f;                                // drawn wall angles always agree with each other.
    return a;
}

static float angle_diff(float a, float b)                     // signed difference a-b, wrapped into (-180, 180]
{
    float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
    return d;
}

// ==================== SECTION 5 : MAZE GEOMETRY ====================

static float ring_half_gap_deg(int i)                         // half-angle (in degrees) of ring i's opening, sized so
{                                                              // every ring's opening is the same physical width (GAP_PX)
    return ((GAP_PX * 0.5f) / RING_R[i]) * RAD2DEG;           // regardless of how far out that ring is
}

static bool ring_angle_in_gap(int i, float angleDeg)          // true if angleDeg falls inside ring i's opening
{
    float half = ring_half_gap_deg(i);
    return fabsf(angle_diff(angleDeg, g_ring_gap_center[i])) <= half;
}

// ==================== SECTION 6 : POWER RAILS (unchanged from Tilt Racer) ====================

esp_err_t board_p4_ldo_init()                                 // switches on the ESP32-P4 internal LDOs used by the display and I2C parts
{
    esp_err_t err = ESP_OK;
    esp_ldo_channel_handle_t ldo3_handle = NULL;
    esp_ldo_channel_config_t ldo3_cfg = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    Serial.println("Initializing LDO3 to 2.5V...");
    err = esp_ldo_acquire_channel(&ldo3_cfg, &ldo3_handle);
    if (err != ESP_OK) {
        Serial.printf("LDO3 Power Error: %s\n", esp_err_to_name(err));
        return err;
    } else {
        Serial.println("LDO3 Power enabled successfully.");
    }
    esp_ldo_channel_handle_t ldo4_handle = NULL;
    esp_ldo_channel_config_t ldo4_cfg = {
        .chan_id = 4,
        .voltage_mv = 3300,
    };
    Serial.println("Initializing LDO4 to 3.3V...");
    err = esp_ldo_acquire_channel(&ldo4_cfg, &ldo4_handle);
    if (err != ESP_OK) {
        Serial.printf("LDO4 Power Error: %s\n", esp_err_to_name(err));
        return err;
    } else {
        Serial.println("LDO4 Power enabled successfully.");
    }
    return ESP_OK;
}

// ==================== SECTION 7 : MOTION SENSOR (unchanged from Tilt Racer) ====================

static bool imu_start(void)                                    // initialises I2C and the LSM6DS3TR-C, returns true on success
{
    i2c_init();                                                // installs the I2C driver on port 0 (SDA=18, SCL=19, 400 kHz) - also used by touch
    if (lsm6ds3_begin() != ESP_OK) return false;               // checks WHO_AM_I=0x6A, resets the chip and configures it; false if it fails
    esp_err_t err = i2c_write_reg(LSM6DS3_I2C_ADDRESS, CTRL1_XL_ADDRESS,
                                  (uint8_t)(LSM6DS3TRC_RATE_104HZ | LSM6DS3TRC_ACC_FSXL_2G | LSM6DS3TRC_ACC_BW0XL_400HZ));  // 104 Hz, +/-2 g
    return (err == ESP_OK);
}

static bool imu_read_accel(float *ax, float *ay)                // reads X and Y acceleration in g, returns false on I2C error
{
    uint8_t raw[6];
    esp_err_t err = i2c_read_reg(LSM6DS3_I2C_ADDRESS, OUTX_L_XL_ADDRESS, raw, 6);  // burst read X, Y, Z output registers
    if (err != ESP_OK) return false;
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    *ax = (float)x * ACC_G_PER_LSB;
    *ay = (float)y * ACC_G_PER_LSB;
    return true;
}

static void read_tilt(void)                                     // called every game tick: updates the filtered horizontal/vertical tilt
{
    float ax = 0.0f, ay = 0.0f;
    if (!g_imu_ok || !imu_read_accel(&ax, &ay)) {
        g_imu_errors++;
        return;                                                // keep the previous filtered values on a read error
    }
    float h_raw = ax * TILT_H_DIRECTION;                        // sensor X -> horizontal (left/right) control
    float v_raw = ay * TILT_V_DIRECTION;                        // sensor Y -> vertical (up/down) control
    if (!g_imu_primed) {
        g_tilt_h_f = h_raw;
        g_tilt_v_f = v_raw;
        g_imu_primed = true;
    } else {
        g_tilt_h_f += TILT_FILTER * (h_raw - g_tilt_h_f);       // exponential low-pass filter
        g_tilt_v_f += TILT_FILTER * (v_raw - g_tilt_v_f);
    }
}

// ==================== SECTION 8 : LVGL DRAWING HELPERS ====================

static lv_obj_t *make_rect(lv_obj_t *parent, int x, int y, int w, int h, uint32_t hex, int radius)  // creates a coloured rectangle
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_color(o, lv_color_hex(hex), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(o, radius, 0);
    return o;
}

static lv_obj_t *make_label(lv_obj_t *parent, const char *text, const lv_font_t *font, uint32_t hex)  // creates a coloured text label
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(hex), 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    return l;
}

static lv_obj_t *make_button(lv_obj_t *parent, int y, int h, uint32_t hex, uint32_t text_hex,
                             const lv_font_t *font, const char *text, lv_event_cb_t cb)  // creates a big rounded touch button
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 184, h);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, y);
    lv_obj_set_style_radius(btn, 26, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(hex), 0);
    lv_obj_set_style_bg_color(btn, lv_color_darken(lv_color_hex(hex), LV_OPA_30), LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x6B7280), LV_STATE_DISABLED);
    lv_obj_set_style_border_width(btn, 4, 0);
    lv_obj_set_style_border_color(btn, lv_color_white(), 0);
    lv_obj_set_style_shadow_width(btn, 14, 0);
    lv_obj_set_style_shadow_color(btn, lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(btn, LV_OPA_40, 0);
    lv_obj_set_style_shadow_ofs_y(btn, 6, 0);
    lv_obj_set_style_translate_y(btn, 4, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(btn, 2, LV_STATE_PRESSED);
    lv_obj_t *lbl = make_label(btn, text, font, text_hex);
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, NULL);
    return btn;
}

static void set_btn_enabled(lv_obj_t *btn, bool enabled)      // greys out / re-enables a button
{
    if (enabled) lv_obj_clear_state(btn, LV_STATE_DISABLED);
    else lv_obj_add_state(btn, LV_STATE_DISABLED);
}

static void show_banner(const char *text)                      // shows a big message in the middle of the arena
{
    lv_label_set_text(ui_banner, text);
    lv_obj_align(ui_banner, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(ui_banner, LV_OBJ_FLAG_HIDDEN);
}

static void hide_banner(void)                                  // hides the centre message
{
    lv_obj_add_flag(ui_banner, LV_OBJ_FLAG_HIDDEN);
}

// ==================== SECTION 9 : DRAWING THE RING WALLS ====================

static lv_obj_t *make_ring_segment(lv_obj_t *parent, float radius)  // creates one blank arc segment sized/centred for a given ring radius
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_remove_style_all(arc);                               // strip the default theme: no background track, no knob, no shadow
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);               // purely decorative - never intercepts touches
    int d = (int)(radius * 2.0f);
    lv_obj_set_size(arc, d, d);
    lv_obj_set_pos(arc, (int)(MAZE_CX - radius), (int)(MAZE_CY - radius));
    lv_arc_set_bg_angles(arc, 0, 360);                           // background track spans the full circle (it's invisible anyway)
    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);  // hide the background track completely
    lv_obj_set_style_arc_width(arc, (int16_t)RING_THICK, LV_PART_INDICATOR);  // wall stroke thickness
    lv_obj_set_style_arc_color(arc, lv_color_hex(WALL_COLOR), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR); // flat wall ends look cleaner than rounded caps
    return arc;
}

static void set_ring_segment(lv_obj_t *arc, float startDeg, float endDeg)  // shows one wall segment from startDeg to endDeg
{                                                                // requires 0 <= startDeg < endDeg <= 360 (no wraparound in one call)
    lv_arc_set_angles(arc, (int16_t)startDeg, (int16_t)endDeg);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_HIDDEN);
}

static void hide_ring_segment(lv_obj_t *arc)                    // hides an unused segment slot
{
    lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
}

static void build_ring_walls(void)                              // (re)lays out all ring wall segments for the CURRENT gap angles
{                                                                // each ring wall covers "everything except the gap", which almost
    for (int i = 0; i < N_RINGS; i++) {                         // always has to wrap past the 0-degree point, so it's drawn as up
        float half = ring_half_gap_deg(i);                      // to 2 separate arc segments that never individually wrap.
        float gapEnd = fmodf(g_ring_gap_center[i] + half + 360.0f, 360.0f);  // angle where the gap ends / the wall begins
        float wallLen = 360.0f - (half * 2.0f);                 // total angular length of wall to draw (everything but the gap)

        float segStart = gapEnd;                                // start drawing right after the gap
        float remaining = wallLen;
        for (int s = 0; s < 2; s++) {
            float segLen = fminf(remaining, 360.0f - segStart);  // this piece runs until either 'remaining' runs out or we hit 360
            if (segLen > 0.5f) {
                set_ring_segment(ui_ring_seg[i][s], segStart, segStart + segLen);
            } else {
                hide_ring_segment(ui_ring_seg[i][s]);
            }
            remaining -= segLen;
            segStart = 0.0f;                                    // any leftover wraps around and continues from angle 0
        }
    }
}

// ==================== SECTION 10 : BUILDING THE ARENA (left 80% of the screen) ====================

static void build_arena(lv_obj_t *scr)                          // creates every object of the play-field
{
    ui_arena = make_rect(scr, 0, 0, ARENA_W, ARENA_H, ARENA_BG, 0);  // light background; also clips its children to 820x600

    float glow_r = RING_R[N_RINGS - 1] + RING_THICK * 0.5f + 18.0f;  // faint "you made it" halo just outside the outer wall
    make_rect(ui_arena, (int)(MAZE_CX - glow_r), (int)(MAZE_CY - glow_r), (int)(glow_r * 2), (int)(glow_r * 2), GOAL_GLOW, LV_RADIUS_CIRCLE);

    for (int i = 0; i < N_RINGS; i++) {                          // ring wall objects (angles are filled in by build_ring_walls())
        for (int s = 0; s < 2; s++) {
            ui_ring_seg[i][s] = make_ring_segment(ui_arena, RING_R[i]);
        }
    }

    ui_sprite = make_rect(ui_arena, 0, 0, SPRITE_D, SPRITE_D, SPRITE_COLORS[0], LV_RADIUS_CIRCLE);  // the ball itself, on top of the rings
    lv_obj_set_style_border_width(ui_sprite, 3, 0);              // thin outline makes it pop against any ring colour behind it
    lv_obj_set_style_border_color(ui_sprite, lv_color_hex(0x1B1B1B), 0);

    ui_banner = make_label(ui_arena, "", &lv_font_montserrat_36, 0xFFFFFF);  // big centre message (hint, win banner)
    lv_obj_set_style_bg_color(ui_banner, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ui_banner, LV_OPA_60, 0);
    lv_obj_set_style_radius(ui_banner, 30, 0);
    lv_obj_set_style_pad_hor(ui_banner, 36, 0);
    lv_obj_set_style_pad_ver(ui_banner, 18, 0);

    if (!g_imu_ok) {                                             // tilt sensor missing - same warning style as Tilt Racer
        lv_obj_t *warn = make_label(ui_arena, "TILT SENSOR NOT FOUND", &lv_font_montserrat_24, 0xFFFFFF);
        lv_obj_set_style_bg_color(warn, lv_color_hex(0xD32F2F), 0);
        lv_obj_set_style_bg_opa(warn, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(warn, 8, 0);
        lv_obj_align(warn, LV_ALIGN_TOP_MID, 0, 8);
    }

    if (!g_touch_ok) {                                           // running in the no-touch fallback (buttons cannot be pressed)
        lv_obj_t *twarn = make_label(ui_arena, "TOUCH NOT WORKING - BUTTONS DISABLED", &lv_font_montserrat_24, 0xFFFFFF);
        lv_obj_set_style_bg_color(twarn, lv_color_hex(0xD32F2F), 0);
        lv_obj_set_style_bg_opa(twarn, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(twarn, 8, 0);
        lv_obj_align(twarn, LV_ALIGN_TOP_MID, 0, g_imu_ok ? 8 : 44);
    }

    lv_obj_move_foreground(ui_banner);                            // banner stays above the rings, sprite AND the warnings above
}

// ==================== SECTION 11 : GAME LOGIC (state machine) ====================

static void update_ring_label(void)                             // writes "RING x / 3" to the panel when it changes
{
    if (g_rings_cleared != g_rings_shown) {
        g_rings_shown = g_rings_cleared;
        lv_label_set_text_fmt(ui_ring_lbl, "RING %d / %d", g_rings_cleared, N_RINGS);
    }
}

static void reset_maze(void)                                     // shuffles a fresh maze and puts the sprite back in the centre
{
    g_px = g_py = 0.0f;
    g_vx = g_vy = 0.0f;
    g_rings_cleared = 0;
    g_rings_shown = -1;
    for (int i = 0; i < N_RINGS; i++) {
        g_ring_gap_center[i] = (float)rnd(0, 3599) / 10.0f;      // random gap angle per ring, 0.0 - 359.9 degrees
    }
    build_ring_walls();
    g_sprite_color_idx = rnd(0, N_SPRITE_COLORS - 1);            // a new random ball colour each game, just for fun
    lv_obj_set_style_bg_color(ui_sprite, lv_color_hex(SPRITE_COLORS[g_sprite_color_idx]), 0);
    update_ring_label();
}

static void set_state(GameState s)                               // switches game phase and updates buttons + banner accordingly
{
    g_state = s;
    switch (s) {
        case STATE_IDLE:
            set_btn_enabled(ui_start_btn, true);
            set_btn_enabled(ui_new_btn, true);
            show_banner("TILT TO ESCAPE\nTAP START");
            break;
        case STATE_PLAYING:
            set_btn_enabled(ui_start_btn, false);                // already going - START has nothing more to do
            set_btn_enabled(ui_new_btn, true);                   // NEW GAME can still abandon and reshuffle at any time
            hide_banner();
            break;
        case STATE_WON:
            set_btn_enabled(ui_start_btn, false);                // force a deliberate NEW GAME tap for a fresh maze
            set_btn_enabled(ui_new_btn, true);
            show_banner("YOU MADE IT!\nGREAT DRIVING!");
            break;
    }
}

static void do_win(void)                                         // called the instant the sprite clears the outer ring
{
    g_vx = g_vy = 0.0f;                                          // freeze physics right where it succeeded
    set_state(STATE_WON);
}

static void update_playing(float dt)                             // rolling-marble physics + ring collisions, one frame's worth
{
    float tiltH = shape_tilt(g_tilt_h_f - g_neutral_h);           // tilt relative to the pose held at START
    float tiltV = shape_tilt(g_tilt_v_f - g_neutral_v);

    g_vx += tiltH * ACCEL_GAIN * dt;                              // tilt accelerates the ball
    g_vy += tiltV * ACCEL_GAIN * dt;
    float drag = clampf(DRAG_PER_S * dt, 0.0f, 1.0f);             // rolling friction bleeds off speed each tick
    g_vx -= g_vx * drag;
    g_vy -= g_vy * drag;
    float spd = sqrtf(g_vx * g_vx + g_vy * g_vy);
    if (spd > MAX_SPEED) {                                        // cap top speed so a hard tilt never feels out of control
        float k = MAX_SPEED / spd;
        g_vx *= k;
        g_vy *= k;
    }

    float oldR = sqrtf(g_px * g_px + g_py * g_py);                // distance from centre before this tick's move
    float nx = g_px + g_vx * dt;                                  // where the ball WOULD be if nothing blocked it
    float ny = g_py + g_vy * dt;
    float nr = sqrtf(nx * nx + ny * ny);
    float theta = angle_deg(nx, ny);

    for (int i = 0; i < N_RINGS; i++) {                           // check every ring wall for a blocked crossing
        bool wasInside = oldR < RING_R[i];
        bool nowInside = nr < RING_R[i];
        if (wasInside == nowInside) continue;                     // didn't cross this ring this tick - nothing to check
        if (ring_angle_in_gap(i, theta)) continue;                 // crossed right at the gap - allowed, free passage
        nr = wasInside ? (RING_R[i] - RING_THICK * 0.5f - SPRITE_R - 1.0f)   // blocked: pin the radius just outside the
                        : (RING_R[i] + RING_THICK * 0.5f + SPRITE_R + 1.0f); // wall on whichever side it came from -
    }                                                               // the ball simply can't cross, but keeps sliding
                                                                     // sideways along the wall since only the RADIAL
    float rad = theta * DEG2RAD;                                   // distance was clamped, not the whole position.
    g_px = nr * cosf(rad);
    g_py = nr * sinf(rad);

    g_rings_cleared = 0;
    for (int i = 0; i < N_RINGS; i++) if (nr > RING_R[i]) g_rings_cleared++;

    if (nr > RING_R[N_RINGS - 1] + RING_THICK * 0.5f + SPRITE_R) {  // fully clear of the outer wall (including its own radius)
        do_win();
    }
}

// ==================== SECTION 12 : RENDERING ====================

static void render_sprite(void)                                   // moves the ball object to match g_px / g_py
{
    lv_obj_set_pos(ui_sprite, (int)(MAZE_CX + g_px - SPRITE_R), (int)(MAZE_CY + g_py - SPRITE_R));
}

// ==================== SECTION 13 : THE GAME TIMER (runs inside LVGL's own task, so no locking is needed) ====================

static void game_tick(lv_timer_t *timer)                          // called by LVGL every TICK_MS milliseconds
{
    LV_UNUSED(timer);
    uint32_t now = lv_tick_get();
    float dt = (float)(now - g_last_tick_ms) / 1000.0f;
    g_last_tick_ms = now;
    dt = clampf(dt, 0.001f, 0.1f);

    read_tilt();                                                   // always sample the accelerometer, even while idle, so the
                                                                    // pose captured at START is fresh and the debug print is live
    if (g_state == STATE_PLAYING) {
        update_playing(dt);
    }

    render_sprite();
    update_ring_label();
}

// ==================== SECTION 14 : BUTTON EVENT HANDLERS ====================

static void on_start_btn(lv_event_t *e)                           // START button
{
    LV_UNUSED(e);
    if (g_state != STATE_IDLE) return;                             // only arms from the idle/ready state
    g_neutral_h = g_tilt_h_f;                                       // whatever pose is held right now becomes "hands off level"
    g_neutral_v = g_tilt_v_f;
    g_vx = g_vy = 0.0f;
    set_state(STATE_PLAYING);
}

static void on_new_btn(lv_event_t *e)                             // NEW GAME button
{
    LV_UNUSED(e);
    reset_maze();                                                  // fresh gaps, fresh colour, sprite back in the centre
    set_state(STATE_IDLE);
}

// ==================== SECTION 15 : BUILDING THE CONTROL PANEL (right 20% of the screen) ====================

static void build_panel(lv_obj_t *scr)                            // creates the title, progress card and the two buttons
{
    ui_panel = make_rect(scr, PANEL_X, 0, PANEL_W, SCREEN_H, 0x14213D, 0);  // deep-navy panel background
    make_rect(ui_panel, 0, 0, 4, SCREEN_H, 0xFFC107, 0);            // 4 px golden divider line on the panel's left edge

    lv_obj_t *title = make_label(ui_panel, "TILT MAZE", &lv_font_montserrat_24, 0xFFC107);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    lv_obj_t *card = make_rect(ui_panel, 10, 50, 184, 96, 0x0B1226, 22);  // dark rounded progress card
    lv_obj_set_style_border_width(card, 3, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0xFFC107), 0);
    lv_obj_t *card_title = make_label(card, "PROGRESS", &lv_font_montserrat_16, 0xFFC107);
    lv_obj_align(card_title, LV_ALIGN_TOP_MID, 0, 10);
    ui_ring_lbl = make_label(card, "RING 0 / 3", &lv_font_montserrat_24, 0xFFFFFF);
    lv_obj_align(ui_ring_lbl, LV_ALIGN_CENTER, 0, 14);

    ui_start_btn = make_button(ui_panel, 280, 88, 0x00C853, 0xFFFFFF, &lv_font_montserrat_30,
                               LV_SYMBOL_PLAY " START", on_start_btn);     // big green START button
    ui_new_btn   = make_button(ui_panel, 390, 72, 0xFF9800, 0x1B1B1B, &lv_font_montserrat_24,
                               LV_SYMBOL_REFRESH " NEW GAME", on_new_btn); // orange NEW GAME button
}

// ==================== SECTION 16 : ARDUINO setup() ====================

void setup()                                                       // runs once at power-up
{
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);
    digitalWrite(32, 0);
    digitalWrite(33, 1);
    Serial.begin(115200);
    Serial.println("Initializing board");

    board_p4_ldo_init();                                            // LDO3 (2.5V MIPI) and LDO4 (3.3V I2C pull-ups)

    g_imu_ok = imu_start();                                         // bring up I2C port 0 + LSM6DS3TR-C, BEFORE board->init()/begin()
                                                                     // (the touch controller shares this bus, and its config skips
                                                                     // re-installing the driver - see esp_panel_board_custom_conf.h)
    Serial.println(g_imu_ok ? "IMU ready (104 Hz, +/-2 g)" : "IMU NOT FOUND - check wiring/address");

    Board *board = new Board();
    board->init();
    board->begin();

    if (board->getLCD() == nullptr) {                               // Board::init() never found a board config (LCD/touch/
        Serial.println("FATAL: no LCD was configured - Board::init() could not find a board configuration.");  // backlight
        Serial.println("       Check that esp_panel_board_custom_conf.h (and esp_panel_drivers_conf.h,");      // never got
        Serial.println("       esp_utils_conf.h, lv_conf.h) are physically inside THIS sketch's folder, next"); // set up at
        Serial.println("       to the .ino, and show up as their own tabs in the Arduino IDE - not just in");  // all, so
        Serial.println("       another sketch's folder. Halting here to avoid a crash on the missing LCD.");   // touching
        return;                                                      // board->getBacklight() etc. below would crash
    }

    Touch *tp = board->getTouch();
    if (tp == nullptr) {
        Serial.println("WARNING: touch is disabled - set ESP_PANEL_BOARD_USE_TOUCH to 1 in esp_panel_board_custom_conf.h");
    }

    bool lvgl_ready = lvgl_port_init(board->getLCD(), tp);
    if (!lvgl_ready) {
        Serial.println("WARNING: lvgl_port_init() returned false with touch enabled - the LVGL task was NOT created.");
        Serial.println("         Retrying with touch disabled so the display can still come up for diagnosis...");
        lvgl_ready = lvgl_port_init(board->getLCD(), nullptr);
        if (lvgl_ready) {
            Serial.println("         Retry OK: display + task are running WITHOUT touch. Touch registration is the fault -");
            Serial.println("         check the GT911 wiring, its I2C address, and its reset/interrupt pins.");
        } else {
            Serial.println("FATAL: display registration itself failed (not just touch). The screen cannot come up.");
            Serial.println("       Check the MIPI-DSI panel wiring and the LDO3/LDO4 power-up messages above.");
        }
        g_touch_ok = false;
    } else {
        Serial.println("LVGL port ready: display + task + touch are all running.");
        g_touch_ok = true;
    }
    if (board->getBacklight() != nullptr) {                          // guard: don't crash if backlight wasn't configured
        board->getBacklight()->setBrightness(0);                     // keep the screen dark while we build the UI
    }

    if (!lvgl_ready) {
        Serial.println("Halting UI setup. Fix the issue above, then re-flash.");
        return;
    }

    lvgl_port_lock(-1);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    randomSeed(esp_random());

    build_arena(scr);                                               // draw the play-field (rings, sprite, banner, warnings)
    build_panel(scr);                                                // draw the control panel (title, progress, buttons)
    reset_maze();                                                    // pick the first random maze + sprite colour
    set_state(STATE_IDLE);                                           // idle phase with hint banner

    g_last_tick_ms = lv_tick_get();
    lv_timer_create(game_tick, TICK_MS, NULL);                       // start the game loop
    lvgl_port_unlock();

    delay(200);
    if (board->getBacklight() != nullptr) {
        board->getBacklight()->setBrightness(100);
    }
    Serial.println("Tilt Maze ready");
}

// ==================== SECTION 17 : ARDUINO loop() ====================

void loop()                                                         // runs forever; the game itself runs in the LVGL task
{
#if TILT_DEBUG
    float raw_ax = 0.0f, raw_ay = 0.0f;
    bool raw_ok = g_imu_ok && imu_read_accel(&raw_ax, &raw_ay);
    Serial.printf("RAW ax=%+.3f g ay=%+.3f g (ok=%d)  |  filtered h=%+.2f g v=%+.2f g  |  neutral h=%+.2f v=%+.2f  |  imu_ok=%d primed=%d i2c_err=%u  |  state=%d rings=%d/%d\n",
                  raw_ax, raw_ay, (int)raw_ok, g_tilt_h_f, g_tilt_v_f, g_neutral_h, g_neutral_v,
                  (int)g_imu_ok, (int)g_imu_primed, (unsigned)g_imu_errors, (int)g_state, g_rings_cleared, N_RINGS);
#endif
    delay(500);
}
