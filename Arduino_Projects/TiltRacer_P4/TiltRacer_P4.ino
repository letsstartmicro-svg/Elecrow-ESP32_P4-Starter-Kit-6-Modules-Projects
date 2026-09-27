/*
 * ============================================================================================
 *  TILT RACER  -  Elecrow All-In-One Starter Kit for ESP32-P4
 *  Display + touch : taken from Lesson 10 (EK79007 MIPI-DSI panel 1024x600, GT911 touch, LVGL v8 port)
 *  Motion sensor   : taken from Lesson 14 (LSM6DS3TR-C on I2C port 0, SDA=GPIO18, SCL=GPIO19, addr 0x6B)
 * ============================================================================================
 *
 *  HOW TO SET UP THE SKETCH FOLDER (Arduino IDE)
 *    Put this file in a folder called  TiltRacer_P4  together with these files from your lessons:
 *      esp_panel_board_custom_conf.h  esp_panel_drivers_conf.h  esp_utils_conf.h  lv_conf.h
 *      lvgl_v8_port.cpp  lvgl_v8_port.h  bsp_i2c.cpp  bsp_i2c.h  bsp_lsm6ds3tr.cpp  bsp_lsm6ds3tr.h
 *
 *  ONE REQUIRED EDIT (the Lesson 14 copy of the board config switches touch OFF):
 *      esp_panel_board_custom_conf.h  ->  #define ESP_PANEL_BOARD_USE_TOUCH   (1)
 *    Leave everything else as-is (touch host SKIP_INIT_HOST stays 1 because bsp_i2c.cpp creates the I2C bus).
 *
 *  HOW TO PLAY
 *    1. Tap CAR COLOR to choose your car.   2. Tap START.   3. Hold the board like a tray and TILT it.
 *       One tilt direction steers the car up/down, the other tilt direction makes it faster/slower.
 *    Stay on the road!  Your score grows the longer (and faster) you drive.
 * ============================================================================================
 */

// ==================== SECTION 1 : INCLUDES & NAMESPACES ====================
#include "Arduino.h"                     // Arduino core: Serial, delay(), pinMode(), digitalWrite()
#include <esp_display_panel.hpp>         // ESP32_Display_Panel library: Board, LCD, Touch, Backlight classes (same as Lesson 10)
#include <lvgl.h>                        // LVGL v8 graphics library: objects, styles, labels, buttons, timers
#include <lvgl_v8_port.h>                // Espressif LVGL glue: lvgl_port_init(), lvgl_port_lock(), lvgl_port_unlock()
#include "esp_ldo_regulator.h"           // ESP32-P4 on-chip LDO regulator driver (needed for the MIPI PHY power rail)
#include "esp_random.h"                  // esp_random(): hardware random number generator (used for random tracks)
#include "bsp_i2c.h"                     // Lesson 14 I2C helper: i2c_init(), i2c_read_reg(), i2c_write_reg()
#include "bsp_lsm6ds3tr.h"               // Lesson 14 IMU driver: register addresses + lsm6ds3_begin()
#include <math.h>                        // sinf(), fabsf() maths functions

using namespace esp_panel::drivers;      // lets us write "LCD" / "Touch" instead of esp_panel::drivers::LCD / Touch
using namespace esp_panel::board;        // lets us write "Board" instead of esp_panel::board::Board

// ==================== SECTION 2 : TUNABLE SETTINGS (change these to change the feel of the game) ====================

// ---- 2.1 Screen layout ----
static const int SCREEN_W = 1024;                       // full panel width in pixels (ESP_PANEL_BOARD_WIDTH in the board config)
static const int SCREEN_H = 600;                        // full panel height in pixels (ESP_PANEL_BOARD_HEIGHT in the board config)
static const int ARENA_W  = 820;                        // play-field width: 820/1024 = 80.1 % of the screen (left side)
static const int ARENA_H  = SCREEN_H;                   // play-field uses the full screen height
static const int PANEL_X  = ARENA_W;                    // the control panel begins exactly where the arena ends
static const int PANEL_W  = SCREEN_W - ARENA_W;         // control panel gets the remaining 204 pixels (right side)

// ---- 2.2 Track shape ----
static const int   COL_W       = 12;                    // the road is drawn as vertical strips ("columns"), each 12 px wide
static const int   N_COLS      = ARENA_W / COL_W + 3;   // number of strips needed to cover the arena plus a little spare (71)
static const int   ROAD_H      = 270;                   // road thickness in pixels (wide = easy for a 4 year old)
static const int   KERB_W      = 10;                    // thickness of the red/white kerb stripes on the road edges
static const float MID_Y       = ARENA_H / 2.0f;        // vertical middle of the arena = centre of a straight road (300)
static const float BEND_A1     = 80.0f;                 // big bend: how far (px) the road swings up/down
static const float BEND_L1     = 240.0f;                // big bend: length scale (bigger number = longer, lazier curves)
static const float BEND_A2     = 30.0f;                 // small wiggle: swing in px
static const float BEND_L2     = 110.0f;                // small wiggle: length scale
static const float RAMP_START  = 700.0f;                // world x (px) where bends begin (before this the road is dead straight)
static const float RAMP_LEN    = 700.0f;                // distance over which bends fade in from nothing to full size

// ---- 2.3 Scenery pools (recycled objects that scroll past) ----
static const int N_BAND        = 7;                     // number of light-green grass stripes
static const int BAND_W        = 90;                    // width of one light stripe in px
static const int BAND_PERIOD   = 180;                   // distance between the start of one stripe and the next
static const int N_TREE        = 11;                    // number of bushes/trees
static const int TREE_GAP      = 90;                    // average spacing between trees (px)
static const int TREE_D_MIN    = 30;                    // smallest tree diameter (px)
static const int TREE_D_MAX    = 52;                    // largest tree diameter (px)
static const int N_DASH        = 9;                     // number of yellow centre-line dashes
static const int DASH_GAP      = 100;                   // distance between dashes (px)
static const int DASH_LEN      = 44;                    // dash length (px)
static const int DASH_H        = 6;                     // dash thickness (px)
static const int START_X       = 360;                   // world x of the chequered START line
static const int START_SQ_W    = 12;                    // width of one chequer square
static const int START_SQ_H    = 25;                    // height of one chequer square
static const int START_ROWS    = 10;                    // number of chequer rows (10 x 25 = 250 px, fits inside the kerbs)

// ---- 2.4 Cars ----
static const int   CAR_K       = 4;                     // car pixel scale: every car part is (units * CAR_K) pixels
static const int   CAR_W       = 20 * CAR_K;            // car width  = 80 px
static const int   CAR_H       = 11 * CAR_K;            // car height = 44 px
static const int   PLAYER_X0   = 190;                   // fixed screen x of the player's car centre
static const float TRACK_FOLLOW = 0.7f;                 // 1.0 = car follows road bends automatically, 0.0 = you must do all the steering
static const int   N_CARS      = 2;                     // how many cars are on the road: 1 player + (N_CARS - 1) AI cars
static const float LANE_OFF[N_CARS] = {-45.0f, 45.0f};  // start-grid lane offsets from the road centre, one per car colour

// ---- 2.5 Driving feel ----
static const float SPEED_BASE  = 320.0f;                // cruising speed (px/s) when the board is held at its neutral pose
static const float SPEED_MIN   = 200.0f;                // slowest allowed speed (car never fully stops)
static const float SPEED_MAX   = 520.0f;                // fastest allowed speed
static const float SPEED_GAIN  = 400.0f;                // extra px/s per 1 g of speed-tilt
static const float SPEED_EASE  = 3.0f;                  // how quickly speed follows the tilt (bigger = snappier)
static const float STEER_GAIN  = 700.0f;                // sideways px/s per 1 g of steering-tilt
static const float TILT_DEADZONE = 0.04f;              // tilts smaller than 0.04 g (~2 degrees) are ignored (hand shake)
static const float TILT_MAX    = 0.60f;                 // tilts bigger than 0.6 g (~37 degrees) count as 0.6 g
static const float TILT_FILTER = 0.30f;                 // low-pass filter strength (1.0 = no smoothing, small = very smooth)
static const float CRASH_MARGIN = 6.0f;                 // extra px the car may overlap the road edge before it counts as a crash
static const float SCORE_PER_PX = 0.04f;                // score points earned per pixel travelled (1 point per 25 px)

// ---- 2.6 AI (computer) cars ----
static const float AI_SPEED[N_CARS] = {340.0f, 310.0f};  // each AI car's own base speed (px/s), indexed by car colour
static const float AI_HOME[N_CARS]  = {560.0f, 430.0f};  // screen x each AI car tries to hover around
static const float AI_FREQ[N_CARS]  = {0.7f, 0.5f};      // how fast each AI car speeds up / slows down (rad/s)
static const float AI_WOBBLE   = 35.0f;                 // +/- px/s of speed variation so the AI cars overtake each other
static const float AI_SPRING   = 0.8f;                  // how strongly AI cars are pulled back towards their "home" x
static const float AI_PULL_MAX = 120.0f;                // maximum speed boost/brake from that pull (px/s)

// ---- 2.7 Timing ----
static const uint32_t TICK_MS       = 33;               // game loop period in ms (about 30 frames per second)
static const uint32_t COUNT_STEP_MS = 800;              // how long each of "3", "2", "1" is shown
static const uint32_t GO_SHOW_MS    = 700;              // how long "GO!" stays on screen after the race begins

// ---- 2.8 Sensor mapping (change these if tilting feels sideways or backwards on your board) ----
#define STEER_USES_SENSOR_Y  1                          // 1: sensor-Y tilt steers, sensor-X tilt = speed.  0: swapped
#define STEER_DIRECTION      (+1.0f)                    // set to -1.0f to reverse the steering direction
#define SPEED_DIRECTION      (+1.0f)                    // set to -1.0f to reverse which tilt is "faster"
#define ACC_G_PER_LSB        (0.000061f)                // LSM6DS3 at +/-2 g full-scale: 0.061 mg per LSB (see bsp_lsm6ds3tr.cpp)
#define TILT_DEBUG           1                          // 1: print tilt values to the Serial Monitor twice a second (for calibrating)
#define REQUIRE_COLOR_PICK   1                          // 1: START stays grey until a car colour has been chosen once

// ---- 2.9 Car colours (RGB hex), names and label text colours ----
static const uint32_t CAR_COLORS[N_CARS] = {0xE53935, 0x1E88E5};  // red, blue
static const char    *CAR_NAMES[N_CARS]  = {"RED", "BLUE"};          // names shown on the colour button
static const uint32_t CAR_TEXT[N_CARS]   = {0xFFFFFF, 0xFFFFFF};  // readable label colour on each button colour
static const uint32_t TREE_COLORS[4] = {0x2E7D32, 0x43A047, 0x00897B, 0xF57C00}; // bush colours: greens, teal, autumn orange

// ==================== SECTION 3 : GAME DATA (global variables) ====================

enum GameState { STATE_IDLE, STATE_COUNTDOWN, STATE_RACING, STATE_CRASHED };  // the four phases of the game

static GameState g_state = STATE_IDLE;                  // current phase of the game
static bool      g_color_chosen = false;                // becomes true when the player has tapped CAR COLOR at least once
static int       g_player = 0;                          // index (0-3) of the car colour the player drives
static int       g_best = 0;                            // best score since power-up (kept in RAM only)
static int       g_score_shown = -1;                    // last score number written to the label (avoids needless redraws)
static int       g_best_shown = -1;                     // last best-score number written to its label

static float g_camera = 0.0f;                           // how far the world has scrolled = distance the player has driven (px)
static float g_speed  = 0.0f;                           // player's current speed in px/s
static float g_off    = 0.0f;                           // player's sideways offset from the road centre in px
static float g_score  = 0.0f;                           // player's score (float so tiny increments accumulate)
static float g_ai_dist[N_CARS] = {0, 0};                // how far each AI car has driven (px)
static float g_ai_t   = 0.0f;                           // seconds since the race started (drives AI speed wobble)
static float g_phase1 = 0.0f;                           // random phase of the big bend (new random track every race)
static float g_phase2 = 0.0f;                           // random phase of the small wiggle

static float g_steer_f = 0.0f;                          // filtered steering tilt in g
static float g_speed_f = 0.0f;                          // filtered speed tilt in g
static float g_neutral_steer = 0.0f;                    // steering tilt captured at "GO" = the "hands-off" pose
static float g_neutral_speed = 0.0f;                    // speed tilt captured at "GO"
static bool  g_imu_ok = false;                          // true if the LSM6DS3 was found at start-up
static bool  g_touch_ok = false;                        // true if the LVGL task ended up running WITH a working touch driver
static bool  g_imu_primed = false;                      // true after the first successful sensor read
static uint32_t g_imu_errors = 0;                       // counts failed sensor reads (shown in the debug print)

static uint32_t g_last_tick_ms = 0;                     // LVGL tick value at the previous game tick (for delta time)
static uint32_t g_state_ms = 0;                         // LVGL tick value when the current state began
static uint32_t g_banner_off_ms = 0;                    // when to hide the "GO!" banner (0 = not scheduled)
static int      g_last_count = -1;                      // last countdown number shown (3,2,1)

static lv_obj_t *ui_arena = NULL;                       // the 820x600 play-field container
static lv_obj_t *ui_panel = NULL;                       // the 204x600 right-hand control panel
static lv_obj_t *ui_banner = NULL;                      // big centre-screen message label
static lv_obj_t *ui_you_tag = NULL;                     // yellow "YOU" label floating above the player's car
static lv_obj_t *ui_score_lbl = NULL;                   // big score number label
static lv_obj_t *ui_best_lbl = NULL;                    // "BEST" label
static lv_obj_t *ui_color_btn = NULL;                   // CAR COLOR button
static lv_obj_t *ui_color_lbl = NULL;                   // text label inside the CAR COLOR button
static lv_obj_t *ui_start_btn = NULL;                   // START button
static lv_obj_t *ui_new_btn = NULL;                     // NEW GAME button
static lv_obj_t *ui_preview_body = NULL;                // body rectangle of the little preview car in the garage card
static lv_obj_t *ui_start_line = NULL;                  // chequered start line container
static lv_obj_t *ui_car[N_CARS] = {NULL, NULL};         // the race cars (one per colour)

static lv_obj_t *ui_band[N_BAND];                       // light-green grass stripe objects
static float     band_wx[N_BAND];                       // world x of each stripe
static lv_obj_t *ui_tree[N_TREE];                       // bush/tree objects
static float     tree_wx[N_TREE];                       // world x of each tree
static int       tree_y[N_TREE];                        // screen y of each tree (fixed for its lifetime)
static lv_obj_t *ui_dash[N_DASH];                       // centre-line dash objects
static float     dash_wx[N_DASH];                       // world x of each dash
static int       dash_y[N_DASH];                        // screen y of each dash (fixed for its lifetime)
static lv_obj_t *ui_col[N_COLS];                        // road strip objects
static int       col_g[N_COLS];                         // which world column number each strip currently shows (-1 = none)
static int       col_y[N_COLS];                         // screen y of each road strip

// ==================== SECTION 4 : SMALL MATH HELPERS ====================

static float clampf(float v, float lo, float hi)        // limits v to the range [lo, hi]
{                                                       // begin of function body
    if (v < lo) return lo;                              // below the range -> return the lower limit
    if (v > hi) return hi;                              // above the range -> return the upper limit
    return v;                                           // inside the range -> unchanged
}                                                       // end of clampf

static int rnd(int lo, int hi)                          // random integer between lo and hi inclusive
{                                                       // begin of function body
    return lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));  // hardware random number folded into the range
}                                                       // end of rnd

static float shape_tilt(float v)                        // turns a raw tilt (in g) into a clean control value
{                                                       // begin of function body
    if (v > TILT_DEADZONE) v -= TILT_DEADZONE;          // tilt beyond the dead-zone: subtract the dead-zone so control starts from 0
    else if (v < -TILT_DEADZONE) v += TILT_DEADZONE;    // same for negative tilt
    else v = 0.0f;                                      // inside the dead-zone: treat as perfectly level
    return clampf(v, -TILT_MAX, TILT_MAX);              // cap at +/- TILT_MAX so extreme tilts do not go crazy
}                                                       // end of shape_tilt

// ==================== SECTION 5 : TRACK GEOMETRY ====================

static float road_center(float wx)                      // y position of the road centre at world x = wx
{                                                       // begin of function body
    float ramp = clampf((wx - RAMP_START) / RAMP_LEN, 0.0f, 1.0f);  // 0 before RAMP_START, rising to 1 -> bends fade in
    float bend = BEND_A1 * sinf(wx / BEND_L1 + g_phase1)            // big slow S-bend
               + BEND_A2 * sinf(wx / BEND_L2 + g_phase2);           // plus a smaller quicker wiggle
    return MID_Y + ramp * bend;                                     // straight at the start, curvy later
}                                                       // end of road_center

// ==================== SECTION 6 : POWER RAILS (copied from Lesson 10) ====================

esp_err_t board_p4_ldo_init()                           // switches on the ESP32-P4 internal LDOs used by the display and I2C parts
{                                                       // begin of function body
    esp_err_t err = ESP_OK;                             // holds the result of each ESP-IDF call
    esp_ldo_channel_handle_t ldo3_handle = NULL;        // handle returned for LDO channel 3
    esp_ldo_channel_config_t ldo3_cfg = {               // configuration for LDO channel 3
        .chan_id = 3,                                   // LDO channel 3 feeds the MIPI D-PHY on this board
        .voltage_mv = 2500,                             // 2.5 V required by the MIPI D-PHY
    };                                                  // end of ldo3_cfg
    Serial.println("Initializing LDO3 to 2.5V...");     // progress message on the serial monitor
    err = esp_ldo_acquire_channel(&ldo3_cfg, &ldo3_handle);  // ask ESP-IDF to switch LDO3 on at 2.5 V
    if (err != ESP_OK) {                                // if that failed...
        Serial.printf("LDO3 Power Error: %s\n", esp_err_to_name(err));  // ...print the reason
        return err;                                     // ...and give up
    } else {                                            // otherwise...
        Serial.println("LDO3 Power enabled successfully.");  // ...report success
    }                                                   // end of LDO3 result check
    esp_ldo_channel_handle_t ldo4_handle = NULL;        // handle returned for LDO channel 4
    esp_ldo_channel_config_t ldo4_cfg = {               // configuration for LDO channel 4
        .chan_id = 4,                                   // LDO channel 4 powers the I2C / touch pull-ups on this board
        .voltage_mv = 3300,                             // 3.3 V
    };                                                  // end of ldo4_cfg
    Serial.println("Initializing LDO4 to 3.3V...");     // progress message
    err = esp_ldo_acquire_channel(&ldo4_cfg, &ldo4_handle);  // switch LDO4 on at 3.3 V
    if (err != ESP_OK) {                                // if that failed...
        Serial.printf("LDO4 Power Error: %s\n", esp_err_to_name(err));  // ...print the reason
        return err;                                     // ...and give up
    } else {                                            // otherwise...
        Serial.println("LDO4 Power enabled successfully.");  // ...report success
    }                                                   // end of LDO4 result check
    return ESP_OK;                                      // both rails are up
}                                                       // end of board_p4_ldo_init

// ==================== SECTION 7 : MOTION SENSOR (Lesson 14 driver + one fast burst read) ====================

static bool imu_start(void)                             // initialises I2C and the LSM6DS3TR-C, returns true on success
{                                                       // begin of function body
    i2c_init();                                         // Lesson 14: installs the I2C driver on port 0 (SDA=18, SCL=19, 400 kHz) - also used by touch
    if (lsm6ds3_begin() != ESP_OK) return false;        // Lesson 14: checks WHO_AM_I=0x6A, resets the chip and configures it; false if it fails
    // The Lesson 14 setup leaves the accelerometer at only 12.5 Hz. Rewrite CTRL1_XL (register 0x10) for a game:
    // bits 7..4 = ODR_XL (0x40 = 104 Hz), bits 3..2 = FS_XL (0x00 = +/-2 g), bits 1..0 = anti-alias filter bandwidth (0x01 = 400 Hz)
    esp_err_t err = i2c_write_reg(LSM6DS3_I2C_ADDRESS, CTRL1_XL_ADDRESS,                                                // (statement continues on the next line)
                                  (uint8_t)(LSM6DS3TRC_RATE_104HZ | LSM6DS3TRC_ACC_FSXL_2G | LSM6DS3TRC_ACC_BW0XL_400HZ));  // one register write = 0x41
    return (err == ESP_OK);                             // true if the write was acknowledged by the chip
}                                                       // end of imu_start

static bool imu_read_accel(float *ax, float *ay)        // reads X and Y acceleration in g, returns false on I2C error
{                                                       // begin of function body
    uint8_t raw[6];                                     // 6 bytes = X_L, X_H, Y_L, Y_H, Z_L, Z_H (little-endian 16-bit each)
    // One I2C transaction: write register address 0x28 (OUTX_L_XL) then read 6 bytes. The chip auto-increments the address
    // (IF_INC bit in CTRL3_C defaults to 1), so we get X, Y and Z in ~0.3 ms instead of the 60 ms the byte-by-byte helper takes.
    esp_err_t err = i2c_read_reg(LSM6DS3_I2C_ADDRESS, OUTX_L_XL_ADDRESS, raw, 6);  // burst read from the accelerometer output registers
    if (err != ESP_OK) return false;                    // bus error: tell the caller to keep its old values
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);      // combine high byte and low byte into a signed 16-bit X value
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);      // same for Y
    *ax = (float)x * ACC_G_PER_LSB;                     // convert raw counts to g (1.0 = Earth gravity)
    *ay = (float)y * ACC_G_PER_LSB;                     // convert Y the same way
    return true;                                        // success
}                                                       // end of imu_read_accel

static void read_tilt(void)                             // called every game tick: updates the filtered steering and speed tilts
{                                                       // begin of function body
    float ax = 0.0f, ay = 0.0f;                         // raw X / Y acceleration in g
    if (!g_imu_ok || !imu_read_accel(&ax, &ay)) {       // no sensor, or the read failed...
        g_imu_errors++;                                 // ...count the error
        return;                                         // ...and keep the previous filtered values
    }                                                   // end of error check
#if STEER_USES_SENSOR_Y                                 // choose which sensor axis does what (compile-time switch)
    float s_raw = ay * STEER_DIRECTION;                 // sensor Y -> steering
    float v_raw = ax * SPEED_DIRECTION;                 // sensor X -> speed
#else                                                   // the other wiring
    float s_raw = ax * STEER_DIRECTION;                 // sensor X -> steering
    float v_raw = ay * SPEED_DIRECTION;                 // sensor Y -> speed
#endif                                                  // end of axis selection
    if (!g_imu_primed) {                                // very first sample?
        g_steer_f = s_raw;                              // start the filter exactly at the first reading (no slow warm-up)
        g_speed_f = v_raw;                              // same for speed
        g_imu_primed = true;                            // remember that we have started
    } else {                                            // later samples
        g_steer_f += TILT_FILTER * (s_raw - g_steer_f); // exponential low-pass filter: move a fraction of the way to the new value
        g_speed_f += TILT_FILTER * (v_raw - g_speed_f); // same for speed
    }                                                   // end of filter update
}                                                       // end of read_tilt

// ==================== SECTION 8 : LVGL DRAWING HELPERS ====================

static lv_obj_t *make_rect(lv_obj_t *parent, int x, int y, int w, int h, uint32_t hex, int radius)  // creates a coloured rectangle
{                                                       // begin of function body
    lv_obj_t *o = lv_obj_create(parent);                // create a base object inside 'parent'
    lv_obj_remove_style_all(o);                         // remove theme styling (no border/padding/shadow/scrollbar) - fastest to draw
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);       // never scroll (children/positions must stay put)
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);        // ignore touches so the arena can never steal a button press
    lv_obj_set_pos(o, x, y);                            // position relative to the parent's top-left corner
    lv_obj_set_size(o, w, h);                           // width and height in pixels
    lv_obj_set_style_bg_color(o, lv_color_hex(hex), 0); // fill colour (selector 0 = main part, default state)
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);        // fully opaque fill (solid fills are the cheapest thing to render)
    lv_obj_set_style_radius(o, radius, 0);              // corner radius in px (LV_RADIUS_CIRCLE makes a circle)
    return o;                                           // hand the object back to the caller
}                                                       // end of make_rect

static lv_obj_t *create_car(lv_obj_t *parent, uint32_t body_hex, int k, lv_obj_t **body_out)  // builds one car sprite from 10 rectangles
{                                                       // begin of function body
    lv_obj_t *car = make_rect(parent, 0, 0, 20 * k, 11 * k, 0x000000, 0);  // invisible container, 20x11 "units" (unit = k pixels)
    lv_obj_set_style_bg_opa(car, LV_OPA_TRANSP, 0);     // container itself is transparent - only its children are visible
    make_rect(car, 3 * k, 0, 4 * k, 2 * k, 0x1A1A1A, k / 2);       // rear wheel, top side
    make_rect(car, 3 * k, 9 * k, 4 * k, 2 * k, 0x1A1A1A, k / 2);   // rear wheel, bottom side
    make_rect(car, 13 * k, 0, 4 * k, 2 * k, 0x1A1A1A, k / 2);      // front wheel, top side
    make_rect(car, 13 * k, 9 * k, 4 * k, 2 * k, 0x1A1A1A, k / 2);  // front wheel, bottom side
    lv_obj_t *body = make_rect(car, 0, 1 * k, 20 * k, 9 * k, body_hex, 3 * k);  // main coloured body with rounded corners
    lv_obj_set_style_border_width(body, 2, 0);          // thin outline gives a cartoon look
    lv_obj_set_style_border_color(body, lv_color_hex(0x1B1B1B), 0);  // near-black outline colour
    make_rect(car, 0, 1 * k, 2 * k, 9 * k, 0x263238, k / 2);         // dark rear spoiler across the back
    make_rect(car, 6 * k, 3 * k, 8 * k, 5 * k, 0xFAFAFA, 2 * k);     // white cabin / roof
    make_rect(car, 10 * k, 3 * k, 4 * k, 5 * k, 0x81D4FA, k);        // light-blue windscreen on the front of the cabin
    make_rect(car, 19 * k, 2 * k, 1 * k, 2 * k, 0xFFF59D, k / 2);    // left headlight
    make_rect(car, 19 * k, 7 * k, 1 * k, 2 * k, 0xFFF59D, k / 2);    // right headlight
    if (body_out) *body_out = body;                     // let the caller keep a pointer to the body (used to recolour the preview car)
    return car;                                         // return the finished car container
}                                                       // end of create_car

static lv_obj_t *make_label(lv_obj_t *parent, const char *text, const lv_font_t *font, uint32_t hex)  // creates a coloured text label
{                                                       // begin of function body
    lv_obj_t *l = lv_label_create(parent);              // create the label inside 'parent'
    lv_label_set_text(l, text);                         // set its text
    lv_obj_set_style_text_font(l, font, 0);             // choose the font (all Montserrat sizes are enabled in lv_conf.h)
    lv_obj_set_style_text_color(l, lv_color_hex(hex), 0);  // text colour
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);  // centre multi-line text
    return l;                                           // return the label
}                                                       // end of make_label

static lv_obj_t *make_button(lv_obj_t *parent, int y, int h, uint32_t hex, uint32_t text_hex,                           // (statement continues on the next line)
                             const lv_font_t *font, const char *text, lv_event_cb_t cb)  // creates a big rounded touch button
{                                                       // begin of function body
    lv_obj_t *btn = lv_btn_create(parent);              // create an LVGL button
    lv_obj_set_size(btn, 184, h);                       // 184 px wide (fits the 204 px panel), h px tall
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, y);          // centred horizontally, y pixels from the top of the panel
    lv_obj_set_style_radius(btn, 26, 0);                // very round corners look friendly
    lv_obj_set_style_bg_color(btn, lv_color_hex(hex), 0);  // normal colour
    lv_obj_set_style_bg_color(btn, lv_color_darken(lv_color_hex(hex), LV_OPA_30), LV_STATE_PRESSED);  // darker while a finger is on it
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x6B7280), LV_STATE_DISABLED);  // grey when the button is not allowed right now
    lv_obj_set_style_border_width(btn, 4, 0);           // chunky white outline
    lv_obj_set_style_border_color(btn, lv_color_white(), 0);  // outline colour
    lv_obj_set_style_shadow_width(btn, 14, 0);          // soft drop-shadow blur size
    lv_obj_set_style_shadow_color(btn, lv_color_black(), 0);  // shadow colour
    lv_obj_set_style_shadow_opa(btn, LV_OPA_40, 0);     // shadow strength
    lv_obj_set_style_shadow_ofs_y(btn, 6, 0);           // shadow sits 6 px below the button
    lv_obj_set_style_translate_y(btn, 4, LV_STATE_PRESSED);   // button sinks 4 px when pressed
    lv_obj_set_style_shadow_ofs_y(btn, 2, LV_STATE_PRESSED);  // shadow shrinks when pressed (looks like a real push)
    lv_obj_t *lbl = make_label(btn, text, font, text_hex);    // text inside the button
    lv_obj_center(lbl);                                 // centre the text in the button
    lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, NULL);     // call 'cb' the instant a finger touches the button (forgiving for small hands)
    return btn;                                         // return the button
}                                                       // end of make_button

static void set_btn_enabled(lv_obj_t *btn, bool enabled)  // greys out / re-enables a button
{                                                       // begin of function body
    if (enabled) lv_obj_clear_state(btn, LV_STATE_DISABLED);  // remove the DISABLED state -> normal colours, touch works
    else lv_obj_add_state(btn, LV_STATE_DISABLED);      // add the DISABLED state -> grey, touch ignored
}                                                       // end of set_btn_enabled

static void show_banner(const char *text)               // shows a big message in the middle of the arena
{                                                       // begin of function body
    lv_label_set_text(ui_banner, text);                 // change the banner text
    lv_obj_align(ui_banner, LV_ALIGN_CENTER, 0, 0);     // re-centre because the text width may have changed
    lv_obj_clear_flag(ui_banner, LV_OBJ_FLAG_HIDDEN);   // make sure it is visible
}                                                       // end of show_banner

static void hide_banner(void)                           // hides the centre message
{                                                       // begin of function body
    lv_obj_add_flag(ui_banner, LV_OBJ_FLAG_HIDDEN);     // hidden objects are skipped by the renderer
}                                                       // end of hide_banner

// ==================== SECTION 9 : SCENERY PLACEMENT (called when an object is created or recycled) ====================

static void place_tree(int i)                           // gives tree i a fresh size, colour and side of the road
{                                                       // begin of function body
    int d = rnd(TREE_D_MIN, TREE_D_MAX);                // random diameter
    int side = (esp_random() & 1) ? 1 : -1;             // +1 = below the road, -1 = above the road (random)
    int gap = rnd(10, 60);                              // random distance between the kerb and the tree
    float cy = road_center(tree_wx[i]) + side * (ROAD_H / 2.0f + gap + d / 2.0f);  // tree centre y: road centre +/- half road + gap + radius
    tree_y[i] = (int)(cy - d / 2.0f);                   // convert centre to top-left y
    lv_obj_set_size(ui_tree[i], d, d);                  // apply the new size
    lv_obj_set_style_bg_color(ui_tree[i], lv_color_hex(TREE_COLORS[rnd(0, 3)]), 0);  // random bush colour
}                                                       // end of place_tree

static void place_dash(int i)                           // computes the y position of centre-line dash i
{                                                       // begin of function body
    dash_y[i] = (int)(road_center(dash_wx[i] + DASH_LEN / 2.0f) - DASH_H / 2.0f);  // sit on the road centre line at the dash middle
}                                                       // end of place_dash

// ==================== SECTION 10 : BUILDING THE ARENA (left 80 % of the screen) ====================

static void build_arena(lv_obj_t *scr)                  // creates every object of the play-field, back to front
{                                                       // begin of function body
    ui_arena = make_rect(scr, 0, 0, ARENA_W, ARENA_H, 0x2FB84F, 0);  // dark-green grass base; it also clips its children to 820x600

    for (int i = 0; i < N_BAND; i++) {                  // light-green stripes: alternating stripes create the feeling of speed
        ui_band[i] = make_rect(ui_arena, 0, 0, BAND_W, ARENA_H, 0x3CD35F, 0);  // one full-height light stripe
        band_wx[i] = (float)(i * BAND_PERIOD);          // spread the stripes along the world
    }                                                   // end of band loop

    for (int i = 0; i < N_TREE; i++) {                  // bushes on the grass
        ui_tree[i] = make_rect(ui_arena, 0, 0, 40, 40, 0x2E7D32, LV_RADIUS_CIRCLE);  // a circle (size/colour are set by place_tree)
        lv_obj_set_style_border_width(ui_tree[i], 4, 0);  // darker rim
        lv_obj_set_style_border_color(ui_tree[i], lv_color_black(), 0);  // rim colour
        lv_obj_set_style_border_opa(ui_tree[i], LV_OPA_30, 0);  // rim is only 30 % black -> looks like a shaded edge
        tree_wx[i] = 40.0f + i * TREE_GAP + rnd(0, 30); // spread trees along the world with a little randomness
    }                                                   // end of tree loop

    for (int j = 0; j < N_COLS; j++) {                  // the road: N_COLS narrow vertical strips
        ui_col[j] = make_rect(ui_arena, 0, 0, COL_W, ROAD_H, 0x4B5563, 0);  // grey tarmac strip, full road thickness
        lv_obj_set_style_border_width(ui_col[j], KERB_W, 0);  // border = the kerb
        lv_obj_set_style_border_side(ui_col[j], LV_BORDER_SIDE_TOP | LV_BORDER_SIDE_BOTTOM, 0);  // kerb only on the top and bottom edges
        col_g[j] = -1;                                  // -1 = "not yet assigned a world column" (forces styling on first frame)
        col_y[j] = 0;                                   // initial y (set properly when assigned)
    }                                                   // end of road loop

    for (int i = 0; i < N_DASH; i++) {                  // yellow dashes down the middle of the road
        ui_dash[i] = make_rect(ui_arena, 0, 0, DASH_LEN, DASH_H, 0xFFEB3B, 2);  // one small yellow rounded bar
        dash_wx[i] = 30.0f + i * DASH_GAP;              // spread dashes along the world
    }                                                   // end of dash loop

    ui_start_line = make_rect(ui_arena, START_X, (int)MID_Y - (START_ROWS * START_SQ_H) / 2,                            // (statement continues on the next line)
                              2 * START_SQ_W, START_ROWS * START_SQ_H, 0xFFFFFF, 0);  // container for the chequered flag pattern
    for (int r = 0; r < START_ROWS; r++) {              // loop over chequer rows
        for (int c = 0; c < 2; c++) {                   // two columns of squares
            uint32_t hex = ((r + c) & 1) ? 0x111111 : 0xFFFFFF;  // alternate black and white like a chess board
            make_rect(ui_start_line, c * START_SQ_W, r * START_SQ_H, START_SQ_W, START_SQ_H, hex, 0);  // one chequer square
        }                                               // end of column loop
    }                                                   // end of row loop

    for (int i = 0; i < N_CARS; i++) {                  // the race cars, one per colour
        ui_car[i] = create_car(ui_arena, CAR_COLORS[i], CAR_K, NULL);  // build the sprite at game scale
    }                                                   // end of car loop

    ui_you_tag = make_label(ui_arena, "YOU", &lv_font_montserrat_16, 0x000000);  // small tag that marks the player's car
    lv_obj_set_style_bg_color(ui_you_tag, lv_color_hex(0xFFEB3B), 0);  // yellow background
    lv_obj_set_style_bg_opa(ui_you_tag, LV_OPA_COVER, 0);  // solid background
    lv_obj_set_style_radius(ui_you_tag, 10, 0);         // rounded pill shape
    lv_obj_set_style_pad_hor(ui_you_tag, 10, 0);        // left/right padding
    lv_obj_set_style_pad_ver(ui_you_tag, 3, 0);         // top/bottom padding
    lv_obj_add_flag(ui_you_tag, LV_OBJ_FLAG_HIDDEN);    // hidden until a colour is chosen

    ui_banner = make_label(ui_arena, "", &lv_font_montserrat_48, 0xFFFFFF);  // big message label (countdown, hints, crash)
    lv_obj_set_style_bg_color(ui_banner, lv_color_black(), 0);  // dark backing so text is readable on any background
    lv_obj_set_style_bg_opa(ui_banner, LV_OPA_60, 0);   // 60 % opaque (semi-transparent)
    lv_obj_set_style_radius(ui_banner, 30, 0);          // very rounded corners
    lv_obj_set_style_pad_hor(ui_banner, 36, 0);         // left/right padding inside the banner
    lv_obj_set_style_pad_ver(ui_banner, 18, 0);         // top/bottom padding inside the banner

    if (!g_imu_ok) {                                    // if the tilt sensor was not found at start-up...
        lv_obj_t *warn = make_label(ui_arena, "TILT SENSOR NOT FOUND", &lv_font_montserrat_24, 0xFFFFFF);  // ...show a warning
        lv_obj_set_style_bg_color(warn, lv_color_hex(0xD32F2F), 0);  // red background
        lv_obj_set_style_bg_opa(warn, LV_OPA_COVER, 0); // solid
        lv_obj_set_style_pad_all(warn, 8, 0);           // padding
        lv_obj_align(warn, LV_ALIGN_TOP_MID, 0, 8);     // top-centre of the arena
    }                                                   // end of IMU warning

    if (!g_touch_ok) {                                  // if we are running in the no-touch fallback (buttons cannot be pressed)...
        lv_obj_t *twarn = make_label(ui_arena, "TOUCH NOT WORKING - BUTTONS DISABLED", &lv_font_montserrat_24, 0xFFFFFF);  // ...show a warning
        lv_obj_set_style_bg_color(twarn, lv_color_hex(0xD32F2F), 0);  // red background
        lv_obj_set_style_bg_opa(twarn, LV_OPA_COVER, 0);  // solid
        lv_obj_set_style_pad_all(twarn, 8, 0);          // padding
        lv_obj_align(twarn, LV_ALIGN_TOP_MID, 0, g_imu_ok ? 8 : 44);  // stack below the IMU warning if both are showing
    }                                                   // end of touch warning
}                                                       // end of build_arena

// ==================== SECTION 11 : GAME LOGIC (state machine) ====================

static void update_score_label(void)                    // writes the score/best numbers to the panel when they change
{                                                       // begin of function body
    int s = (int)g_score;                               // whole-number score
    if (s != g_score_shown) {                           // only touch LVGL when the number actually changed
        g_score_shown = s;                              // remember it
        lv_label_set_text_fmt(ui_score_lbl, "%d", s);   // printf-style text update of the big score label
        lv_obj_align(ui_score_lbl, LV_ALIGN_CENTER, 0, 14);  // keep it centred in the score card as its width changes
    }                                                   // end of change check
    if (g_best != g_best_shown) {                       // LVGL redraws a label every time its text is set, so only do it on change
        g_best_shown = g_best;                          // remember it
        lv_label_set_text_fmt(ui_best_lbl, "BEST %d", g_best);  // best-score label
    }                                                   // end of best-score change check
}                                                       // end of update_score_label

static void bring_player_to_front(void)                 // makes sure the player's car is drawn on top of the AI cars
{                                                       // begin of function body
    lv_obj_move_foreground(ui_car[g_player]);           // move the player's car to the top of the z-order
    lv_obj_move_foreground(ui_you_tag);                 // "YOU" tag above the car
    lv_obj_move_foreground(ui_banner);                  // banner above everything
}                                                       // end of bring_player_to_front

static void apply_player_color(void)                    // refreshes everything that depends on the chosen car colour
{                                                       // begin of function body
    uint32_t hex = g_color_chosen ? CAR_COLORS[g_player] : 0x5C6BC0;  // button colour: car colour, or neutral indigo before the first pick
    lv_obj_set_style_bg_color(ui_color_btn, lv_color_hex(hex), 0);    // recolour the CAR COLOR button
    lv_obj_set_style_bg_color(ui_color_btn, lv_color_darken(lv_color_hex(hex), LV_OPA_30), LV_STATE_PRESSED);  // matching pressed colour
    lv_obj_set_style_text_color(ui_color_lbl, lv_color_hex(g_color_chosen ? CAR_TEXT[g_player] : 0xFFFFFF), 0);  // readable text colour
    if (g_color_chosen) {                               // a colour has been picked...
        lv_label_set_text_fmt(ui_color_lbl, "CAR COLOR\n%s", CAR_NAMES[g_player]);  // ...show its name
        lv_obj_clear_flag(ui_you_tag, LV_OBJ_FLAG_HIDDEN);  // ...and show the "YOU" tag
    } else {                                            // nothing picked yet
        lv_label_set_text(ui_color_lbl, "PICK YOUR\nCAR COLOR");  // invite the player to tap
        lv_obj_add_flag(ui_you_tag, LV_OBJ_FLAG_HIDDEN);  // no "YOU" tag yet
    }                                                   // end of colour-chosen check
    lv_obj_set_style_bg_color(ui_preview_body, lv_color_hex(CAR_COLORS[g_player]), 0);  // recolour the big preview car in the garage card
    g_off = LANE_OFF[g_player];                         // the player's car sits in its own grid lane on the start line
    bring_player_to_front();                            // draw the chosen car on top
}                                                       // end of apply_player_color

static void set_state(GameState s)                      // switches game phase and updates buttons + banner accordingly
{                                                       // begin of function body
    g_state = s;                                        // store the new phase
    bool can_pick = (s == STATE_IDLE || s == STATE_CRASHED);  // colour/start are only allowed between races
    bool can_start = can_pick && (g_color_chosen || !REQUIRE_COLOR_PICK);  // START also needs a colour (if REQUIRE_COLOR_PICK)
    set_btn_enabled(ui_color_btn, can_pick);            // grey the colour button during the race
    set_btn_enabled(ui_start_btn, can_start);           // grey the START button when it may not be used
    g_banner_off_ms = 0;                                // cancel any scheduled banner hide
    char buf[48];                                       // scratch buffer for banner text
    switch (s) {                                        // choose the banner for the new phase
        case STATE_IDLE:                                // waiting on the start grid
            show_banner(g_color_chosen ? "PRESS START!" : "TAP CAR COLOR\nTO PICK YOUR CAR");  // hint text
            break;                                      // end of IDLE case
        case STATE_COUNTDOWN:                           // 3-2-1
            g_last_count = 3;                           // we begin at 3
            show_banner("3");                           // show the first number immediately
            break;                                      // end of COUNTDOWN case
        case STATE_RACING:                              // racing
            hide_banner();                              // clear the screen for driving
            break;                                      // end of RACING case
        case STATE_CRASHED:                             // game over
            snprintf(buf, sizeof(buf), "CRASH!\nSCORE %d", (int)g_score);  // build "CRASH! SCORE n"
            show_banner(buf);                           // show it
            break;                                      // end of CRASHED case
    }                                                   // end of switch
}                                                       // end of set_state

static void reset_world(void)                           // puts the track, scenery and cars back to the start grid
{                                                       // begin of function body
    g_camera = 0.0f;                                    // world scroll back to zero
    g_speed = 0.0f;                                     // standing still
    g_score = 0.0f;                                     // score back to zero
    g_ai_t = 0.0f;                                      // AI clock back to zero
    for (int i = 0; i < N_CARS; i++) g_ai_dist[i] = 0.0f;  // every AI car back on the start line
    g_off = LANE_OFF[g_player];                         // player back in its grid lane
    g_phase1 = (esp_random() % 6283) / 1000.0f;         // random 0..2*pi phase -> a different big bend every race
    g_phase2 = (esp_random() % 6283) / 1000.0f;         // random phase for the small wiggle
    for (int j = 0; j < N_COLS; j++) col_g[j] = -1;     // force every road strip to be restyled for the new track
    for (int i = 0; i < N_BAND; i++) band_wx[i] = (float)(i * BAND_PERIOD);  // reset stripe positions
    for (int i = 0; i < N_DASH; i++) {                  // reset dashes
        dash_wx[i] = 30.0f + i * DASH_GAP;              // world position
        place_dash(i);                                  // matching y on the new road
    }                                                   // end of dash loop
    for (int i = 0; i < N_TREE; i++) {                  // reset trees
        tree_wx[i] = 40.0f + i * TREE_GAP + rnd(0, 30); // world position
        place_tree(i);                                  // new size/colour/side for the new road
    }                                                   // end of tree loop
    g_score_shown = -1;                                 // force the score label to refresh
    update_score_label();                               // write "0"
}                                                       // end of reset_world

static void go_idle(void)                               // back to the start grid, ready to pick a colour / press START
{                                                       // begin of function body
    reset_world();                                      // clean track and cars
    apply_player_color();                               // refresh colour-dependent widgets
    set_state(STATE_IDLE);                              // idle phase + hint banner
}                                                       // end of go_idle

static void begin_countdown(void)                       // starts a new race: fresh track, then 3-2-1-GO
{                                                       // begin of function body
    reset_world();                                      // fresh track and cars
    apply_player_color();                               // make sure the player's lane/colour are applied
    g_state_ms = lv_tick_get();                         // remember when the countdown started
    set_state(STATE_COUNTDOWN);                         // countdown phase (banner shows "3")
}                                                       // end of begin_countdown

static void do_crash(void)                              // called when the player leaves the road
{                                                       // begin of function body
    if ((int)g_score > g_best) g_best = (int)g_score;   // update the best score
    g_speed = 0.0f;                                     // everything stops
    set_state(STATE_CRASHED);                           // game-over banner and buttons
    update_score_label();                               // refresh BEST label
}                                                       // end of do_crash

static float player_y(void)                             // computes the player's car centre y on screen
{                                                       // begin of function body
    float pwx = PLAYER_X0 + g_camera;                   // the player's x position in world coordinates
    return TRACK_FOLLOW * road_center(pwx) + (1.0f - TRACK_FOLLOW) * MID_Y + g_off;  // blend "follow the bend" with "fixed to screen", plus steering offset
}                                                       // end of player_y

static void update_racing(float dt, uint32_t now)       // physics for one frame while racing
{                                                       // begin of function body
    g_ai_t += dt;                                       // advance the AI clock
    float steer = shape_tilt(g_steer_f - g_neutral_steer);  // steering tilt relative to the pose held at "GO"
    float spd_in = shape_tilt(g_speed_f - g_neutral_speed); // speed tilt relative to the pose held at "GO"
    float target = clampf(SPEED_BASE + spd_in * SPEED_GAIN, SPEED_MIN, SPEED_MAX);  // tilt -> desired speed, within limits
    g_speed += (target - g_speed) * clampf(dt * SPEED_EASE, 0.0f, 1.0f);  // ease towards the target (smooth acceleration)
    g_off += steer * STEER_GAIN * dt;                   // tilt moves the car sideways relative to the road
    g_off = clampf(g_off, -(ROAD_H / 2.0f + 40.0f), ROAD_H / 2.0f + 40.0f);  // stop the offset growing without limit
    g_camera += g_speed * dt;                           // the world scrolls by speed * time
    g_score += g_speed * dt * SCORE_PER_PX;             // score grows with distance, so going faster scores faster

    for (int i = 0; i < N_CARS; i++) {                  // move every AI car
        if (i == g_player) continue;                    // the player's colour is not an AI car
        float sx = PLAYER_X0 + g_ai_dist[i] - g_camera; // this AI car's current screen x
        float v = AI_SPEED[i] + AI_WOBBLE * sinf(g_ai_t * AI_FREQ[i] + i * 1.7f);  // its own speed with gentle speeding up/slowing down
        v += clampf((AI_HOME[i] - sx) * AI_SPRING, -AI_PULL_MAX, AI_PULL_MAX);     // elastic pull towards its "home" x keeps it in view
        g_ai_dist[i] += v * dt;                         // advance its distance
    }                                                   // end of AI loop

    if (g_banner_off_ms != 0 && now >= g_banner_off_ms) {  // is it time to hide "GO!"?
        hide_banner();                                  // hide it
        g_banner_off_ms = 0;                            // and unschedule
    }                                                   // end of banner timing

    float py = player_y();                              // where the player's car is now
    float pwx = PLAYER_X0 + g_camera;                   // player's world x
    float limit = ROAD_H / 2.0f - CAR_H / 2.0f + CRASH_MARGIN;  // max distance between car centre and road centre before the car is off the road
    if (fabsf(py - road_center(pwx + CAR_W / 2.0f)) > limit ||   // check the car's nose...
        fabsf(py - road_center(pwx - CAR_W / 2.0f)) > limit) {   // ...and its tail
        do_crash();                                     // off the road = crash
    }                                                   // end of crash test
}                                                       // end of update_racing

static void game_tick(lv_timer_t *timer);               // forward declaration (defined in section 13)

// ==================== SECTION 12 : RENDERING (moves LVGL objects to match the game state) ====================

static void render_scenery(int cam)                     // scrolls stripes, trees, road strips, dashes and the start line
{                                                       // begin of function body
    for (int i = 0; i < N_BAND; i++) {                  // grass stripes
        int sx = (int)band_wx[i] - cam;                 // screen x = world x - camera
        while (sx < -BAND_W) {                          // fully off the left edge?
            band_wx[i] += N_BAND * BAND_PERIOD;         // jump it to the far right of the ring
            sx = (int)band_wx[i] - cam;                 // recompute screen x
        }                                               // end of recycle loop
        lv_obj_set_x(ui_band[i], sx);                   // move the stripe
    }                                                   // end of stripe loop

    for (int i = 0; i < N_TREE; i++) {                  // trees
        int sx = (int)tree_wx[i] - cam;                 // screen x
        while (sx < -TREE_D_MAX) {                      // off the left edge?
            tree_wx[i] += N_TREE * TREE_GAP;            // recycle to the right
            place_tree(i);                              // give it a new look and position on the road at that spot
            sx = (int)tree_wx[i] - cam;                 // recompute screen x
        }                                               // end of recycle loop
        lv_obj_set_pos(ui_tree[i], sx, tree_y[i]);      // move the tree
    }                                                   // end of tree loop

    int first_g = cam / COL_W;                          // world column number of the left-most visible strip
    int base = first_g % N_COLS;                        // which object slot that column uses
    for (int j = 0; j < N_COLS; j++) {                  // every road strip object
        int g = first_g + ((j - base + N_COLS) % N_COLS);  // the world column this slot must show (slot j always shows g with g % N_COLS == j)
        if (col_g[j] != g) {                            // the slot is showing an old column -> recycle it
            col_g[j] = g;                               // remember the new column number
            col_y[j] = (int)(road_center(g * COL_W + COL_W / 2.0f) - ROAD_H / 2.0f);  // top y so the strip is centred on the road centre
            lv_obj_set_style_border_color(ui_col[j], ((g / 3) & 1) ? lv_color_hex(0xE53935) : lv_color_hex(0xFAFAFA), 0);  // kerb: 3 strips red, 3 strips white, repeating
        }                                               // end of recycle
        lv_obj_set_pos(ui_col[j], g * COL_W - cam, col_y[j]);  // place the strip at its scrolled x
    }                                                   // end of road loop

    for (int i = 0; i < N_DASH; i++) {                  // centre dashes
        int sx = (int)dash_wx[i] - cam;                 // screen x
        while (sx < -DASH_LEN) {                        // off the left edge?
            dash_wx[i] += N_DASH * DASH_GAP;            // recycle to the right
            place_dash(i);                              // y for its new spot on the road
            sx = (int)dash_wx[i] - cam;                 // recompute screen x
        }                                               // end of recycle loop
        lv_obj_set_pos(ui_dash[i], sx, dash_y[i]);      // move the dash
    }                                                   // end of dash loop

    int line_x = START_X - cam;                         // screen x of the start line (becomes hugely negative as the race goes on)
    if (line_x < -100) line_x = -100;                   // clamp: LVGL coordinates are 16-bit, so park it just off-screen instead of overflowing
    lv_obj_set_x(ui_start_line, line_x);                // the chequered start line scrolls away with the world
}                                                       // end of render_scenery

static void place_car(lv_obj_t *car, float cx, float cy, int index)  // positions one car by its centre; hides it when off-screen
{                                                       // begin of function body
    static bool hidden[N_CARS] = {false, false};        // remembers each car's visibility so we only touch the flag on change
    bool off = (cx < -CAR_W || cx > ARENA_W + CAR_W);   // completely outside the arena horizontally?
    if (off != hidden[index]) {                         // visibility changed?
        hidden[index] = off;                            // remember
        if (off) lv_obj_add_flag(car, LV_OBJ_FLAG_HIDDEN);       // hide it
        else lv_obj_clear_flag(car, LV_OBJ_FLAG_HIDDEN);         // show it
    }                                                   // end of visibility change
    if (!off) lv_obj_set_pos(car, (int)cx - CAR_W / 2, (int)cy - CAR_H / 2);  // convert centre to top-left and move
}                                                       // end of place_car

static void render_cars(void)                           // positions the four cars and the YOU tag
{                                                       // begin of function body
    float fade = clampf(g_ai_t, 0.0f, 1.0f);            // AI lane wobble fades in during the first second of the race
    for (int i = 0; i < N_CARS; i++) {                  // each car colour
        if (i == g_player) continue;                    // the player's car is placed separately below
        float wx = PLAYER_X0 + g_ai_dist[i];            // AI car's world x
        float sx = wx - g_camera;                       // AI car's screen x
        float y = road_center(wx) + LANE_OFF[i] + fade * 5.0f * sinf(g_ai_t * 1.4f + i * 2.1f);  // AI drives exactly along the road in its lane
        place_car(ui_car[i], sx, y, i);                 // move the AI car
    }                                                   // end of AI loop
    float py = player_y();                              // player's y
    place_car(ui_car[g_player], (float)PLAYER_X0, py, g_player);  // player's car is always at the same screen x
    lv_obj_set_pos(ui_you_tag, PLAYER_X0 - 26, (int)py - CAR_H / 2 - 30);  // "YOU" tag rides above the player's car
}                                                       // end of render_cars

// ==================== SECTION 13 : THE GAME TIMER (runs inside LVGL's own task, so no locking is needed) ====================

static void game_tick(lv_timer_t *timer)                // called by LVGL every TICK_MS milliseconds
{                                                       // begin of function body
    LV_UNUSED(timer);                                   // we do not use the timer argument
    uint32_t now = lv_tick_get();                       // LVGL millisecond clock
    float dt = (float)(now - g_last_tick_ms) / 1000.0f; // seconds since the last tick
    g_last_tick_ms = now;                               // store for next time
    dt = clampf(dt, 0.001f, 0.1f);                      // guard against a huge dt if drawing was slow

    read_tilt();                                        // sample the accelerometer (updates g_steer_f / g_speed_f)

    if (g_state == STATE_COUNTDOWN) {                   // 3-2-1 phase
        uint32_t el = now - g_state_ms;                 // ms since the countdown started
        if (el >= 3 * COUNT_STEP_MS) {                  // 3, 2, 1 have all been shown -> GO
            g_neutral_steer = g_steer_f;                // the current pose becomes "hands off" (neutral)
            g_neutral_speed = g_speed_f;                // same for speed tilt
            set_state(STATE_RACING);                    // start racing
            show_banner("GO!");                         // flash GO!
            g_banner_off_ms = now + GO_SHOW_MS;         // ...and hide it shortly
        } else {                                        // still counting
            int n = 3 - (int)(el / COUNT_STEP_MS);      // 3, then 2, then 1
            if (n != g_last_count) {                    // the number changed
                g_last_count = n;                       // remember
                char b[4];                              // tiny buffer
                snprintf(b, sizeof(b), "%d", n);        // number to text
                show_banner(b);                         // show it
            }                                           // end of number change
        }                                               // end of countdown branch
    } else if (g_state == STATE_RACING) {               // driving phase
        update_racing(dt, now);                         // physics, scoring, AI and crash detection
    }                                                   // end of state handling

    render_scenery((int)g_camera);                      // move all scenery to match the camera
    render_cars();                                      // move all cars
    update_score_label();                               // refresh score text if it changed
}                                                       // end of game_tick

// ==================== SECTION 14 : BUTTON EVENT HANDLERS ====================

static void on_color_btn(lv_event_t *e)                 // CAR COLOR button
{                                                       // begin of function body
    LV_UNUSED(e);                                       // event details not needed
    if (g_state == STATE_COUNTDOWN || g_state == STATE_RACING) return;  // colour is locked during a race
    if (g_state == STATE_CRASHED) go_idle();            // after a crash, choosing a colour first returns to the grid
    if (!g_color_chosen) g_color_chosen = true;         // first tap: pick the current (red) car
    else g_player = (g_player + 1) % N_CARS;            // later taps: cycle red -> blue -> red
    apply_player_color();                               // update button, preview car, lane and YOU tag
    set_state(STATE_IDLE);                              // re-evaluate button states (START becomes enabled) and the hint banner
}                                                       // end of on_color_btn

static void on_start_btn(lv_event_t *e)                 // START button
{                                                       // begin of function body
    LV_UNUSED(e);                                       // event details not needed
    if (g_state == STATE_COUNTDOWN || g_state == STATE_RACING) return;  // ignore while already racing
    if (REQUIRE_COLOR_PICK && !g_color_chosen) return;  // must choose a car first
    begin_countdown();                                  // new track + 3-2-1-GO
}                                                       // end of on_start_btn

static void on_new_btn(lv_event_t *e)                   // NEW GAME button
{                                                       // begin of function body
    LV_UNUSED(e);                                       // event details not needed
    go_idle();                                          // abandon whatever is happening and return to the start grid
}                                                       // end of on_new_btn

// ==================== SECTION 15 : BUILDING THE CONTROL PANEL (right 20 % of the screen) ====================

static void build_panel(lv_obj_t *scr)                  // creates score card, garage preview and the three buttons
{                                                       // begin of function body
    ui_panel = make_rect(scr, PANEL_X, 0, PANEL_W, SCREEN_H, 0x14213D, 0);  // deep-navy panel background
    make_rect(ui_panel, 0, 0, 4, SCREEN_H, 0xFFC107, 0);  // 4 px golden divider line on the panel's left edge

    lv_obj_t *title = make_label(ui_panel, "TILT RACER", &lv_font_montserrat_24, 0xFFC107);  // game title in gold
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);        // top-centre

    lv_obj_t *card = make_rect(ui_panel, 10, 40, 184, 112, 0x0B1226, 22);  // dark rounded score card
    lv_obj_set_style_border_width(card, 3, 0);          // gold outline
    lv_obj_set_style_border_color(card, lv_color_hex(0xFFC107), 0);  // outline colour
    lv_obj_t *score_title = make_label(card, "SCORE", &lv_font_montserrat_20, 0xFFC107);  // small heading
    lv_obj_align(score_title, LV_ALIGN_TOP_MID, 0, 8);  // top-centre of the card
    ui_score_lbl = make_label(card, "0", &lv_font_montserrat_48, 0xFFFFFF);  // BIG score number
    lv_obj_align(ui_score_lbl, LV_ALIGN_CENTER, 0, 14); // centred a little below the middle
    ui_best_lbl = make_label(ui_panel, "BEST 0", &lv_font_montserrat_20, 0xB0BEC5);  // best score under the card
    lv_obj_align(ui_best_lbl, LV_ALIGN_TOP_MID, 0, 158);  // centred, 158 px from the top

    lv_obj_t *garage = make_rect(ui_panel, 10, 190, 184, 96, 0x22365E, 22);  // "garage" card that shows the chosen car large
    create_car(garage, CAR_COLORS[0], 6, &ui_preview_body);  // preview car at scale 6 (120 x 66 px); body pointer saved for recolouring
    lv_obj_t *pc = lv_obj_get_child(garage, 0);         // the preview car container (first child of the garage card)
    lv_obj_set_pos(pc, (184 - 120) / 2, (96 - 66) / 2); // centre the 120x66 car inside the 184x96 card

    ui_color_btn = make_button(ui_panel, 296, 76, 0x5C6BC0, 0xFFFFFF, &lv_font_montserrat_20,                           // (statement continues on the next line)
                               "PICK YOUR\nCAR COLOR", on_color_btn);  // button 1: choose the car colour
    ui_color_lbl = lv_obj_get_child(ui_color_btn, 0);   // keep a pointer to its text label so we can change it later
    ui_start_btn = make_button(ui_panel, 382, 88, 0x00C853, 0xFFFFFF, &lv_font_montserrat_30,                           // (statement continues on the next line)
                               LV_SYMBOL_PLAY " START", on_start_btn);  // button 2: start the race (big green)
    ui_new_btn = make_button(ui_panel, 480, 72, 0xFF9800, 0x1B1B1B, &lv_font_montserrat_24,                             // (statement continues on the next line)
                             LV_SYMBOL_REFRESH " NEW GAME", on_new_btn);  // button 3: new game (orange); bottom 48 px left free for the LVGL FPS monitor
}                                                       // end of build_panel

// ==================== SECTION 16 : ARDUINO setup() ====================

void setup()                                            // runs once at power-up
{                                                       // begin of function body
    pinMode(32, OUTPUT);                                // GPIO32 as output (board control line, same as Lessons 10 and 14)
    pinMode(33, OUTPUT);                                // GPIO33 as output (board control line)
    digitalWrite(32, 0);                                // GPIO32 low (same as Lessons 10 and 14)
    digitalWrite(33, 1);                                // GPIO33 high (same as Lessons 10 and 14)
    Serial.begin(115200);                               // serial monitor at 115200 baud
    Serial.println("Initializing board");               // progress message

    board_p4_ldo_init();                                // Lesson 10: switch on LDO3 (2.5 V MIPI) and LDO4 (3.3 V I2C pull-ups)

    g_imu_ok = imu_start();                             // Lesson 14: bring up I2C port 0 + LSM6DS3TR-C. MUST happen before board->init()
                                                        // because the touch controller shares this I2C bus (SKIP_INIT_HOST = 1)
    Serial.println(g_imu_ok ? "IMU ready (104 Hz, +/-2 g)" : "IMU NOT FOUND - check wiring/address");  // report the sensor result

    Board *board = new Board();                         // Lesson 10: create the board object (reads esp_panel_board_custom_conf.h)
    board->init();                                      // Lesson 10: initialise LCD, touch and backlight drivers
    board->begin();                                     // Lesson 10: start them running

    Touch *tp = board->getTouch();                      // pointer to the GT911 touch driver (NULL if touch is disabled in the config)
    if (tp == nullptr) {                                // touch missing?
        Serial.println("WARNING: touch is disabled - set ESP_PANEL_BOARD_USE_TOUCH to 1 in esp_panel_board_custom_conf.h");  // tell the user how to fix it
    }                                                   // end of touch check

    bool lvgl_ready = lvgl_port_init(board->getLCD(), tp);  // Lesson 10: start LVGL, register the display and touch input, launch the LVGL task
    // lvgl_port_init() only creates its mutex and its background task AFTER the touch (indev) driver has been registered.
    // If touch registration fails, it returns false early and NO task is ever created - so nothing would ever pump
    // lv_timer_handler(), the screen would stay blank forever, and no lv_timer (including our game loop) would ever fire.
    if (!lvgl_ready) {                                  // did initialisation actually finish?
        Serial.println("WARNING: lvgl_port_init() returned false with touch enabled - the LVGL task was NOT created.");  // explain what happened
        Serial.println("         Retrying with touch disabled so the display can still come up for diagnosis...");  // fallback plan
        lvgl_ready = lvgl_port_init(board->getLCD(), nullptr);  // retry: register only the display, skip touch entirely
        if (lvgl_ready) {                               // did the retry succeed?
            Serial.println("         Retry OK: display + task are running WITHOUT touch. Touch registration is the fault -");  // confirms the theory
            Serial.println("         check the GT911 wiring, its I2C address, and its reset/interrupt pins.");  // what to check next
        } else {                                        // retry also failed
            Serial.println("FATAL: display registration itself failed (not just touch). The screen cannot come up.");  // different, deeper fault
            Serial.println("       Check the MIPI-DSI panel wiring and the LDO3/LDO4 power-up messages above.");  // what to check next
        }                                                // end of retry result
        g_touch_ok = false;                             // either way, we are not running with a working touch driver
    } else {                                            // initialisation completed normally on the first try
        Serial.println("LVGL port ready: display + task + touch are all running.");  // confirms the task exists
        g_touch_ok = true;                              // touch registered successfully alongside the display
    }                                                    // end of lvgl_ready check
    board->getBacklight()->setBrightness(0);            // keep the screen dark while we build the UI (avoids a flash of garbage)

    if (!lvgl_ready) {                                  // both attempts failed - lvgl_mux and the LVGL task do not exist
        // Calling lvgl_port_lock()/lv_obj_* now would touch a null mutex and undefined LVGL state, so stop here instead
        // of doing that blind. The FATAL message above already explains what to check; loop() will keep reporting IMU
        // readings over Serial so the sensor and I2C bus can still be verified independently of the display fault.
        Serial.println("Halting UI setup. Fix the issue above, then re-flash.");  // final status line for this boot
        return;                                         // skip the rest of setup() - no lv_obj_* calls, no lv_timer_create()
    }                                                    // end of hard stop on total failure

    lvgl_port_lock(-1);                                 // take the LVGL mutex (wait forever) because the LVGL task is already running
    lv_obj_t *scr = lv_scr_act();                       // the active screen object
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);  // black screen background
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);        // fully opaque
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);     // the screen must never scroll

    randomSeed(esp_random());                           // seed Arduino's random() (not used directly, but keeps things unpredictable)
    g_phase1 = (esp_random() % 6283) / 1000.0f;         // random big-bend phase so scenery creation has a valid road shape
    g_phase2 = (esp_random() % 6283) / 1000.0f;         // random small-wiggle phase

    build_arena(scr);                                   // draw the play-field (left 80 %)
    build_panel(scr);                                   // draw the control panel (right 20 %)
    reset_world();                                      // place scenery and cars on the start grid
    apply_player_color();                               // initial button/preview state
    set_state(STATE_IDLE);                              // idle phase with hint banner

    g_last_tick_ms = lv_tick_get();                     // initialise the delta-time clock
    lv_timer_create(game_tick, TICK_MS, NULL);          // start the game loop: LVGL will call game_tick() every 33 ms
    lvgl_port_unlock();                                 // release the LVGL mutex so the LVGL task can start drawing

    delay(200);                                         // short pause (same as Lesson 10)
    board->getBacklight()->setBrightness(100);          // switch the backlight fully on
    Serial.println("Tilt Racer ready");                 // done
}                                                       // end of setup

// ==================== SECTION 17 : ARDUINO loop() ====================

void loop()                                             // runs forever; the game itself runs in the LVGL task, so this only prints debug info
{                                                       // begin of function body
#if TILT_DEBUG                                          // compile the debug print only if enabled
    float raw_ax = 0.0f, raw_ay = 0.0f;                 // scratch variables for one direct, un-filtered sensor read
    bool raw_ok = g_imu_ok && imu_read_accel(&raw_ax, &raw_ay);  // read straight from the chip, bypassing the game's filter entirely
    Serial.printf("RAW ax=%+.3f g ay=%+.3f g (ok=%d)  |  filtered steer=%+.2f g speed=%+.2f g  |  neutral s=%+.2f v=%+.2f  |  imu_ok=%d primed=%d i2c_err=%u\n",// (statement continues on the next line)
                  raw_ax, raw_ay, (int)raw_ok, g_steer_f, g_speed_f, g_neutral_steer, g_neutral_speed,
                  (int)g_imu_ok, (int)g_imu_primed, (unsigned)g_imu_errors);  // raw values alongside the derived ones, so a wiring fault and a "board is flat" reading can be told apart
#endif                                                  // end of debug print
    delay(500);                                         // twice a second is plenty
}                                                       // end of loop
