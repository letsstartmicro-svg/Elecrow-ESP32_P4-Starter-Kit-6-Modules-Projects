// ==================== SECTION 1: INCLUDES ====================
#include "Arduino.h"                     // Arduino core: pinMode, digitalWrite, delay, Serial, random, millis
#include <esp_display_panel.hpp>         // Espressif ESP_Panel library: Board, LCD, Touch, Backlight classes (also pulls in the LDO API used below)
#include <lvgl.h>                        // LVGL v8.3.11 graphics library (version confirmed in the uploaded lvgl.h)
#include <lvgl_v8_port.h>                // Espressif's LVGL<->ESP_Panel glue: lvgl_port_init(), lvgl_port_lock(), lvgl_port_unlock()
#include "Adafruit_NeoPixel.h"           // Driver for the two WS2812 RGB LEDs on the kit (used here for game feedback flashes)
#include "esp_random.h"                  // esp_random(): hardware true-random-number generator, used to seed the food placement

// ==================== SECTION 2: HARDWARE CONSTANTS (COPIED FROM LESSON 10) ====================
#define LED_NUM     2                    // Number of NeoPixel LEDs on the strip (same value as Lesson 10 / config.h)
#define LED_PIN     8                    // GPIO 8 drives the NeoPixel data line (same value as Lesson 10 / config.h)

Adafruit_NeoPixel strip(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);  // Create the strip object: 2 LEDs, pin 8, GRB byte order, 800 kHz WS2812 timing

using namespace esp_panel::drivers;      // Lets us write LCD / Touch instead of esp_panel::drivers::LCD etc.
using namespace esp_panel::board;        // Lets us write Board instead of esp_panel::board::Board

// ==================== SECTION 3: GAME CONFIGURATION ====================
#define SCREEN_W        1024             // Panel width in pixels (ESP_PANEL_BOARD_WIDTH in esp_panel_board_custom_conf.h)
#define SCREEN_H        600              // Panel height in pixels (ESP_PANEL_BOARD_HEIGHT in esp_panel_board_custom_conf.h)
#define CELL            30               // Size of one grid cell in pixels (30 px is large enough to see clearly on the 1024x600 panel)
#define GRID_COLS       22               // Number of grid columns: 22 * 30 = 660 px wide arena
#define GRID_ROWS       20               // Number of grid rows: 20 * 30 = 600 px tall arena (full screen height)
#define ARENA_W         (GRID_COLS * CELL)   // Arena width in pixels = 660
#define ARENA_H         (GRID_ROWS * CELL)   // Arena height in pixels = 600
#define PANEL_W         (SCREEN_W - ARENA_W) // Right control panel width = 1024 - 660 = 364 px
#define MAX_LEN         (GRID_COLS * GRID_ROWS)  // Worst case snake length = every cell of the grid = 440 segments
#define START_PERIOD_MS 170              // Initial time between snake moves in milliseconds (bigger = slower)
#define MIN_PERIOD_MS   70               // Fastest allowed move period; the game never gets quicker than this
#define POINTS_PER_FOOD 10               // Score awarded for each piece of food eaten

#define DIR_UP          0                // Direction id: up (paired with DIR_DOWN so that (a ^ 1) is the opposite direction)
#define DIR_DOWN        1                // Direction id: down
#define DIR_LEFT        2                // Direction id: left (paired with DIR_RIGHT)
#define DIR_RIGHT       3                // Direction id: right

#define STATE_READY     0                // Game state: waiting for the first direction press
#define STATE_RUNNING   1                // Game state: snake is moving
#define STATE_OVER      2                // Game state: crashed (or won), waiting for NEW GAME

// Pick the biggest Montserrat fonts that are actually enabled in lv_conf.h, so the sketch compiles either way.
#if LV_FONT_MONTSERRAT_48                // If the 48 px font was enabled in lv_conf.h ...
    #define FONT_HUGE (&lv_font_montserrat_48)  // ... use it for the big score number
#elif LV_FONT_MONTSERRAT_30              // otherwise, if the 30 px font is enabled (Lesson 10 uses it) ...
    #define FONT_HUGE (&lv_font_montserrat_30)  // ... use that instead
#else                                    // otherwise fall back to the always-on default font
    #define FONT_HUGE (&lv_font_montserrat_14)  // 14 px is enabled in the uploaded lv_conf.h
#endif                                   // end of the FONT_HUGE selection
#if LV_FONT_MONTSERRAT_30                // Same idea for the "big" font used for titles and arrow symbols
    #define FONT_BIG  (&lv_font_montserrat_30)  // 30 px font (used in Lesson 10)
#elif LV_FONT_MONTSERRAT_20              // else try the 20 px font (also used in Lesson 10)
    #define FONT_BIG  (&lv_font_montserrat_20)  // 20 px font
#else                                    // else fall back
    #define FONT_BIG  (&lv_font_montserrat_14)  // 14 px font
#endif                                   // end of the FONT_BIG selection
#if LV_FONT_MONTSERRAT_20                // And the "medium" font used for small captions
    #define FONT_MED  (&lv_font_montserrat_20)  // 20 px font
#else                                    // else fall back
    #define FONT_MED  (&lv_font_montserrat_14)  // 14 px font
#endif                                   // end of the FONT_MED selection

// ==================== SECTION 4: GAME STATE VARIABLES ====================
static int snake_x[MAX_LEN];             // Column index of every snake segment; index 0 is the HEAD
static int snake_y[MAX_LEN];             // Row index of every snake segment; index 0 is the HEAD
static int snake_len = 3;                // Current number of segments in the snake
static int dir = DIR_RIGHT;              // Direction the snake moved on its most recent step
static int next_dir = DIR_RIGHT;         // Direction requested by the latest button press, applied on the next step
static int food_x = 0;                   // Column of the food item
static int food_y = 0;                   // Row of the food item
static int score = 0;                    // Current score
static int best_score = 0;               // Highest score reached since power-on (kept in RAM only)
static int game_state = STATE_READY;     // Which of the three game states we are in
static int hue_base = 0;                 // Rotating color offset (0-359) that makes the rainbow flow along the snake
static int led_ticks_left = 0;           // How many game ticks remain before the NeoPixels are switched off again

static const int8_t DX[4] = {0, 0, -1, 1};   // Horizontal step for UP, DOWN, LEFT, RIGHT (indexed by direction id)
static const int8_t DY[4] = {-1, 1, 0, 0};   // Vertical step for UP, DOWN, LEFT, RIGHT (screen Y grows downward, so UP = -1)

// ==================== SECTION 5: LVGL OBJECT HANDLES ====================
static lv_obj_t *arena = NULL;           // The custom-drawn playfield object (left side of the screen)
static lv_obj_t *overlay_label = NULL;   // Message box shown over the arena (start hint / game over)
static lv_obj_t *score_value_label = NULL;   // Big number inside the score card
static lv_obj_t *info_label = NULL;      // Small line showing snake length and best score
static lv_timer_t *game_timer = NULL;    // LVGL timer that calls game_tick() every move period

// ==================== SECTION 6: POWER RAILS (COPIED FROM LESSON 10) ====================
// ESP32-P4's MIPI D-PHY requires specific voltage to function.
// LDO3 is typically routed to the MIPI power rail on P4 hardware.
esp_err_t board_p4_ldo_init()            // Returns ESP_OK if both regulators were switched on
{                                        // start of board_p4_ldo_init
    esp_err_t err = ESP_OK;              // Holds the result code of each ESP-IDF call
    esp_ldo_channel_handle_t ldo3_handle = NULL;   // Handle that the driver fills in when LDO channel 3 is acquired
    esp_ldo_channel_config_t ldo3_cfg = {          // Configuration struct for LDO channel 3
        .chan_id = 3,                    // Use the P4's on-chip LDO channel number 3
        .voltage_mv = 2500,              // Output 2500 mV (2.5 V) for the MIPI DSI PHY supply
    };                                   // end of ldo3_cfg

    Serial.println("Initializing LDO3 to 2.5V...");    // Progress message on the serial monitor
    err = esp_ldo_acquire_channel(&ldo3_cfg, &ldo3_handle);   // Ask the driver to enable LDO3 at the requested voltage
    if (err != ESP_OK) {                 // If enabling failed ...
        Serial.printf("LDO3 Power Error: %s\n", esp_err_to_name(err));   // ... print the human-readable error name
        return err;                      // ... and abort, because the display cannot work without this rail
    } else {                             // Otherwise the rail is up
        Serial.println("LDO3 Power enabled successfully.");   // Confirm success
    }                                    // end of LDO3 result check

    esp_ldo_channel_handle_t ldo4_handle = NULL;   // Handle for LDO channel 4
    esp_ldo_channel_config_t ldo4_cfg = {          // Configuration struct for LDO channel 4
        .chan_id = 4,                    // Use LDO channel number 4
        .voltage_mv = 3300,              // Output 3300 mV (3.3 V), used for I2C / touch pull-ups per Lesson 10
    };                                   // end of ldo4_cfg

    Serial.println("Initializing LDO4 to 3.3V...");    // Progress message
    err = esp_ldo_acquire_channel(&ldo4_cfg, &ldo4_handle);   // Enable LDO4 at 3.3 V
    if (err != ESP_OK) {                 // If enabling failed ...
        Serial.printf("LDO4 Power Error: %s\n", esp_err_to_name(err));   // ... print the error
        return err;                      // ... and abort
    } else {                             // Otherwise success
        Serial.println("LDO4 Power enabled successfully.");   // Confirm success
    }                                    // end of LDO4 result check

    return ESP_OK;                       // Both rails are on
}                                        // end of board_p4_ldo_init

// ==================== SECTION 7: LED FEEDBACK HELPER ====================
static void led_flash(uint8_t r, uint8_t g, uint8_t b, int ticks)   // Light both NeoPixels a color for a number of game ticks
{                                        // start of led_flash
    strip.setPixelColor(0, strip.Color(r, g, b));   // Write the color into the RAM buffer for LED 0
    strip.setPixelColor(1, strip.Color(r, g, b));   // Write the same color for LED 1
    strip.show();                        // Push the buffer out on GPIO 8 using the WS2812 one-wire protocol
    led_ticks_left = ticks;              // Remember how long until game_tick() should turn the LEDs off
}                                        // end of led_flash

// ==================== SECTION 8: HUD / OVERLAY HELPERS ====================
static void update_hud(void)             // Refresh the score number and the info line from the game variables
{                                        // start of update_hud
    lv_label_set_text_fmt(score_value_label, "%d", score);   // printf-style update of the big score label (LVGL copies the string)
    lv_label_set_text_fmt(info_label, "Length %d   Best %d", snake_len, best_score);   // Update the small stats line
}                                        // end of update_hud

static void set_overlay(const char *text)   // Show a message over the arena, or hide the message if text is NULL
{                                        // start of set_overlay
    if (text == NULL) {                  // NULL means "hide the box"
        lv_obj_add_flag(overlay_label, LV_OBJ_FLAG_HIDDEN);   // Setting the HIDDEN flag stops LVGL drawing the object
    } else {                             // Otherwise show the requested text
        lv_label_set_text(overlay_label, text);               // Replace the text inside the label
        lv_obj_clear_flag(overlay_label, LV_OBJ_FLAG_HIDDEN); // Remove the HIDDEN flag so the box is drawn
        lv_obj_align(overlay_label, LV_ALIGN_CENTER, 0, 0);   // Re-center it because the new text may have a different size
    }                                    // end of if/else
}                                        // end of set_overlay

// ==================== SECTION 9: GAME LOGIC ====================
static void place_food(void)             // Put the food on a random cell that the snake does not occupy
{                                        // start of place_food
    bool on_snake = true;                // Assume the first candidate is bad so the loop runs at least once
    while (on_snake) {                   // Keep picking random cells until we find a free one
        food_x = random(GRID_COLS);      // Random column 0 .. GRID_COLS-1 (Arduino random(max) excludes max)
        food_y = random(GRID_ROWS);      // Random row 0 .. GRID_ROWS-1
        on_snake = false;                // Optimistically assume this candidate is free
        for (int i = 0; i < snake_len; i++) {   // Compare the candidate with every snake segment
            if (snake_x[i] == food_x && snake_y[i] == food_y) {   // If a segment sits on the candidate cell ...
                on_snake = true;         // ... the candidate is invalid
                break;                   // ... and there is no need to check the remaining segments
            }                            // end of collision check
        }                                // end of for loop over segments
    }                                    // end of while loop
}                                        // end of place_food

static void reset_game(void)             // Put everything back to the starting position (READY state)
{                                        // start of reset_game
    snake_len = 3;                       // Start with three segments
    for (int i = 0; i < snake_len; i++) {   // Lay the three segments out in a horizontal line
        snake_x[i] = (GRID_COLS / 2) - i;   // Head at the middle column, body extending to the left
        snake_y[i] = GRID_ROWS / 2;         // All segments on the middle row
    }                                    // end of for loop
    dir = DIR_RIGHT;                     // The snake initially faces right
    next_dir = DIR_RIGHT;                // No pending turn
    score = 0;                           // Score restarts at zero
    game_state = STATE_READY;            // Wait for the player to press a direction
    lv_timer_set_period(game_timer, START_PERIOD_MS);   // Restore the slow starting speed
    place_food();                        // Choose the first food position
    update_hud();                        // Show the fresh numbers on screen
    set_overlay("TAP A DIRECTION\nTO START");   // Tell the player what to do
    led_flash(0, 0, 40, 6);              // Dim blue LED flash for a moment to signal a new game
    lv_obj_invalidate(arena);            // Mark the arena dirty so LVGL redraws it on the next refresh
}                                        // end of reset_game

static void finish_game(const char *title)   // Enter the OVER state and show the result message
{                                        // start of finish_game
    game_state = STATE_OVER;             // Stop the snake from moving
    if (score > best_score) {            // If this run beat the previous record ...
        best_score = score;              // ... store the new best score
    }                                    // end of best score check
    update_hud();                        // Refresh the "Best" number
    lv_label_set_text_fmt(overlay_label, "%s\nScore: %d\nPress NEW GAME", title, score);   // Build the multi-line result message
    lv_obj_clear_flag(overlay_label, LV_OBJ_FLAG_HIDDEN);   // Make sure the message box is visible
    lv_obj_align(overlay_label, LV_ALIGN_CENTER, 0, 0);     // Re-center after the text change
    led_flash(120, 0, 0, 14);            // Red flash for ~14 ticks to signal the end of the game
    lv_obj_invalidate(arena);            // Redraw the arena
}                                        // end of finish_game

// This runs every move period inside the LVGL task, so it may call LVGL functions without taking the mutex.
static void game_tick(lv_timer_t *t)     // LVGL timer callback; 't' is the timer that fired (unused)
{                                        // start of game_tick
    (void)t;                             // Silence the "unused parameter" compiler warning

    if (led_ticks_left > 0) {            // If a feedback flash is active ...
        led_ticks_left--;                // ... count one tick off
        if (led_ticks_left == 0) {       // ... and when it reaches zero ...
            strip.clear();               // ... clear the LED buffer (all black)
            strip.show();                // ... and send it to the LEDs
        }                                // end of expiry check
    }                                    // end of LED timing

    if (game_state != STATE_RUNNING) {   // Only move the snake while the game is running
        return;                          // Otherwise do nothing this tick
    }                                    // end of state check

    dir = next_dir;                      // Commit the requested direction (already validated in the button handler)
    int nx = snake_x[0] + DX[dir];       // Column where the head will be after this step
    int ny = snake_y[0] + DY[dir];       // Row where the head will be after this step

    if (nx < 0 || nx >= GRID_COLS || ny < 0 || ny >= GRID_ROWS) {   // Did the head leave the arena?
        finish_game("GAME OVER");        // Hitting a wall ends the game
        return;                          // Stop processing this tick
    }                                    // end of wall check

    bool eating = (nx == food_x && ny == food_y);   // Will the head land on the food?
    int check_len = eating ? snake_len : (snake_len - 1);   // If not eating, the tail cell is vacated this step so it is safe to enter
    for (int i = 0; i < check_len; i++) {   // Test the new head cell against the body
        if (snake_x[i] == nx && snake_y[i] == ny) {   // Overlap means the snake bit itself
            finish_game("GAME OVER");    // Self-collision ends the game
            return;                      // Stop processing this tick
        }                                // end of overlap test
    }                                    // end of body loop

    if (eating && snake_len < MAX_LEN) { // Growing is only possible if the array still has room
        snake_len++;                     // Add one segment; the old tail position becomes the new last element below
    }                                    // end of grow check

    for (int i = snake_len - 1; i > 0; i--) {   // Shift every segment one slot toward the tail (back to front so nothing is overwritten)
        snake_x[i] = snake_x[i - 1];     // Segment i takes the column of the segment ahead of it
        snake_y[i] = snake_y[i - 1];     // Segment i takes the row of the segment ahead of it
    }                                    // end of shift loop
    snake_x[0] = nx;                     // Place the head at its new column
    snake_y[0] = ny;                     // Place the head at its new row

    hue_base = (hue_base + 6) % 360;     // Rotate the rainbow a little so the colors seem to flow along the body

    if (eating) {                        // Handle everything that happens when food is eaten
        score += POINTS_PER_FOOD;        // Add points
        int period = START_PERIOD_MS - (score / 50) * 10;   // Every 50 points make the snake 10 ms faster
        if (period < MIN_PERIOD_MS) {    // Clamp to the fastest allowed speed
            period = MIN_PERIOD_MS;      // Use the minimum period
        }                                // end of clamp
        lv_timer_set_period(game_timer, period);   // Apply the new move period to the LVGL timer
        led_flash(0, 120, 0, 3);         // Short green flash as a "yum" indicator
        if (snake_len >= MAX_LEN) {      // Did the snake fill the entire grid?
            update_hud();                // Show the final numbers
            finish_game("YOU WIN!");     // Perfect game
            return;                      // Nothing more to do
        }                                // end of win check
        place_food();                    // Otherwise spawn new food
    }                                    // end of eating block

    update_hud();                        // Refresh score and length labels
    lv_obj_invalidate(arena);            // Ask LVGL to redraw the arena with the new snake position
}                                        // end of game_tick

// ==================== SECTION 10: ARENA DRAWING ====================
// Fill 'out' with the pixel rectangle of grid cell (cx, cy), shrunk by 'inset' pixels on every side.
// 'origin' is the arena's absolute on-screen rectangle.
static void cell_area(const lv_area_t *origin, int cx, int cy, int inset, lv_area_t *out)   // Grid cell -> pixel area
{                                        // start of cell_area
    out->x1 = origin->x1 + cx * CELL + inset;           // Left edge of the cell plus inset
    out->y1 = origin->y1 + cy * CELL + inset;           // Top edge of the cell plus inset
    out->x2 = origin->x1 + (cx + 1) * CELL - 1 - inset; // Right edge (LVGL areas are inclusive, hence the -1)
    out->y2 = origin->y1 + (cy + 1) * CELL - 1 - inset; // Bottom edge (inclusive)
}                                        // end of cell_area

// Draw a small filled circle centered on (px, py) with the given diameter and color (used for the snake's eyes).
static void draw_dot(lv_draw_ctx_t *dc, int px, int py, int diameter, lv_color_t color)   // Helper for eyes
{                                        // start of draw_dot
    lv_draw_rect_dsc_t d;                // Descriptor that tells LVGL how to paint a rectangle
    lv_draw_rect_dsc_init(&d);           // Fill the descriptor with LVGL defaults (opaque white rect, no border)
    d.bg_color = color;                  // Set the fill color
    d.radius = LV_RADIUS_CIRCLE;         // A radius larger than half the size turns the rectangle into a circle
    d.border_width = 0;                  // No outline
    lv_area_t a;                         // Pixel rectangle to paint
    a.x1 = px - diameter / 2;            // Left edge
    a.y1 = py - diameter / 2;            // Top edge
    a.x2 = a.x1 + diameter - 1;          // Right edge (inclusive)
    a.y2 = a.y1 + diameter - 1;          // Bottom edge (inclusive)
    lv_draw_rect(dc, &d, &a);            // Render the circle into the current draw buffer (clipped automatically)
}                                        // end of draw_dot

// Called by LVGL every time part of the arena needs repainting. 'draw_ctx' is the current partial frame buffer.
static void arena_draw_cb(lv_event_t *e) // Event callback registered for LV_EVENT_DRAW_MAIN_END
{                                        // start of arena_draw_cb
    lv_obj_t *obj = lv_event_get_target(e);              // The object being drawn (our arena)
    lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);        // The drawing context: destination buffer + clip rectangle
    lv_area_t oa;                        // Will hold the arena's absolute coordinates
    lv_obj_get_coords(obj, &oa);         // Fill 'oa' with the arena's on-screen rectangle (0,0)-(659,599)

    lv_draw_rect_dsc_t d;                // Reusable rectangle descriptor
    lv_area_t a;                         // Reusable pixel rectangle

    // ---- checkerboard: faint white squares on alternate cells ----
    lv_draw_rect_dsc_init(&d);           // Reset descriptor to defaults
    d.bg_color = lv_color_white();       // White squares ...
    d.bg_opa = LV_OPA_10;                // ... at 10% opacity, so they only slightly lighten the dark background
    d.radius = 0;                        // Sharp corners
    d.border_width = 0;                  // No border
    for (int y = 0; y < GRID_ROWS; y++) {       // Loop over every row
        for (int x = 0; x < GRID_COLS; x++) {   // Loop over every column
            if (((x + y) & 1) == 0) {    // Only every second cell gets a square (checkerboard pattern)
                continue;                // Skip the others
            }                            // end of parity check
            cell_area(&oa, x, y, 0, &a); // Compute this cell's pixel rectangle
            if (_lv_area_is_on(&a, dc->clip_area)) {   // Only draw if it overlaps the region currently being repainted
                lv_draw_rect(dc, &d, &a);  // Paint the square
            }                            // end of clip test
        }                                // end of column loop
    }                                    // end of row loop

    // ---- food: glowing pink-red ball ----
    lv_draw_rect_dsc_init(&d);           // Reset descriptor
    d.bg_color = lv_palette_main(LV_PALETTE_PINK);   // Pink fill
    d.radius = LV_RADIUS_CIRCLE;         // Round shape
    d.border_width = 2;                  // Thin outline ...
    d.border_color = lv_color_white();   // ... in white for a shiny look
    d.shadow_width = 18;                 // Soft glow extending 18 px around the ball
    d.shadow_color = lv_palette_main(LV_PALETTE_PINK);   // Glow uses the same pink
    d.shadow_opa = LV_OPA_70;            // 70% glow strength
    cell_area(&oa, food_x, food_y, 4, &a);   // Food cell inset by 4 px so it is smaller than the snake
    lv_draw_rect(dc, &d, &a);            // Paint the food

    // ---- snake body: drawn from tail to head so the head ends up on top ----
    for (int i = snake_len - 1; i >= 0; i--) {   // Iterate from the last segment down to the head
        lv_draw_rect_dsc_init(&d);       // Reset descriptor
        int hue = (hue_base + i * 12) % 360;     // Each segment is 12 degrees further around the color wheel than the one before
        d.bg_color = lv_color_hsv_to_rgb(hue, 85, 100);   // Convert hue/saturation/value (0-360, 0-100, 0-100) to an RGB565-ready color
        d.radius = (i == 0) ? 10 : 8;    // Head slightly rounder than the body
        d.border_width = (i == 0) ? 2 : 0;   // Only the head gets an outline
        d.border_color = lv_color_white();   // Head outline color
        int inset = (i == 0) ? 1 : 2;    // Head fills more of the cell than the body
        cell_area(&oa, snake_x[i], snake_y[i], inset, &a);   // Pixel rectangle of this segment
        if (_lv_area_is_on(&a, dc->clip_area)) {   // Only paint if it overlaps the region being repainted
            lv_draw_rect(dc, &d, &a);    // Paint the segment
        }                                // end of clip test
    }                                    // end of body loop

    // ---- snake eyes: two dots on the front half of the head ----
    int hx = oa.x1 + snake_x[0] * CELL + CELL / 2;   // Pixel X of the head's center
    int hy = oa.y1 + snake_y[0] * CELL + CELL / 2;   // Pixel Y of the head's center
    int fx = DX[dir] * 7;                // Forward offset X (7 px toward the direction of travel)
    int fy = DY[dir] * 7;                // Forward offset Y
    int sx = -DY[dir] * 7;               // Sideways offset X (perpendicular to travel)
    int sy = DX[dir] * 7;                // Sideways offset Y
    draw_dot(dc, hx + fx + sx, hy + fy + sy, 10, lv_color_white());   // Left eye white
    draw_dot(dc, hx + fx - sx, hy + fy - sy, 10, lv_color_white());   // Right eye white
    draw_dot(dc, hx + fx + sx + DX[dir], hy + fy + sy + DY[dir], 5, lv_color_black());   // Left pupil nudged forward
    draw_dot(dc, hx + fx - sx + DX[dir], hy + fy - sy + DY[dir], 5, lv_color_black());   // Right pupil nudged forward
}                                        // end of arena_draw_cb

// ==================== SECTION 11: TOUCH BUTTON HANDLERS ====================
// LVGL's touch driver (lvgl_v8_port.cpp -> GT911) turns finger presses into LV_EVENT_PRESSED on the button under the finger.
// LVGL automatically applies LV_STATE_PRESSED styling, so no manual state changes are needed (unlike Lesson 10).
static void dir_button_cb(lv_event_t *e) // Shared handler for the four arrow buttons
{                                        // start of dir_button_cb
    int wanted = (int)(intptr_t)lv_event_get_user_data(e);   // Direction id stored as user data when the button was created

    if (game_state == STATE_OVER) {      // After a crash the arrows do nothing
        return;                          // The player must press NEW GAME
    }                                    // end of over check

    if ((wanted ^ 1) != dir) {           // XOR 1 gives the opposite direction; reject a 180-degree reversal into the neck
        next_dir = wanted;               // Accept the turn; it is applied at the next game tick
    }                                    // end of reversal check

    if (game_state == STATE_READY) {     // The first press starts the game
        game_state = STATE_RUNNING;      // Switch to running
        set_overlay(NULL);               // Hide the "tap to start" message
        led_flash(0, 0, 0, 0);           // Turn the LEDs off (all zero color, no timer)
    }                                    // end of ready check
}                                        // end of dir_button_cb

static void new_game_cb(lv_event_t *e)   // Handler for the NEW GAME button
{                                        // start of new_game_cb
    (void)e;                             // The event object is not needed
    reset_game();                        // Restart from the READY state
}                                        // end of new_game_cb

// ==================== SECTION 12: UI BUILDING HELPERS ====================
// Create one square, rounded, colored arrow button inside 'parent' at (x, y).
static void make_dir_button(lv_obj_t *parent, int x, int y, const char *symbol, lv_palette_t pal, int dir_id)   // Build one D-pad key
{                                        // start of make_dir_button
    lv_obj_t *btn = lv_btn_create(parent);               // Create a button widget as a child of the panel
    lv_obj_set_size(btn, 100, 100);                      // 100x100 px is a comfortable finger target
    lv_obj_set_pos(btn, x, y);                           // Position relative to the parent's top-left corner
    lv_obj_set_style_radius(btn, 24, LV_STATE_DEFAULT);  // Rounded corners
    lv_obj_set_style_bg_color(btn, lv_palette_main(pal), LV_STATE_DEFAULT);        // Normal fill color
    lv_obj_set_style_bg_color(btn, lv_palette_darken(pal, 3), LV_STATE_PRESSED);   // Darker fill while the finger is down
    lv_obj_set_style_border_width(btn, 3, LV_STATE_DEFAULT);                       // 3 px outline
    lv_obj_set_style_border_color(btn, lv_palette_lighten(pal, 3), LV_STATE_DEFAULT);   // Light tint of the same color for the outline
    lv_obj_set_style_shadow_width(btn, 16, LV_STATE_DEFAULT);                      // Soft glow around the button
    lv_obj_set_style_shadow_color(btn, lv_palette_main(pal), LV_STATE_DEFAULT);    // Glow in the button's color
    lv_obj_set_style_shadow_opa(btn, LV_OPA_50, LV_STATE_DEFAULT);                 // Half-strength glow
    lv_obj_t *lbl = lv_label_create(btn);                // Create the arrow label inside the button
    lv_label_set_text(lbl, symbol);                      // LV_SYMBOL_* are UTF-8 strings that map to icon glyphs inside the Montserrat fonts
    lv_obj_set_style_text_font(lbl, FONT_BIG, LV_PART_MAIN);                       // Use the big font so the arrow is easy to see
    lv_obj_set_style_text_color(lbl, lv_color_white(), LV_PART_MAIN);              // White arrow
    lv_obj_center(lbl);                                  // Center the arrow in the button
    lv_obj_add_event_cb(btn, dir_button_cb, LV_EVENT_PRESSED, (void *)(intptr_t)dir_id);   // Fire immediately on touch-down, passing the direction id as user data
}                                        // end of make_dir_button

// ==================== SECTION 13: UI CONSTRUCTION ====================
static void build_ui(void)               // Create every LVGL widget of the game
{                                        // start of build_ui
    lv_obj_t *scr = lv_scr_act();        // The active screen: the root object everything else hangs from
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);   // Black screen background
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);         // Fully opaque background
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);                   // Prevent accidental screen scrolling when dragging a finger

    // ---- left: the arena ----
    arena = lv_obj_create(scr);          // A plain object; we paint the snake ourselves in arena_draw_cb
    lv_obj_set_size(arena, ARENA_W, ARENA_H);                         // 660 x 600 px
    lv_obj_set_pos(arena, 0, 0);         // Flush against the top-left corner of the screen
    lv_obj_set_style_radius(arena, 0, 0);                             // Square corners (the theme would round them)
    lv_obj_set_style_border_width(arena, 0, 0);                       // No border so grid math starts at pixel 0
    lv_obj_set_style_pad_all(arena, 0, 0);                            // No padding
    lv_obj_set_style_bg_color(arena, lv_color_hex(0x0B1026), 0);      // Deep navy at the top ...
    lv_obj_set_style_bg_grad_color(arena, lv_color_hex(0x2A0B3D), 0); // ... fading to purple at the bottom
    lv_obj_set_style_bg_grad_dir(arena, LV_GRAD_DIR_VER, 0);          // Vertical gradient direction
    lv_obj_clear_flag(arena, LV_OBJ_FLAG_SCROLLABLE);                 // The arena must never scroll
    lv_obj_clear_flag(arena, LV_OBJ_FLAG_CLICKABLE);                  // Arena ignores touches; only the buttons are interactive
    lv_obj_add_event_cb(arena, arena_draw_cb, LV_EVENT_DRAW_MAIN_END, NULL);   // Call our painter after the background is drawn

    overlay_label = lv_label_create(arena);   // Message box shown on top of the arena
    lv_label_set_text(overlay_label, "");     // Start with empty text
    lv_obj_set_style_text_font(overlay_label, FONT_BIG, 0);           // Big font
    lv_obj_set_style_text_color(overlay_label, lv_color_white(), 0);  // White text
    lv_obj_set_style_text_align(overlay_label, LV_TEXT_ALIGN_CENTER, 0);   // Center each text line
    lv_obj_set_style_bg_color(overlay_label, lv_color_black(), 0);    // Black box behind the text
    lv_obj_set_style_bg_opa(overlay_label, LV_OPA_70, 0);             // 70% opaque so the arena shows through slightly
    lv_obj_set_style_pad_all(overlay_label, 20, 0);                   // 20 px padding inside the box
    lv_obj_set_style_radius(overlay_label, 18, 0);                    // Rounded box corners
    lv_obj_set_style_border_width(overlay_label, 3, 0);               // Colored outline ...
    lv_obj_set_style_border_color(overlay_label, lv_palette_main(LV_PALETTE_CYAN), 0);   // ... in cyan

    // ---- right: the control panel ----
    lv_obj_t *panel = lv_obj_create(scr);     // Container for score and buttons
    lv_obj_set_size(panel, PANEL_W, SCREEN_H);                        // 364 x 600 px
    lv_obj_set_pos(panel, ARENA_W, 0);        // Starts where the arena ends (x = 660)
    lv_obj_set_style_radius(panel, 0, 0);     // Square corners
    lv_obj_set_style_pad_all(panel, 0, 0);    // No padding so children use exact coordinates
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x101830), 0);      // Dark blue top ...
    lv_obj_set_style_bg_grad_color(panel, lv_color_hex(0x000000), 0); // ... to black bottom
    lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_VER, 0);          // Vertical gradient
    lv_obj_set_style_border_width(panel, 4, 0);                       // 4 px border, but only on one side (next line)
    lv_obj_set_style_border_side(panel, LV_BORDER_SIDE_LEFT, 0);      // Draw the border only on the left edge as a divider
    lv_obj_set_style_border_color(panel, lv_palette_main(LV_PALETTE_CYAN), 0);   // Cyan divider line
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);                 // Panel never scrolls

    lv_obj_t *title = lv_label_create(panel); // Game title
    lv_label_set_text(title, "SNAKE");        // Title text
    lv_obj_set_style_text_font(title, FONT_BIG, 0);                   // Big font
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_LIGHT_GREEN), 0);   // Lime green
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);                     // Top-center, 10 px down

    lv_obj_t *card = lv_obj_create(panel);    // Score card background
    lv_obj_set_size(card, 320, 120);          // Card size
    lv_obj_align(card, LV_ALIGN_TOP_MID, 0, 52);                      // Centered horizontally, 52 px from the top
    lv_obj_set_style_radius(card, 20, 0);     // Rounded card
    lv_obj_set_style_pad_all(card, 0, 0);     // No padding
    lv_obj_set_style_bg_color(card, lv_palette_main(LV_PALETTE_PURPLE), 0);           // Purple top ...
    lv_obj_set_style_bg_grad_color(card, lv_palette_main(LV_PALETTE_ORANGE), 0);      // ... to orange bottom
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);           // Vertical gradient
    lv_obj_set_style_border_width(card, 3, 0);                        // Outline
    lv_obj_set_style_border_color(card, lv_color_white(), 0);         // White outline
    lv_obj_set_style_shadow_width(card, 24, 0);                       // Glow
    lv_obj_set_style_shadow_color(card, lv_palette_main(LV_PALETTE_ORANGE), 0);       // Orange glow
    lv_obj_set_style_shadow_opa(card, LV_OPA_50, 0);                  // Half-strength glow
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);                  // No scrolling
    lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);                   // Not touchable

    lv_obj_t *score_caption = lv_label_create(card);                  // "SCORE" caption inside the card
    lv_label_set_text(score_caption, "SCORE");                        // Caption text
    lv_obj_set_style_text_font(score_caption, FONT_MED, 0);           // Medium font
    lv_obj_set_style_text_color(score_caption, lv_color_white(), 0);  // White text
    lv_obj_align(score_caption, LV_ALIGN_TOP_MID, 0, 8);              // Top-center of the card

    score_value_label = lv_label_create(card);                        // The dynamic score number
    lv_label_set_text(score_value_label, "0");                        // Initial value
    lv_obj_set_style_text_font(score_value_label, FONT_HUGE, 0);      // Largest available font
    lv_obj_set_style_text_color(score_value_label, lv_color_white(), 0);   // White digits
    lv_obj_align(score_value_label, LV_ALIGN_BOTTOM_MID, 0, -8);      // Bottom-center of the card

    info_label = lv_label_create(panel);      // Length / best score line
    lv_label_set_text(info_label, "");        // Filled in by update_hud()
    lv_obj_set_style_text_font(info_label, FONT_MED, 0);              // Medium font
    lv_obj_set_style_text_color(info_label, lv_palette_lighten(LV_PALETTE_CYAN, 2), 0);   // Light cyan
    lv_obj_align(info_label, LV_ALIGN_TOP_MID, 0, 180);               // Under the score card

    lv_obj_t *new_btn = lv_btn_create(panel); // NEW GAME button
    lv_obj_set_size(new_btn, 320, 44);        // Wide, short button
    lv_obj_align(new_btn, LV_ALIGN_TOP_MID, 0, 212);                  // Under the info line
    lv_obj_set_style_radius(new_btn, 14, 0);  // Rounded corners
    lv_obj_set_style_bg_color(new_btn, lv_palette_main(LV_PALETTE_TEAL), LV_STATE_DEFAULT);        // Teal normally
    lv_obj_set_style_bg_color(new_btn, lv_palette_darken(LV_PALETTE_TEAL, 3), LV_STATE_PRESSED);   // Darker when pressed
    lv_obj_t *new_lbl = lv_label_create(new_btn);                     // Button text
    lv_label_set_text(new_lbl, LV_SYMBOL_REFRESH " NEW GAME");        // Refresh icon followed by text (string literal concatenation)
    lv_obj_set_style_text_font(new_lbl, FONT_MED, 0);                 // Medium font
    lv_obj_set_style_text_color(new_lbl, lv_color_white(), 0);        // White text
    lv_obj_center(new_lbl);                   // Center the text
    lv_obj_add_event_cb(new_btn, new_game_cb, LV_EVENT_CLICKED, NULL);   // Fire on a full tap (press + release)

    // ---- D-pad: 3 rows x 3 columns of 100 px cells with 10 px gaps ----
    make_dir_button(panel, 132, 268, LV_SYMBOL_UP,    LV_PALETTE_BLUE,   DIR_UP);      // Top-center: UP
    make_dir_button(panel,  22, 378, LV_SYMBOL_LEFT,  LV_PALETTE_GREEN,  DIR_LEFT);    // Middle-left: LEFT
    make_dir_button(panel, 242, 378, LV_SYMBOL_RIGHT, LV_PALETTE_PURPLE, DIR_RIGHT);   // Middle-right: RIGHT
    make_dir_button(panel, 132, 488, LV_SYMBOL_DOWN,  LV_PALETTE_ORANGE, DIR_DOWN);    // Bottom-center: DOWN

    lv_obj_t *hub = lv_obj_create(panel);     // Decorative disc in the middle of the D-pad
    lv_obj_set_size(hub, 100, 100);           // Same size as a button cell
    lv_obj_set_pos(hub, 132, 378);            // Center cell of the 3x3 layout
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);                // Perfect circle
    lv_obj_set_style_bg_color(hub, lv_color_hex(0x1B2444), 0);        // Dark blue fill
    lv_obj_set_style_border_width(hub, 4, 0);                         // Thick ring
    lv_obj_set_style_border_color(hub, lv_palette_main(LV_PALETTE_CYAN), 0);   // Cyan ring
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_CLICKABLE);                    // Decoration only
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_SCROLLABLE);                   // No scrolling
}                                        // end of build_ui

// ==================== SECTION 14: ARDUINO setup() ====================
void setup()                             // Runs once after reset
{                                        // start of setup
    pinMode(32, OUTPUT);                 // GPIO 32 as output (board-specific line set up by Lesson 10; kept identical)
    pinMode(33, OUTPUT);                 // GPIO 33 as output (same)
    digitalWrite(32, 0);                 // GPIO 32 low (as in Lesson 10)
    digitalWrite(33, 1);                 // GPIO 33 high (as in Lesson 10)
    Serial.begin(115200);                // Open the USB serial port at 115200 baud

    Serial.println("Initializing board");   // Progress message

    board_p4_ldo_init();                 // Switch on the 2.5 V MIPI and 3.3 V rails BEFORE touching the display

    strip.begin();                       // Configure the NeoPixel output pin and driver
    strip.show();                        // Send the initial all-off state to the LEDs
    strip.setBrightness(50);             // Limit brightness to 50/255 so the LEDs are not blinding

    randomSeed(esp_random());            // Seed Arduino's random() with a true random 32-bit value from the hardware RNG

    Board *board = new Board();          // Create the board object; it reads esp_panel_board_custom_conf.h (1024x600, MIPI-DSI EK79007, GT911 touch)
    board->init();                       // Allocate and configure the LCD, touch, and backlight drivers
    board->begin();                      // Actually start the drivers (MIPI DSI link, I2C touch controller)

    lvgl_port_init(board->getLCD(), board->getTouch());   // Initialize LVGL, register the display and touch input, and start the LVGL task
    board->getBacklight()->setBrightness(0);              // Keep the backlight off while the UI is built to avoid showing garbage

    lvgl_port_lock(-1);                  // Take the LVGL mutex (wait forever) because the LVGL task is already running in the background
    build_ui();                          // Create all widgets
    game_timer = lv_timer_create(game_tick, START_PERIOD_MS, NULL);   // Create the periodic timer that moves the snake
    reset_game();                        // Place the snake and food, set the HUD, show the start message
    lvgl_port_unlock();                  // Release the mutex so the LVGL task can render the screen

    delay(200);                          // Give LVGL a moment to draw the first frame
    board->getBacklight()->setBrightness(100);   // Turn the backlight on at 100 %
}                                        // end of setup

// ==================== SECTION 15: ARDUINO loop() ====================
void loop()                              // Runs forever after setup()
{                                        // start of loop
    delay(1000);                         // Nothing to do here: the LVGL task handles drawing/touch and the LVGL timer runs the game
}                                        // end of loop
