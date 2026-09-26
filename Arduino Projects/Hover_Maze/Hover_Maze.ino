#include "Arduino.h"
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"

/////// 1. PIN CONNECTIONS
const byte TRIGGER_PIN   = 13;   // Sonic transmitter pulse pin
const byte ECHO_PIN      = 12;   // Sonic receiver echo pin
const int  BL_PIN_A      = 32;   // Primary screen backlight line
const int  BL_PIN_B      = 33;   // Secondary screen backlight line
const int  TOUCH_PAD_PIN = 2;    // Touch pad module (TTP223). -1 = touch screen only
const bool TOUCH_PAD_ACTIVE_HIGH = true; // TTP223 modules output HIGH while touched

/////// 2. DISPLAY & PLAYER
const int SCREEN_WIDTH  = 1024;  // Screen width pixels
const int SCREEN_HEIGHT = 600;   // Screen height pixels
const int ICON_X        = 200;   // Fixed horizontal position of the player
const int ICON_SIZE     = 56;    // Player face diameter (big for small children)
const int HIT_MARGIN    = 10;    // Forgiving hitbox: walls may overlap the face edge by this much
const int START_Y       = (SCREEN_HEIGHT - ICON_SIZE) / 2; // Player always starts in the middle
const int PLAY_TOP      = 10;                              // Highest player Y
const int PLAY_BOTTOM   = SCREEN_HEIGHT - ICON_SIZE - 10;  // Lowest player Y

/////// 3. HAND CONTROL TUNING
const float HAND_MIN_CM   = 5.0;  // Hand this close (or closer) = player at the bottom
const float HAND_MAX_CM   = 30.0; // Hand this high (or higher) = player at the top
const float HAND_LOST_CM  = 45.0; // Readings beyond this mean "no hand" -> player holds still
const unsigned long SENSOR_INTERVAL_MS = 40; // Time between ultrasonic pings (lets each echo finish)
const int   MISSES_BEFORE_LOST = 5; // Missed pings in a row before the hand counts as gone
const float HAND_SMOOTHING = 0.45; // 0..1  higher = reacts faster, lower = steadier
const float GLIDE          = 0.25; // 0..1  how quickly the player glides to the hand position
const float DEADBAND_PX    = 4.0;  // Ignore tiny hand wobbles smaller than this

/////// 4. MAZE TUNING
const int GAP_SIZE       = 240;  // Height of the open path
const int SEGMENT_WIDTH  = 128;  // Width of one hedge column
const int NUM_SEGMENTS   = 10;   // Columns needed to cover the screen plus the entrance
const int GAP_MARGIN     = 70;   // Keep the path away from the very top/bottom
const int MAX_GAP_STEP   = 45;   // Max up/down change between neighbouring columns (gentle curves)
const int FLAT_SEGMENTS  = 5;    // Straight entrance columns lined up with the middle
const int START_OFFSET   = 560;  // Maze starts this far right so the entrance is visible
const int WALL_OVERHANG  = 30;   // Walls extend off-screen so only the path-side corners are rounded
const int BASE_SPEED     = 4;    // Pixels per frame at the start
const int MAX_SPEED      = 8;    // Top speed
const int POINTS_PER_SPEEDUP = 15; // Speed rises by 1 every this many points
const int START_LIVES    = 3;    // Bumps allowed before the round ends (set to 1 for no lives)
const unsigned long INVINCIBLE_MS      = 2000; // Safe blinking time after a bump
const unsigned long COUNTDOWN_STEP_MS  = 800;  // Time for each of 3, 2, 1
const unsigned long RESTART_LOCKOUT_MS = 1200; // Ignore taps right after game over

/////// 5. COLOURS
#define COLOR_SKY_TOP      lv_color_hex(0x6EC6FF)
#define COLOR_SKY_BOTTOM   lv_color_hex(0xE3F6FF)
#define COLOR_HEDGE_A      lv_color_hex(0x43A047)
#define COLOR_HEDGE_B      lv_color_hex(0x66BB6A)
#define COLOR_HEDGE_EDGE   lv_color_hex(0x2E7D32)
#define COLOR_PLAYER       lv_color_hex(0xFFD54F)
#define COLOR_PLAYER_EDGE  lv_color_hex(0xF57F17)
#define COLOR_DARK         lv_color_hex(0x263238)
#define COLOR_ACCENT       lv_color_hex(0xFF7043)
#define COLOR_TITLE        lv_color_hex(0x1E88E5)

/////// 6. COLUMN DATA
struct MazeSegment {
    int x;                 // Left edge
    int gapY;              // Top of the open path
    bool passed;           // Already scored
    lv_obj_t *top_wall;
    lv_obj_t *bottom_wall;
};
MazeSegment maze[NUM_SEGMENTS];
int lastGapY = 0;          // Gap of the right-most column, used to build the next one

/////// 7. GAME STATE
enum GameState { STATE_TITLE, STATE_READY, STATE_PLAYING, STATE_GAMEOVER };
GameState state = STATE_TITLE;
unsigned long stateTimer = 0;
unsigned long invincibleUntil = 0;
float iconY = START_Y;
int score = 0;
int bestScore = 0;
int lives = START_LIVES;

/////// 8. SHARED WITH loop() (sensor + touch pad run outside the LVGL task)
volatile float handTargetY = START_Y;
volatile bool handPresent = false;
volatile bool tapRequested = false;

/////// 9. DISPLAY OBJECTS
lv_obj_t *player_icon    = NULL;
lv_obj_t *hud_score      = NULL;
lv_obj_t *score_label    = NULL;
lv_obj_t *hud_lives      = NULL;
lv_obj_t *life_icons[START_LIVES];
lv_obj_t *title_cont     = NULL;
lv_obj_t *game_over_cont = NULL;
lv_obj_t *go_score_label = NULL;
lv_obj_t *countdown_cont = NULL;
lv_obj_t *countdown_label = NULL;
lv_obj_t *hint_cont      = NULL;

/////// 10. SMALL UI HELPERS
static void make_plain(lv_obj_t *o) {
    lv_obj_remove_style_all(o); // Drop theme padding/borders/shadows
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE); // Let taps fall through to the screen
}

static void set_visible(lv_obj_t *o, bool visible) {
    bool hidden = lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN);
    if (visible && hidden) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
    else if (!visible && !hidden) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *create_panel(lv_obj_t *parent, int w, int h, lv_color_t border) {
    lv_obj_t *p = lv_obj_create(parent);
    make_plain(p);
    lv_obj_set_size(p, w, h);
    lv_obj_set_style_radius(p, 36, 0);
    lv_obj_set_style_bg_color(p, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(p, border, 0);
    lv_obj_set_style_border_width(p, 6, 0);
    lv_obj_set_style_shadow_width(p, 24, 0);
    lv_obj_set_style_shadow_opa(p, LV_OPA_30, 0);
    lv_obj_align(p, LV_ALIGN_CENTER, 0, 0);
    return p;
}

static lv_obj_t *create_label(lv_obj_t *parent, const char *text, const lv_font_t *font, lv_color_t color) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    return l;
}

// Rounded orange "button" with a gently pulsing message, e.g. "Tap the Touch Pad to Start"
static void pulse_cb(void *obj, int32_t v) {
    lv_obj_set_style_text_opa((lv_obj_t *)obj, v, 0);
}

static lv_obj_t *create_tap_prompt(lv_obj_t *parent, const char *text) {
    lv_obj_t *pill = lv_obj_create(parent);
    make_plain(pill);
    lv_obj_set_size(pill, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(pill, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(pill, COLOR_ACCENT, 0);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(pill, 36, 0);
    lv_obj_set_style_pad_ver(pill, 16, 0);

    lv_obj_t *l = create_label(pill, text, &lv_font_montserrat_32, lv_color_white());
    lv_obj_center(l);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, l);
    lv_anim_set_exec_cb(&a, pulse_cb);
    lv_anim_set_values(&a, 110, 255);
    lv_anim_set_time(&a, 700);
    lv_anim_set_playback_time(&a, 700);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
    return pill;
}

// Friendly round face used for the player, the lives and the menus
static lv_obj_t *create_face(lv_obj_t *parent, int size) {
    lv_obj_t *body = lv_obj_create(parent);
    make_plain(body);
    lv_obj_set_size(body, size, size);
    lv_obj_set_style_radius(body, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(body, COLOR_PLAYER, 0);
    lv_obj_set_style_bg_opa(body, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(body, COLOR_PLAYER_EDGE, 0);
    lv_obj_set_style_border_width(body, LV_MAX(2, size / 14), 0);

    int eye = size * 26 / 100;
    int pupil = eye / 2;
    for (int side = -1; side <= 1; side += 2) {
        lv_obj_t *e = lv_obj_create(body);
        make_plain(e);
        lv_obj_set_size(e, eye, eye);
        lv_obj_set_style_radius(e, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(e, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(e, LV_OPA_COVER, 0);
        lv_obj_align(e, LV_ALIGN_CENTER, side * size * 17 / 100, -size * 12 / 100);

        lv_obj_t *p = lv_obj_create(e);
        make_plain(p);
        lv_obj_set_size(p, pupil, pupil);
        lv_obj_set_style_radius(p, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(p, COLOR_DARK, 0);
        lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
        lv_obj_align(p, LV_ALIGN_CENTER, eye / 6, 0); // Looking right, towards the maze
    }

    lv_obj_t *smile = lv_arc_create(body);
    lv_obj_remove_style_all(smile);
    lv_obj_clear_flag(smile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(smile, size * 46 / 100, size * 46 / 100);
    lv_arc_set_bg_angles(smile, 20, 160); // Bottom half of a circle = smile
    lv_obj_set_style_arc_width(smile, LV_MAX(2, size / 14), LV_PART_MAIN);
    lv_obj_set_style_arc_color(smile, COLOR_DARK, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(smile, true, LV_PART_MAIN);
    lv_obj_align(smile, LV_ALIGN_CENTER, 0, size * 4 / 100);
    return body;
}

static lv_obj_t *create_wall(lv_obj_t *parent, lv_color_t color) {
    lv_obj_t *w = lv_obj_create(parent);
    make_plain(w);
    lv_obj_set_style_radius(w, 24, 0);
    lv_obj_set_style_bg_color(w, color, 0);
    lv_obj_set_style_bg_opa(w, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(w, COLOR_HEDGE_EDGE, 0);
    lv_obj_set_style_border_width(w, 4, 0);
    return w;
}

static void create_cloud(lv_obj_t *parent, int x, int y, int w) {
    lv_obj_t *c = lv_obj_create(parent);
    make_plain(c);
    lv_obj_set_size(c, w, w * 2 / 5);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(c, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_80, 0);
}

/////// 11. MAZE HELPERS
static int next_gap() {
    lastGapY = constrain(lastGapY + (int)random(-MAX_GAP_STEP, MAX_GAP_STEP + 1),
                         GAP_MARGIN, SCREEN_HEIGHT - GAP_SIZE - GAP_MARGIN);
    return lastGapY;
}

static void apply_wall_size(int i) {
    lv_obj_set_size(maze[i].top_wall, SEGMENT_WIDTH + 2, maze[i].gapY + WALL_OVERHANG);
    lv_obj_set_size(maze[i].bottom_wall, SEGMENT_WIDTH + 2,
                    SCREEN_HEIGHT - (maze[i].gapY + GAP_SIZE) + WALL_OVERHANG);
}

static void apply_wall_pos(int i) {
    lv_obj_set_pos(maze[i].top_wall, maze[i].x, -WALL_OVERHANG);
    lv_obj_set_pos(maze[i].bottom_wall, maze[i].x, maze[i].gapY + GAP_SIZE);
}

static void update_score_label() {
    lv_label_set_text_fmt(score_label, "Score: %d", score);
}

static void update_lives() {
    for (int i = 0; i < START_LIVES; i++) set_visible(life_icons[i], i < lives);
}

/////// 12. ROUND SETUP
void resetMaze() {
    lastGapY = (SCREEN_HEIGHT - GAP_SIZE) / 2; // Path starts level with the middle of the screen
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        maze[i].x = START_OFFSET + i * SEGMENT_WIDTH;
        maze[i].gapY = (i < FLAT_SEGMENTS) ? lastGapY : next_gap();
        maze[i].passed = false;
        apply_wall_size(i);
        apply_wall_pos(i);
    }
}

void startRound() {
    score = 0;
    lives = START_LIVES;
    iconY = START_Y;
    invincibleUntil = 0;
    resetMaze();
    update_score_label();
    update_lives();
    lv_obj_set_pos(player_icon, ICON_X, (int)iconY);
    set_visible(player_icon, true);
    set_visible(hud_score, true);
    set_visible(hud_lives, true);
    set_visible(title_cont, false);
    set_visible(game_over_cont, false);
    lv_label_set_text(countdown_label, "3");
    set_visible(countdown_cont, true);
    state = STATE_READY;
    stateTimer = millis();
}

void endRound() {
    if (score > bestScore) bestScore = score;
    lv_label_set_text_fmt(go_score_label, "Score: %d      Best: %d", score, bestScore);
    set_visible(player_icon, true);
    set_visible(hint_cont, false);
    set_visible(countdown_cont, false);
    set_visible(game_over_cont, true);
    state = STATE_GAMEOVER;
    stateTimer = millis();
}

/////// 13. GAMEPLAY FRAME
static void update_play(unsigned long now) {
    // Follow the hand smoothly; with no hand in view the player simply stays put
    if (handPresent) iconY += (handTargetY - iconY) * GLIDE;
    iconY = constrain(iconY, (float)PLAY_TOP, (float)PLAY_BOTTOM);

    int speed = min(MAX_SPEED, BASE_SPEED + score / POINTS_PER_SPEEDUP);
    for (int i = 0; i < NUM_SEGMENTS; i++) maze[i].x -= speed;

    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (maze[i].x + SEGMENT_WIDTH < 0) { // Recycle columns that left the screen
            int rightmost = maze[0].x;
            for (int j = 1; j < NUM_SEGMENTS; j++) rightmost = max(rightmost, maze[j].x);
            maze[i].x = rightmost + SEGMENT_WIDTH;
            maze[i].gapY = next_gap();
            maze[i].passed = false;
            apply_wall_size(i);
        }
        if (!maze[i].passed && maze[i].x + SEGMENT_WIDTH < ICON_X) { // One point per column passed
            maze[i].passed = true;
            score++;
            update_score_label();
        }
        apply_wall_pos(i);
    }
    lv_obj_set_pos(player_icon, ICON_X, (int)iconY);

    // Collision with a forgiving hitbox
    if (now >= invincibleUntil) {
        int left = ICON_X + HIT_MARGIN, right = ICON_X + ICON_SIZE - HIT_MARGIN;
        int top = (int)iconY + HIT_MARGIN, bottom = (int)iconY + ICON_SIZE - HIT_MARGIN;
        for (int i = 0; i < NUM_SEGMENTS; i++) {
            if (right > maze[i].x && left < maze[i].x + SEGMENT_WIDTH &&
                (top < maze[i].gapY || bottom > maze[i].gapY + GAP_SIZE)) {
                lives--;
                update_lives();
                if (lives <= 0) { endRound(); return; }
                invincibleUntil = now + INVINCIBLE_MS;
                break;
            }
        }
    }

    // Blink while safe after a bump
    set_visible(player_icon, now >= invincibleUntil || ((now / 120) % 2 == 0));

    // Hide "GO!" shortly after the start
    if (now - stateTimer > COUNTDOWN_STEP_MS) set_visible(countdown_cont, false);
}

/////// 14. MAIN GAME TICK (runs inside the LVGL task every 30ms)
static void game_tick_cb(lv_timer_t *timer) {
    unsigned long now = millis();
    bool tapped = tapRequested;
    tapRequested = false;

    switch (state) {
    case STATE_TITLE:
        if (tapped) startRound();
        break;

    case STATE_READY: {
        // Player waits in the middle while counting down 3, 2, 1
        unsigned long step = (now - stateTimer) / COUNTDOWN_STEP_MS;
        if (step >= 3) {
            lv_label_set_text(countdown_label, "GO!");
            state = STATE_PLAYING;
            stateTimer = now;
        } else {
            static const char *nums[] = {"3", "2", "1"};
            if (strcmp(lv_label_get_text(countdown_label), nums[step]) != 0) {
                lv_label_set_text(countdown_label, nums[step]);
            }
        }
        set_visible(hint_cont, !handPresent);
        break;
    }

    case STATE_PLAYING:
        set_visible(hint_cont, !handPresent);
        update_play(now);
        break;

    case STATE_GAMEOVER:
        if (tapped && now - stateTimer > RESTART_LOCKOUT_MS) startRound();
        break;
    }
}

static void screen_tap_cb(lv_event_t *e) {
    tapRequested = true; // Tapping anywhere on the screen works as well as the touch pad
}

/////// 15. HAND SENSOR (median of 3 + smoothing, called from loop())
// The echo is timed by an interrupt instead of a busy-wait. While the maze scrolls, the screen
// drawing task interrupts loop(), which made busy-wait timings far too long and the hand "vanish".
volatile unsigned long echoStartUs = 0;
volatile unsigned long echoEndUs = 0;
volatile bool echoDone = false;

void IRAM_ATTR echoISR() {
    if (digitalRead(ECHO_PIN)) {
        echoStartUs = micros();
    } else {
        echoEndUs = micros();
        echoDone = true;
    }
}

static void triggerPing() {
    echoDone = false;
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
}

// Distance from the previous ping in cm, or -1 if no echo came back
static float readLastPing() {
    if (!echoDone) return -1;
    return (echoEndUs - echoStartUs) / 58.0;
}

static float median3(float a, float b, float c) {
    return max(min(a, b), min(max(a, b), c));
}

void updateHand() {
    static float samples[3];
    static int sampleCount = 0, sampleIdx = 0, missCount = 0;
    static float smoothY = START_Y;

    float d = readLastPing();
    triggerPing(); // Start the next measurement; its echo is timed in the background

    if (d <= 0 || d > HAND_LOST_CM) {
        if (++missCount >= MISSES_BEFORE_LOST) { // Ignore short dropouts
            handPresent = false;
            sampleCount = 0;
        }
        return;
    }
    missCount = 0;

    samples[sampleIdx] = d;
    sampleIdx = (sampleIdx + 1) % 3;
    if (sampleCount < 3) sampleCount++;
    float m = (sampleCount < 3) ? d : median3(samples[0], samples[1], samples[2]); // Rejects spikes

    float frac = (constrain(m, HAND_MIN_CM, HAND_MAX_CM) - HAND_MIN_CM) / (HAND_MAX_CM - HAND_MIN_CM);
    float y = PLAY_BOTTOM + frac * (PLAY_TOP - PLAY_BOTTOM); // Hand high = player high

    if (!handPresent) smoothY = y; // Hand just arrived: no lag from old readings
    else if (fabsf(y - smoothY) > DEADBAND_PX) smoothY += (y - smoothY) * HAND_SMOOTHING;
    handTargetY = smoothY;
    handPresent = true;
}

void pollTouchPad() {
    if (TOUCH_PAD_PIN < 0) return;
    static bool wasTouched = false;
    bool touched = digitalRead(TOUCH_PAD_PIN) == (TOUCH_PAD_ACTIVE_HIGH ? HIGH : LOW);
    if (touched && !wasTouched) tapRequested = true; // Trigger once per tap
    wasTouched = touched;
}

/////// 16. SETUP
void setup()
{
    pinMode(BL_PIN_A, OUTPUT); pinMode(BL_PIN_B, OUTPUT);
    digitalWrite(BL_PIN_A, 0); digitalWrite(BL_PIN_B, 1);
    if (TOUCH_PAD_PIN >= 0) pinMode(TOUCH_PAD_PIN, INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

    Serial.begin(115200);

    esp_panel::board::Board *board = new esp_panel::board::Board();
    board->init();
    board->begin();

    lvgl_port_init(board->getLCD(), board->getTouch());
    board->getBacklight()->setBrightness(100);

    lvgl_port_lock(-1);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE); // Off-screen walls must not make the screen scroll
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(scr, COLOR_SKY_TOP, 0);
    lv_obj_set_style_bg_grad_color(scr, COLOR_SKY_BOTTOM, 0);
    lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_VER, 0);
    lv_obj_add_event_cb(scr, screen_tap_cb, LV_EVENT_PRESSED, NULL);

    create_cloud(scr, 90, 60, 170);
    create_cloud(scr, 470, 110, 220);
    create_cloud(scr, 800, 40, 150);

    for (int i = 0; i < NUM_SEGMENTS; i++) {
        lv_color_t c = (i % 2 == 0) ? COLOR_HEDGE_A : COLOR_HEDGE_B;
        maze[i].top_wall = create_wall(scr, c);
        maze[i].bottom_wall = create_wall(scr, c);
    }

    player_icon = create_face(scr, ICON_SIZE);
    lv_obj_set_pos(player_icon, ICON_X, START_Y);

    // HUD: score (top left) and lives (top right)
    hud_score = lv_obj_create(scr);
    make_plain(hud_score);
    lv_obj_set_size(hud_score, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(hud_score, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hud_score, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(hud_score, LV_OPA_80, 0);
    lv_obj_set_style_pad_hor(hud_score, 22, 0);
    lv_obj_set_style_pad_ver(hud_score, 8, 0);
    lv_obj_align(hud_score, LV_ALIGN_TOP_LEFT, 16, 12);
    score_label = create_label(hud_score, "Score: 0", &lv_font_montserrat_32, COLOR_DARK);

    hud_lives = lv_obj_create(scr);
    make_plain(hud_lives);
    lv_obj_set_size(hud_lives, START_LIVES * 44 + 24, 56);
    lv_obj_set_style_radius(hud_lives, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hud_lives, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(hud_lives, LV_OPA_80, 0);
    lv_obj_align(hud_lives, LV_ALIGN_TOP_RIGHT, -16, 12);
    for (int i = 0; i < START_LIVES; i++) {
        life_icons[i] = create_face(hud_lives, 36);
        lv_obj_align(life_icons[i], LV_ALIGN_LEFT_MID, 16 + i * 44, 0);
    }

    // "Hold your hand over the sensor" reminder
    hint_cont = lv_obj_create(scr);
    make_plain(hint_cont);
    lv_obj_set_size(hint_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(hint_cont, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hint_cont, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(hint_cont, LV_OPA_90, 0);
    lv_obj_set_style_pad_hor(hint_cont, 24, 0);
    lv_obj_set_style_pad_ver(hint_cont, 10, 0);
    lv_obj_align(hint_cont, LV_ALIGN_BOTTOM_MID, 0, -16);
    create_label(hint_cont, "Hold your hand over the sensor", &lv_font_montserrat_26, COLOR_DARK);

    // Countdown bubble: 3, 2, 1, GO!
    countdown_cont = lv_obj_create(scr);
    make_plain(countdown_cont);
    lv_obj_set_size(countdown_cont, 150, 150);
    lv_obj_set_style_radius(countdown_cont, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(countdown_cont, COLOR_ACCENT, 0);
    lv_obj_set_style_bg_opa(countdown_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(countdown_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(countdown_cont, 6, 0);
    lv_obj_align(countdown_cont, LV_ALIGN_CENTER, 120, 0);
    countdown_label = create_label(countdown_cont, "3", &lv_font_montserrat_48, lv_color_white());
    lv_obj_center(countdown_label);

    // Title screen
    title_cont = create_panel(scr, 720, 470, COLOR_TITLE);
    lv_obj_t *title_face = create_face(title_cont, 96);
    lv_obj_align(title_face, LV_ALIGN_TOP_MID, 0, 24);
    lv_obj_t *title = create_label(title_cont, "Hover Maze!", &lv_font_montserrat_48, COLOR_TITLE);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 132);
    lv_obj_t *how = create_label(title_cont,
                                 LV_SYMBOL_UP "  Hand up = fly up\n" LV_SYMBOL_DOWN "  Hand down = fly down",
                                 &lv_font_montserrat_28, COLOR_DARK);
    lv_obj_set_style_text_line_space(how, 10, 0);
    lv_obj_align(how, LV_ALIGN_TOP_MID, 0, 205);
    lv_obj_t *start_prompt = create_tap_prompt(title_cont, "Tap the Touch Pad to Start");
    lv_obj_align(start_prompt, LV_ALIGN_BOTTOM_MID, 0, -30);

    // Game over screen
    game_over_cont = create_panel(scr, 680, 420, COLOR_ACCENT);
    lv_obj_t *go_face = create_face(game_over_cont, 96);
    lv_obj_align(go_face, LV_ALIGN_TOP_MID, 0, 24);
    lv_obj_t *go_title = create_label(game_over_cont, "Well done!", &lv_font_montserrat_48, COLOR_ACCENT);
    lv_obj_align(go_title, LV_ALIGN_TOP_MID, 0, 132);
    go_score_label = create_label(game_over_cont, "", &lv_font_montserrat_32, COLOR_DARK);
    lv_obj_align(go_score_label, LV_ALIGN_TOP_MID, 0, 200);
    lv_obj_t *again_prompt = create_tap_prompt(game_over_cont, "Tap the Touch Pad to Play Again");
    lv_obj_align(again_prompt, LV_ALIGN_BOTTOM_MID, 0, -30);

    // Start on the title screen with the maze entrance showing behind it
    resetMaze();
    set_visible(player_icon, false);
    set_visible(hud_score, false);
    set_visible(hud_lives, false);
    set_visible(hint_cont, false);
    set_visible(countdown_cont, false);
    set_visible(game_over_cont, false);
    state = STATE_TITLE;

    lv_timer_create(game_tick_cb, 30, NULL);
    lvgl_port_unlock();
}

/////// 17. LOOP: read the hand sensor and touch pad (never touches LVGL directly)
void loop()
{
    static unsigned long lastSensorMs = 0;
    if (millis() - lastSensorMs >= SENSOR_INTERVAL_MS) {
        lastSensorMs = millis();
        updateHand();
    }
    pollTouchPad();
    delay(5);
}
