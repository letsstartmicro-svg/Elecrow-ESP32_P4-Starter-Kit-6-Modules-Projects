// scene.cpp - the playground scene and all its animation
//
// How it is built (this is what keeps it fast and light on memory):
//
//   1. BACKGROUND PICTURE - everything that never moves (sky, hills, grass,
//      tree, slide, sandpit, swing frame, cubby house...) is painted ONCE at
//      start into a single 1024x600 picture held in PSRAM. Redrawing behind a
//      moving object is then just a quick copy from this picture.
//
//   2. SPRITES - each moving thing (a swing with its kid, the see-saw, the duck,
//      a cloud...) is ONE LVGL object that draws all its own parts. Few objects
//      = very little of LVGL's small memory used. One timer (scene_tick, ~30
//      fps) updates them using the latest sensor values.
//
//   3. ONE REDRAW PER SPRITE - each sprite reports one rectangle to redraw per
//      frame, and only when it moved. (LVGL can only track 32 rectangles;
//      beyond that it redraws the whole screen, which is slow.)
//
// Screen is 1024 x 600. The top of the grass is at y = 440.

#include <Arduino.h>
#include <lvgl.h>
#include <math.h>
#include "scene.h"
#include "audio.h"
#include "sign_img.h"
#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

// ----------------------------------------------------------------------------
// Tweakables
// ----------------------------------------------------------------------------
#define SCR_W           1024
#define SCR_H           600
#define GROUND_Y        440
#define SUN_X           830
#define SUN_Y           10      // where the sun sits in the day
#define SUN_HIDDEN_Y    380     // behind the hill
#define TICK_MS         33      // animation step (~30 frames per second)

#define HAND_NEAR_CM    5.0f    // hand this close  -> see-saw fully one way
#define HAND_FAR_CM     40.0f   // hand this far    -> see-saw fully the other way
#define DUCK_SOUND_GAIN 50.0f   // how hard noise pushes the duck (bigger = wobblier)

// Colours (0xRRGGBB)
#define C_SKY_TOP       0x3F9BE0
#define C_SKY_BOTTOM    0xCDEBFF
#define C_HILL_BACK     0xA5DB93
#define C_HILL_FRONT    0x8BCF78
#define C_GRASS         0x6CC04A
#define C_GRASS_EDGE    0x5DAF3E
#define C_SKIN          0xF7C8A0
#define C_DARK          0x333333
#define C_METAL         0x7A8894
#define C_FRAME         0x3B82C4
#define C_DUCK          0xFFD23F
#define C_DUSK          0xFF7F50    // sunset tint
#define C_NIGHT         0x0A1440    // night tint

// The front hill is a big circle; the sun's "window" needs to know it too.
#define HILL_FRONT_CX   820
#define HILL_FRONT_CY   700
#define HILL_FRONT_D    740

// Cubby house doorway
#define DOOR_X          848
#define DOOR_Y          535
#define DOOR_W          46
#define DOOR_H          62

// The "Elecrow Playground" sign on the right-hand hill (top-left corner)
#define SIGN_X          722
#define SIGN_Y          172

// Spring-rider duck
#define DUCK_BASE_X     645
#define DUCK_BASE_Y     590
#define DUCK_SPRING     50

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float approach(float v, float target, float maxStep)
{
    if (v < target) return fminf(v + maxStep, target);
    return fmaxf(v - maxStep, target);
}

// ============================================================================
// Part 1: painting the background picture
// ============================================================================
static lv_obj_t *bg;          // the canvas (background picture)

static void *big_alloc(size_t n)
{
#ifdef ESP_PLATFORM
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);   // PSRAM - plenty of room
    if (p) return p;
#endif
    return malloc(n);
}

static void p_rect(int x, int y, int w, int h, uint32_t color, int radius = 0,
                   int border = 0, uint32_t border_color = 0)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(color);
    d.radius = radius;
    d.border_width = border;
    d.border_color = lv_color_hex(border_color);
    lv_canvas_draw_rect(bg, x, y, w, h, &d);
}

static void p_circle(int cx, int cy, int d, uint32_t color, int border = 0, uint32_t border_color = 0)
{
    p_rect(cx - d / 2, cy - d / 2, d, d, color, LV_RADIUS_CIRCLE, border, border_color);
}

static void p_line(int x1, int y1, int x2, int y2, int w, uint32_t color)
{
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(color);
    d.width = w;
    d.round_start = 1;
    d.round_end = 1;
    lv_point_t pts[2] = {{(lv_coord_t)x1, (lv_coord_t)y1}, {(lv_coord_t)x2, (lv_coord_t)y2}};
    lv_canvas_draw_line(bg, pts, 2, &d);
}

static void p_arc(int cx, int cy, int r, int start, int end, int w, uint32_t color)
{
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color = lv_color_hex(color);
    d.width = w;
    d.rounded = 1;
    lv_canvas_draw_arc(bg, cx, cy, r, start, end, &d);
}

static void p_triangle(int x1, int y1, int x2, int y2, int x3, int y3, uint32_t color)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(color);
    lv_point_t pts[3] = {{(lv_coord_t)x1, (lv_coord_t)y1}, {(lv_coord_t)x2, (lv_coord_t)y2},
                         {(lv_coord_t)x3, (lv_coord_t)y3}};
    lv_canvas_draw_polygon(bg, pts, 3, &d);
}

static void paint_sky_and_hills()
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_grad.dir = LV_GRAD_DIR_VER;
    d.bg_grad.stops_count = 2;
    d.bg_grad.stops[0].color = lv_color_hex(C_SKY_TOP);
    d.bg_grad.stops[0].frac = 0;
    d.bg_grad.stops[1].color = lv_color_hex(C_SKY_BOTTOM);
    d.bg_grad.stops[1].frac = 255;
    lv_canvas_draw_rect(bg, 0, 0, SCR_W, GROUND_Y, &d);

    p_circle(250, 660, 640, C_HILL_BACK);
    p_circle(HILL_FRONT_CX, HILL_FRONT_CY, HILL_FRONT_D, C_HILL_FRONT);
    p_rect(0, GROUND_Y, SCR_W, SCR_H - GROUND_Y, C_GRASS);
    p_rect(0, GROUND_Y, SCR_W, 8, C_GRASS_EDGE);
}

static void paint_tree()
{
    p_rect(62, 300, 28, 175, 0x8B5A2B, 6);                // trunk
    p_circle(50, 260, 100, 0x3E9B3E);
    p_circle(110, 250, 100, 0x3E9B3E);
    p_circle(78, 200, 110, 0x4CAF50);
    p_circle(40, 215, 70, 0x4CAF50);
    p_circle(60, 225, 16, 0xE63946);                      // apples
    p_circle(105, 205, 16, 0xE63946);
    p_circle(95, 265, 16, 0xE63946);
}

static void paint_slide()
{
    p_line(182, 300, 182, 215, 5, C_METAL);               // flag pole
    p_line(172, 300, 172, 472, 7, C_METAL);               // ladder
    p_line(207, 300, 207, 472, 7, C_METAL);
    for (int y = 325; y <= 450; y += 28) p_line(172, y, 207, y, 5, C_METAL);
    p_rect(162, 292, 96, 16, 0xFFC53D, 5);                // platform
    p_line(255, 300, 392, 455, 24, 0xFF5A5F);             // chute
    p_line(392, 455, 428, 458, 24, 0xFF5A5F);
    p_line(258, 294, 395, 449, 6, 0xFF8A8E);              // highlight stripe
}

static void paint_sandpit()
{
    p_rect(225, 505, 300, 78, 0xF2D16B, 38, 5, 0xD9B44A);
    p_rect(290, 522, 34, 30, 0xE63946, 5);                // bucket
    p_arc(307, 523, 16, 180, 360, 3, C_DARK);             // bucket handle
    p_line(352, 548, 382, 516, 5, C_FRAME);               // spade
    p_rect(340, 540, 20, 20, C_FRAME, 4);
    p_rect(420, 525, 56, 38, 0xE3BC52, 3);                // sand castle
    for (int i = 0; i < 3; i++) p_rect(420 + i * 22, 513, 12, 14, 0xE3BC52, 2);
    p_rect(441, 540, 14, 23, 0xC99A2E, 7);
}

static void paint_swing_frame()
{
    p_line(462, 472, 502, 262, 12, C_FRAME);
    p_line(542, 472, 502, 262, 12, C_FRAME);
    p_line(660, 472, 700, 262, 12, C_FRAME);
    p_line(740, 472, 700, 262, 12, C_FRAME);
    p_line(492, 262, 710, 262, 14, C_FRAME);
}

static void paint_seesaw_stand()
{
    p_rect(850, 430, 44, 45, 0x5C6B7A, 8);
}

static void paint_cubby_house()
{
    p_rect(830, 515, 160, 90, 0xF7B2BD);                          // walls
    p_triangle(812, 524, 910, 460, 1008, 524, 0xD1495B);          // roof
    p_rect(DOOR_X - 4, DOOR_Y - 4, DOOR_W + 8, DOOR_H + 8, 0xFFFFFF, 5);  // door frame
    p_rect(DOOR_X, DOOR_Y, DOOR_W, DOOR_H + 4, 0x3A2418, 3);      // dark inside
    p_rect(908, 535, 58, 40, 0xBDE0FE, 3, 5, 0xFFFFFF);           // window
    p_line(937, 538, 937, 572, 3, 0xFFFFFF);
    p_line(911, 555, 963, 555, 3, 0xFFFFFF);
}

static void paint_duck_base()
{
    p_rect(DUCK_BASE_X - 36, DUCK_BASE_Y - 6, 72, 14, 0x555555, 7);
}

static void paint_flowers()
{
    const int fx[] = {30, 150, 535, 745, 775};
    const int fy[] = {540, 580, 555, 588, 530};
    const uint32_t petals[] = {0xFFFFFF, 0xFF9EAA, 0xC77DFF, 0xFFFFFF, 0xFF9EAA};
    for (int i = 0; i < 5; i++) p_circle(fx[i], fy[i], 22, 0xFFD93B, 6, petals[i]);
}

// ============================================================================
// Part 2: plain objects (used for the sun, door, flag and night tint)
// ============================================================================
static lv_obj_t *box(lv_obj_t *par, int x, int y, int w, int h, uint32_t color,
                     int radius = 0, lv_opa_t opa = LV_OPA_COVER)
{
    lv_obj_t *o = lv_obj_create(par);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(o, opa, 0);
    lv_obj_set_style_radius(o, radius, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

static lv_obj_t *circle(lv_obj_t *par, int cx, int cy, int d, uint32_t color,
                        lv_opa_t opa = LV_OPA_COVER)
{
    return box(par, cx - d / 2, cy - d / 2, d, d, color, LV_RADIUS_CIRCLE, opa);
}

// An invisible touch area that calls cb when pressed.
static lv_obj_t *hotspot(lv_obj_t *par, int x, int y, int w, int h, lv_event_cb_t cb)
{
    lv_obj_t *o = lv_obj_create(par);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(o, cb, LV_EVENT_PRESSED, NULL);
    return o;
}

static void set_visible(lv_obj_t *o, bool v)
{
    if (v) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
    else   lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}

// ============================================================================
// Part 3: sprites
//
// A sprite has a "draw" function made of d_circle / d_line / d_rect / d_arc.
// That same function is used twice:
//   - to MEASURE the sprite (so its object is sized to fit - small redraws)
//   - to DRAW it when LVGL redraws that part of the screen
// All coordinates are screen coordinates.
// ============================================================================
static lv_draw_ctx_t *dctx = nullptr;     // null = measuring
static lv_area_t      mbox;
static bool           mAny;
static lv_opa_t       dOpa = LV_OPA_COVER;

static void m_add(float x1, float y1, float x2, float y2)
{
    lv_area_t a = {(lv_coord_t)floorf(x1), (lv_coord_t)floorf(y1), (lv_coord_t)ceilf(x2), (lv_coord_t)ceilf(y2)};
    if (!mAny) { mbox = a; mAny = true; return; }
    if (a.x1 < mbox.x1) mbox.x1 = a.x1;
    if (a.y1 < mbox.y1) mbox.y1 = a.y1;
    if (a.x2 > mbox.x2) mbox.x2 = a.x2;
    if (a.y2 > mbox.y2) mbox.y2 = a.y2;
}

static void d_rect(float x, float y, float w, float h, uint32_t color, int radius = 0,
                   int border = 0, uint32_t border_color = 0, lv_border_side_t side = LV_BORDER_SIDE_FULL)
{
    if (!dctx) { m_add(x, y, x + w, y + h); return; }
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(color);
    d.bg_opa = dOpa;
    d.radius = radius;
    d.border_width = border;
    d.border_color = lv_color_hex(border_color);
    d.border_opa = dOpa;
    d.border_side = side;
    lv_area_t a = {(lv_coord_t)lroundf(x), (lv_coord_t)lroundf(y),
                   (lv_coord_t)(lroundf(x + w) - 1), (lv_coord_t)(lroundf(y + h) - 1)};
    lv_draw_rect(dctx, &d, &a);
}

static void d_circle(float cx, float cy, float dia, uint32_t color)
{
    d_rect(cx - dia / 2, cy - dia / 2, dia, dia, color, LV_RADIUS_CIRCLE);
}

static void d_line(float x1, float y1, float x2, float y2, int w, uint32_t color)
{
    if (!dctx) { m_add(fminf(x1, x2) - w, fminf(y1, y2) - w, fmaxf(x1, x2) + w, fmaxf(y1, y2) + w); return; }
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(color);
    d.width = w;
    d.opa = dOpa;
    d.round_start = 1;
    d.round_end = 1;
    lv_point_t a = {(lv_coord_t)lroundf(x1), (lv_coord_t)lroundf(y1)};
    lv_point_t b = {(lv_coord_t)lroundf(x2), (lv_coord_t)lroundf(y2)};
    lv_draw_line(dctx, &d, &a, &b);
}

// angles: 0 = right, 90 = down, 180 = left, 270 = up
static void d_arc(float cx, float cy, float r, int a0, int a1, int w, uint32_t color, bool rounded = true)
{
    if (!dctx) { m_add(cx - r - 1, cy - r - 1, cx + r + 1, cy + r + 1); return; }
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color = lv_color_hex(color);
    d.width = w;
    d.opa = dOpa;
    d.rounded = rounded;
    lv_point_t c = {(lv_coord_t)lroundf(cx), (lv_coord_t)lroundf(cy)};
    lv_draw_arc(dctx, &d, &c, (uint16_t)r, a0, a1);
}

static void d_triangle(float x1, float y1, float x2, float y2, float x3, float y3, uint32_t color)
{
    if (!dctx) { m_add(fminf(x1, fminf(x2, x3)), fminf(y1, fminf(y2, y3)), fmaxf(x1, fmaxf(x2, x3)), fmaxf(y1, fmaxf(y2, y3))); return; }
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(color);
    d.bg_opa = dOpa;
    lv_point_t p[3] = {{(lv_coord_t)lroundf(x1), (lv_coord_t)lroundf(y1)}, {(lv_coord_t)lroundf(x2), (lv_coord_t)lroundf(y2)},
                       {(lv_coord_t)lroundf(x3), (lv_coord_t)lroundf(y3)}};
    lv_draw_polygon(dctx, &d, p, 3);
}

static void d_heart(float x, float y, float size, uint32_t color)
{
    float r = size * 0.3f;
    d_circle(x - r, y, 2 * r + 1, color);
    d_circle(x + r, y, 2 * r + 1, color);
    d_triangle(x - 2 * r, y + r * 0.3f, x + 2 * r, y + r * 0.3f, x, y + size * 0.85f, color);
}

struct Sprite {
    lv_obj_t *obj;
    void (*draw)();
    bool      dirty;         // moved this frame
    bool      hadArea;
    lv_area_t before;        // where it was at the start of the frame
};

static void sprite_draw_event(lv_event_t *e)
{
    Sprite *s = (Sprite *)lv_event_get_user_data(e);
    dctx = lv_event_get_draw_ctx(e);
    s->draw();
    dctx = nullptr;
    dOpa = LV_OPA_COVER;
}

static void sprite_init(Sprite &s, lv_obj_t *par, void (*draw)())
{
    s.draw = draw;
    s.obj = lv_obj_create(par);
    lv_obj_remove_style_all(s.obj);
    lv_obj_clear_flag(s.obj, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s.obj, sprite_draw_event, LV_EVENT_DRAW_MAIN, &s);
}

// Re-measure the sprite and fit its object around it
static void sprite_update(Sprite &s)
{
    dctx = nullptr;
    mAny = false;
    s.draw();
    dOpa = LV_OPA_COVER;
    s.dirty = true;
    if (!mAny) { set_visible(s.obj, false); return; }
    set_visible(s.obj, true);
    lv_area_t pc;
    lv_obj_get_coords(lv_obj_get_parent(s.obj), &pc);    // positions are relative to the parent
    lv_obj_set_pos(s.obj, mbox.x1 - 1 - pc.x1, mbox.y1 - 1 - pc.y1);
    lv_obj_set_size(s.obj, lv_area_get_width(&mbox) + 2, lv_area_get_height(&mbox) + 2);
}

static bool sprite_area(Sprite &s, lv_area_t *a)
{
    if (lv_obj_has_flag(s.obj, LV_OBJ_FLAG_HIDDEN)) return false;
    lv_obj_get_coords(s.obj, a);
    return true;
}

// ---- A kid: legs, body, head with hair and eyes ----------------------------
// (sx,sy) = where the kid sits, (ux,uy) = "up" direction, (fx,fy) = facing direction
static void draw_kid(float sx, float sy, float ux, float uy, float fx, float fy, uint32_t shirt, uint32_t hair)
{
    d_line(sx, sy, sx + 24 * fx - 12 * ux, sy + 24 * fy - 12 * uy, 8, 0x2F4B7C);   // legs
    d_line(sx, sy, sx + 30 * ux, sy + 30 * uy, 16, shirt);                         // body
    float hx = sx + 50 * ux - 17, hy = sy + 50 * uy - 17;
    d_rect(hx, hy, 34, 34, C_SKIN, LV_RADIUS_CIRCLE, 8, hair, LV_BORDER_SIDE_TOP);  // head + hair
    d_rect(hx + 9, hy + 15, 5, 6, C_DARK, LV_RADIUS_CIRCLE);                       // eyes
    d_rect(hx + 20, hy + 15, 5, 6, C_DARK, LV_RADIUS_CIRCLE);
}

// ============================================================================
// Scene state
// ============================================================================
struct Cloud  { float x, y, speed, scale; };
struct Swing  { float px, py, L, th, om; bool hasKid; };
struct SeeSaw { float cx, cy, phi, boost, handMix, handPhi; };
struct Duck   { float psi, om, beak, lastPsi, lastBeak; };

static lv_obj_t *scr;
static lv_obj_t *sun, *sunEyeL, *sunEyeR;
static lv_obj_t *flag, *door, *night, *litWindow;

static Cloud   clouds[3];
static Swing   swings[2];
static SeeSaw  seesaw;
static Duck    duck;

static Sprite sClouds[3], sSwings[2], sSeesaw, sDuck, sPup, sDog, sSlide, sBird, sRainbow, sStars;
static Sprite *moving[] = {&sClouds[0], &sClouds[1], &sClouds[2], &sSwings[0], &sSwings[1],
                           &sSeesaw, &sDuck, &sPup, &sDog, &sSlide, &sBird, &sRainbow};
#define NUM_MOVING (int)(sizeof(moving) / sizeof(moving[0]))

// plain objects that move in the tick, tracked the same way
struct Tracked { lv_obj_t *obj; bool dirty, hadArea; lv_area_t before; };
static Tracked tSun, tDoor, tFlag;
static Tracked *tracked[] = {&tSun, &tDoor, &tFlag};

static lv_timer_t *tickTimer;
static float       t = 0;                 // running time in seconds
static uint32_t    lastTickMs = 0;

// Inputs from the sensors (set by scene_set_...)
static float    dayTarget = 1, dayShown = 1, dayApplied = -1;
static float    soundLevel = 0;
static float    handCm = -1;
static bool     padWasTouched = false;

// Running effects
static float    doorOpen = 0, pupUp = 0, pupBob = 0;
static float    dogT = -1;                // -1 = the puppy is at home
static float    slideT = -1;              // -1 = nobody on the slide
static float    birdT = -1;               // -1 = no bird
static float    rainbowT = -1;            // -1 = no rainbow
static float    rainbowOpa = 0;
static lv_opa_t starsOpa = 0;
static float    flagBoostT = 0;
static float    lastFlagW = 0;

// ============================================================================
// Sprite drawings
// ============================================================================
static void draw_cloud(const Cloud &c)
{
    float s = c.scale, x = c.x, y = c.y;
    d_rect(x + 15 * s, y + 55 * s, 180 * s, 50 * s, 0xFFFFFF, LV_RADIUS_CIRCLE);
    d_circle(x + 50 * s, y + 65 * s, 75 * s, 0xFFFFFF);
    d_circle(x + 100 * s, y + 50 * s, 100 * s, 0xFFFFFF);
    d_circle(x + 155 * s, y + 65 * s, 80 * s, 0xFFFFFF);
}
static void draw_cloud0() { draw_cloud(clouds[0]); }
static void draw_cloud1() { draw_cloud(clouds[1]); }
static void draw_cloud2() { draw_cloud(clouds[2]); }

static void draw_swing(const Swing &s)
{
    float sn = sinf(s.th), cs = cosf(s.th);
    float sx = s.px + s.L * sn, sy = s.py + s.L * cs;    // seat centre
    float ax = 24 * cs, ay = -24 * sn;                    // half-seat along the seat
    d_line(s.px - 20, s.py, sx - ax, sy - ay, 3, 0x666666);
    d_line(s.px + 20, s.py, sx + ax, sy + ay, 3, 0x666666);
    if (s.hasKid) draw_kid(sx, sy - 6 * cs, -sn, -cs, cs, -sn, 0x3D7DD8, 0x6B3E26);
    d_line(sx - ax, sy - ay, sx + ax, sy + ay, 9, 0xE63946);
}
static void draw_swing0() { draw_swing(swings[0]); }
static void draw_swing1() { draw_swing(swings[1]); }

static void draw_seesaw()
{
    const SeeSaw &ss = seesaw;
    float dx = cosf(ss.phi), dy = sinf(ss.phi);   // along the plank
    float ux = dy, uy = -dx;                      // "up" from the plank
    float cx = ss.cx, cy = ss.cy;
    draw_kid(cx - 106 * dx + 8 * ux, cy - 106 * dy + 8 * uy, ux, uy, dx, dy, 0xFF8C42, 0x2B1B10);
    draw_kid(cx + 106 * dx + 8 * ux, cy + 106 * dy + 8 * uy, ux, uy, -dx, -dy, 0x9B5DE5, 0xE0B040);
    d_line(cx - 118 * dx, cy - 118 * dy, cx + 118 * dx, cy + 118 * dy, 14, 0xF4A259);
    float lx = cx - 86 * dx, ly = cy - 86 * dy, rx = cx + 86 * dx, ry = cy + 86 * dy;
    d_line(lx, ly, lx + 26 * ux, ly + 26 * uy, 6, C_METAL);
    d_line(rx, ry, rx + 26 * ux, ry + 26 * uy, 6, C_METAL);
    d_circle(cx, cy, 12, 0x5C6B7A);               // bolt
}

static void draw_duck()
{
    const Duck &d = duck;
    float s = sinf(d.psi), c = cosf(d.psi);
    // a point on the duck, given as an offset from the top of the spring, tilted
    auto X = [&](float dx, float dy) { return DUCK_BASE_X + DUCK_SPRING * s + dx * c - dy * s; };
    auto Y = [&](float dx, float dy) { return DUCK_BASE_Y - DUCK_SPRING * c + dx * s + dy * c; };

    // zig-zag spring following a gentle curve
    float tx = X(0, 0), ty = Y(0, 0), px = DUCK_BASE_X, py = DUCK_BASE_Y;
    for (int k = 1; k <= 8; k++) {
        float u = k / 8.0f;
        float bx = (1 - u) * (1 - u) * DUCK_BASE_X + 2 * (1 - u) * u * DUCK_BASE_X + u * u * tx;
        float by = (1 - u) * (1 - u) * DUCK_BASE_Y + 2 * (1 - u) * u * (DUCK_BASE_Y - DUCK_SPRING / 2) + u * u * ty;
        float nx = bx + ((k == 8) ? 0 : ((k & 1) ? 8 : -8));
        d_line(px, py, nx, by, 5, 0xB8C0CA);
        px = nx; py = by;
    }
    d_circle(X(-38, -26), Y(-38, -26), 24, C_DUCK);                        // tail
    d_rect(X(0, -20) - 37, Y(0, -20) - 23, 74, 46, C_DUCK, LV_RADIUS_CIRCLE); // body
    d_rect(X(-6, -20) - 17, Y(-6, -20) - 10, 34, 20, 0xFFE680, LV_RADIUS_CIRCLE); // wing
    d_rect(X(46, -45 + d.beak) - 8, Y(46, -45 + d.beak) - 3.5f, 16, 7, 0xE8740C, 3); // lower beak
    d_rect(X(48, -52) - 10, Y(48, -52) - 4, 20, 8, 0xFF8C1A, 3);            // upper beak
    d_circle(X(26, -52), Y(26, -52), 40, C_DUCK);                           // head
    d_rect(X(26, -52) + 3, Y(26, -52) - 9, 8, 9, C_DARK, LV_RADIUS_CIRCLE); // eye
}

static void draw_pup()
{
    float e = pupUp * pupUp * (3 - 2 * pupUp);          // smooth start/stop
    float x = DOOR_X, y = DOOR_Y + DOOR_H - (DOOR_H - 6) * e + pupBob;
    d_rect(x + 2, y + 6, 12, 26, 0x7B4B2A, 6);          // ears
    d_rect(x + 32, y + 6, 12, 26, 0x7B4B2A, 6);
    d_circle(x + 23, y + 26, 36, 0xC68642);             // head
    d_rect(x + 14, y + 20, 5, 6, C_DARK, LV_RADIUS_CIRCLE);   // eyes
    d_rect(x + 27, y + 20, 5, 6, C_DARK, LV_RADIUS_CIRCLE);
    d_rect(x + 18, y + 29, 10, 7, 0x222222, LV_RADIUS_CIRCLE); // nose
    d_rect(x + 20, y + 36, 7, 10, 0xFF7B9C, 3);         // tongue
}

// ---- The puppy's big adventure (touch pad) ----
// dogT timeline (seconds):
//   0.0-0.6 door opens    0.5-1.3 puppy peeks out and barks
//   1.3-1.7 jumps out     1.7-3.0 runs across the grass
//   3.0-4.0 happy hops with hearts (and another bark)
//   4.0-5.0 runs home     5.0-5.6 door closes
#define DOG_HOME_X   871.0f
#define DOG_FAR_X    748.0f
#define DOG_FEET_Y   592.0f

static bool dog_pose(float *x, float *y, float *dir, float *run, float *hop)
{
    float T = dogT;
    *run = 0; *hop = 0; *dir = -1;
    if (T < 1.3f || T >= 5.0f) return false;
    if (T < 1.7f) {                                   // jump out of the door
        float u = (T - 1.3f) / 0.4f;
        *x = DOG_HOME_X + (835 - DOG_HOME_X) * u;
        *y = DOG_FEET_Y - 34 * sinf(u * 3.14159f);
    } else if (T < 3.0f) {                            // run away from the house
        float u = (T - 1.7f) / 1.3f;
        *x = 835 + (DOG_FAR_X - 835) * u;
        *y = DOG_FEET_Y - 3 * fabsf(sinf(T * 14));
        *run = T * 14;
    } else if (T < 4.0f) {                            // happy hops
        float u = (T - 3.0f);
        *x = DOG_FAR_X;
        *y = DOG_FEET_Y - 26 * fabsf(sinf(u * 6.2832f));
        *hop = 1;
    } else {                                          // run home
        float u = (T - 4.0f) / 1.0f;
        *x = DOG_FAR_X + (DOG_HOME_X - DOG_FAR_X) * u;
        *y = DOG_FEET_Y - 3 * fabsf(sinf(T * 14));
        *dir = 1;
        *run = T * 14;
    }
    return true;
}

static void draw_dog()
{
    float x, y, d, run, hop;
    if (!dog_pose(&x, &y, &d, &run, &hop)) return;
    const uint32_t fur = 0xC68642, dark = 0x7B4B2A, legc = 0xA86F35;

    // tail (wags fast)
    float wag = sinf(t * 22);
    d_line(x - d * 25, y - 30, x - d * (33 + 4 * wag), y - 46 + 3 * wag, 6, fur);
    // legs (swing when running)
    float a = run ? 7 * sinf(run) : 0;
    d_line(x - d * 15, y - 16, x - d * 15 + a, y, 6, legc);
    d_line(x + d * 15, y - 16, x + d * 15 - a, y, 6, legc);
    // body
    d_rect(x - 28, y - 38, 56, 28, fur, LV_RADIUS_CIRCLE);
    d_line(x - d * 15, y - 16, x - d * 15 - a, y, 6, fur);
    d_line(x + d * 15, y - 16, x + d * 15 + a, y, 6, fur);
    // head
    float hx = x + d * 27, hy = y - 44;
    d_rect(hx - 4, hy + 6, 8, 6, 0xE63946, 2);                        // collar
    d_circle(hx, hy, 30, fur);
    d_rect(hx + d * 8 - 9, hy - 1, 18, 13, 0xE0A96D, 6);                // snout
    d_circle(hx + d * 16, hy + 1, 7, 0x222222);                          // nose
    d_circle(hx + d * 3, hy - 6, 5, 0x222222);                           // eye
    d_rect(hx - d * 7 - 5, hy - 10, 10, 20, dark, 5);                    // floppy ear
    if (hop || run == 0) d_rect(hx + d * 8 - 3, hy + 10, 7, 9, 0xFF7B9C, 3);  // tongue out

    // hearts float up during the happy hops
    if (dogT >= 3.0f && dogT < 4.6f) {
        for (int k = 0; k < 3; k++) {
            float u = (dogT - 3.0f - 0.25f * k) / 1.0f;
            if (u < 0 || u > 1) continue;
            dOpa = (lv_opa_t)(255 * (1 - u));
            d_heart(DOG_FAR_X - 18 + 18 * k, 520 - 70 * u, 16, 0xFF4D6D);
            dOpa = LV_OPA_COVER;
        }
    }
}

// Slide path: along the chute, then along the little lip at the bottom
static void slide_pos(float u, float *x, float *y)
{
    const float ax = 262, ay = 304, bx = 392, by = 452, cx = 425, cy = 455;
    const float l1 = hypotf(bx - ax, by - ay), l2 = hypotf(cx - bx, cy - by);
    float d = u * (l1 + l2);
    if (d <= l1) { *x = ax + (bx - ax) * d / l1; *y = ay + (by - ay) * d / l1; }
    else { d -= l1; *x = bx + (cx - bx) * d / l2; *y = by + (cy - by) * d / l2; }
}

static void draw_slide_kid()
{
    if (slideT < 0 || slideT >= 2.3f) return;           // nobody there
    float x, y, ux = 0, uy = -1, fx = 0.662f, fy = 0.749f;
    if (slideT < 0.5f) {                                // sitting at the top
        slide_pos(0, &x, &y);
    } else if (slideT < 1.5f) {                         // wheee!
        float u = (slideT - 0.5f) / 1.0f;
        slide_pos(u * u, &x, &y);                       // speeds up as it goes
        ux = 0.40f; uy = -0.92f;                        // leaning back a bit
    } else {                                            // at the bottom
        slide_pos(1, &x, &y);
        fx = 1; fy = 0;
    }
    draw_kid(x + 8, y - 14, ux, uy, fx, fy, 0x2EC4B6, 0xF4D35E);
}

static void draw_bird()
{
    if (birdT < 0) return;
    float x = -60 + birdT * 230;
    float y = 175 + 18 * sinf(birdT * 2.5f);
    if (x > SCR_W + 20) return;
    d_line(x + 18, y + 12, x + 10, y + 12 - 18 * sinf(birdT * 14 + 0.6f), 7, 0x2B6CD4);  // back wing
    d_rect(x + 4, y + 6, 28, 22, 0x3A86FF, LV_RADIUS_CIRCLE);    // body
    d_circle(x + 31, y + 9, 18, 0x3A86FF);                        // head
    d_rect(x + 34, y + 5, 4, 4, C_DARK, LV_RADIUS_CIRCLE);        // eye
    d_rect(x + 38, y + 8, 6, 5, 0xFF9F1C, 2);                     // beak
    d_line(x + 14, y + 12, x + 2, y + 12 - 22 * sinf(birdT * 14), 7, 0x5FA0FF); // front wing
}

static void draw_rainbow()
{
    if (rainbowOpa <= 0) return;
    const uint32_t cols[6] = {0xFF4D4D, 0xFF9F1C, 0xFFE66D, 0x6BD425, 0x3A86FF, 0x8338EC};
    dOpa = (lv_opa_t)(rainbowOpa * 230);
    for (int i = 0; i < 6; i++) d_arc(512, 310, 240 - i * 13, 200, 340, 14, cols[i], false);
}

static void draw_stars()
{
    if (starsOpa == 0) return;
    dOpa = starsOpa;
    const int sx[] = {40, 120, 230, 410, 470, 590, 690, 760, 300, 520};
    const int sy[] = {40, 110, 30, 120, 50, 20, 90, 30, 170, 150};
    for (int i = 0; i < 10; i++) d_circle(sx[i], sy[i], (i % 3) ? 6 : 9, 0xFFF6C8);
    // sleepy moon
    d_circle(340, 80, 72, 0xFFF3C4);
    d_circle(358, 102, 12, 0xF0DE9C);
    d_arc(326, 72, 8, 20, 160, 3, 0xB8932A);     // closed eyes
    d_arc(352, 72, 8, 20, 160, 3, 0xB8932A);
    d_arc(340, 86, 12, 40, 140, 3, 0xB8932A);    // smile
    // the sign's light bulbs switch on
    for (int i = 0; i < 11; i++) {
        float a = (-32 + 64 * i / 10.0f) * 0.0174533f;
        float bx = SIGN_X + 150 + 217 * sinf(a), by = SIGN_Y + 235 - 217 * cosf(a);
        d_circle(bx, by, 11, 0xFFE066);
    }
}

// ============================================================================
// Building the plain objects
// ============================================================================

// The sun lives inside a clipped "window" that also holds a copy of the front
// hill's edge in front of it - so the sun can rise and set behind the hill even
// though the real hill is part of the background picture.
static void make_sun()
{
    const int wx = 815, wy = 0, ww = 195, wh = 380;
    lv_obj_t *win = box(scr, wx, wy, ww, wh, 0, 0, LV_OPA_TRANSP);

    sun = box(win, SUN_X - wx, SUN_HIDDEN_Y, 160, 160, 0, 0, LV_OPA_TRANSP);
    lv_obj_add_flag(sun, LV_OBJ_FLAG_CLICKABLE);
    circle(sun, 80, 80, 150, 0xFFFFFF, LV_OPA_40);        // soft glow
    circle(sun, 80, 80, 110, 0xFFD93B);                   // sun
    sunEyeL = box(sun, 58, 60, 12, 16, 0x5A3E00, LV_RADIUS_CIRCLE);
    sunEyeR = box(sun, 90, 60, 12, 16, 0x5A3E00, LV_RADIUS_CIRCLE);
    circle(sun, 52, 94, 16, 0xFF9EAA, LV_OPA_60);         // cheeks
    circle(sun, 108, 94, 16, 0xFF9EAA, LV_OPA_60);

    lv_obj_t *smile = lv_arc_create(sun);
    lv_obj_remove_style_all(smile);
    lv_obj_set_pos(smile, 50, 55);
    lv_obj_set_size(smile, 60, 60);
    lv_arc_set_bg_angles(smile, 25, 155);
    lv_obj_set_style_arc_width(smile, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_color(smile, lv_color_hex(0x5A3E00), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(smile, true, LV_PART_MAIN);
    lv_obj_clear_flag(smile, LV_OBJ_FLAG_CLICKABLE);
    tSun.obj = sun;

    // hill edge in front of the sun (same colour/position as the painted hill)
    circle(win, HILL_FRONT_CX - wx, HILL_FRONT_CY - wy, HILL_FRONT_D, C_HILL_FRONT);
}

// The sign is a picture with see-through parts, stored as plain RGBA and
// converted once at start-up into LVGL's own colour format (in PSRAM).
static void make_sign()
{
    static lv_img_dsc_t dsc;
    const int n = SIGN_W * SIGN_H;
    uint8_t *buf = (uint8_t *)big_alloc(n * LV_IMG_PX_SIZE_ALPHA_BYTE);
    if (!buf) return;
    uint8_t *p = buf;
    for (int i = 0; i < n; i++) {
        const uint8_t *q = &SIGN_RGBA[i * 4];
        lv_color_t c = lv_color_make(q[0], q[1], q[2]);
#if LV_COLOR_DEPTH == 32
        c.ch.alpha = q[3];
        memcpy(p, &c, 4);
#else
        memcpy(p, &c, sizeof(lv_color_t));
        p[LV_IMG_PX_SIZE_ALPHA_BYTE - 1] = q[3];
#endif
        p += LV_IMG_PX_SIZE_ALPHA_BYTE;
    }
    dsc.header.always_zero = 0;
    dsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    dsc.header.w = SIGN_W;
    dsc.header.h = SIGN_H;
    dsc.data_size = n * LV_IMG_PX_SIZE_ALPHA_BYTE;
    dsc.data = buf;
    lv_obj_t *img = lv_img_create(scr);
    lv_img_set_src(img, &dsc);
    lv_obj_set_pos(img, SIGN_X, SIGN_Y);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
}

// ============================================================================
// Night / day
// ============================================================================
static uint32_t mix_color(uint32_t a, uint32_t b, float f)
{
    int r = (int)(((a >> 16) & 255) * (1 - f) + ((b >> 16) & 255) * f);
    int g = (int)(((a >> 8) & 255) * (1 - f) + ((b >> 8) & 255) * f);
    int bl = (int)((a & 255) * (1 - f) + (b & 255) * f);
    return (r << 16) | (g << 8) | bl;
}

static float sun_y_for(float day)
{
    float f = clampf((0.7f - day) / 0.5f, 0, 1);         // 0 = up, 1 = set
    return SUN_Y + f * (SUN_HIDDEN_Y - SUN_Y);
}

// (called with LVGL's normal redraw tracking switched on)
static void apply_daylight(float d)
{
    // tint: none in full day, orange at sunset, deep blue at night
    uint32_t col = C_DUSK;
    float opa = 0;
    if (d < 0.85f && d >= 0.55f) {
        opa = 70 * (0.85f - d) / 0.30f;
    } else if (d < 0.55f) {
        float f = (0.55f - d) / 0.55f;
        col = mix_color(C_DUSK, C_NIGHT, f);
        opa = 70 + (165 - 70) * f;
    }
    set_visible(night, opa > 1);
    lv_obj_set_style_bg_color(night, lv_color_hex(col), 0);
    lv_obj_set_style_bg_opa(night, (lv_opa_t)opa, 0);

    // stars, moon, window light
    starsOpa = (lv_opa_t)(255 * clampf((0.45f - d) / 0.3f, 0, 1));
    sprite_update(sStars);
    lv_obj_invalidate(sStars.obj);
    lv_obj_set_style_bg_opa(litWindow, starsOpa, 0);
    set_visible(litWindow, starsOpa > 0);
}

// ============================================================================
// The animation tick (runs ~30 times a second in the LVGL task)
// ============================================================================
static void anim_y(void *o, int32_t v)      { lv_obj_set_y((lv_obj_t *)o, (lv_coord_t)v); }
static void anim_height(void *o, int32_t v) { lv_obj_set_height((lv_obj_t *)o, (lv_coord_t)v); }

static void swing_step(Swing &s, float dt)
{
    const float w0sq = 8.2f;                 // gravity / length -> ~2.2 s per swing
    s.om += -w0sq * sinf(s.th) * dt;
    s.om *= 0.996f;                          // air friction
    s.th += s.om * dt;

    // keep a gentle idle sway going, and cap big swings
    float amp = sqrtf(s.th * s.th + (s.om * s.om) / w0sq);
    if (amp < 0.10f) s.om += (s.om >= 0 ? 0.006f : -0.006f);
    if (amp > 1.10f) s.om *= 0.97f;
}

static void scene_tick(lv_timer_t *timer)
{
    uint32_t now = lv_tick_get();
    float dt = lastTickMs ? (now - lastTickMs) / 1000.0f : TICK_MS / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    lastTickMs = now;
    t += dt;

    // ---- 1. Day / night (re-tinting redraws the whole screen, so we only
    //         do it when the light has changed a bit, not every frame)
    dayShown = approach(dayShown, dayTarget, 0.8f * dt);
    if (fabsf(dayShown - dayApplied) > 0.03f || (dayShown != dayApplied && (dayShown == 0 || dayShown == 1))) {
        apply_daylight(dayShown);
        dayApplied = dayShown;
    }

    // ---- 2. Note where everything is, then pause LVGL's own redraw tracking
    lv_disp_t *disp = lv_disp_get_default();
    lv_obj_update_layout(scr);
    for (int i = 0; i < NUM_MOVING; i++) {
        moving[i]->hadArea = sprite_area(*moving[i], &moving[i]->before);
        moving[i]->dirty = false;
    }
    for (Tracked *tr : tracked) {
        lv_obj_get_coords(tr->obj, &tr->before);
        tr->dirty = false;
    }
    lv_disp_enable_invalidation(disp, false);

    // clouds drift, wrap around
    for (int i = 0; i < 3; i++) {
        Cloud &c = clouds[i];
        c.x += c.speed * dt;
        if (c.x > SCR_W + 10) c.x = -210 * c.scale - random(0, 250);
        sprite_update(sClouds[i]);
    }

    // swings
    for (int i = 0; i < 2; i++) { swing_step(swings[i], dt); sprite_update(sSwings[i]); }

    // see-saw: rocks by itself, or follows a hand over the ultrasonic sensor
    {
        SeeSaw &ss = seesaw;
        ss.boost *= 0.985f;
        float rock = (0.10f + ss.boost) * sinf(t * 1.6f);
        ss.handMix = approach(ss.handMix, handCm > 0 ? 1.0f : 0.0f, (handCm > 0 ? 5.0f : 1.5f) * dt);
        if (handCm > 0) {
            float f = clampf((handCm - HAND_NEAR_CM) / (HAND_FAR_CM - HAND_NEAR_CM), 0, 1);
            float target = 0.30f - 0.60f * f;           // near = right side down
            ss.handPhi += (target - ss.handPhi) * fminf(1.0f, dt * 8);
        }
        ss.phi = ss.handMix * ss.handPhi + (1 - ss.handMix) * rock;
        sprite_update(sSeesaw);
    }

    // duck on a spring: wobbles with sound, beak opens to "sing"
    {
        Duck &d = duck;
        const float k = 60.0f, damping = 2.2f;
        d.om += (-k * d.psi - damping * d.om) * dt;
        if (soundLevel > 0.02f) d.om += soundLevel * DUCK_SOUND_GAIN * dt * (d.om >= 0 ? 1 : -1);
        d.psi += d.om * dt;
        if (d.psi > 0.5f) { d.psi = 0.5f; d.om = -fabsf(d.om) * 0.5f; }
        if (d.psi < -0.5f) { d.psi = -0.5f; d.om = fabsf(d.om) * 0.5f; }
        d.beak = approach(d.beak, soundLevel * 12.0f, 60.0f * dt);
        if (fabsf(d.psi - d.lastPsi) > 0.003f || fabsf(d.beak - d.lastBeak) > 0.3f) {
            sprite_update(sDuck);
            d.lastPsi = d.psi; d.lastBeak = d.beak;
        }
    }

    // cubby house door + puppy
    {
        float before = dogT;
        if (dogT >= 0) dogT += dt;
        bool wantOpen = dogT >= 0 && dogT < 5.1f;
        float oldDoor = doorOpen, oldPup = pupUp;
        doorOpen = approach(doorOpen, wantOpen ? 1.0f : 0.0f, 3.0f * dt);
        if (dogT >= 1.3f) pupUp = 0;                                     // jumped out
        else pupUp = approach(pupUp, (dogT >= 0.5f && doorOpen > 0.8f) ? 1.0f : 0.0f, 4.0f * dt);
        if (doorOpen != oldDoor) {
            float e = doorOpen * doorOpen * (3 - 2 * doorOpen);
            lv_obj_set_width(door, (lv_coord_t)(DOOR_W - (DOOR_W - 7) * e));
            tDoor.dirty = true;
        }
        pupBob = (pupUp >= 1.0f) ? 2.0f * sinf(t * 7.0f) : 0;
        if (pupUp != oldPup || pupUp >= 1.0f) sprite_update(sPup);
        if (dogT >= 0) {
            if (before < 0.9f && dogT >= 0.9f) audio_play(SFX_WOOF);
            if (before < 3.05f && dogT >= 3.05f) audio_play(SFX_WOOF);
            if ((before >= 1.2f && before < 5.1f) || (dogT >= 1.2f && dogT < 5.1f)) sprite_update(sDog);
            if (dogT >= 5.6f && doorOpen == 0) dogT = -1;
        }
    }

    // a kid goes down the slide
    if (slideT >= 0) {
        float before = slideT;
        slideT += dt;
        if (before < 0.5f && slideT >= 0.5f) audio_play(SFX_WHEE);
        sprite_update(sSlide);
        if (slideT >= 2.3f) slideT = -1;
    }

    // a bird flies across
    if (birdT >= 0) {
        birdT += dt;
        if (-60 + birdT * 230 > SCR_W + 20) birdT = -1;
        sprite_update(sBird);
    }

    // rainbow fades in, stays, fades out
    if (rainbowT >= 0) {
        rainbowT += dt;
        float a = rainbowT < 0.4f ? rainbowT / 0.4f : (rainbowT < 3.5f ? 1 : 1 - (rainbowT - 3.5f) / 0.6f);
        float na = clampf(a, 0, 1);
        if (rainbowT > 4.1f) { rainbowT = -1; na = 0; }
        if (na != rainbowOpa) { rainbowOpa = na; sprite_update(sRainbow); }
    }

    // flag flutters (faster when the servo button is pressed)
    flagBoostT = fmaxf(0, flagBoostT - dt);
    {
        float amp = flagBoostT > 0 ? 10 : 4, speed = flagBoostT > 0 ? 16 : 7;
        float w = 28 + amp * sinf(t * speed);
        if (fabsf(w - lastFlagW) >= 1) {
            lv_obj_set_width(flag, (lv_coord_t)w);
            lastFlagW = w;
            tFlag.dirty = true;
        }
    }

    // sun sets / rises with the light (unless it is busy jumping or rising)
    if (!lv_anim_get(sun, anim_y)) {
        lv_coord_t y = (lv_coord_t)sun_y_for(dayShown);
        if (lv_obj_get_y(sun) != y) { lv_obj_set_y(sun, y); tSun.dirty = true; }
    }

    // ---- 3. Apply the moves, switch tracking back on, and redraw one
    //         rectangle per moved thing: where it was + where it is now
    lv_obj_update_layout(scr);
    lv_disp_enable_invalidation(disp, true);
    for (int i = 0; i < NUM_MOVING; i++) {
        Sprite &s = *moving[i];
        if (!s.dirty) continue;
        lv_area_t a;
        bool hasNow = sprite_area(s, &a);
        if (s.hadArea && hasNow) {
            if (s.before.x1 < a.x1) a.x1 = s.before.x1;
            if (s.before.y1 < a.y1) a.y1 = s.before.y1;
            if (s.before.x2 > a.x2) a.x2 = s.before.x2;
            if (s.before.y2 > a.y2) a.y2 = s.before.y2;
        } else if (s.hadArea) {
            a = s.before;
        } else if (!hasNow) {
            continue;
        }
        lv_obj_invalidate_area(scr, &a);
    }
    for (Tracked *tr : tracked) {
        if (!tr->dirty) continue;
        lv_area_t a;
        lv_obj_get_coords(tr->obj, &a);
        if (tr->before.x1 < a.x1) a.x1 = tr->before.x1;
        if (tr->before.y1 < a.y1) a.y1 = tr->before.y1;
        if (tr->before.x2 > a.x2) a.x2 = tr->before.x2;
        if (tr->before.y2 > a.y2) a.y2 = tr->before.y2;
        lv_obj_invalidate_area(scr, &a);
    }
}

// ============================================================================
// Touch reactions
// ============================================================================
static void on_sun(lv_event_t *e)
{
    if (lv_anim_get(sun, anim_y)) return;   // still moving
    lv_coord_t y0 = lv_obj_get_y(sun);
    lv_anim_t a;
    lv_anim_init(&a);                        // jump...
    lv_anim_set_var(&a, sun);
    lv_anim_set_exec_cb(&a, anim_y);
    lv_anim_set_values(&a, y0, y0 - 30);
    lv_anim_set_time(&a, 200);
    lv_anim_set_playback_time(&a, 350);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    lv_obj_t *eyes[2] = {sunEyeL, sunEyeR};  // ...and blink
    for (int i = 0; i < 2; i++) {
        lv_anim_init(&a);
        lv_anim_set_var(&a, eyes[i]);
        lv_anim_set_exec_cb(&a, anim_height);
        lv_anim_set_values(&a, 16, 2);
        lv_anim_set_time(&a, 120);
        lv_anim_set_playback_time(&a, 160);
        lv_anim_start(&a);
    }
}

static void on_swings(lv_event_t *e)
{
    lv_point_t p;
    lv_indev_get_point(lv_indev_get_act(), &p);
    Swing &s = (abs(p.x - (int)swings[0].px) < abs(p.x - (int)swings[1].px)) ? swings[0] : swings[1];
    s.om += (s.om >= 0 ? 2.0f : -2.0f);      // push!
}

static void on_seesaw(lv_event_t *e) { seesaw.boost = 0.20f; }

static void on_duck(lv_event_t *e)
{
    duck.om += (duck.om >= 0 ? 3.0f : -3.0f);
    audio_play(SFX_BOING);
}

static void dog_start() { if (dogT < 0) dogT = 0; }
static void on_house(lv_event_t *e) { dog_start(); }

// ============================================================================
// Public API
// ============================================================================
bool scene_create()
{
    scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // 1. Background picture
    void *buf = big_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(SCR_W, SCR_H));
    if (!buf) {
        Serial.println("ERROR: no memory for the background - is PSRAM enabled in Tools menu?");
        return false;
    }
    bg = lv_canvas_create(scr);
    lv_canvas_set_buffer(bg, buf, SCR_W, SCR_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE);
    paint_sky_and_hills();
    paint_tree();
    paint_sandpit();
    paint_slide();
    paint_swing_frame();
    paint_seesaw_stand();
    paint_cubby_house();
    paint_duck_base();
    paint_flowers();

    // 2. Moving things. Order matters: later = in front.
    make_sun();
    make_sign();
    sprite_init(sRainbow, scr, draw_rainbow);

    clouds[0] = {120, 20, 14, 1.00f};
    clouds[1] = {520, 70, 22, 0.70f};
    clouds[2] = {-150, 35, 18, 0.85f};
    void (*cloudDraw[3])() = {draw_cloud0, draw_cloud1, draw_cloud2};
    for (int i = 0; i < 3; i++) sprite_init(sClouds[i], scr, cloudDraw[i]);

    sprite_init(sBird, scr, draw_bird);
    flag = box(scr, 185, 215, 30, 20, 0x2EC4B6, 3);
    tFlag.obj = flag;

    swings[0] = {558, 268, 150, 0.08f, 0, true};
    swings[1] = {648, 268, 150, -0.05f, 0, false};
    sprite_init(sSwings[0], scr, draw_swing0);
    sprite_init(sSwings[1], scr, draw_swing1);

    seesaw = {872, 430, 0, 0, 0, 0};
    sprite_init(sSeesaw, scr, draw_seesaw);
    sprite_init(sSlide, scr, draw_slide_kid);

    duck = {0, 0, 0, 99, 99};
    sprite_init(sDuck, scr, draw_duck);

    // the puppy lives inside a clipped doorway, so it can rise up from the floor
    lv_obj_t *doorway = box(scr, DOOR_X, DOOR_Y, DOOR_W, DOOR_H, 0, 0, LV_OPA_TRANSP);
    sprite_init(sPup, doorway, draw_pup);
    door = box(scr, DOOR_X, DOOR_Y, DOOR_W, DOOR_H, 0xB5651D, 3);
    box(door, 34, 30, 7, 7, 0xFFD93B, LV_RADIUS_CIRCLE);           // knob
    tDoor.obj = door;
    sprite_init(sDog, scr, draw_dog);                              // in front of the house

    // night tint over everything, with the glowing things in front of it
    night = box(scr, 0, 0, SCR_W, SCR_H, C_NIGHT, 0, LV_OPA_TRANSP);
    set_visible(night, false);
    sprite_init(sStars, scr, draw_stars);
    litWindow = box(scr, 913, 540, 48, 30, 0xFFD166, 2, LV_OPA_TRANSP);

    // 3. Invisible touch areas (on top of everything)
    lv_obj_add_event_cb(sun, on_sun, LV_EVENT_PRESSED, NULL);
    hotspot(scr, 455, 250, 295, 230, on_swings);
    hotspot(scr, 745, 360, 270, 105, on_seesaw);
    hotspot(scr, 590, 480, 120, 120, on_duck);
    hotspot(scr, 810, 465, 214, 135, on_house);

    // first placement of every sprite
    lv_obj_update_layout(scr);
    for (int i = 0; i < NUM_MOVING; i++) sprite_update(*moving[i]);
    apply_daylight(1);

    tickTimer = lv_timer_create(scene_tick, TICK_MS, NULL);
    lv_timer_pause(tickTimer);    // starts when the scene wakes
    return true;
}

void scene_wake()
{
    lastTickMs = 0;
    lv_timer_resume(tickTimer);

    // sunrise from behind the hill, with a little bounce at the top
    // (if it is dark, the sun stays down)
    lv_anim_del(sun, NULL);
    lv_obj_set_y(sun, SUN_HIDDEN_Y);
    dayShown = dayTarget;
    lv_coord_t target = (lv_coord_t)sun_y_for(dayShown);
    if (target < SUN_HIDDEN_Y) {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, sun);
        lv_anim_set_exec_cb(&a, anim_y);
        lv_anim_set_values(&a, SUN_HIDDEN_Y, target);
        lv_anim_set_time(&a, 1800);
        lv_anim_set_path_cb(&a, lv_anim_path_overshoot);
        lv_anim_start(&a);
    }
    apply_daylight(dayShown);
    dayApplied = dayShown;

    swings[0].om += 1.5f;     // the swing kid says hello
    seesaw.boost = 0.15f;
    audio_play(SFX_HELLO);
}

void scene_sleep()
{
    lv_timer_pause(tickTimer);
    lv_anim_del(sun, NULL);
    lv_obj_set_y(sun, SUN_HIDDEN_Y);   // ready to rise again next time
}

// ---- Sensor inputs ----
void scene_set_daylight(float day) { dayTarget = clampf(day, 0, 1); }
void scene_set_sound(float level)  { soundLevel = clampf(level, 0, 1); }
void scene_set_hand(float cm)      { handCm = cm; }

void scene_set_touchpad(bool touching)
{
    if (touching && !padWasTouched) dog_start();     // each new touch starts the show
    padWasTouched = touching;
}

// ---- Button actions ----
void scene_slide()
{
    if (slideT < 0) slideT = 0;
}

void scene_bird()
{
    if (birdT >= 0) return;
    birdT = 0;
    audio_play(SFX_TWEET);
}

void scene_rainbow()
{
    rainbowT = (rainbowT > 0.4f && rainbowT < 3.5f) ? 0.4f : 0;
    audio_play(SFX_SPARKLE);
}

void scene_flag_wave() { flagBoostT = 2.5f; }
