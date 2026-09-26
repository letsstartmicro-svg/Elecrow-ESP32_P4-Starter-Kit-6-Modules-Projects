// =============================================================================
//  Interactive Playground  -  Stage 3: sensors, puppy adventure, sign
//  Elecrow All-in-one Starter Kit for ESP32-P4 (7" 1024x600, LVGL 8.3)
//
//  What each part of the kit does:
//   PIR sensor      - wakes the playground up (screen dark until someone moves)
//   Light sensor    - cover it: sunset, then night with stars, moon and a lit window
//   Microphone      - noise makes the spring duck wobble and "sing" (beak opens)
//   Ultrasonic      - hand distance tilts the see-saw and plays a musical note
//   Touch pad       - puppy adventure: door opens, puppy barks, runs out, hops, runs home
//   Button 1        - servo waves (and the flag flaps)
//   Button 2        - RGB LEDs party + a rainbow in the sky
//   Button 3        - a kid goes down the slide ("wheee")
//   Button 4        - a bird flies across ("tweet")
//   Screen taps     - sun, swings, see-saw, duck and cubby house all react too
//
//  Files:
//   Playground.ino  - this file: wake/sleep and connecting sensors to the scene
//   scene.cpp/.h    - the picture and all animation
//   sign_img.h      - the "Elecrow Playground" sign picture
//   hardware.cpp/.h - sensors, servo and LEDs
//   audio.cpp/.h    - speaker sounds and microphone loudness
//   the rest        - Elecrow's display/touch setup (unchanged, from Lesson 10)
//
//  Libraries: same as Elecrow's lessons (ESP32_Display_Panel, lvgl 8.3.11,
//  Adafruit NeoPixel). The servo library is NOT needed.
// =============================================================================

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "scene.h"
#include "hardware.h"
#include "audio.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ----- Behaviour -----
#define SLEEP_AFTER_MS   (3UL * 60UL * 1000UL)   // go dark after 3 minutes of nothing
#define BRIGHTNESS_MAX   100    // backlight %, lower it if the screen is too bright
#define FADE_IN_STEP     4      // % per loop (loop runs every 20 ms)
#define FADE_OUT_STEP    1

// Light sensor: at or below LUX_NIGHT it is night, at or above LUX_DAY full day.
// Watch the Serial Monitor to see what your room reads, then adjust.
#define LUX_NIGHT        3.0f
#define LUX_DAY          60.0f

#define DEBUG_SENSORS    1      // 1 = print sensor readings once a second

// Notes for the hand tone (a pentatonic scale always sounds nice)
static const float NOTES[] = {523, 587, 659, 784, 880, 1047, 1175, 1319};
#define NUM_NOTES   8
#define HAND_NEAR   5.0f
#define HAND_FAR    40.0f

Board *board = nullptr;

enum PlayState { ASLEEP, AWAKE, FADING_OUT };
PlayState state          = ASLEEP;
int       brightness     = 0;
uint32_t  lastActivityMs = 0;
bool      activitySeen   = false;

// ESP32-P4's MIPI display needs these power rails (same as Elecrow's examples)
static void board_p4_ldo_init()
{
    esp_ldo_channel_handle_t ldo3 = NULL, ldo4 = NULL;
    esp_ldo_channel_config_t ldo3_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    esp_ldo_channel_config_t ldo4_cfg = { .chan_id = 4, .voltage_mv = 3300 };
    if (esp_ldo_acquire_channel(&ldo3_cfg, &ldo3) != ESP_OK) Serial.println("LDO3 error");
    if (esp_ldo_acquire_channel(&ldo4_cfg, &ldo4) != ESP_OK) Serial.println("LDO4 error");
}

// Light level, smoothed over about a second, and frozen while the RGB LEDs
// flash (they are right next to the sensor and would fool it).
static float read_light()
{
    static float smooth = -1;
    float raw = hw_light_lux();
    if (raw < 0) return smooth;
    if (hw_leds_busy() || hw_servo_busy()) return smooth;   // LEDs / servo arm fool the sensor
    smooth = (smooth < 0) ? raw : smooth + (raw - smooth) * 0.05f;   // loop runs ~50x a second
    return smooth;
}

static float lux_to_daylight(float lux)
{
    if (lux < 0) return 1;                       // no light sensor: always day
    float lo = log10f(LUX_NIGHT + 1), hi = log10f(LUX_DAY + 1);
    float d = (log10f(lux + 1) - lo) / (hi - lo);
    return d < 0 ? 0 : (d > 1 ? 1 : d);
}

// Hand distance -> musical note (with a little "stickiness" so it doesn't flicker)
static float hand_to_note(float cm)
{
    static int note = -1;
    if (cm < 0) { note = -1; return 0; }
    float f = (cm - HAND_NEAR) / (HAND_FAR - HAND_NEAR);
    f = f < 0 ? 0 : (f > 1 ? 1 : f);
    float pos = (1 - f) * (NUM_NOTES - 1);       // closer = higher
    if (note < 0 || fabsf(pos - note) > 0.7f) note = (int)lroundf(pos);
    return NOTES[note];
}

void setup()
{
    // Same start-up pin settings as all of Elecrow's display examples
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);
    digitalWrite(32, 0);
    digitalWrite(33, 1);

    Serial.begin(115200);
    Serial.println("Playground stage 3 starting");

    board_p4_ldo_init();
    board = new Board();
    board->init();
    board->begin();
    lvgl_port_init(board->getLCD(), board->getTouch());
    board->getBacklight()->setBrightness(0);     // start dark

    hw_begin();                                  // after the display (shares its I2C bus)
    if (!audio_begin()) Serial.println("Audio not available - carrying on without sound");

    lvgl_port_lock(-1);
    bool ok = scene_create();
    lvgl_port_unlock();
    if (!ok) {
        board->getBacklight()->setBrightness(BRIGHTNESS_MAX);   // show whatever we have
        while (true) delay(1000);
    }

    Serial.println("Ready - waiting for movement");
}

void loop()
{
    uint32_t now = millis();

    // ---- Read everything ----
    bool  motion = hw_pir();
    bool  pad    = hw_touchpad();
    int   button = hw_button_pressed();
    float lux    = read_light();
    float handCm = hw_hand_cm();
    float sound  = audio_mic_level();
    hw_update();

    lvgl_port_lock(-1);
    uint32_t touchIdleMs = lv_disp_get_inactive_time(NULL);   // time since last screen touch
    lvgl_port_unlock();
    bool screenTouched = (now > 3000) && (touchIdleMs < 300);  // ignore the first 3 s after boot

    // Anything a child does counts as "someone is here"
    bool someone = motion || pad || button >= 0 || handCm > 0 || screenTouched;
    if (someone) { lastActivityMs = now; activitySeen = true; }
    uint32_t idleMs = activitySeen ? (now - lastActivityMs) : UINT32_MAX;

    // ---- Wake / sleep ----
    switch (state) {
    case ASLEEP:
        if (someone) {
            Serial.println("Waking up");
            audio_enable(true);
            lvgl_port_lock(-1);
            scene_set_daylight(lux_to_daylight(lux));
            scene_wake();
            lvgl_port_unlock();
            state = AWAKE;
        }
        break;

    case AWAKE:
        if (idleMs > SLEEP_AFTER_MS) {
            Serial.println("Nobody around - fading out");
            state = FADING_OUT;
        }
        break;

    case FADING_OUT:
        if (someone) {
            state = AWAKE;                        // someone came back
        } else if (brightness == 0) {
            lvgl_port_lock(-1);
            scene_sleep();
            lvgl_port_unlock();
            audio_tone(0);
            audio_enable(false);
            hw_leds_off();
            state = ASLEEP;
            Serial.println("Asleep");
        }
        break;
    }

    // ---- Feed the playground ----
    if (state != ASLEEP) {
        lvgl_port_lock(-1);
        scene_set_daylight(lux_to_daylight(lux));
        scene_set_sound(sound);
        scene_set_hand(handCm);
        scene_set_touchpad(pad);
        switch (button) {
        case 0: hw_servo_wave(); scene_flag_wave(); break;
        case 1: hw_led_party();  scene_rainbow();   break;
        case 2: scene_slide();                      break;
        case 3: scene_bird();                       break;
        }
        lvgl_port_unlock();
        audio_tone(hand_to_note(handCm));
    }
    if (button >= 0) Serial.printf("Button %d pressed (%u mV)\n", button + 1, (unsigned)hw_button_mv());

#if DEBUG_SENSORS
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.printf("light %.0f lux (day %.2f) | hand %.0f cm | mic %.0f dB (level %.2f) | "
                      "buttons %u mV | pad %d | PIR %d\n",
                      lux, lux_to_daylight(lux), handCm, audio_mic_db(), sound,
                      (unsigned)hw_button_mv(), pad, motion);
    }
#endif

    // ---- Backlight fade ----
    int target = (state == AWAKE) ? BRIGHTNESS_MAX : 0;
    static uint32_t lastBacklightMs = 0;
    if (brightness != target) {
        if (brightness < target) brightness = min(brightness + FADE_IN_STEP, target);
        else                     brightness = max(brightness - FADE_OUT_STEP, target);
        board->getBacklight()->setBrightness(brightness);
        lastBacklightMs = now;
    } else if (now - lastBacklightMs > 2000) {
        board->getBacklight()->setBrightness(brightness);   // safety net: re-apply every 2 s
        lastBacklightMs = now;
    }

    // This pause only paces the sensor checks. The animation runs in its own
    // LVGL task, so it keeps moving smoothly regardless.
    delay(20);
}
