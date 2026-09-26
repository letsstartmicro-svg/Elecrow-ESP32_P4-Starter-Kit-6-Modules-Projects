// hardware.cpp - sensors and gadgets. Pins are from Elecrow's lessons.
//
// Nothing in here waits around: every function returns straight away (the
// ultrasonic check takes at most ~4 ms), so the animation keeps flowing.

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2c.h>
#include "hardware.h"

// ----- Pins -----
#define PIR_PIN        24     // Lesson 4, HIGH = movement
#define TOUCH_PIN      2      // Lesson 3, LOW = touched
#define BUTTON_PIN     16     // Lesson 7, four buttons on one analogue pin
#define US_TRIG_PIN    13     // Lesson 11, ultrasonic
#define US_ECHO_PIN    12
#define SERVO_PIN      25     // Lesson 8
#define RGB_PIN        8      // Lesson 2, two RGB LEDs
// NOTE: do NOT use GPIO 5 - on this kit it is the display's reset line
// (Elecrow's non-display lessons use it as an LED, but that blanks the screen).

// ----- Settings -----
#define SERVO_360      0      // 1 for kit V1.0 (360° servo), 0 for V1.1 (180° servo)
#define HAND_MAX_CM    45     // further than this = "no hand"
#define LIGHT_I2C_ADDR 0x5C   // BH1750 light sensor (Lesson 13)

// The servo signal is made by the RMT peripheral (the same kind of hardware
// that drives the RGB LEDs), NOT the LEDC/PWM timers - the screen backlight
// uses those, and sharing them made the screen dim when the servo moved.

static Adafruit_NeoPixel rgb(2, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// Setup
// ============================================================================
static bool lightOk = false;

static bool light_cmd(uint8_t cmd)
{
    // The light sensor shares the I2C wires with the touch screen. The display
    // library has already started that bus, so we just use it.
    return i2c_master_write_to_device(I2C_NUM_0, LIGHT_I2C_ADDR, &cmd, 1, pdMS_TO_TICKS(50)) == ESP_OK;
}

void hw_begin()
{
    pinMode(PIR_PIN, INPUT);
    pinMode(TOUCH_PIN, INPUT);
    pinMode(US_TRIG_PIN, OUTPUT);
    digitalWrite(US_TRIG_PIN, LOW);
    pinMode(US_ECHO_PIN, INPUT);
    analogSetPinAttenuation(BUTTON_PIN, ADC_11db);

    rgb.begin();
    rgb.setBrightness(60);
    rgb.show();

    lightOk = light_cmd(0x01) && light_cmd(0x10);   // power on, continuous high-res mode
    Serial.println(lightOk ? "Light sensor OK" : "Light sensor not found");
}

// ============================================================================
// Simple inputs
// ============================================================================
bool hw_pir()      { return digitalRead(PIR_PIN) == HIGH; }
bool hw_touchpad() { return digitalRead(TOUCH_PIN) == LOW; }

// ============================================================================
// Buttons: each button gives a different voltage (from Elecrow's key example)
// ============================================================================
static uint32_t lastButtonMv = 0;

static int button_from_mv(uint32_t mv)
{
    if (mv >= 1450 && mv < 1950) return 0;
    if (mv >= 1950 && mv < 2360) return 1;
    if (mv >= 2360 && mv < 2680) return 2;
    if (mv >= 2680 && mv < 3050) return 3;
    return -1;
}

int hw_button_pressed()
{
    static int last = -1, stable = -1;
    lastButtonMv = analogReadMilliVolts(BUTTON_PIN);
    int b = button_from_mv(lastButtonMv);
    int result = -1;
    if (b == last && b != stable) {          // same reading twice in a row = real
        if (b >= 0 && stable < 0) result = b; // new press
        stable = b;
    }
    last = b;
    return result;
}

uint32_t hw_button_mv() { return lastButtonMv; }

// ============================================================================
// Light sensor (read 5 times a second)
// ============================================================================
float hw_light_lux()
{
    static float lux = -1;
    static uint32_t lastMs = 0;
    if (!lightOk || millis() - lastMs < 200) return lux;
    lastMs = millis();
    uint8_t d[2];
    if (i2c_master_read_from_device(I2C_NUM_0, LIGHT_I2C_ADDR, d, 2, pdMS_TO_TICKS(50)) == ESP_OK)
        lux = ((d[0] << 8) | d[1]) / 1.2f;
    return lux;
}

// ============================================================================
// Ultrasonic (a new reading every 70 ms, smoothed)
// ============================================================================
float hw_hand_cm()
{
    static float hist[3] = {-1, -1, -1};
    static float result = -1;
    static uint32_t lastMs = 0, lastSeenMs = 0;
    if (millis() - lastMs < 70) return result;
    lastMs = millis();

    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);
    unsigned long us = pulseIn(US_ECHO_PIN, HIGH, 4000);   // 4 ms ≈ 68 cm max
    float cm = (us == 0) ? -1 : us / 58.0f;
    if (cm > HAND_MAX_CM) cm = -1;

    hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = cm;
    if (cm > 0) lastSeenMs = millis();

    // median of the last three valid readings removes odd spikes
    float v[3]; int n = 0;
    for (int i = 0; i < 3; i++) if (hist[i] > 0) v[n++] = hist[i];
    if (n == 0) {
        if (millis() - lastSeenMs > 250) result = -1;      // hand really gone
    } else if (n == 1) result = v[0];
    else if (n == 2) result = (v[0] + v[1]) / 2;
    else {
        float a = v[0], b = v[1], c = v[2];
        result = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
    }
    return result;
}

// ============================================================================
// Servo wave (a little sequence of positions)
// ============================================================================
struct Step { int16_t pos; uint16_t ms; };
#if SERVO_360
static const Step servoSteps[] = {{100, 700}, {80, 700}, {90, 100}};           // gentle spin one way, then back
static const int  SERVO_MIN_US = 1000, SERVO_MAX_US = 2000;
#else
static const Step servoSteps[] = {{40, 350}, {140, 350}, {40, 350}, {140, 350}, {90, 400}};
static const int  SERVO_MIN_US = 500, SERVO_MAX_US = 2500;
#endif
static int      servoStep = -1;
static uint32_t servoStepAt = 0;
static bool     servoAttached = false;
static uint32_t servoEndMs = 0;

static void servo_write(int angle)
{
    int us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
    rmt_data_t pulse;                 // one 20 ms frame: high for 'us', then low
    pulse.level0 = 1;
    pulse.duration0 = us;
    pulse.level1 = 0;
    pulse.duration1 = 20000 - us;
    rmtWriteLooping(SERVO_PIN, &pulse, 1);   // repeats by itself, 50 times a second
}

void hw_servo_wave()
{
    if (servoStep >= 0) return;                              // already waving
    if (!servoAttached) {
        servoAttached = rmtInit(SERVO_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000);  // 1 µs ticks
        if (!servoAttached) { Serial.println("Servo signal failed"); return; }
    }
    servoStep = 0;
    servoStepAt = millis();
    servo_write(servoSteps[0].pos);
}

// ============================================================================
// LED party
// ============================================================================
static uint32_t ledsUntil = 0;   // when the LED party ends (0 = none yet)

void hw_led_party()
{
    ledsUntil = millis() + 3500;
}

bool hw_servo_busy()
{
    // the servo arm can shade the light sensor, so ignore it while moving
    return servoStep >= 0 || (servoEndMs && millis() - servoEndMs < 1500);
}

bool hw_leds_busy()
{
    // the LEDs sit right next to the light sensor, so their flashing would
    // look like day/night changes - ignore the light sensor until 1.5 s after
    return ledsUntil && millis() < ledsUntil + 1500;
}

void hw_leds_off()
{
    if (ledsUntil > millis()) ledsUntil = millis();
    rgb.clear();
    rgb.show();
}

// ============================================================================
// Keep sequences running
// ============================================================================
void hw_update()
{
    uint32_t now = millis();

    if (servoStep >= 0 && now - servoStepAt >= servoSteps[servoStep].ms) {
        servoStepAt = now;
        servoStep++;
        if (servoStep >= (int)(sizeof(servoSteps) / sizeof(servoSteps[0]))) {
            servoStep = -1;
            rmtDeinit(SERVO_PIN);                            // stop pulses: no buzzing
            servoAttached = false;
            pinMode(SERVO_PIN, OUTPUT);
            digitalWrite(SERVO_PIN, LOW);
            servoEndMs = now;
        } else {
            servo_write(servoSteps[servoStep].pos);
        }
    }

    static bool ledsOn = false;
    if (ledsUntil && now < ledsUntil) {
        static uint32_t lastMs = 0;
        if (now - lastMs >= 30) {
            lastMs = now;
            uint16_t hue = (now * 40) & 0xFFFF;              // rainbow cycle
            rgb.setPixelColor(0, rgb.gamma32(rgb.ColorHSV(hue)));
            rgb.setPixelColor(1, rgb.gamma32(rgb.ColorHSV(hue + 32768)));
            rgb.show();
            ledsOn = true;
        }
    } else if (ledsOn) {
        ledsOn = false;
        hw_leds_off();
    }
}
