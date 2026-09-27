// audio.cpp - a tiny synthesiser for the speaker, and a loudness meter for
// the microphone. Each runs in its own background task on core 0, so sound
// never makes the animation stutter.
//
// Pins and settings are the same as Elecrow's Lesson 16 (mic) and 17 (speaker).

#include <Arduino.h>
#include <ESP_I2S.h>
#include <math.h>
#include "audio.h"

// ----- Pins -----
#define AUDIO_GPIO_CTRL     6     // amplifier enable, LOW = on
#define AUDIO_GPIO_LRCLK    21
#define AUDIO_GPIO_BCLK     22
#define AUDIO_GPIO_SDATA    23
#define MIC_GPIO_CLK        3
#define MIC_GPIO_SDIN       4

// ----- Settings -----
#define SAMPLE_RATE         16000
#define BLOCK               256          // samples per speaker write (16 ms)
#define MASTER_VOLUME       0.35f        // 0..1  <- turn this down if it is too loud
#define MIC_SENSITIVITY_DB  24.0f        // smaller = more sensitive (dB above room noise for "full")
#define MIC_THRESHOLD_DB    6.0f         // ignore sounds less than this above room noise

static I2SClass spk;
static I2SClass mic;

// ----------------------------------------------------------------------------
// Sound effects: lists of notes. f0 -> f1 is a slide in pitch; f0 = 0 is a rest.
// ----------------------------------------------------------------------------
struct Note {
    uint16_t f0, f1;   // start / end frequency (Hz)
    uint16_t ms;       // length
    uint8_t  vol;      // 0..100
    uint8_t  buzzy;    // 0 = smooth (sine), 1 = buzzy (for the bark)
};

static const Note fxHello[]   = {{523, 523, 110, 70, 0}, {659, 659, 110, 70, 0}, {784, 784, 110, 70, 0}, {1047, 1047, 260, 70, 0}};
static const Note fxWoof[]    = {{330, 170, 110, 90, 1}, {0, 0, 70, 0, 0}, {300, 150, 140, 90, 1}};
static const Note fxWhee[]    = {{500, 1400, 260, 70, 0}, {1400, 600, 700, 70, 0}};
static const Note fxTweet[]   = {{2300, 3200, 70, 50, 0}, {0, 0, 50, 0, 0}, {2300, 3200, 70, 50, 0}, {0, 0, 50, 0, 0}, {2600, 3400, 90, 50, 0}};
static const Note fxSparkle[] = {{1047, 1047, 90, 55, 0}, {1319, 1319, 90, 55, 0}, {1568, 1568, 90, 55, 0}, {2093, 2093, 300, 55, 0}};
static const Note fxBoing[]   = {{180, 520, 120, 80, 0}, {520, 260, 250, 70, 0}};

#define FX(a) {a, sizeof(a) / sizeof(a[0])}
static const struct { const Note *notes; int n; } effects[SFX_COUNT] = {
    FX(fxHello), FX(fxWoof), FX(fxWhee), FX(fxTweet), FX(fxSparkle), FX(fxBoing)
};

// ----------------------------------------------------------------------------
// Shared state (written by other tasks, read by the audio tasks)
// ----------------------------------------------------------------------------
static volatile int      pendingSfx = -1;
static volatile float    toneTarget = 0;
static volatile bool     enabled = false;
static volatile uint32_t speakerBusyUntil = 0;
static volatile float    micLevel = 0;
static volatile float    micDb = 0;
static bool              started = false;

// ----------------------------------------------------------------------------
// Speaker task
// ----------------------------------------------------------------------------
static void speaker_task(void *)
{
    static int16_t buf[BLOCK];
    const float TWO_PI_F = 6.2831853f;

    float tonePhase = 0, toneFreq = 0, toneAmp = 0;
    const Note *fx = nullptr;
    int fxN = 0, fxIdx = 0, notePos = 0, noteLen = 0;
    float fxPhase = 0;

    for (;;) {
        if (!enabled) {                       // screen asleep: stay quiet
            fx = nullptr;
            toneAmp = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int req = pendingSfx;
        if (req >= 0) {                       // start a new effect
            pendingSfx = -1;
            fx = effects[req].notes;
            fxN = effects[req].n;
            fxIdx = 0;
            notePos = 0;
            noteLen = fx[0].ms * SAMPLE_RATE / 1000;
        }

        bool active = false;
        for (int i = 0; i < BLOCK; i++) {
            float s = 0;

            // --- the hand tone (smooth glide between notes) ---
            float target = toneTarget;
            if (target > 0) {
                if (toneAmp < 0.01f) toneFreq = target;
                toneFreq += (target - toneFreq) * 0.003f;
                toneAmp += (1.0f - toneAmp) * 0.004f;
            } else {
                toneAmp *= 0.997f;
            }
            if (toneAmp > 0.001f) {
                tonePhase += toneFreq / SAMPLE_RATE;
                if (tonePhase >= 1) tonePhase -= 1;
                s += toneAmp * 0.55f * (sinf(TWO_PI_F * tonePhase) + 0.2f * sinf(2 * TWO_PI_F * tonePhase));
                active = true;
            }

            // --- sound effect ---
            if (fx) {
                const Note &n = fx[fxIdx];
                if (n.f0 > 0) {
                    float t = (float)notePos / noteLen;
                    float f = n.f0 + (n.f1 - n.f0) * t;
                    float env = fminf(1.0f, fminf(notePos / 80.0f, (noteLen - notePos) / 160.0f));
                    fxPhase += f / SAMPLE_RATE;
                    if (fxPhase >= 1) fxPhase -= 1;
                    float w = sinf(TWO_PI_F * fxPhase);
                    if (n.buzzy) w = tanhf(3.0f * w) * 0.9f;
                    s += env * (n.vol / 100.0f) * w;
                }
                active = true;
                if (++notePos >= noteLen) {
                    notePos = 0;
                    if (++fxIdx >= fxN) fx = nullptr;
                    else noteLen = fx[fxIdx].ms * SAMPLE_RATE / 1000;
                }
            }

            float v = s * MASTER_VOLUME * 32767.0f;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            buf[i] = (int16_t)v;
        }

        if (active) speakerBusyUntil = millis() + 200;   // tells the mic to ignore us
        spk.write((uint8_t *)buf, sizeof(buf));
    }
}

// ----------------------------------------------------------------------------
// Microphone task: measures loudness about 30 times a second
// ----------------------------------------------------------------------------
static void mic_task(void *)
{
    static int16_t buf[512];
    float floorDb = -1;
    float level = 0;

    for (;;) {
        size_t got = mic.readBytes((char *)buf, sizeof(buf)) / 2;
        if (got == 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        float mean = 0;
        for (size_t i = 0; i < got; i++) mean += buf[i];
        mean /= got;
        float sq = 0;
        for (size_t i = 0; i < got; i++) { float d = buf[i] - mean; sq += d * d; }
        float db = 20.0f * log10f(sqrtf(sq / got) + 1.0f);
        micDb = db;

        // Room-noise level: follows quiet moments quickly, loud moments slowly
        if (floorDb < 0 || db < floorDb) floorDb = db;
        else floorDb += 0.01f;

        float target = (db - floorDb - MIC_THRESHOLD_DB) / MIC_SENSITIVITY_DB;
        if (target < 0) target = 0;
        if (target > 1) target = 1;
        if (millis() < speakerBusyUntil) target = 0;     // don't react to our own sounds

        level = (target > level) ? target : level * 0.85f; // quick up, gentle down
        micLevel = level;
    }
}

// ----------------------------------------------------------------------------
// Public functions
// ----------------------------------------------------------------------------
bool audio_begin()
{
    pinMode(AUDIO_GPIO_CTRL, OUTPUT);
    digitalWrite(AUDIO_GPIO_CTRL, HIGH);              // amplifier off for now

    mic.setPinsPdmRx(MIC_GPIO_CLK, MIC_GPIO_SDIN);
    if (!mic.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
        Serial.println("Microphone start failed");
        return false;
    }
    spk.setPins(AUDIO_GPIO_BCLK, AUDIO_GPIO_LRCLK, AUDIO_GPIO_SDATA);
    if (!spk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_BOTH)) {
        Serial.println("Speaker start failed");
        return false;
    }
    xTaskCreatePinnedToCore(speaker_task, "speaker", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(mic_task, "mic", 4096, NULL, 4, NULL, 0);
    started = true;
    return true;
}

void audio_enable(bool on)
{
    if (!started) return;
    enabled = on;
    digitalWrite(AUDIO_GPIO_CTRL, on ? LOW : HIGH);
}

void audio_play(Sfx s)
{
    if (s >= 0 && s < SFX_COUNT) pendingSfx = s;
}

void audio_tone(float hz)
{
    toneTarget = hz;
}

float audio_mic_level() { return micLevel; }
float audio_mic_db()    { return micDb; }
