// audio.h - speaker sounds and microphone loudness
// These functions are safe to call from any task (loop, LVGL, ...).
#pragma once

enum Sfx {
    SFX_HELLO,      // little "ta-da" when the playground wakes up
    SFX_WOOF,       // puppy bark
    SFX_WHEE,       // going down the slide
    SFX_TWEET,      // bird
    SFX_SPARKLE,    // rainbow
    SFX_BOING,      // duck tapped
    SFX_COUNT
};

bool  audio_begin();                 // start speaker + microphone (call once in setup)
void  audio_enable(bool on);         // amplifier on/off (off while the screen sleeps)
void  audio_play(Sfx s);             // play a sound effect
void  audio_tone(float hz);          // musical tone that follows the hand; 0 = silent
float audio_mic_level();             // 0 (quiet) .. 1 (loud)
float audio_mic_db();                // raw loudness in dB, for tuning
