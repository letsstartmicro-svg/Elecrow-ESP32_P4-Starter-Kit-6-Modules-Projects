// scene.h - the playground picture and everything that moves in it.
// All functions must be called with the LVGL lock held (lvgl_port_lock).
#pragma once

bool scene_create();   // build the whole playground (call once in setup); false = out of memory
void scene_wake();     // "hello" animation: sun rises, swing gets a push
void scene_sleep();    // pause all animation while the screen is dark

// Sensor inputs (call as often as you like)
void scene_set_daylight(float day);     // 0 = night ... 1 = full day   (light sensor)
void scene_set_sound(float level);      // 0 = quiet ... 1 = loud       (microphone)
void scene_set_hand(float cm);          // hand distance in cm, -1 = none (ultrasonic)
void scene_set_touchpad(bool touching); // opens the cubby house door   (touch pad)

// Button actions
void scene_slide();      // a kid goes down the slide
void scene_bird();       // a bird flies across
void scene_rainbow();    // a rainbow appears
void scene_flag_wave();  // the flag flaps hard (goes with the servo)
