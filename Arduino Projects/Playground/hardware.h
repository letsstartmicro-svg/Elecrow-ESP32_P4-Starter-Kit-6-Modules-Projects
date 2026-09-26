// hardware.h - the sensors and gadgets on the kit's module board
// All functions are called from loop() only.
#pragma once
#include <stdint.h>

void  hw_begin();            // call once in setup, AFTER the display has started

bool  hw_pir();              // true = movement
bool  hw_touchpad();         // true = finger on the touch pad
int   hw_button_pressed();   // 0..3 when a button has just been pressed, else -1
float hw_light_lux();        // latest light reading (lux), -1 if not available
float hw_hand_cm();          // distance to a hand (cm), -1 if nothing in range
uint32_t hw_button_mv();     // raw button voltage, for tuning

void  hw_servo_wave();       // start the servo doing a little wave
void  hw_led_party();        // rainbow on the two RGB LEDs for a few seconds
void  hw_leds_off();
bool  hw_servo_busy();       // true while the servo moves (and a moment after)
bool  hw_leds_busy();        // true while the RGB LEDs are flashing (and a moment after)
void  hw_update();           // keeps servo / LED sequences running - call every loop
