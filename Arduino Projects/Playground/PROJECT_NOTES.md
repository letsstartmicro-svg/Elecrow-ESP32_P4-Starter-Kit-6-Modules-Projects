# Interactive Playground - project notes

Kit: Elecrow All-in-one Starter Kit for ESP32-P4 (7" 1024x600 touch, LVGL 8.3.11,
Arduino core 3.3.3, Elecrow's library bundle). Built for a 4-year-old.

## Status (26 Sep 2026)
- Stage 1 (scene + PIR wake/sleep): tested on the real board - "looks perfect".
- Stage 2 (all sensors): running on the board; everything works except
  button 2 (rainbow) made the screen dim and stay dim.
- Stage 3 (26 Sep, awaiting test on board):
  * Dimming fix: light sensor ignored while RGB LEDs flash (they sit next to it),
    light readings smoothed, backlight re-applied every 2 s.
  * Touch pad now starts a "puppy adventure" (door opens, peek + woof, runs out,
    happy hops with hearts, runs home). Screen-tapping the house does the same.
  * Vintage "Elecrow Playground" sign on the right hill (sign_img.h, made by
    make_sign.py.txt with the TeX Gyre Bonum Bold font); its bulbs light at night.
  Result: button 2 dimming fixed. Button 1 (servo) still dimmed the screen ->
  servo moved from LEDC to RMT + light sensor frozen while servo moves (awaiting test).
  If the screen only dims WHILE the servo moves and recovers, suspect a power dip
  (try a stronger USB supply / powered hub).

## Design (important if changing anything)
- Static scenery is painted once into a 1024x600 canvas in PSRAM.
- Each moving thing is one "sprite" object that draws its own parts
  (keeps LVGL's 48 KB memory at ~36%; 130 separate objects hit 96% and froze).
- scene_tick moves sprites with redraw tracking paused, then invalidates one
  rectangle per moved sprite (LVGL's 32-area limit otherwise forces full-screen redraws).
- Light sensor shares I2C port 0 (SDA 18 / SCL 19) with touch; uses the legacy
  i2c driver the display library already installed - do not call i2c_init or use Wire.
- GPIO 5 is the LCD reset line - never use it (the lessons' "LED on pin 5" blanks the screen).
- Servo signal is generated with RMT (rmtWriteLooping), NOT LEDC: the backlight
  uses LEDC and sharing it (ESP32Servo, or even a separate LEDC channel) dimmed
  the screen when the servo moved. Light sensor is also ignored while the servo
  moves (its arm can shade the sensor).

## Pins
PIR 24 | touch pad 2 (LOW=touched) | buttons 16 (analog ladder) | ultrasonic trig 13 / echo 12
servo 25 | RGB LEDs 8 | mic PDM clk 3 / data 4 | speaker BCLK 22, LRCLK 21, DOUT 23,
amp enable 6 (LOW=on) | backlight 20

## Settings to tune
- Playground.ino: LUX_NIGHT, LUX_DAY, SLEEP_AFTER_MS, BRIGHTNESS_MAX, DEBUG_SENSORS
- audio.cpp: MASTER_VOLUME, MIC_SENSITIVITY_DB, MIC_THRESHOLD_DB
- scene.cpp: DUCK_SOUND_GAIN, HAND_NEAR_CM / HAND_FAR_CM, DOOR_STAY_MS
- hardware.cpp: SERVO_360 (1 = kit V1.0), button voltage bands in button_from_mv()

## Ideas not done yet
- Construction-site scene with a crane (ultrasonic lifts the hook)
- More screen-tap reactions, twinkling stars
