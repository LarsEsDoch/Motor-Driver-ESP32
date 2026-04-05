#include <Arduino.h>
#include <FastLED.h>

#define STATUS_LED_PIN 48

CRGB leds[1];

#define POT_PIN 4

#define EMERGENCY_STOP_BUTTON_PIN 5

#define SPEAKER_PIN 7
#define MOTOR_PIN 6

void setup() {
    Serial.begin(921600);
    delay(2000);

    pinMode(EMERGENCY_STOP_BUTTON_PIN, INPUT_PULLUP);

    pinMode(STATUS_LED_PIN, OUTPUT);

    FastLED.addLeds<WS2812B, STATUS_LED_PIN, GRB>(leds, 1);
    FastLED.setBrightness(127);

    pinMode(MOTOR_PIN, OUTPUT);

    Serial.println("System ready!");
}

void test() {
    Serial.println("Motor starts up...");
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
        analogWrite(MOTOR_PIN, dutyCycle);
        delay(20);
    }

    Serial.println("Maximum speed reached.");
    delay(3000);

    Serial.println("Motor slows down...");
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        analogWrite(MOTOR_PIN, dutyCycle);
        delay(20);
    }
}
void loop() {
    FastLED.setBrightness(127);
    leds[0] = CRGB::Green;
    FastLED.show();
    
    test();

    delay(10);
}