#include <Arduino.h>
#include <FastLED.h>
#include "test.h"
#include "config.h"
#include "globals.h"

void test() {
    Serial.println("Starting Frequency Sweep (200Hz - 10000Hz)...");

    for (int freq = 100; freq <= 20000; freq += 100) {
        ledcWriteTone(speakerChannel, freq);
        Serial.printf("Frequency: %i\n", freq);
        delay(40);
    }

    ledcWriteTone(speakerChannel, 0);
    Serial.println("Sweep finished.");

    Serial.println("Motor starts up...");
    for (int dutyCycle = 0; dutyCycle <= 4095; dutyCycle += 8) {
        ledcWrite(motorChannel, dutyCycle);
        Serial.printf("Speed: %i\n", dutyCycle);
        delay(20);
    }

    Serial.println("Maximum speed reached.");
    delay(1000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);
    Serial.println("0");
    delay(1000);

    Serial.println("Motor slows down...");
    for (int dutyCycle = 4095; dutyCycle >= 0; dutyCycle -= 8) {
        ledcWrite(motorChannel, dutyCycle);
        Serial.printf("Speed: %i\n", dutyCycle);
        delay(20);
    }

    Serial.println("Start LED rainbow test.");
    FastLED.setBrightness(255);
    for (int hue = 0; hue < 256; hue++) {
        leds[0] = CHSV(hue, 255, 255);
        FastLED.show();
        Serial.printf("Hue: %i\n", hue);
        delay(15);
    }

    leds[0] = CRGB::White;
    FastLED.setBrightness(ledBrightness);
    FastLED.show();

    Serial.println("Finished test.");
}
